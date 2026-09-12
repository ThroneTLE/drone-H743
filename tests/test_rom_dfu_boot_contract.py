"""Safety and reset-context contract for the STM32H743 factory USB DFU path."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "app_boot.h"
SOURCE = ROOT / "App" / "Src" / "app_boot.c"
# 2026-09-11：跳 ROM 的**机制**整组搬到 BSP（寄存器、栈切换、备份域），
# app_boot.c 只留"什么时候准跳"的判据——那部分是纯逻辑，宿主侧能测。
MECHANISM = ROOT / "BSP" / "Src" / "bsp_rom_bootloader.c"
CONTROL = ROOT / "App" / "Src" / "app_control.c"
PROTO = ROOT / "App" / "Inc" / "app_proto.h"
MAIN = ROOT / "Core" / "Src" / "main.c"
CMAKE = ROOT / "CMakeLists.txt"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def c_function_body(source: str, signature: str) -> str:
    start = source.rindex(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_boot_command_is_explicit_scheduled_and_safety_gated() -> None:
    header = read(HEADER)
    source = read(SOURCE)
    control = read(CONTROL)
    delay = re.search(r"APP_BOOT_DFU_SCHEDULE_DELAY_MS\s+(\d+)U", header)

    assert delay is not None and int(delay.group(1)) >= 500
    assert "#define APP_BOOT_ESC_SAFE_MAX_US       1100U" in header
    assert 'strcmp(tokens[0], "BOOT?") == 0' in control
    assert 'strcmp(tokens[1], "DFU") != 0' in control
    assert 'strcmp(tokens[2], "CONFIRM") != 0' in control
    assert '"ERR usage BOOT DFU CONFIRM\\r\\n"' in control

    handle = c_function_body(
        control, "static void app_control_handle_boot(char **tokens, uint32_t count)"
    )
    request_at = handle.index("APP_Boot_RequestDfu()")
    usb_at = handle.index("app_control_send_boot_scheduled()")
    confirm_at = handle.index("APP_Boot_ConfirmDfuScheduled()")
    assert request_at < usb_at < confirm_at
    assert "APP_PROTO_MSG_BOOT_STATUS" in handle
    assert '"BOOT mode=dfu state=refused reason=%s\\r\\n"' in control
    assert '"BOOT mode=dfu state=cancelled reason=%s\\r\\n"' in control

    request = c_function_body(
        source, "APP_BootRequestResult APP_Boot_RequestDfu(void)"
    )
    tick = c_function_body(source, "APP_BootEvent APP_Boot_Tick(void)")
    for body in (request, tick):
        assert "app_boot_read_safety" in body
        assert "APP_Boot_IsVectorReasonable" in body
    assert "app_boot_request_sequence = sample.sequence" in request
    assert "APP_BOOT_STATE_WAITING_USB_REPLY" in request
    assert "SVC_Timestamp_Us()" in request
    assert "BSP_RomBootloader_WriteRequestMagic" in tick
    assert tick.index("app_boot_read_safety") < tick.index(
        "BSP_RomBootloader_WriteRequestMagic")
    assert "BSP_RomBootloader_Jump" not in tick
    assert tick.index("BSP_Critical_Enter();") < tick.index("app_boot_read_safety")
    assert tick.index("app_boot_read_vector") < tick.index("BSP_PWM_DisableEsc(1U)")
    assert "BSP_PWM_DisableEsc(2U)" in tick
    assert "APP_Boot_HasSequenceAdvanced" in tick

    usb_reply = c_function_body(
        control, "static uint8_t app_control_send_boot_scheduled(void)"
    )
    assert "APP_USB_CDC_Write" in usb_reply
    assert "100U" in usb_reply
    assert usb_reply.index("APP_USB_CDC_Write") < usb_reply.index("osMessageQueuePut")


def test_reset_magic_moves_the_rom_jump_out_of_the_freertos_task() -> None:
    header = read(HEADER)
    source = read(SOURCE)
    main = read(MAIN)

    mechanism = read(MECHANISM)

    assert "APP_BOOT_DFU_REQUEST_MAGIC" in header
    # 魔数由 App 传进来，备份寄存器由 BSP 写——两边都不越界。
    assert "BSP_RomBootloader_WriteRequestMagic(APP_BOOT_DFU_REQUEST_MAGIC)" in source
    assert "RTC->BKP0R = magic" in mechanism
    assert "RTC->BKP1R = (uint32_t)~magic" in mechanism
    assert "RTC->BKP0R = 0U" in mechanism
    assert "RTC->BKP1R = 0U" in mechanism
    assert "RTC->BKP" not in source, "备份寄存器不该再出现在 App 层"
    backup_access = c_function_body(
        mechanism, "static uint8_t rom_bootloader_open_backup("
    )
    assert "#if defined(__HAL_RCC_PWR_CLK_ENABLE)" in backup_access
    assert "__HAL_RCC_PWR_CLK_ENABLE();" in backup_access
    assert "HAL_PWR_EnableBkUpAccess();" in backup_access
    assert "readback = PWR->CR1 & PWR_CR1_DBP;" in backup_access
    assert "__HAL_RCC_RTC_CLK_ENABLE();" in backup_access
    assert "readback = RCC->APB4ENR & RCC_APB4ENR_RTCAPBEN;" in backup_access
    assert backup_access.count("__DSB();") >= 2
    assert "attempts = 1024U;" in backup_access
    assert "return (readback != 0U) ? 1U : 0U;" in backup_access
    assert "HAL_Delay" not in backup_access

    tick = c_function_body(source, "APP_BootEvent APP_Boot_Tick(void)")
    # USB 栈的停机 2026-09-11 收进 BSP 的薄壳（hUsbDeviceFS 是 CubeMX 生成的句柄）。
    # 顺序不变且仍被钉住：先跟主机说"设备要走了"，再复位；反过来的话上位机会
    # 留着一个没人应答的串口，看起来像是飞控挂了。
    assert "BSP_UsbCdc_Teardown();" in tick
    assert "BSP_RomBootloader_SystemReset();" in tick
    assert tick.index("BSP_UsbCdc_Teardown") < tick.index("BSP_RomBootloader_SystemReset")
    usb_shim = read(ROOT / "BSP" / "Src" / "bsp_usb_cdc.c")
    assert "USBD_Stop(&hUsbDeviceFS)" in usb_shim
    assert "USBD_DeInit(&hUsbDeviceFS)" in usb_shim
    assert "NVIC_SystemReset()" in read(MECHANISM)
    assert "__enable_irq" not in tick

    begin = main.index("/* USER CODE BEGIN 1 */")
    hook = main.index("APP_Boot_TryRomDfu();", begin)
    end = main.index("/* USER CODE END 1 */", hook)
    mpu = main.index("MPU_Config();", end)
    assert begin < hook < end < mpu
    assert '#include "app_boot.h"' in main

    early = c_function_body(source, "void APP_Boot_TryRomDfu(void)")
    # 取魔数时就清掉，且必须发生在校验向量之前：任何一步失败都该正常启动应用，
    # 而不是再跳一次——"永远进 DFU"要拆机才救得回来。
    assert early.index("BSP_RomBootloader_TakeRequestMagic") < early.index(
        "APP_Boot_IsVectorReasonable"
    )
    assert early.index("APP_Boot_IsVectorReasonable") < early.index(
        "BSP_RomBootloader_Jump"
    )
    take = c_function_body(
        mechanism, "uint8_t BSP_RomBootloader_TakeRequestMagic(uint32_t magic)")
    assert "rom_bootloader_clear_magic();" in take

    jump = c_function_body(
        mechanism,
        "void BSP_RomBootloader_Jump(uint32_t vector_address,")
    assert "rom_bootloader_quiesce();" in jump
    assert "SCB->VTOR = vector_address;" in jump
    assert "rom_bootloader_branch(initial_msp, reset_handler);" in jump
    assert "SCB->VTOR" not in source, "向量表切换不该再出现在 App 层"

    branch = c_function_body(
        mechanism,
        "static void __attribute__((naked, noreturn)) rom_bootloader_branch(",
    )
    for instruction in ("msr msp, r0", "msr control, r2", "cpsie i", "bx r1"):
        assert instruction in branch
    assert "naked, noreturn" in mechanism


def test_rom_entry_clears_interrupt_cache_and_mpu_state() -> None:
    source = read(SOURCE)
    mechanism = read(MECHANISM)
    # 两个静默动作 2026-09-11 合成一个 BSP 内部函数：它们从来只被一起调用，
    # 分开只是让调用点多写一行、多一次漏调的机会。
    quiesce = c_function_body(mechanism, "static void rom_bootloader_quiesce(void)")

    for statement in (
        "SysTick->CTRL = 0U",
        "SysTick->LOAD = 0U",
        "SysTick->VAL = 0U",
        "NVIC->ICER[index] = 0xFFFFFFFFUL",
        "NVIC->ICPR[index] = 0xFFFFFFFFUL",
        "SCB_CleanInvalidateDCache()",
        "SCB_DisableDCache()",
        "SCB_InvalidateICache()",
        "SCB_DisableICache()",
        "HAL_MPU_Disable()",
    ):
        assert statement in quiesce
    for leaked in ("SysTick->", "NVIC->", "SCB_DisableDCache", "HAL_MPU_Disable"):
        assert leaked not in source, f"{leaked} 不该再出现在 App 层"

    assert "APP_BOOT_ROM_DFU_VECTOR_ADDRESS 0x1FF09800UL" in read(HEADER)
    assert "APP_PROTO_REQ_BOOT           0x101FU" in read(PROTO)
    assert "APP_PROTO_MSG_BOOT_STATUS       0x2220U" in read(PROTO)
    cmake = read(CMAKE)
    assert "App/Src/app_boot.c" in cmake
    assert "BSP/Src/bsp_rom_bootloader.c" in cmake


PURE_RUNTIME_HARNESS = r"""
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

int main(void)
{
    CHECK(APP_Boot_EvaluateSafety(0U, 0U, 0U, 0U) ==
          APP_BOOT_SAFETY_NO_VALID_SNAPSHOT, 1);
    CHECK(APP_Boot_EvaluateSafety(1U, 1U, 1000U, 1000U) ==
          APP_BOOT_SAFETY_ARMED, 2);
    CHECK(APP_Boot_EvaluateSafety(1U, 0U, 1100U, 1100U) ==
          APP_BOOT_SAFETY_OK, 3);
    CHECK(APP_Boot_EvaluateSafety(1U, 0U, 1101U, 1000U) ==
          APP_BOOT_SAFETY_ESC_HIGH, 4);
    CHECK(APP_Boot_EvaluateSafety(1U, 0U, 1000U, 1101U) ==
          APP_BOOT_SAFETY_ESC_HIGH, 5);

    CHECK(APP_Boot_IsVectorReasonable(0x20020000U, 0x1FF09801U) == 1U, 6);
    CHECK(APP_Boot_IsVectorReasonable(0x24080000U, 0x1FF1FFFDU) == 1U, 7);
    CHECK(APP_Boot_IsVectorReasonable(0U, 0x1FF09801U) == 0U, 8);
    CHECK(APP_Boot_IsVectorReasonable(0xFFFFFFFFU, 0x1FF09801U) == 0U, 9);
    CHECK(APP_Boot_IsVectorReasonable(0x20020000U, 0U) == 0U, 10);
    CHECK(APP_Boot_IsVectorReasonable(0x20020000U, 0xFFFFFFFFU) == 0U, 11);
    CHECK(APP_Boot_IsVectorReasonable(0x2001FFFDU, 0x1FF09801U) == 0U, 12);
    CHECK(APP_Boot_IsVectorReasonable(0x20020000U, 0x1FF09800U) == 0U, 13);
    CHECK(APP_Boot_IsVectorReasonable(0x20020000U, 0x08000001U) == 0U, 14);
    CHECK(APP_Boot_IsSnapshotFresh(200000U, 100000U) == 1U, 15);
    CHECK(APP_Boot_IsSnapshotFresh(200001U, 100000U) == 0U, 16);
    CHECK(APP_Boot_IsSnapshotFresh(99999U, 100000U) == 0U, 17);
    CHECK(APP_Boot_HasSequenceAdvanced(101U, 100U) == 1U, 18);
    CHECK(APP_Boot_HasSequenceAdvanced(100U, 100U) == 0U, 19);
    CHECK(APP_Boot_HasSequenceAdvanced(0U, 0xFFFFFFFFU) == 1U, 20);

    puts("ok");
    return 0;
}
"""


def test_safety_and_vector_predicates_compile_and_run_on_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("ROM DFU contract requires host gcc or clang")

    source = read(SOURCE)
    start = source.index("static uint8_t app_boot_msp_in_sram")
    end = source.index("static void app_boot_read_vector", start)
    pure_functions = source[start:end]
    definitions = r"""
#include <stdint.h>
#define APP_BOOT_ESC_SAFE_MAX_US 1100U
#define APP_BOOT_SNAPSHOT_MAX_AGE_US 100000ULL
#define APP_BOOT_ROM_ADDRESS_MIN 0x1FF00000UL
#define APP_BOOT_ROM_ADDRESS_MAX 0x1FF20000UL
typedef enum {
    APP_BOOT_SAFETY_OK = 0U,
    APP_BOOT_SAFETY_NO_VALID_SNAPSHOT = 1U,
    APP_BOOT_SAFETY_ARMED = 2U,
    APP_BOOT_SAFETY_ESC_HIGH = 3U,
    APP_BOOT_SAFETY_SNAPSHOT_STALE = 4U,
} APP_BootSafety;
"""

    isolated = tmp_path / "boot_predicates.c"
    harness = tmp_path / "boot_predicates_harness.c"
    executable = tmp_path / "boot_predicates.exe"
    isolated.write_text(definitions + pure_functions, encoding="ascii")
    harness.write_text(
        definitions
        + "APP_BootSafety APP_Boot_EvaluateSafety(uint8_t, uint8_t, uint16_t, uint16_t);\n"
        + "uint8_t APP_Boot_IsVectorReasonable(uint32_t, uint32_t);\n"
        + "uint8_t APP_Boot_IsSnapshotFresh(uint64_t, uint64_t);\n"
        + "uint8_t APP_Boot_HasSequenceAdvanced(uint32_t, uint32_t);\n"
        + PURE_RUNTIME_HARNESS,
        encoding="ascii",
    )
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(isolated),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    assert result.stdout.strip() == "ok"

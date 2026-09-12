"""换板子时应该只改板级绑定那几处，不该逐个文件重写。

把固件从自制 H743 板移到 MicoAir743V2，最费时间的改动几乎都不是算法，而是
**外设实例名被写进了上层逻辑**：PWM 从 TIM5/TIM2 搬到 TIM1/TIM4、GPS 从 USART2
分到 USART3、IMU 从 SPI1 换到 SPI2/SPI3、DRDY 从 PC0/EXTI0 散到三个引脚……
每一处写死的名字都要人去找出来改，而漏掉一处的表现通常是"整机不动"，
不是编译错误。

所以做了一次全层解耦，本文件是它的回归闸门。判据不靠感觉，靠 `tools/decoupling_survey.py`
去注释、去字符串之后扫三样东西：**外设实例名**、**HAL / CubeMX 头**、
**`HAL_*` 调用与裸 CMSIS 内建**。

注释里写"本板在 UART8"是对的——那是在解释绑定在哪；字符串里写 `"UART1 rx=%lu"`
也是对的——那是线上的标签。只有**代码**里出现才算耦合。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.decoupling_survey import scan, strip_comments  # noqa: E402


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ------------------------------------------------------------------ 全层闸门

# 唯一允许直接碰硬件的 App/Services 文件，连同理由。
#
# 加东西进这张表前先问一句：这个文件是不是**只**做"把某个硬件能力翻译成上层
# 语言"这一件事？不是的话，该做的是把硬件那半边挪进 BSP，而不是在这里开口子。
ALLOWED = {
    "Services/Src/svc_timestamp.c":
        "时基服务本身。它的职责就是把某个硬件计数器翻译成单调的 µs/ms，"
        "硬件那一面躲不掉，也不该躲——它就是全仓库唯一一处摸时钟的地方。",
}


def test_app_and_services_do_not_touch_hardware() -> None:
    offenders = {}
    for row in scan():
        if row["path"] in ALLOWED:
            continue
        offenders[row["path"]] = {
            "instances": row["instances"],
            "hal_include": row["include"],
            "hal_calls": row["calls"],
            "cmsis": row["cmsis"],
        }

    assert not offenders, (
        "这些 App/Services 文件又直接碰硬件了。把硬件那一半挪进 BSP/，"
        "上层按角色调用；确实躲不掉的，连同理由加进 ALLOWED：\n"
        + "\n".join(f"  {path}: {detail}" for path, detail in offenders.items())
    )


def test_every_allowed_exception_still_exists_and_still_needs_it() -> None:
    """豁免表不能变成坟场。

    文件被删了、或者它其实已经不碰硬件了，豁免就该跟着去掉——留着的话，
    下一个人会以为这里本来就允许碰硬件。
    """
    scanned = {row["path"] for row in scan()}
    for path, reason in ALLOWED.items():
        assert (ROOT / path).is_file(), f"豁免表里的 {path} 已经不存在了"
        assert path in scanned, (
            f"{path} 已经不碰硬件了，请把它从 ALLOWED 里删掉")
        assert len(reason) > 20, f"{path} 的豁免理由太短，说不清为什么躲不掉"


# ------------------------------------------------------------ 绑定的唯一事实源


def test_the_maintenance_link_binding_lives_in_exactly_one_place() -> None:
    """"哪个 UART 是维护口"必须只有一个事实源。

    分散成两处的典型后果：改了收、忘了发，于是命令进得来、回包出不去，
    而两边都"看起来改过了"。
    """
    bsp = read("BSP/Src/bsp_uart.c")

    assert "#define BSP_UART_MAINT_HANDLE     (&huart8)" in bsp
    assert '#define BSP_UART_MAINT_NAME       "uart8"' in bsp

    # 维护口那段代码只经宏引用句柄。只查这一段：同文件里的
    # BSP_UART_GetHandle(bus_index) 是诊断用的"总线号 → 句柄"表，
    # 那里出现 huart8 是对的——它的入参就是板上丝印编号。
    maint = strip_comments(bsp).split("void BSP_UART_MaintInit", 1)[1]
    maint = maint.split("UART_HandleTypeDef *BSP_UART_GetHandle", 1)[0]
    assert "huart8" not in maint
    assert maint.count("BSP_UART_MAINT_HANDLE") >= 4


def test_the_uart_interrupt_routing_table_lives_in_bsp() -> None:
    """HAL 的四个弱回调必须由 BSP 实现，App 里一个都不许有。

    它们的参数只有一个 `UART_HandleTypeDef *`，所以谁实现它们，谁就得知道
    这块板上每个实例归谁——而这次移植里那张表整个变了。
    """
    events = read("BSP/Src/bsp_uart_events.c")
    for callback in ("HAL_UARTEx_RxEventCallback", "HAL_UART_RxCpltCallback",
                     "HAL_UART_TxCpltCallback", "HAL_UART_ErrorCallback"):
        assert f"void {callback}(" in events, f"{callback} 不在 BSP 里"

    for rel in sorted((ROOT / "App" / "Src").glob("*.c")):
        code = strip_comments(rel.read_text(encoding="utf-8"))
        for callback in ("HAL_UARTEx_RxEventCallback", "HAL_UART_RxCpltCallback",
                         "HAL_UART_TxCpltCallback", "HAL_UART_ErrorCallback",
                         "HAL_GPIO_EXTI_Callback"):
            assert f"void {callback}(" not in code, (
                f"{rel.name} 又实现了 HAL 的弱回调 {callback}")


def test_the_imu_drdy_interrupt_is_registered_not_implemented() -> None:
    """DRDY 是控制环的节拍源，引脚归属必须留在 BSP。

    本次移植里它就从 PC0/EXTI0 散成了 PC15(BMI088) 与 PB7(BMI270)。
    """
    assert "void BSP_IMU_SetDrdyHandler(" in read("BSP/Inc/bsp_imu.h")
    assert "void HAL_GPIO_EXTI_Callback(" in read("BSP/Src/bsp_imu.c")
    assert "BSP_IMU_SetDrdyHandler(APP_IMU_OnDataReady)" in read("App/Src/app.c")
    assert "void APP_IMU_OnDataReady(void)" in read("App/Src/app_sensor.c")


def test_the_uart_link_mechanism_is_role_based_not_instance_based() -> None:
    """收发机制按角色取用，实例表只在 BSP 一处。"""
    header = read("BSP/Inc/bsp_uart_link.h")
    source = read("BSP/Src/bsp_uart_link.c")

    for api in ("BSP_UartLink_StartRxToIdle", "BSP_UartLink_RxFilled",
                "BSP_UartLink_TakeErrors", "BSP_UartLink_TransmitDma"):
        assert f"{api}(BSP_UartRole role" in header, f"{api} 不是按角色取的"

    # 错误分类必须是可移植的位，不是透传 HAL 的错误码。
    for bit in ("BSP_UART_LINK_ERR_OVERRUN", "BSP_UART_LINK_ERR_FRAMING",
                "BSP_UART_LINK_ERR_NOISE", "BSP_UART_LINK_ERR_PARITY"):
        assert bit in header
    assert "HAL_UART_ERROR_ORE" not in strip_comments(header)
    assert "huart1" in source and "huart6" in source   # 表就该在这儿


def test_the_dma_tx_engine_is_not_written_for_one_uart() -> None:
    """发送引擎按参数绑定 UART，不按板子写死。

    写死的话，下一条口要提速就得再抄一遍，两份实现随后各自长歪。
    """
    for path in ("BSP/Src/bsp_uart_tx.c", "BSP/Inc/bsp_uart_tx.h"):
        code = strip_comments(read(path))
        found = re.findall(r"\b(?:h?(?:uart|usart)\d|UART\d|USART\d)\b", code)
        assert not found, f"{path} 把实例名写死了: {sorted(set(found))}"

    header = read("BSP/Inc/bsp_uart_tx.h")
    assert "void BSP_UartTx_Attach(BSP_UartTx *tx, UART_HandleTypeDef *huart," in header


def test_the_pwm_diagnostic_asks_bsp_which_timer_it_is() -> None:
    """`PWM?` 曾经写死 `TIM2->CR1`。

    搬到 TIM1/TIM4 之后那段代码没人改，于是它一直在报一颗**本板没初始化**的
    定时器，实测 `cr1=0x00000000 psc=0 arr=0`——查"PWM 没输出"的人看到这行会
    认定定时器没配好，而真正的 TIM1/TIM4 好好的。诊断说谎比诊断缺失更糟。
    """
    assert "void BSP_PWM_GetEscTimerDebug(" in read("BSP/Inc/bsp_pwm.h")
    report = read("App/Src/app_cmd_system.c")
    assert "BSP_PWM_GetEscTimerDebug(&timer)" in report
    assert "BSP_PWM_GetServoTimerDebug(&timer)" in report
    # 定时器名字从快照里来，不是格式串里的常量。
    assert '"tim1"' not in report and '"tim2"' not in report


def test_the_rom_bootloader_mechanism_is_in_bsp_but_the_policy_is_not() -> None:
    """跳 DFU：怎么跳归 BSP，什么时候准跳归 App（后者有宿主单测）。"""
    bsp = read("BSP/Src/bsp_rom_bootloader.c")
    app = read("App/Src/app_boot.c")

    for mechanism in ("SCB->VTOR", "NVIC->ICER", "RTC->BKP0R", "msr msp, r0"):
        assert mechanism in bsp, f"{mechanism} 应该在 BSP 里"
        assert mechanism not in app, f"{mechanism} 不该留在 App 里"

    # 判据留在 App：它们是纯函数，宿主侧能测。
    for policy in ("APP_Boot_EvaluateSafety", "APP_Boot_IsVectorReasonable",
                   "APP_Boot_IsSnapshotFresh"):
        assert policy in app


# ------------------------------------------------------------ App 层的替代设施


def test_the_app_layer_has_a_hal_free_millisecond_clock() -> None:
    """超时判断不该是"为了拿个时间"就把 HAL 拖进 App 层的理由。

    `HAL_GetTick()` 到处散落曾是本仓库里 App→HAL 依赖最常见的来源（67 处）。
    """
    header = read("Services/Inc/svc_timestamp.h")

    assert "uint32_t SVC_Timestamp_Ms(void);" in header
    # 回绕语义必须写在契约里，否则调用方迟早写成 `now > deadline`。
    assert "49.7" in header
    # 必须与 Driver 层打时间戳用的是同一个计数器，否则上下两半在比两个钟。
    assert "HAL 时基那个计数器" in header

    for rel in sorted((ROOT / "App").rglob("*.[ch]")):
        code = strip_comments(rel.read_text(encoding="utf-8"))
        assert "HAL_GetTick" not in code, f"{rel.name} 又直接调 HAL_GetTick 了"


def test_the_app_layer_has_a_hal_free_critical_section() -> None:
    """关中断和内存屏障也要经 BSP，否则一行 `__disable_irq()` 就把 CMSIS 拖进来。"""
    header = read("BSP/Inc/bsp_critical.h")

    assert "uint32_t BSP_Critical_Enter(void);" in header
    assert "void BSP_Critical_MemoryBarrier(void);" in header
    # 必须成对、必须还原原值——无条件开中断会拆掉外层临界区。
    assert "__set_PRIMASK(state)" in read("BSP/Src/bsp_critical.c")
    assert "无条件" in header


def test_the_usb_device_stack_is_behind_a_bsp_shim() -> None:
    """`CDC_Transmit_FS` / `hUsbDeviceFS` 来自 CubeMX 生成的 USB_DEVICE/。"""
    assert "uint8_t BSP_UsbCdc_Transmit(" in read("BSP/Inc/bsp_usb_cdc.h")
    assert "void BSP_UsbCdc_Teardown(void);" in read("BSP/Inc/bsp_usb_cdc.h")
    for rel in sorted((ROOT / "App" / "Src").glob("*.c")):
        code = strip_comments(rel.read_text(encoding="utf-8"))
        assert "hUsbDeviceFS" not in code, f"{rel.name} 直接抓了 CubeMX 的 USB 句柄"
        assert "CDC_Transmit_FS" not in code, f"{rel.name} 直接调了 USB 栈"


def test_the_dma_buffer_lives_where_dma_can_reach_it() -> None:
    """发送缓冲必须在 `.dma_buffer` 段。

    默认的 .bss 在 DTCM（0x20000000），DMA1/DMA2 根本够不到，症状是
    "HAL 返回 OK 但一个字节都没出去"——最像"蓝牙模块坏了"的一种固件缺陷。
    """
    bsp = read("BSP/Src/bsp_uart.c")
    linker = read("STM32H743XX_FLASH.ld")

    assert '__attribute__((section(".dma_buffer"), aligned(32)))' in bsp
    assert "bsp_uart_maint_tx_buffer" in bsp
    dma_section = linker.split(".dma_buffer", 1)[1].split("}", 1)[0]
    assert "ALIGN(32)" in dma_section
    assert ">RAM_D2" in linker.split(".dma_buffer", 1)[1][:400]


def test_the_tx_path_cleans_the_cache_before_handing_the_buffer_to_dma() -> None:
    """RAM_D2 是 cacheable 的，发前不 clean 就会偶尔发出旧内容。

    "偶尔"取决于 cache 行何时被换出，所以它不会稳定复现，只会在某次长时间
    运行里冒出一帧坏数据——不写这条断言，未来某次重构顺手删掉它都不会有人发现。
    """
    source = strip_comments(read("BSP/Src/bsp_uart_tx.c"))
    kick = source.split("static void uart_tx_kick", 1)[1].split("\nvoid ", 1)[0]

    assert "BSP_Cache_CleanDCache(chunk, length)" in kick
    assert kick.index("BSP_Cache_CleanDCache") < kick.index("HAL_UART_Transmit_DMA"), (
        "clean 必须发生在启动 DMA 之前")

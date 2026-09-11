"""换板子时应该只改板级绑定那一处，不该逐个文件重写。

这次把固件从自制 H743 板移到 MicoAir743V2，最费时间的改动几乎都不是算法，
而是"外设实例名被写进了上层逻辑"：PWM 从 TIM5/TIM2 搬到 TIM1/TIM4、GPS 从
USART2 分到 USART3、蓝牙落在 UART8……每一处写死的实例名都要人去逐个找出来改。

所以维护口（本板 = 板载蓝牙）这条链路改造时一并做了解耦，本文件把它钉住：

  * **谁是维护口**只在 `BSP/Src/bsp_uart.c` 写一次；
  * App 层拿不到、也不需要 HAL 句柄；
  * DMA 发送引擎对"哪个 UART"无知，靠参数绑定；
  * 索引算法连 HAL 和 RTOS 都不认识，因此能在 PC 上单测。

这些断言都扫**去掉注释后**的代码：注释里写"本板在 UART8"是应该的，
那是在解释绑定在哪；代码里出现 UART8 才是耦合。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

# 本仓库用到的外设实例名。新增外设时补进来。
INSTANCE_NAMES = re.compile(
    r"\b(?:h?(?:uart|usart)\d|UART\d|USART\d|SPI\d|hspi\d|I2C\d|hi2c\d|"
    r"TIM\d+|htim\d+|SDMMC\d|GPIO[A-K])\b"
)

HAL_INCLUDES = re.compile(
    r'#\s*include\s+[<"](?:main\.h|usart\.h|spi\.h|i2c\.h|tim\.h|dma\.h|gpio\.h|'
    r'stm32h7xx[^">]*)[">]'
)


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_comments(source: str) -> str:
    """去掉块注释与行注释。注释里提硬件是解释，不是耦合。"""
    source = re.sub(r"/\*.*?\*/", " ", source, flags=re.S)
    source = re.sub(r"//[^\n]*", " ", source)
    return source


def test_the_maintenance_link_app_module_names_no_peripheral() -> None:
    """app_maint_uart 只认"维护口"这个角色，不认 UART8。

    这是这次解耦的主目标：换一块把蓝牙接在 USART2 的板子，这个文件一行都不用动。
    """
    for path in ("App/Src/app_maint_uart.c", "App/Inc/app_maint_uart.h"):
        code = strip_comments(read(path))
        found = INSTANCE_NAMES.findall(code)
        assert not found, f"{path} 里出现了外设实例名: {sorted(set(found))}"


def test_the_maintenance_link_app_module_does_not_reach_for_hal() -> None:
    """D1-2：App 层不直接依赖 HAL。

    这个模块以前 `#include "usart.h"` 并直接调 `HAL_UART_Receive_IT(&huart8, ...)`，
    换板时它就是必改项之一。现在收发、恢复、时间都经 BSP / Services。
    """
    for path in ("App/Src/app_maint_uart.c", "App/Inc/app_maint_uart.h"):
        code = strip_comments(read(path))
        assert not HAL_INCLUDES.search(code), f"{path} 仍然包含 HAL 头"
        assert "HAL_" not in code, f"{path} 仍然直接调用 HAL"


def test_the_binding_lives_in_exactly_one_place() -> None:
    """"哪个 UART 是维护口"必须只有一个事实源。

    分散成两处的典型后果：改了收、忘了发，于是命令进得来、回包出不去，
    而两边都"看起来改过了"。
    """
    bsp = read("BSP/Src/bsp_uart.c")

    assert "#define BSP_UART_MAINT_HANDLE     (&huart8)" in bsp
    assert '#define BSP_UART_MAINT_NAME       "uart8"' in bsp

    # 维护口那一段代码只经宏引用句柄，不再直接写 huart8。
    #
    # 只查这一段：同文件里的 BSP_UART_GetHandle(bus_index) 是**诊断**用的
    # "总线号 → 句柄"表，那里出现 huart8 是对的——它的入参就是板上丝印编号。
    maint = strip_comments(bsp).split("void BSP_UART_MaintInit", 1)[1]
    maint = maint.split("UART_HandleTypeDef *BSP_UART_GetHandle", 1)[0]
    assert "huart8" not in maint
    assert maint.count("BSP_UART_MAINT_HANDLE") >= 4


def test_the_dma_tx_engine_is_not_written_for_one_uart() -> None:
    """发送引擎按参数绑定 UART，不按板子写死。

    这样同一份引擎既能给蓝牙用，也能给数传或任何别的口用；写死的话，
    下一条口要提速就得再抄一遍，两份实现随后各自长歪。
    """
    for path in ("BSP/Src/bsp_uart_tx.c", "BSP/Inc/bsp_uart_tx.h"):
        code = strip_comments(read(path))
        found = INSTANCE_NAMES.findall(code)
        assert not found, f"{path} 把实例名写死了: {sorted(set(found))}"

    header = read("BSP/Inc/bsp_uart_tx.h")
    # 绑定靠参数传入，而且上下文是调用方持有的，可以有多个实例。
    assert "void BSP_UartTx_Attach(BSP_UartTx *tx, UART_HandleTypeDef *huart," in header


def test_the_app_layer_has_a_hal_free_millisecond_clock() -> None:
    """超时判断不该是"为了拿个时间"就把 HAL 拖进 App 层的理由。

    `HAL_GetTick()` 到处散落是本仓库里 App→HAL 依赖最常见的来源。
    换时基（换芯片、或改用 RTOS tick）时，散落的每一处都要改。
    """
    header = read("Services/Inc/svc_timestamp.h")
    source = read("Services/Src/svc_timestamp.c")

    assert "uint32_t SVC_Timestamp_Ms(void);" in header
    assert "SVC_Timestamp_Ms(void)" in source
    # 回绕语义必须写在契约里，否则调用方迟早写成 `now > deadline`。
    assert "49.7" in header


def test_the_dma_buffer_lives_where_dma_can_reach_it() -> None:
    """发送缓冲必须在 `.dma_buffer` 段。

    默认的 .bss 在 DTCM（0x20000000），DMA1/DMA2 根本够不到，症状是
    "HAL 返回 OK 但一个字节都没出去"——最像"蓝牙模块坏了"的一种固件缺陷。
    """
    bsp = read("BSP/Src/bsp_uart.c")
    linker = read("STM32H743XX_FLASH.ld")

    assert '__attribute__((section(".dma_buffer"), aligned(32)))' in bsp
    assert "bsp_uart_maint_tx_buffer" in bsp
    # 段本身要落在 D2 域，并且按 cache 行对齐。
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
    clean_at = kick.index("BSP_Cache_CleanDCache")
    start_at = kick.index("HAL_UART_Transmit_DMA")
    assert clean_at < start_at, "clean 必须发生在启动 DMA 之前"

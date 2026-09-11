"""CubeMX 生成代码必须与 drone-H743.ioc 同步。

**本文件全红 = 还没跑 Generate Code，不是代码写错了。**

移植到 MicoAir743v2 改的是 .ioc（引脚、时钟、外设），而 Core/ 与 USB_DEVICE/ 下的
文件是 CubeMX 从 .ioc 生成的、禁止手改。两者之间存在一个必然的时间差：
.ioc 已经改完，生成代码要等有人在 STM32CubeIDE 里点一次 Generate Code 才会跟上。

这个时间差很危险，因为它**不会导致编译失败**，只会让固件按旧配置跑：
  - PLL3 还按 12 MHz 晶振的倍频配 → USB 拿不到 48 MHz → 枚举不上，
    而"USB 认不到"最容易被误判成线材或驱动问题；
  - 定时器还在旧的脚上 → 电机与舵机毫无反应，或者更糟，输出到别的功能脚上。

所以把这个状态显式变成一条红测试，而不是让它藏在"下次谁想起来再说"里。

修复办法只有一个：
    1. 用 STM32CubeIDE 打开 drone-H743.ioc
    2. 核对引脚视图与时钟树（时钟树不能有红色）
    3. Generate Code
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

REGENERATE = (
    "请先在 STM32CubeIDE 里打开 drone-H743.ioc 并 Generate Code"
    "（见 tools/micoair743v2_ioc_migrate.py 的说明）"
)


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


def ioc_value(key: str) -> str:
    ioc = read("drone-H743.ioc")
    match = re.search(rf"^{re.escape(key)}=(.*)$", ioc, re.MULTILINE)
    assert match is not None, f"missing {key} in drone-H743.ioc"
    return match.group(1).strip()


def test_usb_pll3_matches_ioc() -> None:
    """USB 全速要求 48 MHz，容限很窄；配错的表现是设备枚举不上。"""
    usbd_conf = read("USB_DEVICE/Target/usbd_conf.c")
    divn3 = ioc_value("RCC.DIVN3")
    divq3 = ioc_value("RCC.DIVQ3")

    assert f"PeriphClkInitStruct.PLL3.PLL3N = {divn3};" in usbd_conf, (
        f".ioc 是 PLL3N={divn3}，生成代码还停在旧值。{REGENERATE}"
    )
    assert f"PeriphClkInitStruct.PLL3.PLL3Q = {divq3};" in usbd_conf, (
        f".ioc 是 PLL3Q={divq3}，生成代码还停在旧值。{REGENERATE}"
    )


def test_actuator_timers_are_generated() -> None:
    """ESC/舵机换了定时器，生成代码里必须有对应的 MX_TIMx_Init 与句柄。"""
    tim_c = read("Core/Src/tim.c")
    tim_h = read("Core/Inc/tim.h")

    for timer in ("TIM1", "TIM4"):
        assert f"void MX_{timer}_Init(void)" in tim_c, f"{timer} 未生成。{REGENERATE}"
        assert f"void MX_{timer}_Init(void);" in tim_h
        assert f"TIM_HandleTypeDef h{timer.lower()};" in tim_c

    # 旧板子的定时器必须消失，否则它们会把 PWM 脚抢回去。
    for timer in ("TIM2", "TIM5", "TIM8"):
        assert f"void MX_{timer}_Init(void)" not in tim_c, (
            f"{timer} 仍在生成代码里。{REGENERATE}"
        )

    assert "PE9     ------> TIM1_CH1" in tim_c, REGENERATE
    assert "PD12     ------> TIM4_CH1" in tim_c, REGENERATE


def test_new_peripherals_are_generated() -> None:
    """SDMMC1（飞行日志）、USART3（GPS）、SPI3（BMI270）都是本次新增的外设。"""
    assert (ROOT / "Core" / "Src" / "sdmmc.c").exists(), (
        f"SDMMC1 尚未生成，飞行日志无处落盘。{REGENERATE}"
    )
    assert "SD_HandleTypeDef hsd1;" in read("Core/Src/sdmmc.c")

    # CubeMX 按"只复制用得到的库文件"工作（ProjectManager.LibraryCopy=1），
    # 所以 SD 的 HAL 模块是随 SDMMC1 一起被拷进来的。少了它，
    # drv_sdblock.c 里的 SD_HandleTypeDef 根本不存在，整个工程编不过。
    assert (ROOT / "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_sd.h").exists(), (
        f"SD 的 HAL 模块尚未随 SDMMC1 一起生成。{REGENERATE}"
    )
    assert "#define HAL_SD_MODULE_ENABLED" in read("Core/Inc/stm32h7xx_hal_conf.h"), (
        f"hal_conf.h 里还没启用 SD 模块。{REGENERATE}"
    )

    usart = read("Core/Src/usart.c")
    assert "void MX_USART3_UART_Init(void)" in usart, f"GPS 串口未生成。{REGENERATE}"
    # 旧板子的 UART5 引脚在本板上是 SDMMC1_CMD，必须让出来。
    assert "void MX_UART5_Init(void)" not in usart, REGENERATE

    # ELRS 搬到板载 RC 口（USART6/PC6/PC7），UART4 随之退场。
    # 少了 huart6，app_elrs.c 连编都编不过。
    assert "void MX_USART6_UART_Init(void)" in usart, (
        f"ELRS 的 USART6 未生成，app_elrs.c 会因 huart6 未声明编译失败。{REGENERATE}"
    )
    assert "UART_HandleTypeDef huart6;" in usart, REGENERATE
    assert "void MX_UART4_Init(void)" not in usart, (
        f"UART4 已不再使用，仍在生成代码里说明还没重新生成。{REGENERATE}"
    )

    spi = read("Core/Src/spi.c")
    assert "void MX_SPI3_Init(void)" in spi, f"BMI270 的 SPI3 未生成。{REGENERATE}"
    assert "void MX_SPI4_Init(void)" not in spi, REGENERATE


def test_imu_drdy_external_interrupts_are_generated() -> None:
    """DRDY 从 PC0/EXTI0 换到 PC15/PC14（BMI088）与 PB7（BMI270）。

    漏了这一步，控制环会一直退到 20 ms 轮询兜底——飞是能飞，但内环节拍全乱。
    """
    gpio = read("Core/Src/gpio.c")

    assert "EXTI15_10_IRQn" in gpio, f"BMI088 的 DRDY 中断未生成。{REGENERATE}"
    assert "EXTI9_5_IRQn" in gpio, f"BMI270 的 DRDY 中断未生成。{REGENERATE}"
    assert "EXTI0_IRQn" not in gpio, (
        f"PC0 在本板上是电池电压采样，不该再有 EXTI0。{REGENERATE}"
    )

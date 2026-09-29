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

    # ELRS 搬到板载 RC 口（USART6/PC6/PC7）。少了 huart6，app_elrs.c 连编都编不过。
    assert "void MX_USART6_UART_Init(void)" in usart, (
        f"ELRS 的 USART6 未生成，app_elrs.c 会因 huart6 未声明编译失败。{REGENERATE}"
    )
    assert "UART_HandleTypeDef huart6;" in usart, REGENERATE
    # UART4 2026-09-10 随 ELRS 退场，2026-09-29 又作为光流口回来（见下一条测试）。
    assert "void MX_UART4_Init(void)" in usart, (
        f"光流的 UART4 未生成，bsp_board.c 会因 huart4 未声明编译失败。{REGENERATE}"
    )

    spi = read("Core/Src/spi.c")
    assert "void MX_SPI3_Init(void)" in spi, f"BMI270 的 SPI3 未生成。{REGENERATE}"
    assert "void MX_SPI4_Init(void)" not in spi, REGENERATE


def test_optical_flow_uart4_generated_code_matches_ioc() -> None:
    """光流 2026-09-29 从 USART2 迁到 UART4，.ioc 与生成代码必须逐项一致。

    USART2 是 DJI 图传 6 针口，该版板子第 1 脚是 12 V；UART4 是 4 针 5 V 口
    （GND / 5V / TX4 / RX4，PA0/PA1）。USART2 原有的两路 DMA stream 原样转给 UART4
    （H7 有 DMAMUX，只换请求名），不新占 stream；USART2 保留为不带 DMA 的普通串口。
    """
    ioc = read("drone-H743.ioc")
    usart = read("Core/Src/usart.c")
    irq = read("Core/Src/stm32h7xx_it.c")
    main = read("Core/Src/main.c")

    ips = re.findall(r"^Mcu\.IP\d+=(.+)$", ioc, re.MULTILINE)
    pins = re.findall(r"^Mcu\.Pin\d+=(.+)$", ioc, re.MULTILINE)
    assert len(ips) == int(ioc_value("Mcu.IPNb")) and "UART4" in ips
    assert len(pins) == int(ioc_value("Mcu.PinsNb"))
    assert "PA0" in pins and "PA1" in pins

    # 引脚与复用：PA0=TX、PA1=RX，均为 AF8。
    assert ioc_value("PA0.Signal") == "UART4_TX"
    assert ioc_value("PA1.Signal") == "UART4_RX"
    msp = usart.split("if(uartHandle->Instance==UART4)", 1)[1].split("else if", 1)[0]
    assert "PA0     ------> UART4_TX\n    PA1     ------> UART4_RX" in msp, REGENERATE
    assert "GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;" in msp
    assert "GPIO_InitStruct.Alternate = GPIO_AF8_UART4;" in msp
    assert "HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);" in msp
    assert f"huart4.Init.BaudRate = {ioc_value('UART4.BaudRate')};" in usart
    assert "huart4.Init.WordLength = UART_WORDLENGTH_8B;" in usart
    assert "huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;" in usart
    assert "huart4.Init.OverSampling = UART_OVERSAMPLING_16;" in usart

    # DMA：请求、stream、方向、模式、优先级与 .ioc 一致，DMA 中断服务的也是它。
    for direction, link in (("RX", "hdmarx"), ("TX", "hdmatx")):
        request = re.search(rf"^Dma\.Request(\d+)=UART4_{direction}$", ioc, re.MULTILINE)
        assert request is not None, f".ioc 里没有 UART4_{direction} 的 DMA 请求"
        prefix = f"Dma.UART4_{direction}.{request.group(1)}."
        stream = ioc_value(prefix + "Instance")
        handle = f"hdma_uart4_{direction.lower()}"
        assert f"{handle}.Instance = {stream};" in msp, REGENERATE
        assert f"{handle}.Init.Request = DMA_REQUEST_UART4_{direction};" in msp
        assert f"{handle}.Init.Direction = {ioc_value(prefix + 'Direction')};" in msp
        assert f"{handle}.Init.Mode = {ioc_value(prefix + 'Mode')};" in msp
        assert f"{handle}.Init.Priority = {ioc_value(prefix + 'Priority')};" in msp
        assert f"__HAL_LINKDMA(uartHandle,{link},{handle});" in msp
        body = irq.split(f"void {stream}_IRQHandler(void)", 1)[1].split("\n}", 1)[0]
        assert f"HAL_DMA_IRQHandler(&{handle});" in body, f"{stream} 的中断没交给 {handle}"
    # 接的正是 USART2 原来那两路；光流靠循环 DMA + 空闲事件收帧。
    assert ioc_value("Dma.UART4_RX.4.Instance") == "DMA1_Stream4"
    assert ioc_value("Dma.UART4_TX.5.Instance") == "DMA1_Stream5"
    assert ioc_value("Dma.UART4_RX.4.Mode") == "DMA_CIRCULAR"

    # 全局中断：优先级照 USART2。
    nvic = ioc_value("NVIC.UART4_IRQn")
    assert nvic == ioc_value("NVIC.USART2_IRQn")
    priority = nvic.split("\\:")[1]
    assert f"HAL_NVIC_SetPriority(UART4_IRQn, {priority}, 0);" in msp
    assert "HAL_NVIC_EnableIRQ(UART4_IRQn);" in msp
    handler = irq.split("void UART4_IRQHandler(void)", 1)[1].split("\n}", 1)[0]
    assert "HAL_UART_IRQHandler(&huart4);" in handler
    assert "void UART4_IRQHandler(void);" in read("Core/Inc/stm32h7xx_it.h")
    usart_h = read("Core/Inc/usart.h")
    assert "extern UART_HandleTypeDef huart4;" in usart_h
    assert "void MX_UART4_Init(void);" in usart_h

    # 初始化在 DMA 之后：DMA 时钟先开，MspInit 里才初始化得了 stream。
    assert "-MX_UART4_Init-UART4-" in ioc_value("ProjectManager.functionlistsort")
    assert main.index("MX_DMA_Init();") < main.index("MX_UART4_Init();")

    # USART2：初始化与中断保留，DMA 全部移走。
    assert "void MX_USART2_UART_Init(void)" in usart and "MX_USART2_UART_Init();" in main
    assert "HAL_NVIC_EnableIRQ(USART2_IRQn);" in usart
    assert re.search(r"^Dma\.Request\d+=USART2_", ioc, re.MULTILINE) is None
    assert "hdma_usart2" not in usart and "hdma_usart2" not in irq

    # 没有新占 stream：每路 stream 只归一个请求，总数仍与请求数一致。
    streams = re.findall(r"^Dma\.\w+\.\d+\.Instance=(DMA\d_Stream\d)$", ioc, re.MULTILINE)
    assert len(streams) == len(set(streams)) == int(ioc_value("Dma.RequestsNb"))


def test_spi2_mspinit_carries_no_old_board_pins() -> None:
    """USER CODE 区里不能再留上一块板的 SPI2 引脚。

    CubeMX 只重新生成它自己那部分，USER CODE BEGIN/END 之间的内容原样保留。
    老板子的 SPI2 是 SCK=PA9 / MOSI=PC1 / MISO=PC2_C，那段手写的 HAL_GPIO_Init
    就这样跟着移植活了下来，并且**排在生成代码之后**，所以它是最后说了算的那个。

    在 MicoAir743v2 上这两个脚另有其人：
        PA9 = USART1_TX（调参/遥测口的发送脚）
        PC1 = BATT_CURRENT_SENS（对面是电流采样运放的**输出**）
    把它们配成 SPI2 的复用推挽，等于让 MCU 和运放两个输出对顶，同时让 USART1
    发不出字节。2026-09-10 实机 `REQ mod=IMUSEL op=BUS` 读到两者都是 mode=2 af=5，
    确认仍然生效。

    这段只能在 STM32CubeIDE 里删（本仓库禁止手改 CubeMX 生成文件），所以把它
    立成一条红测试，而不是留在某个人的记忆里。
    """
    spi = read("Core/Src/spi.c")

    begin = spi.find("USER CODE BEGIN SPI2_MspInit 1")
    end = spi.find("USER CODE END SPI2_MspInit 1")
    assert begin != -1 and end > begin, "找不到 SPI2_MspInit 的 USER CODE 区"
    block = spi[begin:end]

    assert "HAL_GPIO_Init(GPIOA" not in block, (
        "SPI2_MspInit 仍在把 GPIOA 的脚配成 SPI2 复用——那是老板子的 SCK(PA9)，"
        "本板上 PA9 是 USART1_TX。请在 STM32CubeIDE 里删掉这段 USER CODE。"
    )
    assert "GPIO_PIN_1|" not in block and "GPIO_PIN_1 |" not in block, (
        "SPI2_MspInit 仍在把 PC1 配成 SPI2 复用——本板上 PC1 是电池电流采样输入，"
        "对面是运放输出，推挽对顶。请在 STM32CubeIDE 里删掉这段 USER CODE。"
    )


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

#!/usr/bin/env python3
"""把 drone-H743.ioc 从自制 H743 板改写成 MicoAir743v2 板级配置。

为什么用脚本而不是手改 .ioc：
    .ioc 有八百多行、引脚/DMA/NVIC/时钟四处互相牵连，手改一处漏一处的概率很高。
    脚本把"改了哪些项"写成可读的数据表，改错了能一眼看出来，也能重复执行。

**这个脚本不生成代码。** 它只改 CubeMX 的工程描述文件。跑完之后必须：
    1. 用 STM32CubeIDE 打开 drone-H743.ioc；
    2. 目视核对引脚视图与时钟树（时钟树不能有红色）；
    3. 点 Generate Code。

引脚依据：doc/micoair743v2/vendor/ardupilot-hwdef.dat 与官方 Betaflight target。

用法：
    python tools/micoair743v2_ioc_migrate.py [--check]
    --check 只报告将要发生的改动，不写文件。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IOC = ROOT / "drone-H743.ioc"

HEADER = "#MicroXplorer Configuration settings - do not modify"

# ---------------------------------------------------------------- 外设清单

# 顺序即 Mcu.IPn 的编号顺序，CubeMX 自己会重排，这里只求完整。
IP_LIST = [
    "CORTEX_M7",
    "DEBUG",
    "DMA",
    "FREERTOS",
    "I2C1",
    "I2C2",
    "MEMORYMAP",
    "NVIC",
    "RCC",
    "SDMMC1",
    "SPI1",
    "SPI2",
    "SPI3",
    "SYS",
    "TIM1",
    "TIM4",
    "TIM17",
    "UART4",
    "UART7",
    "UART8",
    "USART1",
    "USART2",
    "USART3",
    "USB_DEVICE",
    "USB_OTG_FS",
]

# 被移除的外设：它们的引脚在 MicoAir 上全被别的功能占了。
REMOVED_IPS = ["SPI4", "TIM2", "TIM5", "TIM8", "UART5"]

# ---------------------------------------------------------------- 引脚表
#
# (CubeMX 引脚名, {属性: 值})
# 属性里 Signal 必填；Mode / GPIO_Label / GPIOParameters 按需要给。
PINS: list[tuple[str, dict[str, str]]] = [
    # --- 电源 / 时钟 / 调试 ---
    ("PH0-OSC_IN (PH0)", {"Mode": "HSE-External-Oscillator", "Signal": "RCC_OSC_IN"}),
    ("PH1-OSC_OUT (PH1)", {"Mode": "HSE-External-Oscillator", "Signal": "RCC_OSC_OUT"}),
    ("PA13 (JTMS/SWDIO)", {"Mode": "Serial_Wire", "Signal": "DEBUG_JTMS-SWDIO"}),
    ("PA14 (JTCK/SWCLK)", {"Mode": "Serial_Wire", "Signal": "DEBUG_JTCK-SWCLK"}),

    # --- USB CDC（引脚与旧板相同） ---
    ("PA11", {"Mode": "Device_Only", "Signal": "USB_OTG_FS_DM"}),
    ("PA12", {"Mode": "Device_Only", "Signal": "USB_OTG_FS_DP"}),

    # --- SPI2：BMI088（加计与陀螺各一个片选） ---
    ("PD3", {"Mode": "Full_Duplex_Master", "Signal": "SPI2_SCK",
             "GPIOParameters": "GPIO_Speed", "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH"}),
    ("PC2_C", {"Mode": "Full_Duplex_Master", "Signal": "SPI2_MISO"}),
    ("PC3_C", {"Mode": "Full_Duplex_Master", "Signal": "SPI2_MOSI",
               "GPIOParameters": "GPIO_Speed", "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH"}),
    ("PD4", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Speed,PinState,GPIO_PuPd,GPIO_Label",
             "GPIO_Label": "BMI088_A_CS", "GPIO_PuPd": "GPIO_PULLUP",
             "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH", "PinState": "GPIO_PIN_SET"}),
    ("PD5", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Speed,PinState,GPIO_PuPd,GPIO_Label",
             "GPIO_Label": "BMI088_G_CS", "GPIO_PuPd": "GPIO_PULLUP",
             "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH", "PinState": "GPIO_PIN_SET"}),

    # --- SPI3：BMI270 ---
    ("PB3 (JTDO/TRACESWO)", {"Mode": "Full_Duplex_Master", "Signal": "SPI3_SCK",
                             "GPIOParameters": "GPIO_Speed",
                             "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH"}),
    ("PB4 (NJTRST)", {"Mode": "Full_Duplex_Master", "Signal": "SPI3_MISO"}),
    ("PD6", {"Mode": "Full_Duplex_Master", "Signal": "SPI3_MOSI",
             "GPIOParameters": "GPIO_Speed", "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH"}),
    ("PA15 (JTDI)", {"Signal": "GPIO_Output",
                     "GPIOParameters": "GPIO_Speed,PinState,GPIO_PuPd,GPIO_Label",
                     "GPIO_Label": "BMI270_CS", "GPIO_PuPd": "GPIO_PULLUP",
                     "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH", "PinState": "GPIO_PIN_SET"}),

    # --- IMU DRDY ---
    # PC14/PC15 在旧板上是 LSE 晶振脚；本板不接 LSE，让给 BMI088 的两路 DRDY。
    #
    # BMI088 把加计和陀螺做成两颗独立芯片，各有一路 DRDY，两路都实打实接到了 MCU
    # （hwdef: PC15 DRDY1_BMI088_G / PC14 DRDY2_BMI088_A）。但**只有陀螺那路配成外部
    # 中断**：角速率是最内环，节拍由陀螺定；加计 ODR 是 1600 Hz，如果也开中断，
    # 每秒会多出约 1600 次进了 EXTI15_10 又立刻被掩码挡掉的空中断——纯浪费，而且浪费
    # 在优先级最高的中断路径上。把它当普通输入脚：需要时可以查电平，但不产生边沿。
    ("PC15-OSC32_OUT (OSC32_OUT)", {"Signal": "GPXTI15",
                                    "GPIOParameters": "GPIO_Label",
                                    "GPIO_Label": "BMI088_G_DRDY"}),
    ("PC14-OSC32_IN (OSC32_IN)", {"Signal": "GPIO_Input",
                                  "GPIOParameters": "GPIO_Label",
                                  "GPIO_Label": "BMI088_A_DRDY"}),
    ("PB7", {"Signal": "GPXTI7", "GPIOParameters": "GPIO_Label",
             "GPIO_Label": "BMI270_DRDY"}),

    # --- SPI1：外部 NOR。本板没有这颗芯片，保留配置让诊断如实报告"探测失败"。 ---
    ("PA5", {"Mode": "Full_Duplex_Master", "Signal": "SPI1_SCK"}),
    ("PA6", {"Mode": "Full_Duplex_Master", "Signal": "SPI1_MISO"}),
    ("PA7", {"Mode": "Full_Duplex_Master", "Signal": "SPI1_MOSI"}),
    ("PA4", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Speed,PinState,GPIO_PuPd,GPIO_Label",
             "GPIO_Label": "FLASH_CS", "GPIO_PuPd": "GPIO_PULLUP",
             "GPIO_Speed": "GPIO_SPEED_FREQ_VERY_HIGH", "PinState": "GPIO_PIN_SET"}),
    # ICM-42688 候选片选。本板上这个脚是 OSD 的 CS（挂在 SPI1），拉动它不会
    # 影响 SPI2 上的 BMI088；SPI2 上没有 ICM 应答，探测会如实失败并跳过。
    ("PB12", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Speed,PinState,GPIO_PuPd,GPIO_Label",
              "GPIO_Label": "IMU_CS", "GPIO_PuPd": "GPIO_PULLUP",
              "GPIO_Speed": "GPIO_SPEED_FREQ_HIGH", "PinState": "GPIO_PIN_SET"}),

    # --- I2C2：气压计 SPL06(0x77) + 板载磁罗盘 QMC5883L(0x0D)。引脚与旧板相同。 ---
    ("PB10", {"Mode": "I2C", "Signal": "I2C2_SCL"}),
    ("PB11", {"Mode": "I2C", "Signal": "I2C2_SDA"}),
    # --- I2C1：外接磁罗盘口 ---
    ("PB8", {"Mode": "I2C", "Signal": "I2C1_SCL"}),
    ("PB9", {"Mode": "I2C", "Signal": "I2C1_SDA"}),

    # --- 串口 ---
    ("PA9", {"Mode": "Asynchronous", "Signal": "USART1_TX"}),    # 调参 / 遥测
    ("PA10", {"Mode": "Asynchronous", "Signal": "USART1_RX"}),
    ("PA2", {"Mode": "Asynchronous", "Signal": "USART2_TX"}),    # MTF 光流 (MicoLink)
    ("PA3", {"Mode": "Asynchronous", "Signal": "USART2_RX"}),
    ("PD8", {"Mode": "Asynchronous", "Signal": "USART3_TX"}),    # GPS
    ("PD9", {"Mode": "Asynchronous", "Signal": "USART3_RX"}),
    ("PA0", {"Mode": "Asynchronous", "Signal": "UART4_TX"}),     # ELRS / CRSF
    ("PA1", {"Mode": "Asynchronous", "Signal": "UART4_RX"}),
    # 总线舵机：新旧两块板唯一引脚同址的外设。半双工单线 + 上拉。
    ("PE8", {"Mode": "Half_duplex(single_wire_mode)", "Signal": "UART7_TX",
             "GPIOParameters": "GPIO_PuPd", "GPIO_PuPd": "GPIO_PULLUP"}),
    ("PE1", {"Mode": "Asynchronous", "Signal": "UART8_TX"}),     # 维护口
    ("PE0", {"Mode": "Asynchronous", "Signal": "UART8_RX"}),

    # --- PWM：ESC 走 TIM1，舵机走 TIM4，各自独立帧率 ---
    ("PE9", {"Signal": "S_TIM1_CH1"}),    # ESC 1  (MOTOR4 焊盘)
    ("PE11", {"Signal": "S_TIM1_CH2"}),   # ESC 2  (MOTOR3 焊盘)
    ("PE13", {"Signal": "S_TIM1_CH3"}),   # 备用   (MOTOR2 焊盘)
    ("PE14", {"Signal": "S_TIM1_CH4"}),   # 备用   (MOTOR1 焊盘)
    ("PD12", {"Signal": "S_TIM4_CH1"}),   # PWM 舵机 1 (MOTOR7 焊盘)
    ("PD13", {"Signal": "S_TIM4_CH2"}),   # PWM 舵机 2 (MOTOR8 焊盘)

    # --- SDMMC1：飞行日志裸块，四线 ---
    ("PC12", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_CK"}),
    ("PD2", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_CMD"}),
    ("PC8", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_D0"}),
    ("PC9", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_D1"}),
    ("PC10", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_D2"}),
    ("PC11", {"Mode": "SD_4_bits_Wide_bus", "Signal": "SDMMC1_D3"}),

    # --- LED / 蜂鸣器 / 杂项 IO ---
    ("PE3", {"Signal": "GPIO_Output", "GPIOParameters": "PinState,GPIO_Label",
             "GPIO_Label": "LED_RED", "PinState": "GPIO_PIN_RESET"}),
    ("PE2", {"Signal": "GPIO_Output", "GPIOParameters": "PinState,GPIO_Label",
             "GPIO_Label": "LED_GREEN", "PinState": "GPIO_PIN_RESET"}),
    ("PE4", {"Signal": "GPIO_Output", "GPIOParameters": "PinState,GPIO_Label",
             "GPIO_Label": "LED_BLUE", "PinState": "GPIO_PIN_RESET"}),
    ("PD15", {"Signal": "GPIO_Output", "GPIOParameters": "PinState,GPIO_Label",
              "GPIO_Label": "BUZZER", "PinState": "GPIO_PIN_RESET"}),
    ("PC13", {"Signal": "GPIO_Output", "GPIOParameters": "PinState,GPIO_PuPd,GPIO_Label",
              "GPIO_Label": "LED7", "GPIO_PuPd": "GPIO_PULLUP",
              "PinState": "GPIO_PIN_SET"}),
    # Ai-WB2 的使能与在位检测。本板上这两个脚属于 USART6 / UART5，
    # 但本工程没有启用那两个串口，所以继续当普通 IO 用。
    ("PC6", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Label", "GPIO_Label": "LED1"}),
    ("PC7", {"Signal": "GPIO_Output", "GPIOParameters": "GPIO_Label", "GPIO_Label": "LED2"}),
    ("PB5", {"Signal": "GPIO_Input"}),
    ("PD10", {"Signal": "GPIO_Output"}),

    # --- 虚拟引脚 ---
    ("VP_FREERTOS_VS_CMSIS_V2", {"Mode": "CMSIS_V2", "Signal": "FREERTOS_VS_CMSIS_V2"}),
    ("VP_MEMORYMAP_VS_MEMORYMAP", {"Mode": "CurAppReg",
                                   "Signal": "MEMORYMAP_VS_MEMORYMAP"}),
    ("VP_SYS_VS_tim3", {"Mode": "TIM3", "Signal": "SYS_VS_tim3"}),
    ("VP_TIM1_VS_ClockSourceINT", {"Mode": "Internal", "Signal": "TIM1_VS_ClockSourceINT"}),
    ("VP_TIM4_VS_ClockSourceINT", {"Mode": "Internal", "Signal": "TIM4_VS_ClockSourceINT"}),
    # TIM17 是 svc_timestamp 的纯计时基准（不产生 PWM），所以是 Enable_Timer 不是 Internal。
    ("VP_TIM17_VS_ClockSourceINT", {"Mode": "Enable_Timer", "Signal": "TIM17_VS_ClockSourceINT"}),
    ("VP_USB_DEVICE_VS_USB_DEVICE_CDC_FS", {"Mode": "CDC_FS",
                                            "Signal": "USB_DEVICE_VS_USB_DEVICE_CDC_FS"}),
]

# 定时器通道的 SH.* 映射（PWM 生成）
SH_ENTRIES = {
    "SH.GPXTI7.0": "GPIO_EXTI7",
    "SH.GPXTI7.ConfNb": "1",
    "SH.GPXTI15.0": "GPIO_EXTI15",
    "SH.GPXTI15.ConfNb": "1",
    "SH.S_TIM1_CH1.0": "TIM1_CH1,PWM Generation1 CH1",
    "SH.S_TIM1_CH1.ConfNb": "1",
    "SH.S_TIM1_CH2.0": "TIM1_CH2,PWM Generation2 CH2",
    "SH.S_TIM1_CH2.ConfNb": "1",
    "SH.S_TIM1_CH3.0": "TIM1_CH3,PWM Generation3 CH3",
    "SH.S_TIM1_CH3.ConfNb": "1",
    "SH.S_TIM1_CH4.0": "TIM1_CH4,PWM Generation4 CH4",
    "SH.S_TIM1_CH4.ConfNb": "1",
    "SH.S_TIM4_CH1.0": "TIM4_CH1,PWM Generation1 CH1",
    "SH.S_TIM4_CH1.ConfNb": "1",
    "SH.S_TIM4_CH2.0": "TIM4_CH2,PWM Generation2 CH2",
    "SH.S_TIM4_CH2.ConfNb": "1",
}

# ---------------------------------------------------------------- 外设参数

PERIPHERAL_SETTINGS = {
    # TIM1：ESC，1 MHz 时基、400 Hz 帧。ARR 运行期由 BSP_PWM_Init 重设，
    # 这里的 Period 只是初值；Prescaler 必须对，否则脉宽单位就不是 us。
    "TIM1.IPParameters": ("Channel-PWM Generation1 CH1,Channel-PWM Generation2 CH2,"
                          "Channel-PWM Generation3 CH3,Channel-PWM Generation4 CH4,"
                          "Prescaler,Period"),
    "TIM1.Channel-PWM\\ Generation1\\ CH1": "TIM_CHANNEL_1",
    "TIM1.Channel-PWM\\ Generation2\\ CH2": "TIM_CHANNEL_2",
    "TIM1.Channel-PWM\\ Generation3\\ CH3": "TIM_CHANNEL_3",
    "TIM1.Channel-PWM\\ Generation4\\ CH4": "TIM_CHANNEL_4",
    "TIM1.Prescaler": "120-1",
    "TIM1.Period": "2499",
    # TIM4：PWM 舵机，1 MHz 时基、50 Hz 帧。
    "TIM4.IPParameters": ("Channel-PWM Generation1 CH1,Channel-PWM Generation2 CH2,"
                          "Prescaler,Period"),
    "TIM4.Channel-PWM\\ Generation1\\ CH1": "TIM_CHANNEL_1",
    "TIM4.Channel-PWM\\ Generation2\\ CH2": "TIM_CHANNEL_2",
    "TIM4.Prescaler": "120-1",
    "TIM4.Period": "19999",
    # USART3：GPS
    "USART3.IPParameters": "VirtualMode-Asynchronous,BaudRate",
    "USART3.VirtualMode-Asynchronous": "VM_ASYNC",
    "USART3.BaudRate": "38400",
    # SPI3：BMI270。片选由软件控制（GPIO），所以是 NSS 硬件模式关闭。
    "SPI3.IPParameters": "VirtualType,Mode,Direction,CalculateBaudRate,VirtualNSS,DataSize,BaudRatePrescaler",
    "SPI3.VirtualType": "VM_MASTER",
    "SPI3.Mode": "SPI_MODE_MASTER",
    "SPI3.Direction": "SPI_DIRECTION_2LINES",
    "SPI3.VirtualNSS": "VM_NSSHARD",
    "SPI3.DataSize": "SPI_DATASIZE_8BIT",
    "SPI3.BaudRatePrescaler": "SPI_BAUDRATEPRESCALER_32",
    "SPI3.CalculateBaudRate": "3.75 MBits/s",
    # SDMMC1：飞行日志裸块。时钟分频保守取值，先保证能认卡再谈速度。
    "SDMMC1.IPParameters": "ClockDiv,BusWide",
    "SDMMC1.ClockDiv": "4",
    "SDMMC1.BusWide": "SDMMC_BUS_WIDE_4B",
}

# ---------------------------------------------------------------- 时钟树
#
# HSE 从 12 MHz 变 8 MHz，PLL 倍频必须跟着改，否则整机时钟全错：
#   PLL1: 8 MHz / DIVM1(1) * DIVN1(60) = 480 MHz VCO → /2 = 240 MHz SYSCLK（不变）
#   PLL3: 8 MHz / DIVM3(1) * DIVN3(24) = 192 MHz VCO → /4 = 48 MHz USB（不变）
# 也就是说：把原来乘 12 的倍数改成乘 8 的等效倍数，下游频率一个都不变。
CLOCK_SETTINGS = {
    "RCC.HSE_VALUE": "8000000",
    "RCC.DIVN1": "60",
    "RCC.DIVN3": "24",
    "RCC.VCOInput1Freq_Value": "8000000",
    "RCC.VCOInput3Freq_Value": "8000000",
}

# ---------------------------------------------------------------- DMA 追加
#
# 旧配置已用 14 条 stream（DMA1 全部 8 条 + DMA2 的 0~5）。
# GPS 搬到 USART3 需要两条，正好用掉 DMA2 剩下的 6 和 7，总数 16 —— 打满。
# SPI3(BMI270) 因此没有 DMA，走阻塞传输：它是备用 IMU，800 Hz 下 12 字节
# 的阻塞读远短于一个周期，不值得为它挤掉 GPS 的 DMA。
DMA_TEMPLATE = {
    "EventEnable": "DISABLE",
    "FIFOMode": "DMA_FIFOMODE_DISABLE",
    "MemDataAlignment": "DMA_MDATAALIGN_BYTE",
    "MemInc": "DMA_MINC_ENABLE",
    "PeriphDataAlignment": "DMA_PDATAALIGN_BYTE",
    "PeriphInc": "DMA_PINC_DISABLE",
    "Polarity": "HAL_DMAMUX_REQ_GEN_RISING",
    "Priority": "DMA_PRIORITY_HIGH",
    "RequestNumber": "1",
    "RequestParameters": ("Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,"
                          "MemDataAlignment,Mode,Priority,FIFOMode,SignalID,Polarity,"
                          "RequestNumber,SyncSignalID,SyncPolarity,SyncEnable,"
                          "EventEnable,SyncRequestNumber"),
    "SignalID": "NONE",
    "SyncEnable": "DISABLE",
    "SyncPolarity": "HAL_DMAMUX_SYNC_NO_EVENT",
    "SyncRequestNumber": "1",
    "SyncSignalID": "NONE",
}

NEW_DMA_REQUESTS = [
    # (请求名, 请求序号, DMA stream, 方向, 模式)
    ("USART3_RX", 14, "DMA2_Stream6", "DMA_PERIPH_TO_MEMORY", "DMA_CIRCULAR"),
    ("USART3_TX", 15, "DMA2_Stream7", "DMA_MEMORY_TO_PERIPH", "DMA_NORMAL"),
]

NEW_NVIC = {
    # DRDY 外部中断：PC15 落在 EXTI15_10，PB7 落在 EXTI9_5。
    # PC14 是普通输入，不占中断（原因见上面 PINS 表里的说明）。
    "NVIC.EXTI15_10_IRQn": "true\\:5\\:0\\:true\\:false\\:true\\:true\\:true\\:true\\:true",
    "NVIC.EXTI9_5_IRQn": "true\\:5\\:0\\:true\\:false\\:true\\:true\\:true\\:true\\:true",
    "NVIC.DMA2_Stream6_IRQn": "true\\:5\\:0\\:false\\:false\\:true\\:true\\:false\\:true\\:true",
    "NVIC.DMA2_Stream7_IRQn": "true\\:5\\:0\\:false\\:false\\:true\\:true\\:false\\:true\\:true",
    "NVIC.USART3_IRQn": "true\\:5\\:0\\:false\\:false\\:true\\:true\\:true\\:true\\:true",
    "NVIC.SDMMC1_IRQn": "true\\:5\\:0\\:false\\:false\\:true\\:true\\:true\\:true\\:true",
}

REMOVED_NVIC = ["NVIC.EXTI0_IRQn"]


def escape_key(name: str) -> str:
    """.ioc 的键里空格要转义成 '\\ '，值里不用。"""
    return name.replace(" ", "\\ ")


def load(path: Path) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition("=")
        entries.append((key, value))
    return entries


def is_pin_key(key: str) -> bool:
    head = key.split(".", 1)[0]
    head = head.replace("\\ ", " ")
    if head.startswith("VP_"):
        return True
    if len(head) >= 3 and head[0] == "P" and head[1] in "ABCDEFGH" and head[2].isdigit():
        return True
    return False


def build(old: list[tuple[str, str]]) -> dict[str, str]:
    out: dict[str, str] = {}

    dropped_prefixes = tuple(f"{ip}." for ip in REMOVED_IPS)

    for key, value in old:
        if is_pin_key(key):
            continue
        if key.startswith("Mcu.Pin") or key == "Mcu.PinsNb":
            continue
        if key.startswith("Mcu.IP"):
            continue
        if key.startswith("SH."):
            continue
        if key.startswith(dropped_prefixes):
            continue
        if key in REMOVED_NVIC:
            continue
        out[key] = value

    # 外设清单
    for index, ip in enumerate(IP_LIST):
        out[f"Mcu.IP{index}"] = ip
    out["Mcu.IPNb"] = str(len(IP_LIST))

    # 引脚
    for index, (pin, attrs) in enumerate(PINS):
        out[f"Mcu.Pin{index}"] = pin
        for attr, value in attrs.items():
            out[f"{escape_key(pin)}.{attr}"] = value
    out["Mcu.PinsNb"] = str(len(PINS))

    # 信号映射
    out.update(SH_ENTRIES)

    # 外设参数与时钟
    out.update(PERIPHERAL_SETTINGS)
    out.update(CLOCK_SETTINGS)
    out.update(NEW_NVIC)

    # DMA 追加
    for name, number, instance, direction, mode in NEW_DMA_REQUESTS:
        out[f"Dma.Request{number}"] = name
        prefix = f"Dma.{name}.{number}."
        out[prefix + "Instance"] = instance
        out[prefix + "Direction"] = direction
        out[prefix + "Mode"] = mode
        for attr, value in DMA_TEMPLATE.items():
            out[prefix + attr] = value
    out["Dma.RequestsNb"] = str(14 + len(NEW_DMA_REQUESTS))

    # 生成函数清单交给 CubeMX 自己重建：留着旧的会引用已删除的外设。
    out.pop("ProjectManager.functionlistsort", None)

    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="只报告改动，不写文件")
    args = parser.parse_args()

    old_entries = load(IOC)
    old_map = dict(old_entries)
    new_map = build(old_entries)

    added = sorted(k for k in new_map if k not in old_map)
    removed = sorted(k for k in old_map if k not in new_map)
    changed = sorted(k for k in new_map
                     if k in old_map and old_map[k] != new_map[k])

    print(f"新增 {len(added)} 项 / 删除 {len(removed)} 项 / 修改 {len(changed)} 项")
    for key in changed:
        print(f"  ~ {key}: {old_map[key]} -> {new_map[key]}")

    if args.check:
        return 0

    lines = [HEADER]
    lines.extend(f"{key}={new_map[key]}" for key in sorted(new_map))
    IOC.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"已写入 {IOC}")
    print("下一步：用 STM32CubeIDE 打开 .ioc，核对引脚与时钟树，再 Generate Code。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

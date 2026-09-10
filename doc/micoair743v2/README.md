# MicoAir743v2 板级参考（备选硬件方案）

> 本目录是**备选方案调研资料**，不是当前实机。当前主线硬件仍以 `drone-H743.ioc` +
> [`doc/hardware-reference.md`](../hardware-reference.md) 为准。本目录不构成 PIPELINE 主线需求，
> 上机前的任何结论都必须先在实物上复核。

## 权威边界

- 板级事实的权威是 [`vendor/`](vendor/) 下的四份上游文件，**不得手改**；要更新就重新抓取并更新本文末尾的抓取版本。
- 本文只是摘要与对照表。**本文与 `vendor/` 冲突时，以 `vendor/` 为准。**
- `vendor/ardupilot-hwdef.dat` 给的是 MCU 引脚↔外设映射，**不等于实物排针引出与丝印顺序**。
  官方仓库没有公开原理图和 pinout 图，凡涉及"哪个焊盘"的问题都归入下面的待确认清单。
- 官方固件仓库（70MB 预编译二进制，`*.hex`/`*.bin` 本仓库已 gitignore）克隆在
  `.tmp/micoair743v2/`，不进版本控制；需要时按末尾命令重新拉取。

## 板级事实

| 项目 | 值 | 出处 |
|---|---|---|
| MCU | STM32H743，2048 KB Flash | hwdef |
| **HSE 晶振** | **8 MHz** | hwdef `OSCILLATOR_HZ` |
| 尺寸/重量 | 36×36×8 mm，10 g，30.5 mm 孔距 | ArduPilot README |
| IMU 1 | BMI088 @ SPI2（SCK PD3 / MISO PC2 / MOSI PC3，CS 陀螺 PD5、加计 PD4，DRDY PC15/PC14） | hwdef |
| IMU 2 | BMI270 @ SPI3（SCK PB3 / MISO PB4 / MOSI PD6，CS PA15，DRDY PB7） | hwdef + BF target |
| 气压计 | SPL06 @ **I2C2**（PB10/PB11），地址 `0x77` | hwdef `I2C_ORDER I2C2 I2C1` + BF `I2CDEV_2` |
| 板载磁罗盘 | QMC5883L @ **I2C2**，地址 `0x0D` | hwdef INTERNAL + BF INTMAG |
| 外部磁罗盘口 | **I2C1**（PB8/PB9） | hwdef EXTERNAL + BF EXTMAG |
| UART | 8 路，见下表 | hwdef |
| PWM | 11 路，全部支持 DShot，1–8 支持双向 DShot | ArduPilot README |
| 日志存储 | microSD，SDMMC1 四线（CK PC12 / CMD PD2 / D0–D3 PC8–PC11） | hwdef + BF |
| **外部 SPI NOR** | **无**（SPI1 被 AT7456E OSD 占用，CS PB12） | hwdef / BF / INAV 均无 dataflash |
| 电压/电流采样 | PC0 / PC1，ArduPilot scale 21.12 / 40.2 | hwdef |
| LED | 红 PE3 / 绿 PE2 / 蓝 PE4；LED strip PD14 | hwdef + BF |
| 蜂鸣器 | PD15（`BEEPER_INVERTED`） | BF target |
| 通用 IO | PE5 / PE6（BF 的 PINIO1/2，即 ArduPilot PWM9/10，TIM15） | BF target |
| USB | OTG_FS，PA11/PA12 | hwdef |
| SWD | PA13 / PA14 | hwdef |
| BEC | 9V/3A + 5V/3A，支持 6S | ArduPilot README |

### UART 与 PWM 引脚

| UART | TX | RX | ArduPilot 默认用途 |
|---|---|---|---|
| USART1 | PA9 | PA10 | MAVLink2 |
| USART2 | PA2 | PA3 | DisplayPort |
| USART3 | PD8 | PD9 | GPS |
| UART4 | PA0 | PA1 | MAVLink2 |
| UART5 | PB6 | PB5 | User |
| USART6 | PC6 | PC7 | RCIN |
| **UART7** | **PE8** | PE7 | ESC 遥测（仅 RX） |
| UART8 | PE1 | PE0 | 蓝牙模块 |

PWM 1–11：PE14 / PE13 / PE11 / PE9（TIM1）、PB1 / PB0（TIM3）、PD12 / PD13（TIM4）、
PE5 / PE6（TIM15）、PD14（TIM4，兼 LED 焊盘）。

## 与当前 drone-H743 的对照

| 子系统 | 当前 drone-H743 | MicoAir743v2 | 结论 |
|---|---|---|---|
| 控制律（Driver 层） | HAL-free 纯 C | — | **不动** |
| 总线舵机 | UART7 半双工 @ **PE8** | `PE8 = UART7_TX` | **引脚同址**，待确认焊盘引出 |
| 光流 | MicoLink 帧 | 同厂 MTF 系列 | 任意 UART，直接用 |
| ELRS / CRSF | UART4 + DMA | USART6 为板载 RC 口 | 可用 |
| GPS | USART2 | USART3 为默认 GPS 口 | 可用 |
| 调参口 / 维护口 | USART1 / UART8 | 均在 | 可用 |
| ESC PWM | TIM1 / TIM8 | TIM1 CH1–4 等 11 路 | 可用，定时器需重映射 |
| USB CDC | OTG_FS PA11/PA12 | 同引脚 | 可用 |
| 气压计 | SPL06 @ **SPI4** | SPL06 @ **I2C2** | 同芯片换总线，寄存器逻辑可复用 |
| 磁罗盘 | IST8310 / HMC5883 @ I2C1 | 板载 QMC5883L @ I2C2 | 需新驱动，**或外接 IST8310 走 I2C1** |
| IMU | ICM-42688 @ SPI2 | BMI088 / BMI270 | **需新驱动**（BMI270 上电要刷配置固件） |
| 参数 + 飞行日志 | GD25Q32 @ SPI1 | **板上无外部 NOR** | 参数改内部 Flash 扇区；日志改 SDMMC + FatFs |
| 时钟 | HSE 12 MHz | HSE 8 MHz | PLL 分频重算 |

## 待实测确认清单

上机前必须逐条确认，全部属于"上游文件回答不了"的实物问题：

1. **PE8 / PE7 焊盘是否引出、丝印编号是什么**。hwdef 只描述 MCU 内部映射。
2. **半双工舵机总线的电平与上拉**：UART pad 是否直连 MCU、需不需要外接上拉。
3. **BEC 带载能力**：`DRV_AIRFRAME_SERVO_MOTOR_MASS_G = 348.6` 那组舵机电机的峰值电流
   对 5V/3A 是什么水平。
4. **气压计实际芯片**：ArduPilot 声明 SPL06，Betaflight 与 INAV 声明 DPS310（两者寄存器高度相似）。
   按芯片 ID 实测判定后再决定复用哪份驱动。
5. **板载磁罗盘与气压计同在 I2C2**：若外接 IST8310 复用现有驱动，须走 I2C1。
6. **机体模型要重算**：[`Driver/Inc/drv_airframe_model.h`](../../Driver/Inc/drv_airframe_model.h) 的
   `DRV_AIRFRAME_BOARD_MASS_G` 记的是 75 g，本板仅 10 g，`DRV_AIRFRAME_CG_Z_M` 与推重比需重新核算。
   舵机 FOPDT 辨识、推力表、力臂等绑机体的量不受影响。

## 上游来源与抓取版本

抓取日期：2026-09-10。

| 本地文件 | 上游 | 上游最近提交 |
|---|---|---|
| `vendor/ardupilot-hwdef.dat` | `ArduPilot/ardupilot` `libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/hwdef.dat` | `fcd4db28` |
| `vendor/ardupilot-README.md` | 同上目录 `README.md` | `fcd4db28` |
| `vendor/px4-default.px4board` | `PX4/PX4-Autopilot` `boards/micoair/h743-v2/default.px4board` | `4890d11d` |
| `vendor/inav-MICOAIR743-target.h` | `iNavFlight/inav` `src/main/target/MICOAIR743/target.h`（V1 目标，仅作旁证） | `54adf500` |
| `vendor/betaflight-MICOAIR743V2_EXTMAG-config.h` | `MicoAir/MicoAir743v2` `Firmware/Betaflight/target/MICOAIR743V2_EXTMAG/config.h` | `455e0849` |
| `vendor/betaflight-MICOAIR743V2_INTMAG-config.h` | 同上，`MICOAIR743V2_INTMAG` | `455e0849` |

PX4 主线有官方 target `boards/micoair/h743-v2`；ArduPilot 主线有官方 hwdef。

重新拉取官方固件仓库（含各家预编译二进制，落在 gitignore 的 `.tmp/` 下）：

```bash
git clone --depth 1 https://github.com/MicoAir/MicoAir743v2.git .tmp/micoair743v2
```

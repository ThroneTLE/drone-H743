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

## 移植进度（2026-09-10）

固件侧改动已完成并提交在分支 `feat/micoair743v2`，**但还差一步 CubeMX 生成**。

**明天上机前必须先做**：用 STM32CubeIDE 打开 `drone-H743.ioc` → 核对引脚视图与时钟树
（时钟树不能有红色）→ **Generate Code** → 编译。
在这之前 `tests/test_micoair743v2_generated_code_sync.py` 会红，那是有意为之的提醒，
不是代码写错了。

| 子系统 | 落地情况 |
|---|---|
| `.ioc` 板级重配 | 由 [tools/micoair743v2_ioc_migrate.py](../../tools/micoair743v2_ioc_migrate.py) 改写，引脚/时钟/DMA/NVIC 见该脚本的数据表 |
| BMI088 驱动 | `drv_bmi088.c` + `drv_bmi088_tables.c`（换算表纯函数、宿主可测） |
| BMI270 驱动 | `drv_bmi270.c` + `drv_bmi270_tables.c` + `drv_bmi270_config.c`（Bosch BSD-3 配置数据） |
| 多 IMU 选型 | `svc_imu.c`（装配变换与记账，无 HAL）+ `bsp_imu.c`（探测顺序 BMI088 → BMI270 → ICM42688） |
| 气压计 | `drv_baro.c` 增加 I2C 通路，寄存器逻辑复用；同时接受 SPL06 与 DPS310 的 `PROD_ID 0x10` |
| 磁罗盘 | 无需新驱动，`drv_mag.c` 已支持 QMC5883L 自动探测，只改总线绑定到 I2C2 |
| 参数存储 | `drv_intflash.c`（Bank2 尾两扇区）+ `svc_param.c` 的 H7 ECC 两阶段提交修正 |
| 飞行日志 | `drv_sdblock.c`（SDMMC 裸块，不引 FatFs），`app_flight_log.c` 主体未改 |
| 持久化路由 | `app_flash_service.c` 变成按地址路由的门面，上层地址空间与几何不变 |
| PWM | ESC → TIM1_CH1/CH2 (PE9/PE11)，舵机 → TIM4_CH1/CH2 (PD12/PD13) |
| 未接入 | 电压/电流采样（PC0/PC1）——板上有，但固件里目前没有消费者，故未配 ADC |

### 移植中发现、并已处理的三个坑

1. **H7 片内 Flash 的 ECC**：`svc_param` 的两阶段提交原本会对同一个 32 字节 flash word
   写两次（NOR 上合法，H7 上直接报错）。记录布局加了 8 字节填充，让状态字独占一个 word。
2. **默认方向码**：全新板子参数区是空的，方向码会停在 legacy 哨兵上，退回**老板子**
   实测的符号补偿——用在 BMI088 上横滚方向是反的。现在哨兵状态按探测到的芯片取默认值。
3. **量程刻度随芯片变**：同一个 `±16 g` 枚举，ICM-42688 是 2048 LSB/g，
   BMI088 映射到 ±24 g 后是 1365 LSB/g。原来 App 层写死了 ICM 的表，已改为按芯片分派。

### 审计复核后又修掉的四个 P1 + 两处接线错位（2026-09-10 第二轮）

首轮移植提交 `eec53b8c` 之后做了一轮软件审计，报告与复现原文在
`data/analysis/2026-09-10/micoair_review_eec53b8c/`（`data/analysis/` 已 gitignore，
只在本机；结论已摘录进下表，不依赖那份文件也能看懂）。
四条 P1 全部复核成立并已修复，六条各自都有能在修复前失败的测试
（`tests/test_micoair743v2_review_fixes.py`，替身见 `tests/_micoair_hostfakes.py`）。

| # | 症状 | 根因 | 落点 |
|---|---|---|---|
| P1-1 | 气压计必然不可用，且周期读取把空 GPIO/SPI 指针交给 HAL | I2C 通路改在了**不参与构建**的 `BSP/Src/bsp_spl06.c`，真正被编译的是 `Driver/Src/drv_baro.c` | I2C 通路移入 `drv_baro.c`；死文件 `bsp_spl06.c/.h` 删除；`bsp_baro.c` 不再引用已不存在的 `Press_cs_*` 标签 |
| P1-2 | SD 读回旧数据，状态码仍是 OK | H7 阻塞版 `HAL_SD_ReadBlocks` 是 **CPU 轮询 FIFO**，不是 DMA；读完再 invalidate 会丢弃 CPU 刚写进 cache 的脏行 | 轮询路径去掉全部缓存维护，连 `DRV_SDBLOCK_Bus` 里的钩子字段一并删除，并写明改用 IDMA 时才需要怎么做 |
| P1-3 | 后台日志与通信任务交错时互相覆盖数据，两边都返回 OK | 存储路由重构时漏掉了原有互斥锁；SD 后端有一个**全局共享块缓冲**，不足整块的写是读-改-写 | 读/写/擦除公开入口整笔持锁；因 `flashBusMutex` 非递归，核心逻辑拆成无锁的 `*_unlocked`，公开入口只做加锁—调用—解锁 |
| P1-4 | 主 IMU 配置失败后无限重试，备用 IMU 一次都轮不到 | 只初始化第一颗 probe 成功的芯片，失败即返回；外层重试又选回同一颗 | 先全部探测记账，再按优先级逐个 init，**谁先成功谁上岗**；`SVC_IMU_SelectionRecord` 只记账，选中改由 `SVC_IMU_SelectionCommit` 在 init 成功后完成 |
| 接线 | ELRS 可能整条收不到 | `ConfigureRxPinBias` 仍配老板子的 PD0/AF8，新板 UART4_RX 是 PA1——两个脚接同一路 AF 输入 | 改配 PA1，并加测试断言它与 `.ioc` 的 UART4_RX 分配一致 |
| 接线 | GPS 回调会去认光流的串口 | 回调写死 `USART2`，而新板 GPS 在 USART3、USART2 成了光流口 | 判据改为从 `BSP_Board_GetGpsBus()` 取，只有一个来源 |

> **GPS 仍然是停用的**，这一点没变：它在 `Core/Src/freertos.c` 的两处被注释掉
> （`APP_Task_GPS_Init` / `APP_Task_GPS_Step`），而那是 CubeMX 生成文件，属于禁止手改的范围。
> 上面修的是"绑定与回调不再自相矛盾"，要真正启用 GPS 需要单独决定，并且得由你在
> CubeMX 的 USER CODE 区里放开。

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

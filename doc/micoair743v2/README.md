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
| 接线 | ELRS 可能整条收不到 | `ConfigureRxPinBias` 仍配老板子的 PD0/AF8，新板 UART4_RX 是 PA1——两个脚接同一路 AF 输入 | 改配 PA1，并加测试断言它与 `.ioc` 的分配一致。**后被下面的实物验证一节取代**：ELRS 已整体搬到 USART6/PC7，那条测试改成连"是哪个串口"都从代码推导 |
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
| ELRS / CRSF | UART4 + DMA | **USART6 (PC6/PC7)**，板载 RC 口 | 已搬过去，接收机按丝印插 |
| GPS | USART2 | USART3 为默认 GPS 口 | 可用 |
| 调参口 / 维护口 | USART1 / UART8 | 均在 | 可用 |
| ESC PWM | TIM1 / TIM8 | TIM1 CH1–4 等 11 路 | 可用，定时器需重映射 |
| USB CDC | OTG_FS PA11/PA12 | 同引脚 | 可用 |
| 气压计 | SPL06 @ **SPI4** | SPL06 @ **I2C2** | 同芯片换总线，寄存器逻辑可复用 |
| 磁罗盘 | IST8310 / HMC5883 @ I2C1 | 板载 QMC5883L @ I2C2 | 需新驱动，**或外接 IST8310 走 I2C1** |
| IMU | ICM-42688 @ SPI2 | BMI088 / BMI270 | **需新驱动**（BMI270 上电要刷配置固件） |
| 参数 + 飞行日志 | GD25Q32 @ SPI1 | **板上无外部 NOR** | 参数改内部 Flash 扇区；日志改 SDMMC + FatFs |
| 时钟 | HSE 12 MHz | HSE 8 MHz | PLL 分频重算 |

## 实物验证结果（2026-09-10，出厂 PX4 固件）

**方法**：板子到手后先不刷机，用**出厂固件**把硬件接口验一遍——出厂固件是厂家验证过的
已知good基准，这时候读出来的任何异常都只可能是硬件问题，不会和我们自己的代码混在一起。
刷完我们的固件就没有这个基准了。

出厂固件：`PX4 1.15.4`，分支 `micoair-1.15.4`，构建于 2025-04-16，
`HW arch: MICOAIR_H743_V2`，`MCU: STM32H7[4|5]xxx, rev. V`。

走 MAVLink `SERIAL_CONTROL` 进 NuttX nsh 控制台取证，全程只读命令
（唯一的写操作是在 SD 卡上建了一个测试文件又删掉）。

| 接口 | 我们的绑定 | 出厂 PX4 实测 | 结论 |
|---|---|---|---|
| BMI088 | SPI2，CS PD5/PD4 | 加计 `Type 0x6A` + 陀螺 `Type 0x66`，**两个独立 device_id，同在 SPI:2**；`error_count 0` | ✅ 分体双片选在硬件上是通的 |
| BMI270 | SPI3，CS PA15 | `Type 0x37`，**SPI:3**，加计陀螺共用 device_id；`error_count 0`，且在被实时陀螺标定 | ✅ |
| 气压计 | `hi2c2` 地址 `0x77` | `Type 0x4F, I2C:2 (0x77)`，`error_count 0`；该 target **只编进了 `GOERTEK_SPL06` 一个气压计驱动** | ✅ 芯片吃 SPL06 的寄存器与补偿模型，上游 SPL06/DPS310 之争对我们无影响 |
| 磁罗盘 | `hi2c2` 地址 `0x0D` | `Type 0x08, I2C:2 (0x0D)` | ✅ |
| I2C2 总线 | 气压计 + 磁罗盘共用 | `i2cdetect -b 2` 扫出且**仅扫出** `0x0D` 与 `0x77` | ✅ 无地址冲突 |
| SDMMC1 | 飞行日志裸块 | `/dev/mmcsd0` 已挂载；写入→读回→删除一轮，内容字节一致 | ✅ 读写都通 |
| ADC | PC0 电压 / PC1 电流 | `adc_report` 的 `channel_id=[10, 11, 20]`，16 位，`v_ref 3.3`。H743 的 ADC ch10/ch11 就是 **PC0/PC1** | ✅ 硬件通，固件侧尚未接入 |
| 串口 | 8 路 | `/dev/ttyS0`～`ttyS7` 全部实例化 | ✅ |
| GPS 口 | `huart3`（USART3, PD8/PD9） | PX4 的 `GPS1 = /dev/ttyS2`，`gps status` 也开在 ttyS2 | ✅ 绑定正确 |
| PWM | ESC→TIM1 PE9/PE11，舵机→TIM4 PD12/PD13 | `pwm_out status` 10 路，分组 `{0-3} {4,5} {6,7} {8,9}` 与 TIM1/TIM3/TIM4/TIM15 布局吻合；已在 400 Hz | ✅ 四个脚都是真实输出 |
| USB CDC | 调参 / 遥测口 | 整轮取证就是走它做的 | ✅ |

### 由此结掉的原"待实测"项

- **气压计到底是 SPL06 还是 DPS310** —— 结了，按 SPL06 处理即可。
- **板载磁罗盘与气压计同在 I2C2** —— 结了，扫描证实只有这两个地址，不冲突。

## 待实测确认清单（仍未结）

以下是出厂固件也回答不了的，必须靠眼睛、示波器或带载：

1. **PE8 / PE7 焊盘是否引出、丝印编号是什么**。hwdef 只描述 MCU 内部映射。
2. **半双工舵机总线的电平与上拉**：UART pad 是否直连 MCU、需不需要外接上拉。
3. **BEC 带载能力**：`airframe.servo_motor_mass_g = 348.6` 那组舵机电机的峰值电流
   对 5V/3A 是什么水平。
4. **两颗 IMU 的实际安装朝向**。出厂 PX4 输出的是它自己旋转之后的机体 FRD，证明不了
   我们的旋转常数。可做的判据：把板子**正面朝上放平**，读 `listener sensor_accel`，
   PX4 的 FRD 下应当是 `z ≈ -9.81`；若读到 `+9.81` 说明板子是倒扣的。
   我们自己的常数最终仍要刷完固件后用倾斜实验复核。
5. **机体模型要重算**：2026-09-11 起机体数据的唯一来源是 Flash（见
   [`Driver/Inc/drv_airframe_params.h`](../../Driver/Inc/drv_airframe_params.h)），
   固件里一个默认值都没有，**没写过机体模型的板子禁止解锁**。
   旧常量里 `board_mass_g` 记的是 75 g，本板仅 10 g；整机质量、重心与推重比
   全部要拿秤和尺重新量，从上位机"机体模型"页写进去。
   舵机 FOPDT 辨识、推力表、力臂等绑机体的量不受影响。
6. **LED**：`led_control` 没编进这份 PX4，LED 由 PX4 的状态逻辑自行驱动，只能靠眼睛确认
   红/绿/蓝三颗（PE3/PE2/PE4）是否都在亮。
7. **蜂鸣器（PD15）是外接焊盘，板上没有发声体**（2026-09-10 实物确认）。
   `tune_control play` 命令能正常返回，但没接蜂鸣器就听不到，因此这条**无法用出厂固件验证**，
   要等接上外置蜂鸣器再说。固件侧目前也没有蜂鸣器驱动，暂不影响。

## 刷我们的固件之前必须注意

1. **SD 卡内容会被毁掉**。卡上现有 `APM/`、`log/`、`params`、`parameters_backup.bson`
   （这张卡先后跑过 ArduPilot 和 PX4）。我们的 `drv_sdblock` 把卡当**裸块**用，
   从第 `DRV_SDBLOCK_BASE_BLOCK = 2048` 块（即 1 MB 处）开始写——而 1 MB 正是最常见的
   分区起始位置，文件系统会立刻被覆盖。第 0 块（MBR）不碰，但那救不了数据。
   → 想留着出厂数据就**换一张空卡**，或者先把卡拷出来。
2. ~~遥控接收机的接法与板子丝印不一致~~ —— **已解决**：ELRS 于 2026-09-10 从
   UART4(PA0/PA1) 搬到了板载 RC 口 **USART6(PC6/PC7)**，接收机按丝印插即可。
   UART4 随之从工程里移除，它的两条 DMA stream 原样交给 USART6（DMA 仍是 16/16）。
   连带修掉两处遗留：`LED1`/`LED2` 标签原先压在 PC6/PC7 上（老板子的 Ai-WB2 使能脚），
   已移到真正的灯 PE3(红)/PE2(绿)；`bsp_aiwb2_power.c` 不再读写 PC6——
   那现在是遥控链路的发送脚，读它当"WiFi 使能状态"会拿到随机的串口数据位。
3. **刷 `0x08000000` 会覆盖 PX4 的 bootloader**，这是预期行为，而且**可完全回退**——
   还原镜像在仓库里（见下），不依赖任何临时目录。

### 烧录前基线（回滚用，2026-09-10 锁定）

板子出厂时在跑的、也是我们**第一次烧录前**的状态：

| 项目 | 值 |
|---|---|
| 固件 | PX4 `Release 1.15.4 (17761535)`，分支 `micoair-1.15.4` |
| PX4 git-hash | `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` |
| 构建时间 | `Apr 16 2025 17:04:09` |
| NuttX | `Release 11.0.0`，git-hash `5d74bc138955e6f010a38e0f87f34e9a9019aecc` |
| HW arch | `MICOAIR_H743_V2` |
| PX4GUID | `0006000000003835323433335104004c0042` |
| 板子实测 | 以上各项由实机 `ver all` 读出，非推测 |

还原镜像就在仓库里 —— [`baseline/`](baseline/)，**已纳入版本控制**：

| 文件 | 大小 | sha256 |
|---|---|---|
| [`baseline/MicoAir743v2-PX4-1.15.4-Bootloader+Firmware.bin`](baseline/MicoAir743v2-PX4-1.15.4-Bootloader%2BFirmware.bin) | 2097152 | `4f553745c2946eaf3ac68bf83626751df71ad710db6e1163e6831b5f545591d2` |
| [`baseline/MicoAir743v2_PX4-1.15.x_bootloader.bin`](baseline/MicoAir743v2_PX4-1.15.x_bootloader.bin) | 41020 | `6f97070a8dade37c151dd26a758c82b95b85a8811a6189901f154e93b4b05b7c` |

第一个是一把还原到出厂状态（刷 `0x08000000`）；第二个只有 bootloader，
刷完可以用 QGC 之类的地面站再装应用层。

> **为什么放进版本控制。** 这两个文件原本只存在于 `.tmp/micoair743v2/` —— 那是公共
> 临时区，任何清理动作（包括别的 agent）都可能把它抹掉，而丢了它就等于失去回到出厂
> 状态的能力。`.gitignore` 里 `*.bin` 本来是忽略的，这里照 `!tests/golden/*.bin`
> 的先例开了白名单：2 MB 的代价换"删不掉"，值。
> `tests/test_micoair743v2_review_fixes.py` 里有一条测试逐字节核对这两个校验和。
>
> 另有 ArduPilot 4.5.x 与 PX4 1.14 / 1.16 的 bootloader 可从官方固件仓库取
> （见本文末尾的 `git clone`），但**回到"烧录前"应当用上面这个 1.15.4**。

## 首刷实测结果（2026-09-11，我们自己的固件）

`0x08000000` 整片擦除后写入，校验通过。启动后 USB CDC 枚举为 `VID:PID=0483:5740`，
命令面应答正常。逐链路点名：

| 链路 | 结果 |
|---|---|
| USB CDC 命令面 | ✅ `PING` → `PONG drone-H743` |
| IMU（BMI270 @SPI3） | ✅ **810 Hz 稳定出数**，`fault=0`，`frame=canonical_flu_ram`；板子倒扣时 `az≈-1008 mg` |
| IMU（BMI088 @SPI2） | ❌ 未上岗，见下节 |
| 气压计 SPL06 @I2C2 | ✅ `pressure_pa=96163`，标定系数全部读出 |
| 磁罗盘 QMC5883L @I2C2 | ✅ 采样计数持续增长 |
| 片内 Flash 参数 | ✅ 加载默认值（刚整片擦过，`cfg_valid=0` 符合预期） |
| ELRS @USART6 | ⏸ `frames=0`，接收机尚未接上 |
| GPS @USART3 | ⏸ 任务仍被注释掉（`Core/Src/freertos.c`，需 CubeMX 侧启用） |
| 光流 @USART2 / SDMMC / PWM | ⏸ 未测 |

软件跳 DFU 可用：`BOOT?` 看安全前置条件，`BOOT DFU CONFIRM` 让固件自己跳进 ROM
bootloader，不用按 BOOT 键。**前提是命令任务没卡死**。

### BMI088 两个缺陷（2026-09-11 已定位并修复）

首刷时 BMI088 被判 `bad_id` 没能上岗，只剩 BMI270 可用。实际是**两个独立缺陷叠在一起**，
每一个单独都足以让它失败：

**其一：`MasterKeepIOState`（SPI CFG2 的 AFCNTR）默认关闭。**
SPE=0 期间 SPI 把引脚控制权交还给 GPIO，SCK 不再被钉在 CPOL 电平上；下一笔事务时
片选已经拉低、随后使能 SPE，SCK 这一跳在从机看来就是一个多余的时钟沿，整串数据错位
——读回全 `0x00`，而 HAL 一路报成功。症状是**一串事务里只有第一笔拿得到数据**，
间隔 50 ms 仍失败、隔 100 ms 以上又全部正常。探测正好踩中（驱动是"丢弃式首读 +
紧接着真读"）。SPI3 上的 BMI270 恰好能容忍这个毛刺，"SPI2 坏 SPI3 好"因此把排查
引向了错误的方向。修法：`BSP_IMU_ConfigureSpiMode` 对两条 IMU 总线强制打开，
`.ioc` 与生成代码的三个 SPI 同步改为 ENABLE。

**其二：寄存器写用了 `HAL_SPI_Transmit`。**
H7 全双工主机模式下发出去的每个字节同时也会收进 RX FIFO，而 HAL 的
`SPI_CloseTransfer` **不清它**。`bmi088_acc_write()` 是写后回读校验，回读先拿到那两个
陈字节，校验永远不符，8 次重试耗尽后返回 `ERROR`——表现为"探测通过、初始化失败"。
修法：加计与陀螺的寄存器写都改成收发等长的 `TransmitReceive`，rx 收下即丢，只为配平。
同一根因还修了 `bmi088_gyro_read`（原先是 `Transmit` + `Receive` 两段式）。

修复后实测：

```
selected=BMI088 chip_id=0x1E  probe=ok  init=ok
IMU ok=1 who=0x1E  rate_hz=1680  fault=0   health level=0（OK，BMI270 时是 1=降级）
ax=-982 ay=213 az=52  →  合成 1006 mg ≈ 1 g
```

定位过程中排除掉的（每条都是实机测过的，留作后来人的负面证据）：引脚复用与 PC2/PC3
模拟开关、SPI 速率（3.75 MHz 与 470 kHz 表现一致）、RX FIFO 残留、SPI2 外设状态
（连 RCC 硬复位都救不回来）、生成代码残留的 PC1/PA9 复用、SPI2 的 NVIC 中断线、
CSB 最小空闲时间、加计的 suspend 模式。

### 采样节拍与初始化稳定性（2026-09-11 已修）

BMI088 上岗后又暴露两件事，一并修掉：

**节拍虚高到 1760 Hz，而陀螺只有 1000 Hz。** 逐个关中断源实测：关掉陀螺 DRDY
速率掉到 810，关掉加计 DRDY 毫无变化——多出来的约 800 Hz 来自 **BMI270**。
读它的 `PWR_CTRL=0x0E`（加计陀螺都还开着），软复位之后速率立刻回到 1000 Hz。

根因是 `HAL_GPIO_EXTI_Callback` 里那个静态掩码同时接受两颗 IMU 的引脚，注释写的
理由是"另一颗没初始化就不会产生边沿"——**冷启动成立，软复位不成立**：
`BOOT DFU CONFIRM`、看门狗复位都不给传感器掉电，上一轮配置过的那颗照旧按自己的
ODR 发边沿。后果不是多几次空唤醒：控制环被以约 1.7 倍于陀螺更新率的节奏唤醒，
三成迭代拿到重复样本，而角速率是最内环，重复样本对 D 项就是噪声放大。

修法两道闸：`BSP_IMU_GetDrdyPin()` 按选中的芯片给出唯一的引脚供回调比对；
`BSP_IMU_RouteDrdy()` 在选型敲定后直接清掉未选中那颗的 EXTI 屏蔽位（用
`EXTI_D1->IMR1` 的单个位，不是关整条共用中断线）。实测 `IMR1=0xFFC08000`：
bit15（PC15/BMI088 陀螺）置位、bit7（PB7/BMI270）清零，`rate_hz=1000`，`level=0`。

**初始化时好时坏，约每三四次复位失败一次。** 失败时 BMI088 探测通过却上不了岗，
退到 BMI270——两颗的安装旋转与量程刻度都不一样，这种不确定性比干脆失败更危险。
根因是 `bmi088_acc_write()` 写完立刻回读校验，而 datasheet 要求写
`ACC_PWR_CONF`/`ACC_PWR_CTRL` 之后至少 **450 µs** 才能再访问；8 次重试挤在一起，
整段都可能落在那个窗口里。写与回读之间加 1 ms 落定后，**连续 8 次复位全部上岗**。

### 同轮修掉的诊断谎报（D5-3）

这几条不是新功能，是原有诊断在说假话，而且都实打实地把排查带偏过：

| 位置 | 谎报内容 | 修法 |
|---|---|---|
| `APP_IMU_GetStatus` | `sample_count` 与 `ax/ay/az/gx/gy/gz` 被硬写成 0，一颗 810 Hz 满血运转的 IMU 看起来和彻底死了一模一样 | 改从稳定器的只读验证快照取数，与 `IMU?` 同源 |
| `APP_Baro_GetStatus` | `product_id` 永远是 0（唯一写它的 `APP_Baro_ReportStartup` 被 `APP_MESSAGE_STARTUP_REPORT_ENABLED=0` 编译掉了），于是好的气压计被判 `ok=0 stage=who_id` | 改为从 `BSP_BARO_GetDevice()` 实时读，不产生总线事务 |
| `APP_Flash_GetStatus` | 同样从不刷新，三个状态字段恒为 0 → 判据读成"全部成功"，给**板上根本不存在的**外部 SPI NOR 开健康证明 `ok=1 stage=ready` | 惰性首刷一次（片选在 SPI1 上接的是 AT7456E OSD，不宜每次 `STATUS?` 都抖） |
| WiFi 诊断 | 仍印 `pin=PC6`，而 PC6 现在是 USART6_TX（ELRS 的发送脚） | 改为 `pin=none` |

另修一处 H7 写法错误：`bmi088_gyro_read` 是全套 IMU 驱动里唯一用
`HAL_SPI_Transmit` + `HAL_SPI_Receive` 两段式的读。在 H7 全双工主机模式下，
发地址字节的同时也会收进一个字节且 HAL 不清，随后的 Receive 先交出那个陈字节，
整串错位一格。已改为单次 `TransmitReceive`。

## 机体模型改为运行时参数（2026-09-11，已实机验证）

固件里不再保留任何机体数据。质量、重心、惯量、力臂、下桨旋向全部改成
`airframe.*` 运行时参数，唯一来源是 Flash，由上位机「机体模型」页写入
（[`Driver/Inc/drv_airframe_params.h`](../../Driver/Inc/drv_airframe_params.h)）。
**没有有效模型时禁止解锁**，原因码 `APP_LED_ARM_BLOCK_AIRFRAME`（LED_3 闪 7 下）。

实机复核（COM22，写 RAM 未保存，复位后已自行清回零）：

| 命令 | 结果 | 说明 |
|---|---|---|
| `AIRFRAME?`（刚烧完） | 全部 `0.000000` | v19→v20 迁移按设计调 `DRV_Airframe_Clear()`，旧常量确实没了 |
| `REQ mod=ARM op=STATUS` | `block=no_rc ... airframe=0 airframe_missing=airframe.mass_kg` | 原因链只报第一条（没插遥控），但条件清单同时暴露机体模型也缺——这正是横幅列全条件的理由 |
| 逐条 `PARAM SET airframe.*` | 全部 `OK` | 派生值当场重算：`mass_kg=0.754600`、`cg_z_m=-0.094558`、`tether_attach_to_cg_m=0.250858`、`rod_to_cg_m=0.890858`，与仓库历史值一致 |
| 再查 `ARM` | `airframe=1 airframe_missing=-` | 闸门如期放行 |
| 自动档下 `PARAM SET airframe.mass_kg 99` | `ERR param target` | 派生值当场拒绝，不是"写进去再被覆盖" |

### 参数存储改用片内 Flash（2026-09-11 已修复并实机验证）

首次验证时 `SAVE` 回 `OK save st=4`。**根因是两个缺陷叠在一起，而且互相遮掩**：

1. **配置记录没有物理落点**。参数区在逻辑上留了三个扇区，物理上只映射了两个；
   配置记录（`APP_CONTROL_CFG_ADDRESS`）正好落在那个被注释成"没人用、留作边界
   缓冲"的第三扇区上——它其实一直有人用。于是 `flash_param_physical()` 找不到
   映射，一路返回 `INVALID_ARG`（= `st=4`）。
2. **v20 记录的 size 算错**。`Save` 写的 `size` 含机体模型块，读回来的校验式不含，
   自己写的记录过不了自己的检查。缺陷 1 让它一直没机会暴露。

修法是把参数区做实：

| 逻辑扇区（顶部起） | 用途 | 物理扇区 | 物理地址 |
|---|---|---|---|
| 顶-5 | 诊断擦写区（`FLASH SCRATCH TEST`，破坏性） | 3 | `0x08160000` |
| 顶-4 | `svc_param` 槽 A（IMU/舵机/舵机型号/朝向标定） | 4 | `0x08180000` |
| 顶-3 | `svc_param` 槽 B | 5 | `0x081A0000` |
| 顶-2 | 配置记录 槽 A（增益 / 遥控映射 / **机体模型**） | 6 | `0x081C0000` |
| 顶-1 | 配置记录 槽 B | 7 | `0x081E0000` |

擦除粒度是 128 KB，所以"能独立擦除的单位"就是一整个扇区——一个逻辑槽必须独占
一个物理扇区，否则擦 A 会把 B 一起抹掉。诊断擦写区也必须单独占一个：它是破坏性的，
跟真数据共用扇区就是"跑一次诊断把标定擦了"的陷阱。代码段随之从 1792K 收到 1408K
（当前用 436K，仍有三倍余量）。

配置记录同时补上 **A/B 双槽 + 两阶段提交**：写另一个槽 → 回读校验 → 再单独写提交字
（独占一个 32 字节 flash word，因为 H7 的 ECC 让每个 word 在两次擦除之间只能编程一次）。
整个过程中当前那个槽一直没被碰过，任何一步掉电都还能读回上一份好配置。
以前是单槽原地擦写，那时丢了只是"增益要重设"；现在这条记录里装着机体模型，
丢了就禁止解锁，代价不一样了。

**实机验证（COM22）**：

```
OK save st=0
HW PARAMSTORE backend=INTFLASH cfg_slot=0x3FE000 cfg_valid=1 cfg_loaded=1
              last_save=0 log_backend=SDBLOCK log_ready=1 nor=absent_on_this_board
```

复位后 `AIRFRAME?` 读回 `mass_kg=0.754600 cg_z_m=-0.094558`，`ARM` 报
`airframe=1 airframe_missing=-`。**重新烧录固件之后机体模型依然在**——参数扇区
在代码段之外，刷固件不会碰它。

`HW FLASH ok=0 ... id=000000` 仍然会显示，那说的是**外部 SPI NOR**，本板确实没有
这颗芯片，是实话。为免被读成"配置保存坏了"，`STATUS?` 紧跟着报一行 `HW PARAMSTORE`
说明参数真正的去向。顺带确认：SD 卡在位可用（`log_ready=1`）。

## 板载蓝牙与 USB / 数传同权（2026-09-11）

蓝牙模块接在 **UART8（TX=PE1 / RX=PE0，115200）**，也就是固件里的"维护口"
（`App/Src/app_maint_uart.c`）。

**命令面本来就是同一套**：维护口收到的行直接交给 `APP_Control_ProcessLine`，
和 USB 走同一个解析器，所有命令（`PARAM SET`、`AIRFRAME?`、`REQ mod=...`、
`SAVE`……）在蓝牙上一字不差地可用。缺的是另外两半，这轮补齐：

| 缺什么 | 后果 | 补法 |
|---|---|---|
| 遥测流没有蓝牙出口 | 蓝牙只能敲命令、看不了波形 | 新增 `APP_TELEM_SINK_BT`，命令 `TELEM SINK bt`；`SINK auto` 现在也认得蓝牙 |
| 异步文本到不了蓝牙 | 蓝牙上看不到 `READY`、心跳和任何没人问也该来的回报 | 结构化文本在蓝牙链路活跃时一并镜像过去 |

两处实现上的取舍：

- **二进制另开入口**。遥测帧里含 `0x00`，`APP_MaintUART_Write` 的形参是
  `const char *`，强转着用能编过、也能发出去大部分字节，坏在哪一帧取决于数据
  内容。所以加了 `APP_MaintUART_WriteRaw(const uint8_t *, uint16_t)`，
  用错了在编译期就是类型错误。
- **镜像有闸门**。`APP_MaintUART_Write` 是阻塞发送（100 ms 超时），无条件镜像
  会让没连蓝牙时每条文本都在 UART8 上白等一次，直接拖慢 UART 任务。所以只在
  最近 30 s 内收到过蓝牙命令时才镜像。

帧长上限与数传同档（`APP_UART_TX_TEXT_SIZE - 开销`），**没有**套用 USB 那条放宽的
上限：115200 下 40 Hz 大约 11.5 kB/s，给它更大的帧只会丢帧，而丢帧表现成波形
断断续续，很容易被误判成蓝牙模块不稳定。

上位机**不需要改**：它从不显式发 `TELEM SINK`，一直用 `auto`，而 `auto` 跟随
"最近一条命令是从哪条链路进来的"。所以把地面站连到蓝牙的虚拟串口上，
开流就自动走蓝牙。

### 实机验证状态

`TELEM SINK bt` 在 USB 上试过，飞控回 `sink=bt`，切回 `usb` 也正常。
**但整条蓝牙链路本身还没实测过**——需要先把蓝牙模块配对、拿到它的虚拟串口号，
再用地面站连上去跑一遍命令与波形。模块的配对方式、默认波特率是否就是 115200，
这两件事要你在实物上确认。

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

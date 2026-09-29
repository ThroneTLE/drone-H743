# MicoAir743v2 电调输出

软件提供编译期选择，默认单向 DShot300。软件测试不是实机电调验证。

| 输出 | 绑定 | 用途 |
|---|---|---|
| M4 / PE9 | TIM1_CH1 | 电调 1 |
| M3 / PE11 | TIM1_CH2 | 电调 2 |
| M7 / PD12 | TIM4_CH1 | 舵机 1，50 Hz PWM |
| M8 / PD13 | TIM4_CH2 | 舵机 2，50 Hz PWM |

**通道 1/2 分别接上桨还是下桨不写死在代码里**（2026-09-13 起）。装配时两根线可能
插反，而上下桨的偏航力臂 ku/kl 不同：接反之后推力分配整体错位、倾转力矩跟着偏，
飞机看起来还能自稳。归属与每一路的旋向由上位机「校准 · 桨叶与电机方向」页标定，
存 CFG 记录（`Driver/Inc/drv_prop_map.h`），**偏航极性由它推出**。
未标定时偏航极性为 0（没有方向），分配式给不出偏航力矩；500 Hz 提交点按角色查通道，
查不到就物理禁用，不退回下标顺序。它**不是**解锁闸门——什么时候需要标定由作者决定。

`PROPCAL SPIN` 是全仓库唯一一条能从上位机让电机转起来的路径：要显式确认词、
只在未解锁时可用、一次只转一路、油门上限由固件裁决，而且**靠心跳续命**——
上位机每 100 ms 重发一次，飞控 300 ms 没听到就断电，超时判定跑在 500 Hz 控制环里
（`App/Src/app_prop_spin.c`）。`APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS` 仍然是 0。

### 油门上限分两层（2026-09-21）

这一页原本只用来看旋向，20% 够了。后来它也被用来在台架上测电流——那件事要把
油门推到能产生可观电流的程度。把常量从 20 改成 100 也能达到目的，但那样看旋向
这条老路径的保护就一起没了：一次误拖滑条就是满推力。

所以拆成 `APP_PROP_SPIN_DEFAULT_MAX_PERCENT`(20) 与 `APP_PROP_SPIN_HARD_MAX_PERCENT`(100)：

```
PROPCAL SPIN ARM confirm=safe [max_pct=1..100]
```

不写 `max_pct` 就是 20%，默认路径一个字没变。写了才可能更高，而且这个上限
**跟着窗口走**：停一次就回到 20%，关窗后回包里的 `max_pct` 也报 20（界面滑条
跟着它缩回去）。窗口开着时重发 ARM **只降不升**——一个已经开着的窗口不该靠重发
一条命令拿到更大的权限，想抬高必须先 `SPIN STOP`。参数非法的 ARM 不算数，但也
不会顺手把正在转的电机停掉。判据见 `tests/test_prop_spin_safety.py`。

### 心跳回包捎带电调回传

`PROPCAL SPIN ARM|SET|STOP` 的回包新增：

```
esc_telem=<0|1> esc_i1=<A|-> esc_i2=<A|-> esc_erpm1=<n|-> esc_erpm2=<n|->
```

挂在这一行上是因为心跳每 100 ms 必定往返一次（它是安全机制，不是为了取数才有
的），所以多报这几个字段**不增加任何一次往返**。`esc_telem=1` 只在双向 DShot 档
且已经收到过回传帧时出现；默认构建是单向 DShot300，这里恒为 0。

缺失一律报 `-` 而不是 0：`0` 在这几路上本来就是合法值（电调回报"未旋转"时
eRPM 就是 0，EDT 电流分辨率 1 A/LSB，小电流下也是 0），拿合法值当"没有"，
上位机就再也分不清"电调说它停着"和"根本没收到回传"。

报的是**电转速 eRPM**，不是机械转速：换算要除以电机极对数，而固件没有这个配置，
换算等于编一个数。上位机 `tools/thrust_bench/` 自 2026-09-21 起直接用 eRPM 记录与建模，
不再要求填写极对数。

### 解码计数单独一行（`PROPCAL escdiag`）

`PROPCAL SPIN ARM|STOP` 与**裸 `PROPCAL SPIN`**（纯状态查询，不开窗）会另发：

```
PROPCAL escdiag avail=<0|1> proto=<0|1|2> fr1=<n> crc1=<n> to1=<n> fr2=<n> crc2=<n> to2=<n>
```

本节下方那张表里，GCR 的 nibble→五位组对应关系已于 2026-09-21 在 bitbang 后端实机钉死；
但换后端、换电调固件或改线路时仍可能回不来数。而"回不来数"这一个现象底下压着三种完全不同的
故障，只看 `esc_i*`/`esc_erpm*` 的 `-` 是分不开的：

| 现象 | 结论 | 该查哪一侧 |
|---|---|---|
| `avail=0` | 本次构建不是双向档 | 烧 `ESC_PROTOCOL=DSHOT300_BIDIR` |
| 只有 `to*` 在涨 | 线上没有回传 | 电调固件是否已切双向 DShot（**不是**电机正反转的 `bi_direction`） |
| `crc*` 在涨 | 有回传但解不开 | 极性 / 校验取反 / GCR 表 / 位宽 |
| `fr*` 在涨 | 链路通 | 剩下的是数值对不对 |

`proto=` 报的是本次构建档位，烧完之后确认"烧进去的是不是双向档"没有别的地方能看。

**不挂在心跳那一行上**：心跳行已接近 `APP_UART_TX_TEXT_SIZE` 的一半，再塞六个
32 位计数会顶到截断边上；而且计数是慢变量，挂 10 Hz 等于常年多占约 800 B/s。
跑一轮前后各读一次，看哪个计数在涨就够了。

总电压/总电流不走这条行——它们是遥测通道 `batt_v` / `batt_i`，由上位机按页面
可见性订阅推送（见 [telemetry-protocol.md](telemetry-protocol.md)），40 Hz 比这里
的 10 Hz 更适合看采样噪声。`batt_i` 携带的是固件里的 25 点块平均。

## 构建与回退

```powershell
cmake --preset Debug -DESC_PROTOCOL=DSHOT300
cmake --build --preset Debug
cmake --preset Debug -B build/BIDIR -DESC_PROTOCOL=DSHOT300_BIDIR
cmake --build build/BIDIR
cmake --preset Debug -B build/PWM -DESC_PROTOCOL=PWM
cmake --build build/PWM
```

切换协议必须另行烧录；不新增 Flash 参数。TIM1_UP 独占 DMA1_Stream2，UART8 接收保留中断，
TX 继续使用 Stream3 DMA。PWM 后端启动前恢复 TIM1 的 1 MHz 计数/400 Hz 帧率，TIM4 不参与切换。

## 接口与时序

- `drv_dshot` 的编码与交错数组来自固定 PX4 v1.16.0，来源和许可见 `tests/fixtures/dshot_px4/README.md`。
- `BSP_PWM_SetEscPulse/GetEscPulse` 在 DShot 模式下表达等效微秒命令；setter 暂存，500 Hz 最终仲裁点的 `BSP_PWM_CommitEsc` 一次提交两路。
- 1100 us 对应 DShot 0；1101..1940 us 映射到 48..2047。不保证与原 PWM 有相同转速或推力。
- `BSP_PWM_DisableEsc` 立即物理禁用；0 us 禁用与发送停止码不同。原 RC 丢失、验收、DFU 路径保留。
- `BSP_DShot_Submit` 接受两路码与有效位掩码，返回 OK/BUSY/INVALID/ERROR。无循环 DMA、不排队重播旧油门。
- 错误锁存并关闭输出，只有显式初始化/重启才尝试恢复。准备期间禁用会取消该次提交；完成回调不会打开输出。
- `BSP_DShot_GetSnapshot` 中 code 是最后接受的码，enabled 是通道使能状态；必须连同 fault/busy 解读。
- 120 MHz 内核时钟：PSC=0、ARR=399，bit 0/1 高电平为 150/300 tick。实际时钟来自 RCC 配置。
- 两路 CCR 交错；先 UG 锁存零，再由更新事件搬入预装载。数据前有低电平准备周期，两个零尾槽排空预装载。
- 160 字节缓冲在 `.dma_buffer`，32 字节对齐并在发送前 clean。禁用使用有界寄存器操作，不依赖 HAL tick/IRQ。
- 单向档不支持双向遥测、转向设置或电调刷写；DMA 完成不代表电调收到或执行。
  双向档见下方「双向 DShot300 转速回传」；特殊命令两档都只开放 `ESC EDT ON|OFF`（命令 13/14，见下方「DShot 特殊命令」），转向/刷写在两档下都不支持。

## 诊断与日志

`PWM?` 增加 `ESC protocol=...` 行。DShot 明确 `command_unit=pwm_equivalent_us ack=unavailable`，
报告 code/enabled/busy/fault/submitted/completed/busy_rejected/errors/cancelled。
原 esc_us 是等效命令；定时器诊断中的 CCR 是硬件 tick，不能当作微秒命令。

FlightLog V10 头末尾 4 个保留字节使用 `[0xD5,1,protocol,0]`：1=PWM、2=DSHOT300；
协议标记占用扇区头保留字节，受原 CRC 保护。导出 meta 的 sectors 与 CSV 每条记录带 esc_protocol/motor_command_unit；逐条诊断的 V11 扩展见下节。
旧零保留区标为 legacy_unspecified；不根据当前固件推测历史协议，不改变历史 CSV 数值。

## 审核者验证

拆桨测 M4/M3 位序、校验、帧率、末位与中止，同时检查 M7/M8 的舵机 PWM。
记录 AM32 55A 固件版本；验证上电停止、两路低油门、失联禁用、DFU 禁用与重连。
并发 IMU/遥测/日志负载下记录 DMA 错误、控制周期和蓝牙收发；实际转速/推力差异另行实测。

## FlightLog V11 逐条诊断（R-DSHOT-2）

解锁录制仍用原来的125Hz门控，记录从V10的776字节升级为V11的808字节。
原记录中CRC之前的772字节布局不变，追加32字节DShot诊断，然后计算全记录CRC。
扩展为小端 `4B2H6I`：present、enabled_mask、busy、fault；code_ch1、code_ch2（按硬件通道编号，不按上/下桨——归属由 PROPCAL 标定）；
submitted、completed、busy_rejected、errors、cancelled、timer_clock_hz。
在 Observe 构建记录时读取 BSP 的临界区快照，排队后不再读取后续发送状态。
code 是最后被 DMA 后端接受的发送码，原 motor_upper_us/motor_lower_us 仍表示等效PWM指令。
completed 仅表示 DMA 完成，单向 DShot 无电调确认/转速反馈；计数为本次初始化后的累计值。
present=0 表示 PWM 后端无 DShot 诊断；历史V10及更早的新增CSV字段为空，不能按当前协议推断历史值。
256字节扇区头和已定义协议标记保留，固件扫描和主机解码继续识别V10。
每4KiB扇区仍容纳4条记录；净记录数据量从97000增至101000 B/s，队列64条新增2048字节D1 RAM。
Contract：数据与日志采集时刻绑定，历史不重解释；Boundary：新 app_esc_log 适配模块/日志格式与解码；Test seam：真实C结构、构建/Observe路径到Python解析及CSV。

## 双向 DShot300 转速回传（ESC_PROTOCOL=DSHOT300_BIDIR）

单向档只知道"我发了什么"，不知道电调有没有执行。双向档让电调在每帧之后回传**电周期**，
这是转速的唯一实测来源——`kv * 电压 * 油门%` 那种估算不是测量值。

与单向档的三点差别：

- **线电平取反**：空闲高、数据位拉低。CCER 两路设成低有效，MOE=0 时的空闲电平由 OIS1/OIS2 给高。
- **校验取反**：12 bit 载荷相同，4 bit 校验是单向档的反码。电调用这一位区分两种模式，
  写错了整帧被忽略。`tests/test_dshot_telemetry.py` 直接和单向编码器对拍这一条。
- **电调回话**：帧尾之后约 30 us，以 5/4 倍波特率（375 kbit/s）发回 21 bit GCR 帧。

### 不需要改 CubeMX

DMA1/DMA2 共 16 条流在本工程里已经分配光了，没有富余给输入捕获。但发送用的 DMA1_Stream2
在两帧之间空闲将近 2 ms，所以接收相**复用同一条流**，靠改 DMAMUX 请求号在两相之间切换
（发送 TIM1_UP=15，接收 TIM1_CH1=11 / TIM1_CH2=12）。两相时间上严格不重叠。

代价是一次只能收一路，**两个电调轮流采**：500 Hz 提交时每路 250 Hz 更新。这是 TIMER 后端的设计，
该后端实机失败；现默认 BITBANG 后端两路同时采，实机各约 500 Hz（见下文「换成 bitbang 后端」）。
收尾不用任何中断——回传在 90 us 内结束而下一拍在 2 ms 后，下次提交进临界区时读一次
DMA 剩余计数即可，给 500 Hz 控制路径增加的中断数是 0。

### 分层

协议本身（GCR 表、校验、指数尾数）在 `Driver/Src/drv_dshot_telemetry.c`，纯整数运算、
无寄存器无 RTOS，由 `tests/test_dshot_telemetry.py` 在宿主上判对错。
寄存器翻面与 DMAMUX 切换在 `BSP/Src/bsp_dshot_rx.c`。

### 2026-09-21：这一档之前从来没有被接进输出链

`bsp_esc_protocol.h` 里定义了 `BSP_ESC_PROTOCOL_IS_DSHOT`（"发送路径在两档 DShot 下
完全一致"），但它**被使用的次数是 0**。所有判断点写的都是
`== BSP_ESC_PROTOCOL_DSHOT300`，于是双向档（`== 2`）在每一处都掉进"不是 DShot"的
分支：`BSP_DShot_Init()` 从不被调用，`BSP_PWM_SetEscPulse()` 既不写 CCR 也不发帧，
`BSP_PWM_DisableEsc()` 却又去调一个从没初始化过的 `BSP_DShot_Disable()`。

之所以能一直全绿：`tests/test_dshot_bsp.py`——**唯一**一个用宿主 gcc 编译真实
`bsp_pwm.c` 并跑仲裁链的装置——`params` 只有 `[0, 1]`。编译得过不等于接得上。
现在是 `[0, 1, 2]`，三档 × 5 case 全部真跑。

修法：8 处改用 `BSP_ESC_PROTOCOL_IS_DSHOT`；两处必须三分支不能合并——
`BSP_PWM_EscProtocol()` 返回 `"DSHOT300_BIDIR"`，日志头协议码 1/2/**3**
（旧值含义未变，老日志照旧可读，宿主解码见 `tools/flight_log_receive.py`）。

### 被禁用的通道，在接收相里是高阻而不是强制无效

`BSP_DShotRx_Start()` **不看 `enabled_mask`**：发完一帧就把本轮的 rx_channel 翻成
输入捕获去听回话，刚被安全禁用的那一路也一样。

这不违反"禁用的通道不许出油门"——输入捕获是高阻，根本不驱动任何电平，比强制
无效更彻底。要紧的是它必须**有界**：下一拍提交里的 `BSP_DShotRx_Harvest()` 把通道
交回输出模式。一个翻过去回不来的通道，等于禁用过一次之后这一路永远发不出帧，
而那是一条静默故障。两条都由 `tests/fixtures/dshot_px4/bsp_harness.c` 的 `races()`
在双向档下钉住。

同理，发送完成后 `CR1.CEN` 仍然开着——那是捕获在跑，不是还在发。"发送已经结束"
的判据是 `DIER.UDE` 清零，不是 CEN。

### DShot 特殊命令：`ESC EDT ON|OFF`（2026-09-21）

逐路电流属于 **EDT（扩展遥测）**，而 EDT 不是电调配置器里的开关，是由飞控发一条
DShot 特殊命令打开的（13 = enable，14 = disable）。在此之前本仓库根本不发特殊命令。

分层与各层的判据：

| 层 | 做什么 | 判据 |
|---|---|---|
| `drv_dshot.c` / `drv_dshot_telemetry.c` | `*_EncodeCommand()`，只收 1..47，telemetry 位强制为 1 | `tests/test_dshot_command_encode.py`：命令 13 手算基准 单向 `0x01BA` / 双向 `0x01B5` |
| `bsp_dshot.c` | `BSP_DShot_SubmitCommand()`，与 Submit 共用同一条发送路径 | `tests/test_dshot_bsp.py` 三档 |
| `app_esc_command.c` | 连发 10 帧、抢占中止、失败整条作废 | `tests/test_esc_command_safety.py` |
| `app_cmd_esc_edt.c` | `ESC EDT ON\|OFF`，只认两个词 | 同上 |

**油门入口仍然拒绝 1..47,这条没有被放宽。** 特殊命令走的是另一个入口，不是把
`DRV_DShot_Encode` 的范围打开——1..47 里躺着 7/8/20/21（改电机转向）和 12（写电调
Flash），一条发错的命令会持久化到电调里，重启也恢复不了。同理，界面和命令行**都
给不了任意命令号**：只有 `ON`/`OFF` 两个词，命令号是编译期常量。

**连发 10 帧，中途失败整条作废。** 电调认的是**连续**若干帧；中间夹一帧油门，计数
就从头开始。所以它占住 500 Hz 提交点连续 10 拍（20 ms），而不是由文本任务见缝插针
地调一次 Submit——那样发出去的是"命令、油门、命令、油门…"，电调一条都不认。
补发也不做：补上去的是一条长度不够的新序列，而上层会以为发成功了。

**抢占比点电机更严：** 解锁 / 验收 / 舵机标定 / 辨识 / 点电机 / 台架，任何一个在跑
都立刻中止。后两者正靠连续的油门帧维持电调解锁，插几帧命令会让电机掉出解锁状态。

**命令帧顶掉那一拍的油门帧，但不动暂存的油门**，下一拍照常提交。清零会让电调掉出
解锁状态，表现成"发个 EDT 命令把电机停了"。

**回包说的是"发出去了"，不是"生效了"。** DShot 单向发送，电调不对特殊命令回 ACK，
所以 `sent=10/10` 只证明 10 帧连续命令帧已提交。EDT 到底开没开，**唯一判据是之后
收不收得到 EDT 帧**——看 `PROPCAL escdiag` 的 fr 计数和电流字段。把前者说成后者，
会让人在下一步去查接收侧。

### 2026-09-21 实机：发送侧已被证明正确，问题在电调侧

**症状**：双向档下电机完全不转，`escdiag` 连续数万帧 `fr=0 crc≈0`，只有超时在涨。

**AM32 的自检机制**（`am32-firmware/AM32` `Src/dshot.c` `computeDshotDMA()`）：

```c
if (!armed) { if (dshot_telemetry == 0) {
    if (getInputPinState()) { high_pin_count++;
        if (high_pin_count > 100) { dshot_telemetry = 1; } } } }
```

**不是配置器里的开关**，是未解锁时每收一帧采样一次引脚、读到高才计数、累计过 100
才切双向；计数从不清零。特殊命令 1..47 需 **6 次连续重复**。

**排查过的两条，都已排除：**

1. **"接收相的高阻窗口把自检机会打了对折"** —— 本模块发完帧就把通道翻成输入捕获，
   要到下一拍提交（约 2 ms 后）才翻回；两路轮流，每个电调只有一半的帧看得到被驱动
   的高电平。加了开机自检宽限期（头 500 帧完全不开接收相，线 100% 驻高）后，
   **仍然 `fr=0`**。假设不成立。
2. **"我们的空闲电平根本不是高"** —— 新增 `BSP_DShot_ReadEscPinLevels()` 直接读
   PE9/PE11 的 IDR，在一个完整帧周期上统计：**98~100% 都是高**（低的那约 2% 正是
   帧数据）。**发送侧在引脚上被证明正确**：空闲高、数据拉低，正是双向 DShot 的规定。

**当时的结论"问题在电调侧"是错的。** 电调固件没问题（官方规格
`AM32_F4A_4IN1_F421_2.17`，明确支持双向 DShot）。真正的原因就是 **DMA burst**——
把后端换成 bitbang 之后一次就通，见下一节。

这段排查过程保留下来，因为它记录了**三条被逐项证伪的假设**（自检窗口占空、
空闲电平、电调固件），下次遇到类似症状不必重走。`PROPCAL escdiag` 的
`pin1=/pin2=` 字段也是那一轮留下的：它是**量出来的引脚电平**，不是回读我们自己
刚写的寄存器，所以能把"我们以为设对了"和"线上真的是这样"分开。

### 2026-09-21 解决：换成 bitbang 后端，一次通过

**实机结果**（`build/BIDIR`，`BSP_ESC_BIDIR_BACKEND=BITBANG`）：

```
fr1=41443  crc1=1  to1=4465(不再增长)   esc_i1=0  edt_age1=991
fr2=38119  crc2=1  to2=7789(不再增长)   esc_i2=0  edt_age2=1723
```

两路各约 500 Hz 稳定解码，累计十几万帧只有 1 次校验错（开机那一下），超时计数
停止增长。`ESC EDT ON` 之后 EDT 电流帧也开始到达。

**bitbang 相对 TIMER 后端的差异，逐条都是它能通的可能原因之一，但真正被证明的
只有"换过去就通了"这一条**——没有做单变量拆解，因为作者的要求是停止在自研路子
上试错、改用参考实现普遍采用的做法：

| | TIMER 后端 | BITBANG 后端 |
|---|---|---|
| 发送 | DMA burst -> `TIM1->DMAR` | DMA -> `GPIOE->BSRR`，逐子槽 |
| 接收 | 定时器输入捕获，**两路轮流** | 端口过采样 `GPIOE->IDR`，**两路同时** |
| DMAMUX | 两相之间要切请求号 | 两相都是 TIM1_UP，不用切 |
| 定时器 | 输出比较通道 | 纯时钟源，不碰 CCMR/CCER |
| 接收窗口线电平 | 高阻（无上下拉） | **输入 + 内部上拉**，没人驱动时仍停在空闲高 |
| 占空精度 | 定时器 CCR，精确 | **8 子槽 = 3/8 与 6/8，精确命中 37.5%/75%** |

8 子槽是刻意的：参考实现常用 3 子槽（33%/67% 近似），而 8 子槽在本板上
120 MHz ÷ 2.4 MHz = 50 整除，没有累积相位误差。时钟不整除时 `BSP_DShot_Init`
**拒绝初始化**而不是带着误差发帧——那种失效的现象和"电调不认帧"完全一样，
从外面分不出来。

### EDT 的节奏是协议本身的，不是缺陷

实测 `edt_age1/2` 在 51~2849 ms 之间，即 **EDT 电流帧每 2~3 秒一次**：它是低速插
在转速帧之间的。新鲜度窗口据此定为 4 秒（`PROPCAL_ESC_CURRENT_FRESH_MS`），
并且**把真实年龄一并报出来**——只放宽窗口而不报年龄，就是把几秒前的读数伪装成
实时值。

这条定义了能力边界：**EDT 电流适合台架上"稳住一个工作点数秒"，不适合看瞬态。**
转速是 500 Hz，那才是推力标定的自变量。

### 两个后端并存，默认 BITBANG

`BSP_ESC_BIDIR_BACKEND` 选择实现，协议仍是 `DSHOT300_BIDIR`——这是实现选择，
不该占一个 `BSP_ESC_PROTOCOL` 取值。TIMER 后端保留且仍被
`tests/test_dshot_bsp.py`（显式 `-DBSP_ESC_BIDIR_BACKEND=0`）覆盖：留着是为了能在
同一块板子上 A/B，删掉就再也没法证明"换后端"确实是那个变量。

BITBANG 后端有自己的绑定装置 `tests/test_dshot_bitbang_bsp.py`，钉的是
**方向位、目标寄存器、引脚翻面、空闲电平、时钟整除守卫**——其中"DMA 方向位写反"
和"两相之间方向位残留"两条尤其要紧：它们错了以后的现象就是"电调不回话"，
和这一整节追了一天的症状一模一样，从外面分不出来。

### 上机前必须先做的两件事

1. **确认所用电调固件支持双向 DShot，并匹配反相线路与校验格式。**
   [AM32 公开实现](https://github.com/am32-firmware/AM32/blob/main/Src/dshot.c)在
   `computeDshotDMA()` 中自动识别反相输入。不能把电机正反转的 `bi_direction`
   设置当成 RPM 回传开关；本机电调已于 2026-09-21 在 BITBANG 后端下实机回传 eRPM（见下方「已验证 / 未验证」），
   换电调或改刷电调固件后须重新核实。EDT 扩展遥测另有启用命令，
   本工具只接收和分类，不自动修改电调配置或持久化设置。
2. **拆桨台架验证**。宿主测试证明的是协议编解码，不是线上时序、方向切换时机和电平。2026-09-21 已在 BITBANG 后端实机验证线上回包（两路各约 500 Hz 解码，见下表）；2026-09-22 起推力台带桨会话也持续记录两路 eRPM。

### 已验证 / 未验证

| 项 | 状态 |
|---|---|
| GCR 编解码、校验、指数尾数、单比特错误拒绝 | 宿主 25 条用例通过 |
| GCR 字母表游程受限（全 65536 字组合最长同电平 3 位） | 宿主穷举通过 |
| 三档构建（DSHOT300 / DSHOT300_BIDIR / PWM）零警告 | 通过 |
| 三档在宿主 seam 上真跑仲裁链（`test_dshot_bsp.py` params=[0,1,2]） | 通过（2026-09-21 起；之前只有 [0,1]） |
| 双向档接入输出链（Init / Submit / Disable / 诊断 / 日志协议码） | 通过，且实机读到 `avail=1 proto=2` |
| nibble→五位组的**对应关系** | **已钉死**（2026-09-21）：bitbang 后端实机连续解出十几万帧，累计 crc 错 1 次 |
| 线上时序、方向切换、电调实际回包（**BITBANG 后端**） | **实机通过**：两路各约 500 Hz、crc=1、超时停止增长；`ESC EDT ON` 后 EDT 电流帧到达 |
| 线上时序、电调实际回包（**TIMER 后端**） | **实机失败**：电机不转、零回传。保留该后端仅为可对照，不作为可用配置 |
| 逐路 EDT 电流是否真的逐路 | **未验证**：本板电调只有一颗合金采样电阻，两路很可能报同一个总电流。电机停转时都是 0 A，分辨不出，须给油门后对比 |

R-THRUST-2 已把两路 eRPM、各自来源时刻和独立 EDT 电流分类接入 `TBENCH?` 快照，
供[推力台程序](thrust-bench.md)使用。eRPM 缺失时拒绝定量建模，不回退到 KV 估计。
2026-09-22/23 推力台实测会话里两路 DShot 电流字段始终为空（原始快照与 `samples.csv` 均无值），
逐路电流采样能力与台架运行时的 EDT 状态仍未核实，电流/功耗模型暂不可用；有协议字段不代表这块硬件已测得。

## AM32 电机 KV 写入（2026-09-23）

AM32 的低转速功率限制按电调内存储的 KV 计算最大占空比；出厂 KV=2220，配 AEO CRM2413-KV1300 时推力在约 50% 油门封顶（推力台实录）。微空 55A 手册（固件 2.19）要求低 KV 电机修改 KV。飞控提供 `ESC KV <kv> CONFIRM`：经 DShot 编程模式（AM32 ≥2.18）发送固定 15 帧 36×6、26、(KV−20)/40、37、12×6，只写 `eepromBuffer.motor_kv`。电调须通电、电机静止；写入后须电调重新上电才生效。回包 `sent=15/15` 只证明帧已连续发出，是否生效以最大推力测试的转速/推力为准。推力台窗口按钮“电调KV设为1300”调用此命令。细节见 `data/analysis/thrust-bench/2026-09-23-esc-kv/review.md`。

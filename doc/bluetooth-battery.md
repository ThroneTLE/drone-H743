# 蓝牙通道与电池电压（R-BT-1 / R-BATT-1）

## 蓝牙

通道下拉新增“蓝牙”，底层复用原SerialTransport的ASCII命令、二进制接收、取消和连接代次。
后台枚举Windows已配对经典SPP设备，把设备名与出站COM按蓝牙地址关联；入站监听COM（地址全零）不作为连接目标。
优先选择上次成功连接的地址，否则自动选唯一的MicoAir743v2。多个匹配设备由用户选择；不可用设备不回退到别的USB/蓝牙口。
COM号和本机某一块板的地址都不写死。用户点“启动连接”后再次核对当前枚举，异步打开串口；停止、切换通道、关闭窗口会使旧扫描/打开结果失效。
蓝牙固定使用115200，USB的串口、指纹和波特率记录分别保留。未配对时提示在Windows完成配对；不扫描并连接任意附近设备。
Windows配对记录可在蓝牙关闭时存在，只有当前串口枚举中存在的口才允许连接。蓝牙身份始终不满足USB刷写/校准的专用身份门。

Contract：自动选择不等于已连接；连接不等于飞控已应用命令。Boundary：设备枚举/匹配、Tk控件、串口会话各自独立。Test seam：真实DronePanel、模拟PnP/Serial，覆盖换号、歧义、取消、延迟打开、旧结果与三尺寸三缩放。

## 电压采样与保护

作者确认3S，并明确要求告警和禁止低压解锁，飞行中不自动切断电机。
PC1=ADC1 CH11/rank1电流；PC0=CH10/rank2电池电压。作者已Generate Code；详见[battery-cubemx-checklist.md](battery-cubemx-checklist.md)。
两通道使用discontinuous group=1；每次Start/Poll/Get只读取一个rank，读完两路后Stop。任一路转换失败则丢弃整对。
**完整一对读完即序列结束，下一对从rank1开始是定义使然，不依赖任何假设**；中途失败时sequencer停在rank2，此时不赌"Stop会把discontinuous子组指针倒回rank1"（本仓库拿不出依据），
而是在下一次取数前重跑一遍`adc_prepare()`——HAL的`HAL_ADCEx_Calibration_Start()`第一步就是`ADC_Disable()`，等于把ADC关掉再打开，状态机整体复位。正常路径不付这个代价。
复位失败或Stop失败则锁存ADC不可用，显式初始化前不再转换，并报ERROR（区别于"尚未初始化"的NOT_READY）。
采样对以开始时刻标时，避免任务抢占后把旧样本标成新鲜；所有ADC读取归现有messageTask，约20ms周期，不受慢存储和PC命令阻塞影响。
电压样本复用电流快照的`sample_ms`（同一对、同一拍），因此它取的是**该对电流转换也成功时**的开始时刻；电流侧被判无效时时间戳不前移，电压只会显得更旧，
即只朝"过期→禁止新解锁"的安全方向偏，不会把旧电压当成新鲜。

总电压标称值：`mV = round(raw * 69696 / 65535)`，即3.3V参考×21.12分压比；比例来自仓库存档的ArduPilot MicoAir743v2 hwdef。未实测校准。
平均单节值为总压除以配置串数，未单独测量每一节。当前默认3S、告警/新解锁门3500mV每节、恢复3600mV每节（整包10.5/10.8V）。
3500mV告警参考[Betaflight默认参数](https://betaflight.com/docs/wiki/guides/current/Cli)，是可调整工程默认值，不能替代具体电池验证。
有效性要求有采样、ADC成功、未饱和、年龄≤250ms。缺失、过期、低压均禁止新的解锁；拒绝一次后电压恢复不会自动解锁，仍需拨杆低→高。
低压不清除已解锁锁存，原RC丢失、IMU、坐标迁移、机体模型和油门门仍照原逻辑执行。上位机解锁横幅和电源页显示告警。
状态灯新增8闪低压/电压失效提示，但**只在未解锁时显示**：已解锁后常亮红灯压过一切拒绝原因（与原有其余7个原因一致）。
3S带载掉到10.5V以下是常态，若让大半时间熄灭的8闪盖掉红灯，桨还在转时旁人会把飞控当成已上锁；飞行中的低压提示走横幅、电源页和ELRS回传。
告警使用独立8闪图案，原LED颜色绑定的枚举和Flash布局保持不变。

配置只在未解锁且服务初始化后允许修改，RAM中生效，重启恢复默认；不插入既有CFG/FCAL Flash布局。
`BATTERY SET <nonce> <cells> <low_cell_mv> <recover_cell_mv>` 原子校验后生效：cells=1..12，low=2500..4100，recover>low且≤4400。
界面只用当前连接的新鲜、已确认未解锁回包开放设置；草稿、待确认和固件回读分开。没有明确ACK不能宣称已应用。

## 电池诊断协议v1

上行沿用ASCII：`BATTERY? [nonce]`。下行新增`0x2232`，既有$X帧/CRC8，payload固定60B、小端；不改变旧function ID或遥测schema。

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | u8 | version=1 |
| 1 | u8 | bit0电压有效、1低压、2电池项允许新解锁、3饱和、4电流有效、5配置明确ACK、6配置拒绝；bit7保留 |
| 2 / 3 | u8 / u8 | 配置串数 / ADC状态（0成功1未就绪2超时3错误） |
| 4 / 8 | u32 / u32 | nonce / 电压样本年龄ms（无样本=0xFFFFFFFF） |
| 12 / 16 | u32 / u32 | 整包电压mV / ADC原始值 |
| 20 / 24 | u32 / u32 | 电压采样数 / 错误数 |
| 28 / 32 | u32 / u32 | 每节告警mV / 每节恢复mV |
| 36 / 40 | i32 / u32 | 电流mA（无效=INT32_MIN） / 电流样本年龄ms |
| 44 / 48 | u32 / u32 | ELRS被DMA接纳帧数 / 拒绝帧数，不是遥控器确认 |
| 52 / 56 | u32 / u32 | RAM配置代次 / 飞控解锁状态（0未知1未解锁2已解锁） |

R-PWR-1 之前「电池电压」页可见时每2秒查询一次（单帧69B，34.5B/s）。该轮询已删除：
页面并入「传感器 → 电源」（`tools/panel_lib/pages/power.py`），实时电压/电流改走遥测流
通道 `batt_v` / `batt_i` 推送，`BATTERY? <nonce>` 只在打开页面时取一次、之后由「刷新诊断」
按钮触发，属于按需突发而不是周期负载。详见 [current-sensor.md](current-sensor.md) 的 R-PWR-1 一节。
配置帧同样属于按需突发。所有写入来自用户操作，不重放旧配置命令。连接变化、nonce不符、长度/版本/标志矛盾不接受；超时/拒绝可见。
总览注册表新增电池电压项，名称/状态/读数仍由固件提供。

## ELRS电池回传

messageTask每500ms尝试一次，使用最新缓存与USART6 RC口的非阻塞DMA；每帧12B，24B/s。DMA忙则丢弃本次，不排队旧读数。
实际`APP_ELRS_SendTelemetryBattery()`已接入周期路径；DMA接纳/拒绝计数可在「传感器 → 电源」页的诊断区读取。电压按0.1V、电流按0.1A编码，12V对应整数120。
无效/过期值发送全FF；没有容量/剩余电量模型，因此capacity与remaining也用全FF，不伪造0mAh或百分比。
单位与全FF处理以[EdgeTX实际接收器](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.cpp)为兼容基准（电压/电流precision=1，全FF字段不刷新）。
全FF语义的具体依据：`getCrossfireTelemetryValue()`逐字节取值时，只有当**至少一个字节不等于0xFF**才把返回值置true；调用方据此跳过`processCrossfireTelemetryValue()`，
所以整字段全FF既不会注册新传感器也不会刷新旧值（界面上表现为`---`或失去更新星号），不会被当成65535×0.1=6553.5V。这条是遥控器固件行为，不是本仓库能自证的，故在此记明出处。
[TBS现行文档](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)的0x08单位注释写成10µV/10µA，与本工程既有API及EdgeTX不一致；本实现不采用该注释，黄金帧锁定0.1单位。
全FF不冒充真实零值；遥控器对旧值的具体超时显示取决于其软件。实机需确认遥控器发现并刷新RxBt/电流传感器，不能把本机DMA计数当成无线链路验收。

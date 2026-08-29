# drone-H743 PC Tools

PC 侧调试与分析脚本。脚本放在本目录顶层，采集到的数据和分析产物统一放在
仓库根目录的 `data/` 下。运行数据自动进入 `YYYY-MM-DD` 子目录，读取工具会递归查找各日期。所有默认路径由 `tools/project_paths.py` 集中定义：

| 目录 | 内容 |
| --- | --- |
| `data/captures/imu_vibration/` | 满速率原始 IMU 采集与频谱报告（`imu_vibration_capture.py`） |
| `data/captures/imu_attitude/` | Fusion 姿态录制与诊断报告（`imu_attitude_tuner.py`） |
| `data/captures/saleae_spi/` | Saleae SPI 抓包 |
| `data/identification/motor/` | 电机 Hammerstein 辨识原始数据与拟合产物 |
| `data/identification/thrust/` | 推力标定数据与拟合曲线 |
| `data/identification/attitude/` | 姿态激励辨识批次 |
| `data/calibration/pressure/` | RS485 压力传感器标定与手册 |
| `data/calibration/airframe/` | V0 坐标发现、RAM 复验、Flash 提交会话与审计报告 |
| `data/calibration/imu_metrology/` | V1 IMUCAP 原始证据、可恢复会话、候选与应用审计 |
| `data/calibration/flight_acceptance_v2/` | V2A 控制链路验收样本、人工机械确认与报告 |
| `data/firmware_updates/` | USB ROM DFU 烧录、校验、复位的逐次日志 |
| `data/flight_logs/` | USART1 导出的原始/转换飞行日志（flight_log_receive.py，bin+csv+json） |
| `data/analysis/` | 系统辨识报告和 Rerun 等派生产物 |

## 飞行器传感器验收 V0

启动现有地面站并打开“校准 / 验收 V0”页：

```powershell
python tools\drone_tcp_panel.py
```

V0 通过固件 `IMU?` 的 `stabilizer_snapshot` 完成 6 个必要动作：水平、机头朝上、左侧朝上、`+roll/+pitch/+yaw`。三个正交静态姿态确定 proper signed-permutation，三个正向旋转验证陀螺积分与 Fusion 方向。开始前必须拆桨，并断开电调动力或可靠固定机体。旧固件没有 snapshot provenance 时页面显示 `UNSUPPORTED`，不会用全零值生成 PASS。

推荐连接方式是飞控 USB CDC：插入飞控 USB 后，在页面顶部选择 `serial`，再选择 Windows 新出现的 USB 虚拟 COM（不要误选 ST-Link VCP）。虚拟 COM 的波特率字段只是兼容参数，实际传输走 USB FS，不受115200 UART吞吐限制。V0轮询和 `IMUCAP DUMP` 共用同一CDC字节流，开始二进制导出前应结束V0会话；固件在导出期间也会暂停普通结构化文本的USB镜像。

采集期间仍禁止普通 SAVE、LOAD、参数/PID写入、舵机、辨识和电机动作。唯一例外是页面内带双重人工确认的 `IMUFRAME` 流程：`APPLY` 只改 RAM，`REVERT` 恢复已持久化值，6 步复验全部 PASS 后 `COMMIT` 才通过 Param 双槽后台写 Flash。诊断命令框不能绕过该授权门，固件还会独立检查 `armed=0` 和安全电机输出。

页面会自动保存每个完成步骤，并在下次打开时恢复最新会话；旧版报告的原始 CSV 也会自动迁移为会话。发现候选后按 `A 临时应用到 RAM` → `B 清空旧样本并开始6步复验` → `C 复验通过后写入 Flash`。应用前软件要求飞机水平静止；复验还要求加速度倾角与 Fusion 姿态中位误差不超过 8°。每次 RAM/Flash 操作另写审计 JSON，且当前固定 `flight_release=false`：坐标映射写入成功不等于整机已允许自由飞行。

发现阶段表格里的 `FAIL` 表示 legacy 轴还没有直接符合 FLU，正是用来推导 signed-permutation 候选的证据，不是最终验收结论。点击 A 后旧记录不会自动改成 PASS；必须点击 B 清空发现阶段样本，并在 RAM 映射生效的 canonical FLU 链路下重新完成 6 个必要步骤。只有这批新样本全部 PASS，C 才会启用。

V0、V1、V2A 和固件升级页面使用纵向滚动容器；鼠标滚轮或右侧滚动条都能访问底部内容。程序会根据屏幕高度适度压低 Windows Tk DPI scaling：1080p 使用更紧凑布局，1440p 约缩小 10%，不会修改系统缩放设置。

V0 页面会扫描所有 `*_session.json` / `*_report.json` 并显示“历史验收”下拉框；带原始样本的 session 优先作为可恢复项，只有报告时则从配套 `samples.csv` 重建会话。“恢复/继续当前历史”使用绿色按钮。新建操作明确命名为“新建独立验收”，切换前会提示：它只清空当前页面工作区并建立新的唯一文件，不覆盖或删除历史报告。

首次使用 `IMUFRAME` 仍需烧入一次支持该协议的新固件；此后更换/校正安装映射只写参数 Flash，无需重新编译程序。V0 只解决离散轴排列与极性，不估计加速度比例/非正交、温漂或整机执行器方向。

## IMU 传感器计量 V1

地面站“IMU 标定 V1”页使用 USB CDC 的 `IMUCAP v4` 原始数据建立可恢复会话。
开始前仍必须通过 V0 的拆桨、动力隔离、新鲜 snapshot、active COM、连接 generation
和 USB 身份门。采集时面板会主动关闭自己对该 COM 的占用，由后台 `CaptureLink`
依次执行 `START → 等待/人工完成 → STOP → DUMP`，保存后再连接面板；Tk 主线程不等待串口。

标准初始化步骤：六个加速度方向各 2000 样本、陀螺静止 4500 样本、手动
`+360° X/Y/Z` 各最多 6000 样本，以及带 `room/cold/warm` 等明确标签的温度平台。
原始 CSV、v4 header metadata、`session.json` 和候选位于
`data/calibration/imu_metrology/YYYY-MM-DD/<session>/`，软件重启会恢复最新 manifest。

V4 ABI 为 48-byte sample 与 56-byte header；V3 的 46/40-byte 数据仅能离线读取，
不能进入 V1。V1 会拒绝 `INVALID_PROVENANCE`，要求 contract=1、base_frame=1、
orientation 0..23、非零 calibration generation 和 firmware CRC。filtered legacy 向量
按固件同一 0..23 表转换为 FLU，温度使用 `raw/132.48+25°C`。

分析结果分别显示 accel、gyro static、gyro +360 和 temperature。六面加速度与静态
陀螺通过后，可生成“室温基础候选”：只应用加速度 bias/3x3 矩阵与陀螺残余零偏，
陀螺矩阵保持单位阵、温漂位保持无效。手扶 `+360°` 只能作为方向证据，不能冒充精密
转台；精密转台与 cold/room/warm 多温点均 PASS 后才生成完整 V1 候选。

步骤 5 会重新加载候选并重放全部原始 CSV，严格一致后通过 `IMUCAL BEGIN/DATA/END`
上传 128-byte 候选，`APPLY` 只发布 RAM preview 并保持电机解锁锁定。二次确认姿态、
静止输出和方向后，步骤 6 才发送 `COMMIT`；目标必须观测到新 generation 与新样本，
再通过 Param 双槽写入外部参数 Flash，并以 `dirty=0` 确认。`REVERT` 随时恢复 Flash
确认值。每次操作在当前 V1 会话目录写独立 `imucal_*.json` 审计记录。

## 控制链路安全验收 V2A

地面站“链路验收 V2A”页要求 V0 已持久化、V1 室温基础位有效、Param `dirty=0`、
实时快照未解锁且人工确认拆桨。`ACCEPT V2 START props=1` 成功后目标端直接把两个
ESC CCR 清零，并发放 500 ms 租约；上位机每 250 ms 续租，USB/软件断开后目标自动
退出并继续保持 ESC 禁用。

V2A 包含 RC 中位与三轴正向、导航静止/前/左、roll/pitch 恢复方向、四个舵机
`中心±50 us` 和 failsafe 步骤。舵机步骤仍需操作者按机体标记目视确认机械方向。
带动力的电机旋向和偏航反扭矩不在 V2A 中执行。当前页提供安全模式、步骤切换和目标
快照；离线严格判定由 `flight_acceptance_v2.py` 提供。V2A 通过仍固定
`flight_release=false`，不会自动解锁自由飞行。页面把收到的全部 `ACCEPT` 原始行和
安全声明保存到 `data/calibration/flight_acceptance_v2/YYYY-MM-DD/<session>/`；机械
方向仍必须由操作者明确确认后才能进入严格报告，原始日志本身不等于 PASS。

## USB ROM DFU 一键固件升级

地面站的“固件升级”页使用 STM32H743 芯片内置 ROM DFU，不维护第二套自定义
Bootloader。默认固件为 `build/Debug/drone-H743.elf`，V0 只接受 ELF/HEX；烧录工具
会从 PATH、STM32CubeProgrammer 和常见 CubeCLT 安装目录查找
`STM32_Programmer_CLI.exe`。

使用时在顶部选择飞控 USB CDC 对应的 `serial` 端口并连接。页面自动轮询实时
`stabilizer_snapshot`，只有 `armed=0` 且 `m1/m2<=1100` 时才允许点击。确认后主机
只授权并发送一次精确命令 `BOOT DFU CONFIRM`；收到
`BOOT mode=dfu state=scheduled` 后，后台等待 CDC 断开以及 Windows 枚举
`STM32 BOOTLOADER`，然后以 argv（不经过 shell）执行：

```text
STM32_Programmer_CLI -c port=USB1 -w <firmware.elf|firmware.hex> -v -s 0x08000000
```

USB DFU 不能使用 JTAG/SWD 专用的 `-rst`。写入校验后使用 `-s 0x08000000`
让 ROM 启动应用；若特定 CubeProgrammer/驱动组合仍未自动离开 DFU，页面会把它明确
标为“写入校验成功、启动失败”，此时按复位键或 USB 断电重插即可，不需要重复擦写。
正常启动后，页面会等待最多 15 秒，并按烧录前保存的 VID/PID、USB 序列号和物理位置
匹配同一块飞控；匹配成功后自动刷新 COM、选中端口、建立 serial 连接，并发送
`PING`/`IMU?` 恢复状态。不会把列表中的第一个 CH340、ST-Link 或另一块 STM32 当成目标。

连接后软件锁定 COM 下拉框，并把安全快照绑定到该连接的 generation 和 active port。
ST-Link VCP、CH340、CP210x、FTDI 与蓝牙串口会明确拒绝；默认只信任本工程
`0483:5740` application CDC，无法识别的 USB 身份必须在升级页人工勾选确认并写入日志。
ELF 会检查 ELF32 little-endian、EM_ARM、entry 和所有 PT_LOAD 的 p_paddr；HEX 会检查
record checksum、扩展地址和全部数据地址。任何待写数据超出
`0x08000000..0x081FFFFF` 都会在发送 BOOT 前拒绝，页面同时显示镜像 SHA-256、大小、
入口和写入范围。

CubeProgrammer 输出和进度会实时显示，完整日志保存到
`data/firmware_updates/YYYY-MM-DD/`。发送 BOOT 前还会先确认系统里没有既存 DFU，
避免把其它设备的 `USB1` 当作本机。CDC 断开/DFU 枚举等待阶段可以取消；真正进入
program/verify 后，取消按钮和关闭窗口会锁定，避免用户一键留下半刷固件。超时或失败
不会自动重试擦写。若应用没有成功跳入 DFU，使用板上的 BOOT0 救援入口进入 ROM
DFU；USB 仍无法恢复时再使用 ST-Link。首次使用一键升级仍需先烧入支持
`BOOT DFU CONFIRM` 的应用固件。

## 振动频谱采集（IMU 原始满速率）

飞行日志（约 250Hz）和 VOFA（40Hz）都是抽取且已滤波的数据，看不到 150~300Hz
的桨叶振动带，因此无法用来确定 AAF 和软件 IIR 的截止频率。`IMUCAP` 采集的是
未抽取、未滤波的原始样本，固件侧见 `App/Src/app_imu_capture.c`。

图形界面（推荐）——下拉选 COM 口、选测试档位，点按钮即可采集并出报告：

```powershell
python tools\imu_vibration_ui.py
```

档位预设为 静止 / 定速 30·50·70·90% / 悬停，会自动按档位给文件打 tag，方便
后续横向对比。选中会让电机转动的档位时，界面会先弹出机架固定确认。串口读写
在后台线程执行，导出过程中界面不会卡死。

命令行等价用法：

```powershell
# 采集约 6 秒并立即分析（--tag 用于标注油门档位）
python tools\imu_vibration_capture.py --port COM7 --capture --analyse --tag thr50

# 只分析已有采集，不连接硬件
python tools\imu_vibration_capture.py --analyse-file data\captures\imu_vibration\YYYY-MM-DD\xxx.csv
```

固件命令：`IMUCAP?` 查状态，`IMUCAP START [samples]` 开始，`IMUCAP STOP` 停止，
`IMUCAP DUMP` 通过 USB CDC 导出，`IMUCAP CANCEL` 取消导出。

采集时的建议档位：静止（噪声底）→ 机架固定后电机定速扫描 30/50/70/90%
（桨叶基频随转速移动，用于区分真实振动和混叠假峰）→ 悬停。定速扫描时机架必须
固定牢靠。

分析报告会给出各轴主频、削顶比例、IIR 实测衰减、Fusion 门限健康度，并在主频
超过 Nyquist 的 80%（可能是混叠）或加速度削顶时明确告警。

## IMU Fusion 录制与分析

飞行时不需要连接 OpenOCD。使用正常 VOFA 串口流录制 28 通道，数据和元数据
直接保存在仓库的 `data/captures/imu_attitude/`：

```powershell
python tools\vofa_serial_capture.py --port COM10 --duration 30 --prefix imu_fusion
```

新增加的通道 23..27 分别记录加速度方向误差、是否拒绝、恢复触发进度、
累计修正次数和加速度模长拒绝。录制后生成 Fusion 分析报告：

```powershell
python tools\imu_attitude_tuner.py analyze `
  data\captures\imu_attitude\YYYY-MM-DD\imu_fusion_YYYYMMDD_HHMMSS.csv
```

`imu_attitude_tuner.py record/run` 仍可在静态台架上通过 OpenOCD telnet `4444`
只读录制，不执行 halt、reset、写内存或刷写，但它不是飞行采集前提。分析器
不会根据单个样本自动改飞行参数；旧 `dwell/tau` 报告属于已移除算法，不能
用于当前 x-io Fusion。新报告只依据真实拒绝、恢复和最终落地残差决定是否需要
进一步审查。

## Direct SoftAP UDP mode

Current firmware starts the Ai-WB2-12F as a SoftAP by default:

- SSID: `DroneH743`
- Password: `12345678`
- Module IP/gateway: `192.168.43.1`
- DHCP clients: `192.168.43.100` to `192.168.43.200`
- UDP server port: `7777`

Connect the PC to `DroneH743`, then send one UDP packet to the module so the
UDP server learns your PC as the peer:

```bash
python3 tools/aiwb2_net_tool.py udp-send --module-ip 192.168.43.1 --module-port 7777 --message PING
```

After that, VOFA or the Python UDP tools can receive telemetry from the module.
If you need to change the AP without rebuilding, send through the maintenance
UART:

```text
WIFI AP MyDrone 12345678 7777 6
```

Print the PC IP used to reach the module:

```bash
python3 tools/aiwb2_net_tool.py ip --target 192.168.223.181
```

Listen for UDP data from the module:

```bash
python3 tools/aiwb2_net_tool.py udp-server --port 6666
```

Listen for UDP data and type replies back to the module:

```bash
python3 tools/aiwb2_net_tool.py udp-reply-console --port 6666
```

For the current Ai-WB2 auto transparent UDP-client mode, the PC must reply to
the module's source port. This command remembers the last source address and
uses it for replies.

Send one UDP packet to the module:

```bash
python3 tools/aiwb2_net_tool.py udp-send --module-ip 192.168.43.1 --module-port 7777 --message PING1234
```

Run a two-way UDP console:

```bash
python3 tools/aiwb2_net_tool.py udp-bridge --local-port 6666 --module-ip 192.168.43.1 --module-port 7777
```

Run a TCP server for the module to connect:

```bash
python3 tools/aiwb2_net_tool.py tcp-server --port 6666
```

## drone-H743 TCP panel

After the Ai-WB2 is configured as TCP client transparent mode to the PC, run
the GUI panel:

```bash
python3 tools/drone_tcp_panel.py
```

Default panel behavior:

- listens as a TCP server on `0.0.0.0:6666`
- shows a minimal ground-station layout with module overview/detail pages for
  GD25Q32 Flash, SPL06 barometer, ICM42688 IMU, and USART1 link diagnostics
- shows a dedicated SPL06 page with realtime/diagnostic fields, temporary data
  capture, CSV export, and optional plotting
- sends line commands such as `STATUS?`, `CONFIG?`, `PARAM?`, `PID?`,
  `BARO?`, `SERVO MOVE 0 1500 500`
- controls two Zhongling bus servos, mapped by default to IDs `1` and `2`
- can ask the board to save/load servo config in the onboard GD25Q32 flash
- keeps unknown board lines in the raw command log so firmware commands can be
  added incrementally without breaking the UI

SPL06 plotting uses `matplotlib` when it is installed. The panel still starts
without it and shows an install hint in the plot area:

```bash
python3 -m pip install matplotlib
```

The parameter/PID page is intentionally line-based first. It currently sends
`CONFIG?`, `PARAM?`, `PID?`, `PARAM SET <name> <value>`,
`PID SET <axis> kp=<v> ki=<v> kd=<v>`, `SAVE`, and `LOAD`; firmware that does
not yet implement every command should reply with its normal `ERR` line, which
the panel logs without crashing.

## drone-H743 flash diagnostics

When the board UART is flooded by periodic IMU sample lines, use the filtered
diagnostic runner instead of pasting commands into a serial terminal:

```bash
python3 tools/flash_diag_test.py --list-ports
python3 tools/flash_diag_test.py --serial COM18 --baud 115200 --final-rtos
```

The default test sends `RTOS?`, `FLASH?`, and the standard `FLASH VERIFY`
ranges one at a time. It prints only matching RTOS/FLASH responses, filters
unrelated IMU stream lines, and returns a non-zero exit code if any required
check fails.

Optional read throughput test:

```bash
python3 tools/flash_diag_test.py --serial COM18 --baud 115200 --bench --final-rtos
```

## 飞行日志接收（flight_log_receive.py）

飞行日志通过 USART1 从飞控导出，flight_log_receive.py 负责接收并转成 .bin（原始）、.csv（解析后）和 _meta.json（元数据）三件套，默认保存到 `data/flight_logs/YYYY-MM-DD/`。

```powershell
# 图形界面：选串口、选保存目录
python tools\flight_log_receive.py
# 命令行：免界面直接导出到默认目录
python tools\flight_log_receive.py --port COM7
# 指定串口、波特率和保存目录
python tools\flight_log_receive.py --port COM7 --baud 57600 --out-dir data\flight_logs\YYYY-MM-DD
```

```

固件端命令：FLOG DUMP 开始导出，FLOG CANCEL 取消。

## 飞行日志系统辨识

使用 `flight_log_sysid.py` 可以把 `flightlog_*.csv` 和匹配的
`flightlog_*_meta.json` 转成可复用的系统辨识摘要：

```bash
python3 tools/flight_log_sysid.py data/flight_logs/2026-07-24/flightlog_20260724_200832.csv
```

也可以被其他脚本直接导入：

```python
from tools.flight_log_sysid import analyze_flight_log

analysis = analyze_flight_log("data/flight_logs/2026-07-24/flightlog_20260724_200832.csv")
print(analysis.gain_groups)
```

工具会自动切分不连续日志，按保存下来的控制参数快照分组，报告倾角/电机饱和、
记录丢包等风险，并拟合主要的“控制命令 -> 执行器输出”映射。

如果需要中文表格和图表界面：

```bash
python3 tools/flight_log_sysid_ui.py data/flight_logs/2026-07-24/flightlog_20260724_200832.csv
```

如果想用一个窗口完成“选文件、选片段、看波形、弹出 Rerun 回放”，使用 All-in-One 工作台：

```powershell
python tools\flight_log_workbench.py data\flight_logs
```

工作台左侧选择日志文件和文件内片段，右侧选择通道并查看当前片段波形；
点击“弹出Rerun回放”会通过 `.tmp/rerun_env` 隔离环境打开该片段的三维现场回放。

如果只是想自己选文件、选通道、快速看波形，也可以单独用离线波形查看器：

```bash
python3 tools/flight_log_waveform_ui.py data/flight_logs
```

也可以直接打开某个 CSV：

```bash
python3 tools/flight_log_waveform_ui.py data/flight_logs/2026-07-27/flightlog_20260727_045138.csv
```

界面左侧选择日志文件，中间搜索/选择通道，右侧点击“画选中”即可查看曲线；
支持多通道同图、分图、归一化、按时间断点断开，并显示选中通道的统计值。
图表区可以选择滚轮模式：总缩放、只缩横轴、只缩纵轴；滚轮会以鼠标所在位置为中心缩放。

如果想用 Rerun 做三维现场回放和时间轴播放，推荐用隔离虚拟环境启动脚本；
它会把 `rerun-sdk` 装到 `.tmp/rerun_env`，避免 Rerun 依赖升级影响主 Python 分析环境。

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 data\flight_logs
```

也可以直接列出最新日志的分片：

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 data\flight_logs --list
```

播放某一个分片：

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 data\flight_logs --play --segment 13
```

如果你已经在当前 Python 环境里安装好了 `rerun-sdk`，也可以直接运行 Python 脚本：

```bash
python3 tools/flight_log_rerun_replay.py data/flight_logs --list
```

工具会按时间断点把日志切成多个片段，导出 `data/analysis/flight_logs/rerun_replay/YYYY-MM-DD/*.rrd`；
Rerun 中可以播放三维机体姿态、粗略轨迹/高度、推力方向估计，以及姿态、舵机、
电机、光流、控制量等时间曲线。

Optional serial auto-configuration through CH340 requires `pyserial`:

```bash
python3 -m pip install pyserial
python3 tools/aiwb2_net_tool.py configure-at \
  --serial-port /dev/cu.wchusbserialXXXX \
  --ssid YOUR_WIFI_SSID \
  --password YOUR_WIFI_PASSWORD \
  --local-port 7777
```

For local convenience, `configure-at` and the loop test also read
`AIWB2_WIFI_SSID`, `AIWB2_WIFI_PASSWORD`, and `AIWB2_TCP_PORT` from the
environment. Do not commit real WiFi credentials.

## VOFA UDP direct mode

The Ai-WB2 is configured as an auto-transparent UDP server on module port
`7777`, so VOFA can talk to it directly without the Python bridge.

Suggested VOFA settings:

- `数据接口`: `UDP`
- `远程IP`: `192.168.43.1` in default SoftAP mode, or the IP shown by `WIFI?`
- `远程端口`: `7777`
- `本地端口`: any free local UDP port, for example `6668`

## 其他工具

| 脚本 | 说明 |
| --- | --- |
| `imu_filter_report.py` | IMU 滤波器效果报告 |
| `aiwb2_tcp_loop_test.py` | Ai-WB2 TCP 回环测试 |
| `tcp_bidirectional_test.py` | TCP 双向转发测试 |
| `vofa_udp_bridge.py` | VOFA UDP 桥接 |
| `pressure_rs485_gui.py` | RS485 压力/称重传感器 GUI（含推力标定） |
| `pressure_rs485_test.py` | RS485 压力传感器命令行自检 |
| `thrust_ident_auto_viewer.py` | 推力标定结果自动查看 |
| `attitude_ident_pid.py` | 姿态激励辨识 PID 分析 |
| `fit_motor_hammerstein.py` | 电机 Hammerstein 拟合 |
| `flow_velocity_filter_eval.py` | 光流速度滤波评估 |
| `decode_saleae_spi_csv.py` | Saleae SPI CSV 解码 |
| `saleae_imu_spi_capture.py` | Saleae 抓取 ICM42688 SPI 总线 |
| `servo_baud_sweep.py` | 舵机总线波特率扫描 |
| `synex_config_builder.py` | Synex 配置生成 |
| `ground_station/` | [drone-H743 专属上位机 / 地面站 (Serial-Studio 二次开发)](ground_station/README.md) |

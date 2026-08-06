# drone-H743 PC Tools

PC 侧调试与分析脚本。脚本放在本目录顶层，采集到的数据和分析产物统一放在
`tools/data/` 下，不要再散落在顶层：

| 目录 | 内容 |
| --- | --- |
| `data/imu_vibration/` | 满速率原始 IMU 采集与频谱报告（`imu_vibration_capture.py`） |
| `data/imu_attitude_data/` | Fusion 姿态录制与诊断报告（`imu_attitude_tuner.py`） |
| `data/motor_ident/` | 电机 Hammerstein 辨识原始数据与拟合产物 |
| `data/thrust_ident/` | 推力标定数据与拟合曲线 |
| `data/ident_runs/` | 姿态激励辨识批次 |
| `data/pressure/` | RS485 压力传感器标定与手册 |
| `data/spi_captures/` | Saleae SPI 抓包 |

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
python tools\imu_vibration_capture.py --analyse-file tools\data\imu_vibration\xxx.csv
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
直接保存在仓库的 `tools/data/imu_attitude_data/`：

```powershell
python tools\vofa_serial_capture.py --port COM10 --duration 30 `
  --out-dir tools\data\imu_attitude_data --prefix imu_fusion
```

新增加的通道 23..27 分别记录加速度方向误差、是否拒绝、恢复触发进度、
累计修正次数和加速度模长拒绝。录制后生成 Fusion 分析报告：

```powershell
python tools\imu_attitude_tuner.py analyze `
  tools\data\imu_attitude_data\imu_fusion_YYYYMMDD_HHMMSS.csv
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

## 飞行日志系统辨识

使用 `flight_log_sysid.py` 可以把 `flightlog_*.csv` 和匹配的
`flightlog_*_meta.json` 转成可复用的系统辨识摘要：

```bash
python3 tools/flight_log_sysid.py flightlog_20260724_200832.csv --out-dir .tmp/sysid_flightlog_20260724_200832
```

也可以被其他脚本直接导入：

```python
from tools.flight_log_sysid import analyze_flight_log

analysis = analyze_flight_log("flightlog_20260724_200832.csv")
print(analysis.gain_groups)
```

工具会自动切分不连续日志，按保存下来的控制参数快照分组，报告倾角/电机饱和、
记录丢包等风险，并拟合主要的“控制命令 -> 执行器输出”映射。

如果需要中文表格和图表界面：

```bash
python3 tools/flight_log_sysid_ui.py flightlog_20260724_200832.csv
```

如果想用一个窗口完成“选文件、选片段、看波形、弹出 Rerun 回放”，使用 All-in-One 工作台：

```powershell
python tools\flight_log_workbench.py log
```

工作台左侧选择日志文件和文件内片段，右侧选择通道并查看当前片段波形；
点击“弹出Rerun回放”会通过 `.tmp/rerun_env` 隔离环境打开该片段的三维现场回放。

如果只是想自己选文件、选通道、快速看波形，也可以单独用离线波形查看器：

```bash
python3 tools/flight_log_waveform_ui.py log
```

也可以直接打开某个 CSV：

```bash
python3 tools/flight_log_waveform_ui.py log/flightlog_20260727_045138.csv
```

界面左侧选择日志文件，中间搜索/选择通道，右侧点击“画选中”即可查看曲线；
支持多通道同图、分图、归一化、按时间断点断开，并显示选中通道的统计值。
图表区可以选择滚轮模式：总缩放、只缩横轴、只缩纵轴；滚轮会以鼠标所在位置为中心缩放。

如果想用 Rerun 做三维现场回放和时间轴播放，推荐用隔离虚拟环境启动脚本；
它会把 `rerun-sdk` 装到 `.tmp/rerun_env`，避免 Rerun 依赖升级影响主 Python 分析环境。

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 log
```

也可以直接列出最新日志的分片：

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 log --list
```

播放某一个分片：

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_flight_log_rerun_replay.ps1 log --play --segment 13
```

如果你已经在当前 Python 环境里安装好了 `rerun-sdk`，也可以直接运行 Python 脚本：

```bash
python3 tools/flight_log_rerun_replay.py log --list
```

工具会按时间断点把日志切成多个片段，导出 `.tmp/rerun_replay/*.rrd`；
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

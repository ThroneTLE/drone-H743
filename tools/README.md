# drone-H743 PC 工具

> 当前工具入口。阶段进度、验收结论和当前应执行的命令以 `PIPELINE.md` 对应 REQ 为准；
> 不要从历史 README 或旧 COM 号推断实机操作。

## 主工作台

```powershell
python tools/drone_tcp_panel.py
```

Python 面板在当前工作树中提供以下能力；哪些版本已经烧录或审核，以 `PIPELINE.md` 为准：

- 状态监视工作台与遥测流 v3；
- V0 机体坐标验收、V1 IMU 室温计量、V2A 无桨链路验收；
- 舵机机械校准、BUS/PWM 类型选择和舵机调试；
- 光流/测距校准与监视；
- 参数、诊断、Flash、日志和 USB ROM DFU 工作流。

面板会恢复上次连接且默认启用自动连接。它不是安全判定的唯一边界；危险命令、标定提交、
执行器动作和解锁条件必须由固件再次验证。实机默认归审核者，没有当前 REQ 或作者授权时
不要启动面板、连接设备或发送命令。

## 日志页

顶部“日志”分为“接收与导入”“波形与回放”“离线分析”。

- 本地 CSV 直接读取；BIN 用现有 FlightLog 解析器转换到新的独立目录，保留源文件。
- 在“接收与导入”点击“从当前连接接收日志”，自动使用顶部已连接的串口及实际波特率，无需关闭连接或改动自动重连设置。导出复用原收发线程，普通轮询暂时让位；取消、完成或失败后恢复普通收发。停止类命令可中断导出，顶部停止仍可断开连接。
- 完整接收后自动选择生成的 CSV；不完整接收会显示缺失字节，保留产物而不自动载入。
- 波形与回放复用 `flight_log_workbench`；离线分析复用 `flight_log_sysid_ui` / `flight_log_sysid`。读取与分析在后台执行，Rerun 仍通过既有隔离脚本启动。
- 报告导出写入新目录；取消目录选择不会写文件。原独立工具入口继续保留。

## 数据位置

所有默认路径由 `tools/project_paths.py` 定义，目录规则见 `data/README.md`。不要在
`tools/` 下新建数据目录，也不要覆盖 `data/calibration/**` 历史证据。

## 常用离线工具

| 目的 | 入口 |
|---|---|
| 飞行日志接收 | `tools/flight_log_receive.py` |
| 飞行日志波形工作台 | `tools/flight_log_workbench.py` |
| Rerun 回放 | `tools/run_flight_log_rerun_replay.ps1` |
| 内环系统辨识（当前） | `tools/sysid/`，界面在面板「系统辨识」页；方法见 [`doc/system-identification.md`](../doc/system-identification.md) |
| 推力台标定（当前） | `python tools/pressure_rs485_gui.py`（`python -m tools.thrust_bench` 同入口）；原称重/砝码标定与H743上下桨控制、实测扫描的单页流程，DShot电转速eRPM、分路电流与带载电压标定（无需极对数），见 [`doc/thrust-bench.md`](../doc/thrust-bench.md) |
| 历史日志事后分析 | `tools/flight_log_sysid.py`、`tools/attitude_ident_pid.py`（静态 OLS / 旧台架，不用于新辨识） |
| 历史推力数据查看 | `tools/thrust_ident_auto_viewer.py` 仅事后读取旧格式 `thrust_ident_auto_*.csv`；仓库内旧CSV已于2026-09-23删除，需要时从外部选择文件；旧ESP控制已从推力台窗口移除 |
| IMU 漂移分析 | `tools/stationary_drift.py`、`tools/drift_ab_check.py` |
| 光流数据评估 | `tools/flow_velocity_filter_eval.py`、`tools/flow_quality_probe.py` |
| Flash 诊断与时序 | `tools/flash_diag_test.py`、`tools/flash_timing_capture.py` |
| Saleae 抓取/解析 | `tools/saleae_imu_spi_capture.py`、`tools/decode_saleae_spi_csv.py` |

完整脚本和入口不要靠本文手工枚举；先查自动生成的
`.agents/skills/drone-h743-project/references/repository-index/host-tools.md`。

## Rerun 隔离环境

`run_flight_log_rerun_replay.ps1` 把 `rerun-sdk` 安装在 `.tmp/rerun_env`，避免污染工程
Python 环境：

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_flight_log_rerun_replay.ps1
```

## Serial Studio

`tools/ground_station/` 保存一套 Serial Studio/Qt 调研与原型。该副线当前是否可投入，
只看 `PIPELINE.md`；不要把其中的仿真工程当成真机工作台。说明见
[`ground_station/README.md`](ground_station/README.md)。

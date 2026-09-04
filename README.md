# drone-H743

STM32H743 + FreeRTOS 共轴双桨飞控。本文只做文档导航，不复制进度、参数或协议细节。

## 当前事实从哪里读

| 需要知道什么 | 唯一入口 |
|---|---|
| 当前主线、REQ、验收缺口 | [`PIPELINE.md`](PIPELINE.md) |
| Agent 执行规则 | [`AGENTS.md`](AGENTS.md) |
| 当前软件架构 | [`doc/current-architecture.md`](doc/current-architecture.md) |
| 稳定工程契约 | [`doc/technical-spec.md`](doc/technical-spec.md) |
| 遥测线上格式 | [`doc/telemetry-protocol.md`](doc/telemetry-protocol.md) |
| 硬件与板级事实 | [`doc/hardware-reference.md`](doc/hardware-reference.md) 与 `drone-H743.ioc` |
| PC 工具入口 | [`tools/README.md`](tools/README.md) |
| 数据目录与证据规则 | [`data/README.md`](data/README.md) |
| 历史资料使用规则 | [`doc/history/README.md`](doc/history/README.md) |

坐标系的机器权威是 `Driver/Inc/drv_frame_contract.h`：右手 FLU，`+X` 前、`+Y` 左、
`+Z` 上。任何旧文档、历史数据或第三方库约定与它冲突时，不得把旧口径当作另一套规范。

## 常用命令

```powershell
cmake --build --preset Debug
python -m pytest tests -q
python tools/drone_tcp_panel.py
```

`drone_tcp_panel.py` 会恢复上次连接，且自动连接默认开启；未经授权不要启动，或先确认本机
状态文件已关闭自动连接。实机默认由审核者持有。没有当前 REQ 或作者明确授权时，不烧录、
不复位、不连接目标板、不发送命令。

# M1 底层与原始数据健康 · 实机作业单

> 目标：一次通电，按顺序拿齐 PIPELINE M1 验收门的全部证据。全程**拆桨**；M1 不发任何舵机/电机指令。
> 前置：昨夜 fix_IMU 分支 7 个主题提交已全部落地，696 项测试通过、Debug 固件构建零警告（见 PIPELINE 证据表 2026-08-30）。

## 1. 烧录当前固件

```powershell
cmake --build --preset Debug
& "D:/Program Files/OpenOCD-20240916-0.12.0/bin/openocd.exe" `
  -f interface/stlink.cfg -f target/stm32h7x.cfg `
  -c "program build/Debug/drone-H743.elf verify reset exit"
```

（jlink MCP 的 openocd 服务器可用时优先走 MCP；上面是 CLI 后备，路径取自 .vscode/launch.json。）

## 2. 连接工作台

```powershell
python tools/drone_tcp_panel.py
```

- 自动重连按 USB 序列号指纹找板，COM 变号（30/31/32 轮换）也能跟上；首次无 `data/panel_state.json` 时手选串口一次即可。
- 连接后先看：`RTOS?` 有回、USBCDC 发送统计（`tx_dropped` 应接近 0 且不随时间增长）。

## 3. M1 证据采集清单（对照 PIPELINE M1 门）

| # | 项目 | 操作 | 通过判据 | 证据落点 |
|---|---|---|---|---|
| 1 | 链路健康 | 连续轮询 ≥2 分钟 | 无断连；tx_dropped 稳定 | 面板日志 |
| 2 | 器件 ID | `FLASH?`（JEDEC C8 40 16）、IMU/气压计诊断回读 | ID 全部匹配 | 命令回显截存 |
| 3 | 采样率/时间戳/丢样 | IMU 页「静止漂移自检」跑 ≥30 s | 无大空洞丢帧、无重复/倒退序号 | `data/calibration/imu_metrology/<日期>/stationary_drift/` 自动落盘 |
| 4 | 温度合理性 | 同上报告的温度跨度字段 | 室温附近、无跳变 | 同上 |
| 5 | 供电 | 万用表量 3V3/5V 轨（面板测不了） | 电压在标称 ±5% | 人工记录进 PIPELINE 证据表 |

## 4. 收尾（强制）

- 把每项结果（含失败）回写 `PIPELINE.md` 证据表；全部通过才允许把 M1 置 ✅ 并把当前位置移到 M2。
- M2 提醒：V0 坐标闭环要在**当前固件**上重跑（历史 2026-08-28 证据仅供对照）。

## 备忘（后续节点的坑，提前知道）

- 固件新规：只带 V1 标定位的旧 FCAL 记录**无法启动 ACCEPT**——到 M4/M6 前需先在舵机机械页完成一次 `SERVOCAL COMMIT`。
- 面板浏览历史验收会话现在是严格只读（昨夜修复），要续采必须点「继续历史会话」或「新建独立验收」。

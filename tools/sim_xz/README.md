# R-SIM-1 X/Z teaching simulator

这是一个纯电脑端的 X/Z 二维教学仿真器。它用于观察控制器、参数和上位机协议的关系，不替代实机飞行、标定或验收证据。

## 上位机一键启动（推荐）

启动此工作树的上位机：

    python tools/drone_tcp_panel.py

在顶部连接栏下的 **仿真 · X–Z 教学实验** 中点击 **一键启动并连接**。上位机会选择空闲回环端口，打开仿真窗口并开始实验，同时切到带四环Kp与波形的临时二维仿真工作区。无需手动输入端口或另外运行命令。

已有设备连接时先手动断开；启动按钮不会切走实机。**停止仿真**会关闭本次启动的子进程、释放监听并恢复原设置（不自动重连）。关闭上位机也会回收仿真进程。失败信息显示在标题栏，详细日志位于simulation日期目录的launcher日志。

## 手动启动（可选）

1. 先启动现有地面站 TCP 服务端，默认监听 `127.0.0.1:6666`；端口不同则使用 `--host` 和 `--port`。
2. 在仓库根目录运行 `python -m tools.sim_xz --host 127.0.0.1 --port 6666`。
3. 点击“开始 / 暂停”，选择位置、速度或俯仰实验；实验目标和电机时间常数可以在窗口中修改。
4. 在地面站参数控件中修改四环参数，先点击“保存 A 参数快照”，再点击“运行 B 并保存”。A/B 会锁定实验类型、目标、物理模型和完整参数快照。
5. 关闭窗口会停止 TCP 客户端和 A/B 导出；每次导出使用唯一 run id，不覆盖同日结果。

This is a host-only teaching tool. It does not provide flight evidence, calibration evidence, or a hardware substitute.

## Start

1. Start the existing ground-station panel as a TCP server on `127.0.0.1:6666`.
2. From the repository root, run:

   ```powershell
   python -m tools.sim_xz
   ```

3. Use `Start / pause` in the simulator window. The simulator is a TCP client and does not open a serial port.
4. In the ground station, request `CAPS?`, `PARAM?`, and `TELEM?`. The simulator answers with the existing `$X` function IDs and CRC.
5. Change one of the four gain channels in the ground station, wait for the parameter echo, click `Save A parameter snapshot`, then click `Run B vs saved A` after the next parameter change.

## Experiments

The defaults are the approved teaching values: position step `0.5 m`, X velocity step `0.2 m/s`, and pitch step `3 deg`. The X velocity experiment keeps Z at `1 m` and bypasses only the horizontal position target by using the existing cascade's velocity feed-forward path. The pitch experiment fixes total thrust to the C bridge's hover value.

The A/B artifact contains a complete parameter snapshot and five comparison channels: `pitch`, `pitch_rate`, `vx`, `x`, and `z`. Artifacts are written below `data/simulation/YYYY-MM-DD/` with `simulation=true` metadata.

## Boundaries

The C bridge is compiled from the real host-pure controller sources. Python owns only the deterministic plant, actuator assumptions, experiment orchestration, TCP adaptation, and Tk snapshots. Drag and thrust time constant are teaching assumptions; tilt time constant, mass, gravity, inertia, and lever arm come from the C-side airframe model. No serial, flash, reset, or target-board command is used.


## 界面与操作（2026-09-08 复核修订）

界面按“实验设置 → 侧视场景 → 响应对比”分栏。机体竖直绘制，上端朝上，同轴双桨在重心下方；机体长轴不等同于水平 X 坐标轴。合推力 T 从桨盘处发出，Fx/Fz 是世界坐标水平/竖直分量，推力箭头随姿态和倾转角变化。示意图尺寸不是机械设计尺寸。

- 左栏选择中文实验、应用目标；从上位机启动本机 TCP 监听后，点击开始。未连接会明确提示。
- 场景显示米制坐标、俯仰角和推力，暂停保持轨迹，复位清除轨迹与控制器状态，保留调参。
- 尚未生成 A/B 时，右栏显示实时响应；保存 A 后在上位机修改参数，再运行 B。结果使用点击时的快照，后台运行中禁止改写 A。
- 五条曲线为俯仰角（度）、俯仰角速度（度/秒）、X速度、X位置、Z高度；金色实线为A，青色虚线为B。每条纵轴独立缩放，读数须结合刻度。
- 输入 nan/inf 会被拒绝，非有限状态或场景越界会暂停并说明原因。导出JSON附带实验目标、模型假设与积分步长。
- 此版支持至少1100×740窗口；已做三尺寸、三Tk缩放的离线布局检查，不等同跨显示器DPI实测。

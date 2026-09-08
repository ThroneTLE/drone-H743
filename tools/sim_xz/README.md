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


## 完整串级与高度实验（R-SIM-3）

上位机仿真标题栏现在可以选择“高度阶跃 · Z”再一键启动，自动进入“高度 P—速度 PID”工作区。仿真窗口同样可以切换实验；切换后复位并暂停，点击开始运行。高度默认从1 m保持1秒后给到1.3 m，高度增量可修改。

- 水平：X位置P → X速度PID → 期望推力/俯仰 → 俯仰角P → 俯仰角速度PID → 同轴分配器。
- 高度：Z位置P → Z速度PID → 竖直加速度/重力补偿 → 合推力；通过姿态与推力矢量与水平链耦合。
- 共12个核心增益。位置和角度环本来就是P，不人为增加I/D；速度和角速度环的P/I/D均调用现有C实现。
- “二维仿真”工作区显示水平8项增益与俯仰/角速度/X速度；“高度 P—速度 PID”显示高度4项增益与Z/vz/实际推力。
- 高度实验A/B显示Z、vz等垂直响应，CSV也包含vz；所有参数、模型来源值和目标随快照保存。
- 增大P不一定改变曲线：例如当前垂直速度上限0.3 m/s，当两组P都已触发限速时，前段响应可能相同。这是保留真实限幅的结果，不会为了让动画有差异而解除限制。

## 物理模型保留内容与边界

仅约束Y位移、滚转、偏航为零，X/Z/俯仰保持非线性旋转、重力及倾转力矩耦合。控制器仍使用原C积分、D滤波、前馈、抗饱和与多速率调度；反馈当前采用理想状态，未加入没有依据的传感器噪声或估计器误差。

- 上下电机：先由真实C推力—PWM—推力函数经过静态曲线/量化换算，再分别更新实际推力和上限；两电机时间常数暂共享可调tau，是待辨识项。
- 俯仰执行器：使用机体头文件中驱动俯仰的BETA/ID2辨识，保留延迟、静态增益、上升/下降不同时间常数。方向按宿主C机械映射推导；编码器中性反馈值不当作机械trim。
- 辨识适用条件是总线舵机、全供电、停电机、正负200us大信号；将反馈位移增益用于倾转角是工程近似，不宣称已经验证当前负载或PWM模式下相同。
- 俯仰力矩复用当前C的有效力臂与PITCH_EFFECTIVENESS源值；作用于实际倾转与实际推力，不直接使用控制器请求力矩。
- 质量/重力/惯量来自在册头文件；惯量仍是估计值。现有控制器默认参数和机械映射并不等于自动读取了实机Flash参数。

以上是同源控制器的平面动力学仿真，不是经过实机辨识验证的完整数字孪生。二维不是删减控制链的理由；未测部分则明确保留为待辨识边界。

# R-SIM-1 X/Z teaching simulator

这是一个纯电脑端的 X/Z 二维教学仿真器。它用于观察控制器、参数和上位机协议的关系，不替代实机飞行、标定或验收证据。

## 中文操作说明

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

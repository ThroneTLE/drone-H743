# R-SIM-1 最终离线复核
复核提交：57a2f141。日期：2026-09-08。仅教学工具软件复核，不替代指定审核者验收，REQ 保持待审核。

## 已验证
- 六个 simulation 测试模块独立复跑：21 passed in 11.86s。
- Physical serial open attempts: 0
- Flashing/probe tool invocation attempts: 0
- Tk display available at session start: True; Tk roots created: 2
- 真实 DronePanel + TcpTransport + SimulatorDevice 测试逐一验证四环 Kp 写入 C 与遥测回读。
- 实际截图 r_sim_ui_screenshot.png 的五量曲线、中文标签与 A/B 图例均完整可见。
- 本轮不重复全仓回归或固件构建；此前缺历史 CSV 的全仓失败仍按执行者证据保留。

## 未关闭问题
1. [P2] experiments.py:set_targets 仅判断 <=0，nan/inf 通过。独立复现 SimulationEngine().set_targets(float('nan'), .2, 3) 成功，目标保存为 nan。step 的范围比较也没有非有限检测。应拒绝所有非有限目标，并对非有限状态显式暂停、说明原因。
2. [P2] app.py:_run_ab 在后台线程启动后才读取 B 参数及 self._baseline_params / plant.thrust_tau_s；运行中保存 A 或继续修改模型可能使比较不再是点击时固定的快照。parameter_snapshot 自身已有锁，但整个实验任务尚未原子冻结。应在提交作业前复制 A/B 参数、模型和目标，把不可变作业交给后台，并限制运行中改写 A。

## 结论
核心教学链路已有可执行验证，以上输入与并发边界仍需修复；不能声明无遗留缺陷或替指定审核者置完成。定时跟进已按作者要求暂停。

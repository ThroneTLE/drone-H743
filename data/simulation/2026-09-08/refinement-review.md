# R-SIM-1 作者要求的直接返修与界面复核
日期：2026-09-08。操作者：父任务直接实施，未再次派工。定时跟进已暂停。

## 改动
- 非有限实验目标拒绝；非有限状态在进入C控制器前暂停，计算异常保持上个有限状态；显示暂停原因。
- A/B点击时复制A、B参数和模型tau，后台运行期间禁止替换A。实验JSON增加目标、模型假设、步长。
- 新建presentation模块隔离绘图，保持控制器和协议不变。
- 三栏深色界面：实验操作、二维场景、响应曲线。机体竖直，上端朝上；双桨位于下部。
- 合推力从桨盘处绘出，标T、Fx、Fz和倾转角；世界向量投影到屏幕，正负俯仰均有测试。
- 场景有米制网格、历史轨迹、状态；曲线有单位、时间轴、数值刻度、A实线/B虚线。无A/B时显示实时响应。

## 证据
- 修bug先红后绿：8 failed, 3 passed in 1.10s → 11 passed in 0.89s。
- 针对性仿真、3尺寸×3缩放UI矩阵、PIPELINE：52 passed in 17.94s，原文refinement-tests.txt。
- 串口与probe调用均0。Debug检查原文：ninja: no work to do.
- 实际Tk窗口截图ui-refinement.png；由运行的真实控件生成，没有使用手绘SVG替代。
- A/B演示数据position_step_ab_070437_578527_f953da7a.json/.csv：同一场景两套参数，仅教学合成演示。

## 工作模式
默认层；simulation（电脑端教学工具）；tk-ui（布局绘图与可用性）；横切修bug（先前复核两项缺陷，单独提交a17cd7b9）。不运行实机基线或历史实录对拍。

## 边界
保持R-SIM-1待审核。机体图形为方向示意而非机械尺寸图；响应曲线每行独立缩放。软件演示不证明实机参数范围或飞行能力。真实跨屏DPI切换未实测。


## 最终全仓软件回归
原文refinement-full-tests.txt：2 failed, 1333 passed, 4 skipped in 252.45s。
- test_z_channel_migration_matches_old_exactly：既有历史CSV缺失，未修改或伪造。
- test_repository_index_is_current：执行全仓测试期间生成了本次截图/报告，索引落后。收齐产物后重建索引并单独复验。
全仓结果不描述为全部通过。本次simulation与UI针对性测试通过。

索引/Pipeline复验原文：10 passed in 4.81s；Physical serial open attempts: 0；Flashing/probe tool invocation attempts: 0。此前final-review.md中的两项P2均已由a17cd7b9及本轮回归关闭。

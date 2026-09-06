# 类别模式：frame-migration（坐标系运行时迁移）

适用于 R-F6 系列，以及此后任何改动 `DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK`
或六个 seam 内部坐标表述的任务。

派本类任务前先读
[flu-coordinate-contract.md](../flu-coordinate-contract.md)；本文件只补该类
任务专属的授权边界，不重复契约内容。

## 为什么单独立一类

这类任务有个别处没有的性质：**改错了不会报错，也不会测出来，会在第一次带桨
起飞时表现为翻机。** 符号错误在离线测试里往往双向抵消——估计器把 Y 取反，
控制器再取反回去，端到端测试全绿，而物理方向是反的。

因此本类任务的验收重心不在"测试通过"，而在"**每个符号的物理含义有人确认过**"。

## 硬规矩

1. **唯一事实源是 `Driver/Inc/drv_frame_contract.h`。** 注释、文档、历史数据、
   遗留符号宏与它冲突时，一律当作迁移证据，不当作另一套约定。

2. **禁止用增益抵消符号。** 发现方向反了就去改表述或适配器，不许改增益正负。
   这条违反即打回，不接受"实机上试出来这样对"。

3. **适配器必须落在具名边界上。** 不许把符号散进控制律中间。一个 seam 迁移
   完成的标志之一，是它的适配常量能被删掉，而不是被改值。

4. **一位一证。** 不许在没有对应可执行测试的情况下置 `DONE_MASK` 的任何一位。
   seam 3 / seam 4 还额外要求拆桨物理方向证据——host 测试不能替代。

5. **纯表述迁移必须数值等价。** 把内部表示换成 FLU 而算法不变时，要用
   `data/` 里的实录数据做迁移前后逐样本对拍，输出应当一致（浮点容差内）。
   对不上就说明这次改动不只是换表述，停下来说清楚改了什么。

6. **历史数据永不重解释。** 迁移后新写的遥测 / 采集 / 标定 / 飞行日志必须带
   `DRV_FRAME_CONTRACT_VERSION` 和显式 frame 标识；旧记录按它自己记录的 frame
   回放，不许因为"现在的源已经迁移了"就当成 FLU 读。

7. **禁止削弱解锁锁。** `APP_Stabilizer_IsImuFrameArmLocked()` 及其调用点、
   `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` 的判定式，都不得为了"方便测试"
   放宽。需要在半迁移状态下动电机，走 REQ 明文授权的拆桨流程，不是改门。
   （AGENTS.md 硬约束 4 的"削弱安全门"在本类里包含这一条。）

8. **一次一个 seam。** 不许在同一个提交里动两个 seam。符号错误的定位成本随
   同时改动的 seam 数指数上升。

## 交付要求（在通用完成协议之上追加）

- 交付说明里必须有一张**符号对照表**：改动涉及的每个量，迁移前语义 → 迁移后
  语义 → 谁负责适配。口头描述不算。
- 必须分开写"**软件证据**"与"**待实机确认的物理边界**"两节。措辞禁止
  "方向已验证"这类跨越了物理证据的说法。
- 每个 seam 的测试文件命名固定为 `tests/test_flu_seam<N>_<name>_frame.py`，
  不要另起名字——`DONE_MASK` 的位与它一一对应是可机检的。

## 必跑

```powershell
python -m pytest tests\test_flu_frame_contract.py -q
python -m pytest tests\test_flu_seam<N>_*.py -q
python -m pytest tests -q
cmake --build --preset Debug
```

改了契约头本身时，`drv_frame_contract.h`、`flu-coordinate-contract.md`、
SKILL.md 的路由段、`test_flu_frame_contract.py` 必须在**同一个任务**里改齐。

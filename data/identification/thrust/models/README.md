# 推力模型目录：哪个在用

**飞控当前只用一张表：`lut/current.json` 指向的推力查补表。** 其余目录都是历史、候选或报告用，不要拿去做飞控换算、对比基准或新验证的依据。

| 目录 | 是什么 | 状态 |
|---|---|---|
| `lut/current.json` | 指针：飞控里正在跑的查补表（由 `python -m tools.thrust_bench.thrust_lut_export` 写入） | **在用** |
| `lut/<日期>/<时间>-<编号>/` | 推力查补表（`thrust_lut.py`，转速表 + 油门表，双线性插值），及该表的台架验证结果 `validation-*.json/html` | 只有 current 指向的那张在用；其余是历史或候选 |
| `throttle/` | 油门 × 电量电压三次多项式（`throttle_model.py`，2026-09-23 首版） | 已被查补表取代 |
| `<日期>/training-*` | 实验库整轮训练（`dataset_model.py`，“生成模型报告”用） | 只用于报告和拟合对比，不是飞控换算 |

另有两处旧值，不在本目录，也不要当作现行数据：

- 固件 `Driver/Src/drv_coax_ctrl.c` 里的 21 点旧曲线，由旧 ESP 台架的 `emit.py` 生成。
- 机体参数 `max_total_thrust_g = 1595.342`，是那条旧曲线的末点。

## 查补表版本（2026-09-24）

| 编号 | 说明 |
|---|---|
| `1281c31206bf` | **当前**。数值与下面两张完全相同，只把可用电量上限放宽到 12.78 V |
| `a5f0068bb431` | 作者重建（数值相同）；差速实测验证 `validation-2c779cd8e38b` 在这张表上做 |
| `c493194dbb35` | 作者首建；同油门实测验证 `validation-1921b5d54e88` 在这张表上做 |
| `dd46a4a837e5` | 主控首次试建，未验证，数值相同 |

## 换表流程

1. 推力台点“建立查补表”，得到候选表。
2. 用“查补表实测验证”和“差速实测验证”检验候选表。
3. 验证通过后运行 `python -m tools.thrust_bench.thrust_lut_export <候选表 thrust_lut.json>`：写固件表并更新 `current.json`。
4. 编译、烧录。用 `THRUSTLUT ?` 确认 model 与 `current.json` 一致。

`tests/test_thrust_lut_firmware.py` 检查固件表、`current.json` 与来源 JSON 三者一致。详细约定见 `doc/thrust-bench-contract.md`，记录见 `data/analysis/thrust-bench/2026-09-23-thrust-lut/`。

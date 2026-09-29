# swing_194914_usb_w90g

台架自由摆 / 挂砝码录制（电机不转、上锁）。时间 2026-09-27T19:49:14（本机时间），时长 240 s，链路 USB（COM22）。

- 机型：drone-H743：MicoAir743v2 飞控，共轴双桨 + 两轴舵机倾转（机体 FLU）
- 固件：.tmp/fw_notch_20260927.elf（烧录时的快照，SHA256 前缀 1e5b2456）
- 台架：杆平行机体 Y（Pitch），杆在飞控板上方 0.04 m；舵机转轴（两轴相交）在板下 0.13 m
- 砝码：{"mass_g": 89.7, "position_as_used_m": {"above_rod_z": 0.227, "fore_aft_x": "±0.053（先前后后）"}, "position_as_stated_by_author": "杆上方 53 mm、前后偏移 ±227 mm", "position_note": "按作者原话（上 53、前后 227）静态偏角与摆频推出的刚度相差约 10 倍；按上 227、前后 53 两者吻合（摆频残差约 1%），分析采用后者，待作者确认"}
- 过程：无砝码松手摆 → 砝码前方静置并松手摆 → 挪到后方静置并松手摆 → 取下
- 结果：{"baseline_deg": -0.086, "front_static_deg": 5.482, "back_static_deg": -4.647, "free_swing_f0_hz": 0.68, "weighted_swing_hz": 0.532, "stiffness_K_n_m_per_rad": 0.725, "I_rod_kg_m2": 0.0415, "method": "U = K(1−cosθ)近似线性 Kθ + 砝码势能 m_w g (z cosθ − x sinθ)；平衡角与小振动频率联合最小二乘解 K、I_rod"}

文件：`samples.csv`（t_us + 姿态等 40 Hz 遥测），`conditions.json`（完整条件与结果，机器可读）。
汇总与最终参数见 `data/analysis/sysid-rig-params/2026-09-27/summary.md`。

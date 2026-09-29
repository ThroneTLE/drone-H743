# swing_200707_bt_nousb_w90g

台架自由摆 / 挂砝码录制（电机不转、上锁）。时间 2026-09-27T20:07:07（本机时间），时长 240 s，链路 蓝牙维护口（COM35），USB 拔掉、电池供电。

- 机型：drone-H743：MicoAir743v2 飞控，共轴双桨 + 两轴舵机倾转（机体 FLU）
- 固件：.tmp/fw_notch_20260927.elf（烧录时的快照，SHA256 前缀 1e5b2456）
- 台架：杆平行机体 Y（Pitch），杆在飞控板上方 0.04 m；舵机转轴（两轴相交）在板下 0.13 m
- 砝码：{"mass_g": 89.7, "position_as_used_m": {"above_rod_z": 0.227, "fore_aft_x": "±0.053"}, "position_note": "同 swing_194914_usb_w90g"}
- 过程：同上一轮
- 结果：{"baseline_deg": 1.129, "front_static_deg": 7.105, "back_static_deg": -3.658, "free_swing_f0_hz": 0.675, "weighted_swing_hz": 0.535, "stiffness_K_n_m_per_rad": 0.694, "I_rod_kg_m2": 0.039, "interpretation": "拔掉 USB 后外部已无连线，K 视为纯重力：d = K/(m g) = 0.694/(1.1453×9.81) = 0.0618 m → 重心在板下 0.0218 m；USB 线只贡献约 4% 刚度并使静止角偏约 1.3°"}

文件：`samples.csv`（t_us + 姿态等 40 Hz 遥测），`conditions.json`（完整条件与结果，机器可读）。
汇总与最终参数见 `data/analysis/sysid-rig-params/2026-09-27/summary.md`。

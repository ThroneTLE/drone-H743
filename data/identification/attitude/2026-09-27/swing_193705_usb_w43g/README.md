# swing_193705_usb_w43g

台架自由摆 / 挂砝码录制（电机不转、上锁）。时间 2026-09-27T19:37:05（本机时间），时长 150 s，链路 USB（COM22）。

- 机型：drone-H743：MicoAir743v2 飞控，共轴双桨 + 两轴舵机倾转（机体 FLU）
- 固件：.tmp/fw_notch_20260927.elf（烧录时的快照，SHA256 前缀 1e5b2456）
- 台架：杆平行机体 Y（Pitch），杆在飞控板上方 0.04 m；舵机转轴（两轴相交）在板下 0.13 m
- 砝码：{"mass_g": 43.2, "position": "杆下方、略偏前后（未量坐标）", "usable": false}
- 过程：静止 → 约 6 s、64 s、90 s 三次松手自由摆（起始幅度 20–40°）；砝码造成的静态偏角在记录里不可辨
- 结果：{"free_swing_f0_hz": 0.669, "damping_ratio": 0.037, "note": "起始幅度 40°，大角度段姿态受加速度影响（fusion_acc_err 最大 3.4°）；频率随幅度法区分重力/线缆失败（误差过大），不用于刚度"}

文件：`samples.csv`（t_us + 姿态等 40 Hz 遥测），`conditions.json`（完整条件与结果，机器可读）。
汇总与最终参数见 `data/analysis/sysid-rig-params/2026-09-27/summary.md`。

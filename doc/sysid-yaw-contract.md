# 吊绳偏航辨识与整定（SYSID MODE YAW）—— 跨侧契约

2026-10-01 清晨。作者："我们第二个偏航，我会尝试用绳子给他吊起来，你可以在姿态页面写一个YAW整定"。
背景：手扶带桨台架作者"我控制YAW基本没感觉到反馈"；日志与模型核算——共轴偏航靠上下桨反扭矩差，
上限 = k·(2·T单max − F)，k=`DRV_COAX_CTRL_PROP9047_YAW_M_PER_N`=0.005（**从未实测**），悬停时约 0.0095 N·m；
回归偏航效能约为模型 1/3（手扶干扰未排除）。`drv_coax_ctrl.c` 注释早有待办："一次系留纯偏航台阶（固定推力差、
记 gyro_z、取初始斜率）即可定住 k/I_zz 这个比值——控制律真正需要的也只是这个比值"。固件与地面站都以本文件为准。

## 台架

- 机体用绳从上方吊住，**推力始终小于机重**（绳子一直绷紧，升不起来）；绳顶最好装转环，偏航自由；绳越长扭转刚度越小。
- 横滚/俯仰由吊挂摆约束，**本模式舵机锁中位、不跑横滚俯仰环**（避免与摆动打架）；只用上下桨差速产生偏航力矩。
- 推力越小、差速余量越大：总推力 F 时差速上限 ΔT ≤ 2·min(F/2, T单max − F/2)。默认 F = 0.5×悬停推力。

## 模式与命令（固件）

- `SYSID MODE YAW`（编号 6；`APP_SYSID_YAW`），新模块 `App/Src/app_sysid_yaw.c/.h`（照 `app_sysid_xy.c` 的结构）。
- 配置：`SYSID YAW inject=diff|rate thrust_mn=<总推力 mN> twist_deg=<90..1440>`，只在空闲时收、只存 RAM；
  上电默认 `diff / 0.5×悬停推力 / 720`。回显一行：`SYSID YAW inject=<..> thrust_mn=<..> twist_deg=<..> control=openloop|closed_loop`
  （diff 为 openloop，rate 为 closed_loop）。`SYSID YAW` 无参数 = 只回显。拒绝回 `SYSID YAW event=rejected reason=running|range|lift|usage`
  （`lift`：thrust_mn > 0.8×机重，会把机体提起来；`range`：< 2000 mN 或参数越界）。
- 幅值沿用激励设置（单位随注入类型）：diff 为差速推力 ΔT [N]，上限 = min(0.8·F, 2·(T单max − F/2))（`T单max` 用 `DRV_COAX_CTRL_SingleMaxThrustN()`）；
  rate 为偏航角速度 [rad/s]，上限 2.0。越界开跑前拒绝 `ERR sysid yaw amp over limit (inject=<..> max=<..>)`。
- 开跑后紧跟溯源行：`SYSID YAWSTART run=<n> yaw_inject=<..> yaw_thrust_mn=<..> yaw_k_um_per_n=<k·1e6> yaw_izz_ugm2=<Izz·1e6>
  yaw_single_max_mn=<T单max mN> yaw_twist_deg=<..>`。

## 阶段与出力

复用 `APP_SysIdPhase`：IDLE → RAMP_UP（1.5 s，总推力由怠速线性升到 F，ΔT=0）→ SETTLE（2 s）→ PREROLL（0.5 s）→
EXCITE（激励剖面）→ RAMP_DOWN（1 s 线性降到最小，结束报 `finished`）。舵机全程中位。

- **diff（开环，辨对象）**：偏航力矩指令 M = k·ΔT，ΔT = amp·shape；用控制器**同一套**分配（新增公开函数
  `DRV_COAX_CTRL_AllocateYawPair(float total_force_n, float yaw_moment_n_m, float *upper_n, float *lower_n, uint8_t *saturated)`，
  内部就是 `coax_ctrl_allocate_motor_thrust`，含偏航极性与单桨上限钳位）→ 推力映射换脉宽（有 `pulses_for_pair` 用它）。
- **rate（闭环，验整定）**：偏航角速度参考 r = amp·shape；用生产偏航角速度环同一组参数（`coax.rate_yaw_kp/ki/kd/ff/i_limit`）算 M，
  再走同一分配。SETTLE/PREROLL 段 r = 0（只保持不转）。
- 控制拍 500 Hz，记录按现有降采样。

## 安全（软停同 ALT/XY：ΔT 与 r 清零，推力 1 s 线性降到最小，报中止理由）

- `yaw_overspeed`：|gyro_z| > 8.0 rad/s（2026-10-01 首轮吊绳 diff 1.5 N/1.2 s 即达 3.9 rad/s，作者"停止阈值太低了我正常激励都会触发"，由 4.0 改 8.0；页面 diff 默认改 0.5 N/0.8 s）。
- `yaw_twist`：开跑起陀螺 z 积分的偏航角 |ψ| > twist_deg（绳子会绞）。
- 当拍交还：遥控失联、上锁、STOP、IMU 失效（同 ALT/XY）。
- thrust_mn 超 0.8×机重在配置时就拒绝（不可能开跑）。

## 记录（沿用 ALT/XY 追加的 7 个尾字段；批头标志新增 `DRV_SYSID_FLAG_YAW 0x0800U`）

| 字段名 | YAW 模式含义 | 单位 |
|---|---|---|
| height | ψ：陀螺 z 积分偏航角（开跑清零） | rad |
| height_raw | 0 | — |
| height_sp | 0 | — |
| vz | 偏航角速度（原始陀螺 z，未陷波） | rad/s |
| vz_sp | 偏航角速度参考 r（diff 为 0） | rad/s |
| az | 差速推力指令 ΔT = T_lower − T_upper（按控制器极性约定，正值对应正 M） | N |
| vbat | 电池电压 | V |

其余：`torque` = 偏航力矩指令 M（饱和前）[N·m]；`thrust` = 总推力 F；`erpm`/`erpm_lower` = 上/下桨转速（关键：实际差速）；
`gz` = 陀螺 z；`angle`/`angle_sp`/`servo_tilt`/`tilt_x`/`tilt_y` = 0。`conditions.json` 由页面写入 `mode="6"`、`yaw`（YAWSTART 字段）与 `yaw_request`。

## 地面站

- 「系统辨识」页组新增子页「偏航（吊绳）」（`tools/panel_lib/pages/sysid/` 下新模块，结构照「XY 速度 / 位置环」页），切到本页＝本轮做 YAW。
  界面：台架说明（吊绳、转环、推力小于机重）、注入类型（diff/rate）、总推力（默认 0.5×悬停推力，显示机重与 0.8 倍上限）、
  幅值与剖面（沿用激励设置，单位随注入类型；显示差速上限）、绞绳上限、实时偏航角速度/偏航角、开始/停止、波形（ψ、ω 与 r、ΔT 与 M、上下桨 eRPM）、结果。
- 分析 `tools/sysid/yaw_analysis.py`：
  - diff：拟合 ω̇ = b·M(t−τ) − d·ω − s·ψ + c（τ 网格搜索 0–150 ms），输出 b（=1/Izz_有效，单位 rad/s² per N·m）、
    与模型 1/Izz 之比（即 k 实测/k 模型 的倍数）、d、s、τ、拟合优度；并按 b、τ 给出偏航角速度环建议 `rate_yaw_kp/ki`
    （目标穿越频率 ω_c = min(1/(4τ), 设定上限)，kp = ω_c/b，ki 取 kp·ω_c/5），同时报告悬停时最大可用偏航力矩与对应最大角加速度。
  - rate：阶跃上升时间、超调、稳态误差、饱和时间占比。
- 存档 `data/identification/yaw/<日期>/yaw_<时分秒>_<id>/`（samples.csv + conditions.json），与 ALT/XY 同格式。

## 实现后澄清（2026-10-01，固件侧交付后定稿，两侧以此为准）

- **ψ 记录饱和**：`height` 缩放 1e-4、int16 只到 ±3.2767 rad（约 ±187°），绞绳上限可设到 1440°——分析侧的 ψ 一律由 `vz` 积分得到，
  `height` 只作 ±187° 内参考。
- **力矩分辨率**：`torque` 分辨率 1e-4 N·m 而偏航 M 约 1e-3 量级——拟合与指标里的 M 一律用 `az × k` 重建（k = `yaw_k_um_per_n`×1e-6）；
  `az` 是**分配钳位后**实际下发的差速，`az = P·(T_lower − T_upper)` 与 M 同号（P 为偏航极性）；`torque` 仍是饱和前指令，可判"是否撞上限"。
- **推力来源**：YAW 恒为自动油门，**不读** `SYSID THROTTLE target_n`，只借其最高油门 % 封顶（两桨分别封）；页面不需再发 `SYSID THROTTLE target_n`。
- **开跑拒绝补充**：`ERR sysid yaw prop map uncalibrated (yaw polarity)`（桨位/旋向未标定时分配器给不出偏航力矩）、
  `thrust over lift limit (thrust_mn>0.8*weight)`、`thrust too low (thrust_mn<2000)`、`thrust over max`、`airframe invalid`；
  `amp over limit` 格式 `(inject=<..> max=<三位小数>)`。
- **lift 判据**用机重 mass×g，不用悬停学习值。
- **rate 模式**用 `DRV_RateControl_Step` 只跑 z 轴、参数每拍现取；没有参考模型的偏航角加速度前馈，`rate_yaw_ff` 在本模式不起作用。
- 电机：直通分支经 `APP_SysId_GetMotorPulsePair()` 取上下桨脉宽；两桨相等时与旧路径逐位相同，不等时按桨位标定分别下发。

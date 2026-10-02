# 水平槽 XY 速度/位置辨识（SYSID MODE XY）—— 跨侧契约

2026-09-30，R-XYID-1。作者要求："实现这个XY的速度和位置的辨识……高度环是竖直起来的槽，这个XY是水平的槽，
我们可以验证位置和速度闭环，可能也会有摩擦力。"固件与地面站两侧都以本文件为准；改契约先改这里。

## 台架与方向

- 与光杆/高度台同一套架子：碳杆穿过机体，杆两端落在**水平**槽里。机体靠推力托住自重（推力 ≈ 悬停推力，
  槽只受很小的法向力），可以绕杆倾斜；绕杆倾斜时推力的水平分量推着机体沿槽平移。
- 杆方位角 ψ 就是现有台架设置里的"杆轴方位角"（`sysid.rig.azimuth_rad`，FLU 机体系，与内环页同一个量）。
  杆可装 ±45° 或平行 Y 轴。**平移方向 u = 水平且垂直于杆 = (−sin ψ, cos ψ)**（FLU 机体系 x、y 分量）。
  沿 +u 为正。台架上机体不偏航，机体系即世界系。
- 测量：光流 + 测距（`SVC_FlowNav_GetVelocity/GetPosition/GetState`），投影到 u 上。垂直于 u 的分量只记录不控制。

## 模式与命令（固件）

- `SYSID MODE XY`（编号 5；`APP_SYSID_XY`）。
- 配置：`SYSID XY inject=tilt|vel|pos win_mm=<30..400> mass_g=<0|500..3000>`，只在空闲时收、只存 RAM，
  上电默认 `tilt / 150 / 0`。回显一行：`SYSID XY inject=<..> win_mm=<..> mass_g=<..> control=openloop|closed_loop`
  （tilt 为 openloop，vel/pos 为 closed_loop）。`SYSID XY` 无参数 = 只回显。拒绝回
  `SYSID XY event=rejected reason=running|range|usage`。
- 开跑前检（`ERR sysid xy <理由>`）：需自动油门（`SYSID THROTTLE target_n>0`，target_n = 托住机体的推力，
  由页面按 `coax.hover_thrust_n` 或 `HOVER?` 学到的值填，且 ≤ 整机最大推力）；幅值上限 tilt ≤ 0.10 rad、
  vel ≤ 0.30 m/s、pos ≤ 0.15 m 且 ≤ 0.7·win（`amp over window: inject=pos amp<=0.7*win`）；光流速度与位置当下有效且新鲜
  （≤ 200 ms，时间戳差取有符号：样本比当前拍晚 1 ms 也算新鲜）；测距有效。光流无效的拒绝带诊断：
  `ERR sysid xy flow invalid (vel_valid=<0|1> height_valid=<0|1> age_ms=<ms，-1=从未出过样本>)`，前缀不变；
  vel_valid 取观测里的 flow_valid（光流速度有效且测距有效）。
- 开跑后紧跟一行溯源：`SYSID XYSTART run=<n> xy_inject=<..> xy_win_mm=<..> xy_mass_g=<..> xy_psi_mrad=<..>
  xy_target_cn=<..> xy_x0_mm=<..> xy_y0_mm=<..>`（起点为开跑那一刻光流位置）。
- `SYSID THR?` 行末尾追加实时量：`xy_pos_mm=<沿 u 相对起点> xy_vel_mms=<沿 u> xy_ok=<0|1> xy_yaw_mrad=<int>`（非 XY 模式也报，
  xy_pos_mm 相对最近一次记下的起点；xy_yaw_mrad 是本轮或最近一轮陀螺 z 积分出的偏航 [mrad]，开跑清零；
  地面站只在 XY 页用）。注意行长预算测试。

## 阶段与出力

复用 `APP_SysIdPhase`：IDLE → RAMP_UP（1.5 s 推力由怠速线性升到 target_n，推力 ≥ 2 N 起保持姿态）→
SETTLE（2 s）→ PREROLL（0.5 s）→ EXCITE（激励剖面）→ RAMP_DOWN（1 s 线性降到遥控油门，阶段结束
`finished`）。推力恒为 target_n（RAMP 段除外）。

- 姿态：与 ALT 相同的两轴保持（`sysid_closed_loop` 的 ALT 分支：生产 DRV_AttitudeControl + DRV_RateControl，
  目标 = 开跑第一拍的静止姿态 att0），XY 在其上叠加目标倾角偏置。
- 倾角偏置由"沿 u 的期望水平加速度 a_u"换算：期望合力 F = m_eff·(a_u·u_x, a_u·u_y, g)，
  m_eff = `DRV_COAX_CTRL_EffectiveMassKg(m_run)`；目标 roll/pitch 用**生产同一公式**（drv_coax_ctrl.c 的
  target_pitch = atan2(Fx, Fz)、target_roll = −atan2(Fy·cos pitch, Fz)），为此在 drv_coax_ctrl 暴露纯函数
  `DRV_COAX_CTRL_TiltFromForce(const float force_n[3], float *roll_rad, float *pitch_rad)`，生产路径改为调用它
  （逐位不变）。倾角夹在 `coax.tilt_limit_rad` 内。
- 注入（shape = 激励发生器按单位幅值归一化的剖面，同 ALT）：
  - **tilt（开环，辨对象）**：直接给沿 u 的倾角 θ = amp·shape（换算成等效 a_u = g·tan θ 后走上面的公式），
    激励段不跑环。用于测"倾角 → 加速度"增益、静摩擦门槛与光流测量滞后。
    **2026-10-01 改**：SETTLE/PREROLL 段改为跑生产速度环、目标速度 0（位置参考跟着实测走，不追开跑点，
    免得静摩擦卡住时积分饱和），把机体按住；PREROLL 末拍的环输出锁成基准 a_trim，激励段
    a_u = a_trim + g·tan θ。原因：杆式台架上"水平"不等于零水平力，浮起推力下摩擦很小，原写法激励前就溜走
    8~11 cm、激励只剩微小响应。记录 az 含基准，分析时扣掉激励起点的 az 再看注入量。
  - **vel（闭环）**：生产 `DRV_POSITION_CONTROL`（沿 u 做一维，用 **x 通道参数** pos_x_kp / vel_x_kp / vel_x_ki /
    vel_x_kd / vel_x_i_limit / accel_xy_max；两轴对称），p_sp = p0 + ∫v_inj dt，速度前馈 v_inj = amp·shape。
  - **pos（闭环）**：p_sp = p0 + amp·shape，无前馈。SETTLE/PREROLL 段 vel/pos 都在 p0 保持。
  - 位置 50 Hz、速度 100 Hz，同生产节拍；积分限幅与饱和反馈同生产。
- 垂直于 u 的方向不加环（槽约束），倾角偏置只沿 u。

## 安全（软停，同 ALT 的软着陆思路）

- 软停 = 倾角偏置清零、位置/速度环停，姿态回 att0，推力 1 s 线性降到遥控油门，走完报中止理由。
  触发（RAMP_DOWN 段都不再判）：
  - `xy_window`：d + s²/(2·0.2) > win。d = hypot(p_u, p_perp)（相对开跑点的位移模长，旋转不变——水平槽上机体
    可偏航、杆方位也可能设错），s = hypot(v_u, v_perp)；0.2 m/s² 是软停后只靠动摩擦滑停的偏保守减速度。
  - `xy_overspeed`：s > 0.4 m/s（vel 幅值上限 0.3 m/s 之上留余量；方向设反闭环正反馈时兜底）。
    同拍同时满足时先报超速。
  - `xy_yaw_limit`：陀螺 z（原始 gyro）按控制拍 dt 积分（开跑清零），|yaw| > 15°。只监视：XY 模式偏航开环、
    没有执行通路，也不做投影旋转。
  - `xy_flow_invalid`：光流无效（无效 / 无 token / 样本年龄 > 200 ms）**连续超过 200 ms** 才软停。缺口内
    位置/速度环不更新（保持上一拍倾角偏置、积分不累加，恢复后按一个正常周期算），投影量保持上一拍值，记录在
    无效拍照旧记 0。
  - 杆轴残差/角度超限（与 ALT 同样放宽为软停）。
- 当拍交还（不软停）：遥控失联、上锁、推油门杆、IMU 失效、推力来源失效、STOP 命令。
- 推力 < 0.5·m_run·g 时（RAMP_UP 早段）任何中止都当拍交还。

## 记录（沿用 ALT 追加的 7 个尾字段，XY 模式下含义如下；非 XY/ALT 轮为 0）

| 字段名（schema） | XY 模式含义 | 单位 |
|---|---|---|
| height | p_u：沿 u 的光流位置，相对起点 p0 | m |
| height_raw | p_perp：垂直于 u 的光流位置，相对起点 | m |
| height_sp | p_sp_u：沿 u 的位置参考（tilt 注入为 0） | m |
| vz | v_u：沿 u 的光流速度 | m/s |
| vz_sp | v_sp_u：速度环参考（含前馈；tilt 注入为 0） | m/s |
| az | a_u：沿 u 的期望加速度（tilt 注入为 g·tan θ） | m/s² |
| vbat | 电池电压 | V |

其余字段照旧：`angle` = 绕杆轴 n 的绝对角度（`roll·cosψ + pitch·sinψ`，含开跑姿态），**正角把推力偏向 −u**，
所以沿 +u 的加速度对应 `−g·tan(angle − 基线)`；`angle_sp` = 目标倾角偏置在杆轴上的分量（同一口径、不含开跑姿态，
tilt 注入 θ>0 时 `angle_sp < 0`）。沿 +u 为正的指令加速度看 `az`（a_u）。
`thrust`、`tilt_x/tilt_y`、`gx/gy/gz` 等。批头标志另加 `DRV_SYSID_FLAG_XY`（新位，不与 ALT 0x0100、封顶 0x0200 冲突）。
`conditions.json` 由页面写入 `mode="5"`、`xy`（XYSTART 字段）与 `xy_request`（页面下发的配置）。

## 地面站

- 「系统辨识 → XY 速度 / 位置环」页（`tools/panel_lib/pages/sysid/horizontal.py` 现为"未实现"占位）改为可用视图，
  结构照「Z 高度」页：注入类型（tilt/vel/pos）、幅值与剖面（沿用激励设置，单位随注入类型）、出窗余量 win_mm、
  随动附加质量、托住推力（默认取 `coax.hover_thrust_n`，可手填）、实时 xy_pos_mm / xy_vel_mms、开始/停止、
  波形（位置与参考、速度与参考、绕杆角与推力）、结果。
- 切到本页 = 本轮做 XY（与 Z 页同一机制）。
- 结果分析 `tools/sysid/xy_analysis.py`：
  - tilt：沿 u 的加速度（由光流速度差分，零相位平滑）对 g·tan θ 的增益、静摩擦门槛角（开始滑动时的 |θ|）、
    光流速度相对 IMU 的滞后。
  - vel/pos：阶跃/双向阶跃的 90% 上升时间、超调、末值误差、稳态抖动。
- 存档目录：`data/identification/xy/<日期>/xy_<时分秒>_<id>/`（samples.csv + conditions.json），与 ALT 同格式。

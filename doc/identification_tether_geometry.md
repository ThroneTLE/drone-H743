# 系统辨识参数与结果记录

本文记录当前绑绳台架系统辨识用到的几何、质量、舵机映射和首轮实验建议。

文档包含两部分：

- 用户原话记录：尽量保持原始表达，便于追溯现场信息来源。
- 提炼后的工程信息：把原始描述整理成坐标、质量表、重心估算、绑绳约束和实验参数。

## 用户原话记录

> 我可以报一些参数，电路板的中心为原点，电路板为75g，最上面是电池232g,电池重心高度109mm，在电路板上方。电路板下方是底座模块，重心高度为-117mm。底座模块，底座模块99g，下面是舵机+电机，重心高度-244mm，重量是348.6g。舵机1号轴在底座模块下方44mm 舵机二号轴在舵机1号轴下面54mm，共轴双桨两个平行桨叶垂线中点距离舵机2号80.5mm，你还需要其他信息吗

> 1.左右前后对称 2.IMU近似原点 3.电路板中心距离绳子绑点156.3mm 4.一号多机管前后，2号舵机管左右 4.1500为90° 500-2500是180°代码应该也有写 电机桨叶的参数在驱动写死了做了百分比和推力的对比了

> 1.是的 2.640mm是绑点到杆子的距离.随后你可以写一个文档，要求包含我说的原话和你提炼出来的信息

## 坐标系与对称假设

- 坐标原点：电路板中心。
- z 轴方向：电路板上方为正，下方为负。
- 左右、前后近似对称，因此第一版模型取 `x = 0`、`y = 0`。
- IMU 近似位于原点，即 `x = 0`、`y = 0`、`z = 0 mm`。
- 绳子绑点在中心轴线上方：

```text
tether_attach = (0, 0, +156.3 mm)
```

- 绑点到固定杆子的距离：

```text
rope_length_attach_to_rod = 640 mm
```

## 质量与重心

| 部件 | 质量 | 重心 z 高度 | 备注 |
|---|---:|---:|---|
| 电路板 | 75 g | 0 mm | 原点 |
| 电池 | 232 g | +109 mm | 电路板上方 |
| 底座模块 | 99 g | -117 mm | 电路板下方 |
| 舵机+电机 | 348.6 g | -244 mm | 最下方动力/执行模块 |

总质量：

```text
m_total = 75 + 232 + 99 + 348.6 = 754.6 g
```

z 向一阶矩：

```text
sum(m_i * z_i)
= 75 * 0 + 232 * 109 + 99 * (-117) + 348.6 * (-244)
= -71353.4 g*mm
```

整体重心高度：

```text
z_cg = -71353.4 / 754.6 = -94.6 mm
```

结论：

```text
center_of_gravity = (0, 0, -94.6 mm)
```

也就是整体重心在电路板中心下方约 `94.6 mm`。

## 舵机与推力矢量几何

用户提供了舵机安装位置和桨叶位置（见用户原话记录）。

### 关键点 z 坐标（原点 = 电路板中心，z+ 向上）

| 点 | z (mm) | 计算 |
|---|---:|---|
| 电路板（原点 / IMU） | 0 | 定义 |
| 整体 CG | −94.6 | 质量加权 |
| 底座模块 CG | −117 | 用户给定 |
| 舵机 1 号轴（pitch / alpha） | −161 | −117 − 44 |
| 舵机 2 号轴（roll / beta） | −215 | −161 − 54 |
| 推力作用点（双桨中点） | −295.5 | −215 − 80.5 |

### 倾转机构

两台舵机构成串联二自由度倾转机构：

- **舵机 1（外层，alpha）**：绕机体 y 轴旋转，控制 pitch（前后）
- **舵机 2（内层，beta）**：安装在舵机 1 输出端，绕（已跟随舵机 1 旋转后的）x 轴旋转，控制 roll（左右）
- **共轴双桨**：安装在舵机 2 输出端，推力沿桨盘法向

倾转顺序：`R_body→thrust = R_y(α) · R_x(β)`。

### 推力方向（机体坐标系）

```text
F_body = R_y(α) · R_x(β) · [0, 0, T]^T
```

小角度近似（α, β ≪ 1 rad）：

```text
F_body ≈ T * [α,  β,  1]^T
```

其中 T 为双桨总推力，α 和 β 是倾转角。

### 推力力臂（推力作用点 → CG）

推力作用点相对 CG 的矢径：

```text
r = thrust_pos − CG
  = (0, 0, −295.5) − (0, 0, −94.6)
  = (0, 0, −200.9 mm)
  = (0, 0, −0.201 m)
```

推力在 CG 下方 **200.9 mm**。

### 推力对 CG 的力矩（小角度）

```text
τ = r × F_body

τ_roll  =  d · T · β   =  0.201 · T · β   [N·m]
τ_pitch = −d · T · α   = −0.201 · T · α   [N·m]
τ_yaw   = 0                               （同轴双桨差速独立提供）
```

其中 d = 0.201 m。

悬停工况（T = mg = 7.40 N）：

```text
τ_roll  ≈  1.49 * β   [N·m/rad]  ≈  0.0260 * β   [N·m/deg]
τ_pitch ≈ −1.49 * α   [N·m/rad]  ≈ −0.0260 * α   [N·m/deg]
```

**与控制器参数对比**：当前控制器设 `tilt_lever_arm_m = 0.18`，实测推力力臂为 0.201 m，偏差约 10%。MBD 仿真建议用 0.201 m。

### 倾角 → 舵机脉宽

舵机行程 500~2500 us 对应 0°~180°（机械），中心 1500 us = 90° 中立。

代码中的校准中心（来自 [Driver/Inc/drv_coax_ctrl.h](Driver/Inc/drv_coax_ctrl.h)）：

```text
ALPHA_CENTER_US = 1441 us     ← 舵机 1（pitch）
BETA_CENTER_US  = 1877 us     ← 舵机 2（roll）
```

换算：

```text
2000 us → 180° 行程
1 us   → 0.09° = 0.001571 rad
1 rad  → 636.6 us
1 deg  → 11.111 us
```

倾角 → 脉宽（中性位置附近）：

```text
alpha_us = 1441 + 636.6 * α    (α 单位 rad)
beta_us  = 1877 + 636.6 * β    (β 单位 rad)
```

代码中 sign 处理：`DRV_COAX_CTRL_SERVO_ALPHA_SIGN = +1.0`、`DRV_COAX_CTRL_SERVO_BETA_SIGN = -1.0`。正 α/β 产生正还是负的姿态力矩需实验确认（见下方 sign test）。

### Sign Convention（待台架确认）

上述力矩符号基于 R_y(α) 先转、正 α 产生 nose-up 推力的假设。实际 sign 需要验证：

```text
IDENT STEP pitch pulse_us=+30 duration_ms=2000
```

观察初始角加速度方向即可确定 α → pitch 力矩的符号。同理验证 roll。

### 共轴双桨推力与偏航

同轴双桨（上下桨反向旋转）推力合力：

```text
T = T_upper + T_lower
```

偏航力矩由上下桨反扭矩差产生：

```text
τ_yaw = (Q_upper − Q_lower)
```

转速-推力关系（来自控制器参数 `thrust_coeff_n_per_rad2`）：

```text
T_per_motor = C_T · ω² = 3.0×10⁻⁵ · ω²    [N]
```

悬停时（T = 3.70 N / 桨），ω ≈ 351 rad/s ≈ 3350 RPM，占最大转速 900 rad/s 的 39%。

## 绑绳几何

绑点相对整体重心的高度差：

```text
d_attach_to_cg = 156.3 - (-94.6) = 250.9 mm
```

这说明绳子绑点在整体重心上方约 `250.9 mm`。该几何会给机体姿态带来明显的重力恢复趋势，台架响应中会包含绑绳约束引入的摆动/恢复力矩。

如果把固定杆到绑点的 `640 mm` 也计入整机作为单摆的整体摆动长度，则固定点到整体重心的近似距离为：

```text
L_rod_to_cg ~= 640 + 250.9 = 890.9 mm
```

需要区分两个现象：

- 机体绕绑点的小角度姿态恢复：主要与 `d_attach_to_cg ~= 250.9 mm`、整机转动惯量有关。
- 整机连同绳子绕固定杆摆动：近似长度可取 `L_rod_to_cg ~= 890.9 mm`。

在没有完整转动惯量数据前，不建议把这些近似直接用于精确控制器设计；它们主要用于判断辨识数据是否合理，以及选择激励时长。

## 舵机与控制轴映射

用户描述：

```text
1 号舵机管前后
2 号舵机管左右
```

结合当前固件注释和控制代码，工程映射整理为：

| 控制轴 | 舵机变量 | 舵机编号 | 方向 |
|---|---|---:|---|
| pitch | alpha | 1 | 前后 |
| roll | beta | 2 | 左右 |

因此系统辨识命令中的轴向含义为：

```text
IDENT STEP pitch ...  -> 主要激励 1 号舵机 alpha
IDENT STEP roll ...   -> 主要激励 2 号舵机 beta
```

## 舵机脉宽与角度

用户描述：

```text
1500 us = 90 deg
500 us ~ 2500 us 对应 180 deg
```

由此可得近似换算：

```text
2000 us -> 180 deg
1 us    -> 0.09 deg
1 deg   -> 11.111 us
```

常用辨识激励幅值对应舵机角度近似为：

| pulse 偏置 | 角度偏置 |
|---:|---:|
| 20 us | 1.8 deg |
| 30 us | 2.7 deg |
| 40 us | 3.6 deg |
| 80 us | 7.2 deg |

当前固件中的机械校准中心来自 `Driver/Inc/drv_coax_ctrl.h`：

```c
#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1441U
#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1877U
#define DRV_COAX_CTRL_SERVO_TRAVEL_DEG       180.0f
#define DRV_COAX_CTRL_SERVO_LIMIT_DEG         90.0f
```

因此辨识模块默认中心为：

```text
alpha center = 1441 us
beta  center = 1877 us
```

若台架机械零位重新调整，应使用：

```text
IDENT CENTER alpha_us=<v> beta_us=<v>
```

临时修改本次辨识使用的中心。

## 电机与桨叶模型

用户说明电机桨叶参数已在驱动中写死，并做了百分比和推力对比。已完成独立的 Hammerstein 辨识实验，结果见下方 [电机推力 Hammerstein 辨识结果](#电机推力-hammerstein-辨识结果)。

当前工程中相关模型位置：

```text
Driver/Inc/drv_motor_model.h          (精简版，21点插值，无 lag update)
D:\stm32hal\drone-H743\tools\0002_hammerstein_model.h        (完整版，含 interp + lag update)
tools/0002_hammerstein_report.md      (辨识报告)
tools/hammerstein_00001_1_check/      (另一组数据，tau 不可用)
```

第一版系统辨识假设：

- 本阶段辨识对象是舵机激励到姿态/角速度响应。
- 油门由遥控器控制。
- ident 模块不主动设置电机输出。

## 第一轮辨识建议

由于绑绳约束明显，第一轮先使用较小舵机偏置确认方向、延迟和安全边界。

建议初始命令：

```text
IDENT ARM
IDENT STEP roll pulse_us=20 duration_ms=3000
IDENT STEP roll pulse_us=-20 duration_ms=3000
IDENT STEP pitch pulse_us=20 duration_ms=3000
IDENT STEP pitch pulse_us=-20 duration_ms=3000
```

确认舵机方向、姿态响应和 abort 机制正常后，再加大到：

```text
IDENT DOUBLET roll pulse_us=30 hold_ms=800 repeat=2
IDENT DOUBLET pitch pulse_us=30 hold_ms=800 repeat=2
IDENT PRBS roll pulse_us=25 bit_ms=250 duration_ms=8000 seed=1
IDENT PRBS pitch pulse_us=25 bit_ms=250 duration_ms=8000 seed=2
```

台架观察重点：

- roll 命令是否主要改变左右姿态。
- pitch 命令是否主要改变前后姿态。
- 正负 pulse 的姿态方向是否符合预期。
- 是否出现约 1 s 量级的机体姿态恢复/摆动成分。
- 是否出现约 2 s 量级的整机绳摆成分。
- `IDENT STOP`、RC disarm、IMU stale、姿态超限是否能立即结束实验并回舵机中心。

---

# 电机推力 Hammerstein 辨识结果

电机推力辨识采用 Hammerstein 模型结构：**静态非线性查表 + 一阶动态滞后**。

$$
\text{thrust\_ss} = f(\text{pwm\_us}) \quad\text{(分段线性插值)}
$$

$$
\dot{x} = \frac{1}{\tau}(\text{thrust\_ss} - x)
$$

## 辨识数据来源

| 数据集 | CSV 文件 | 说明 |
|---|---|---|
| `0002` | `tools/0002.csv` | 主要数据集，tau 可用 |
| `00001_1` | `tools/00001_1.csv` | 备选数据集，tau 不可用（过渡段信噪比不足） |

## 数据集 0002 辨识结果

### 实验条件

- 激励方式：21 级阶梯扫描（0%→100%，步长 5%，每级 dwell 2000ms）
- 尾段稳态样本数：25
- 尾段 trim fraction：0.10
- tau 拟合区间：z = 0.10 ~ 0.90

### 静态非线性表

PWM 范围 1100us ~ 1940us，推力范围 0g ~ 1363.4g。

最大稳态噪声：2.7g（95% 油门处）。最大首尾漂移：53.5g（50% 油门处）。

| PWM (us) | 油门 (%) | 稳态推力 (g) |
|---|---:|---:|
| 1100 | 0 | -0.0 |
| 1142 | 5 | 19.5 |
| 1184 | 10 | 52.0 |
| 1226 | 15 | 98.3 |
| 1268 | 20 | 154.7 |
| 1310 | 25 | 218.6 |
| 1352 | 30 | 289.3 |
| 1394 | 35 | 378.3 |
| 1436 | 40 | 457.1 |
| 1478 | 45 | 545.3 |
| 1520 | 50 | 648.0 |
| 1562 | 55 | 732.9 |
| 1604 | 60 | 833.4 |
| 1646 | 65 | 919.3 |
| 1688 | 70 | 1003.3 |
| 1730 | 75 | 1079.1 |
| 1772 | 80 | 1156.7 |
| 1814 | 85 | 1226.3 |
| 1856 | 90 | 1298.0 |
| 1898 | 95 | 1349.6 |
| 1940 | 100 | 1363.4 |

### 动态参数

- 拟合过渡段数：20（全部相邻阶梯对）
- 选中用于全局 tau 的过渡段：10（20%~90% 油门范围，R² ≥ 0.90）
- **推荐全局 tau = 0.1766 s**

### 预测误差

| 指标 | 全响应 (2100 样本) | 尾段 (525 样本) |
|---|---|---|
| RMSE | 23.08 g | 1.05 g |
| MAE | 10.13 g | 0.66 g |
| Max Abs Error | 104.96 g | 4.01 g |

全响应误差主要来自过渡段模型简化（单一 tau 替代变 tau）。稳态尾段精度良好（~1g RMSE），说明静态查表足够准确。

### C 模型文件

完整模型在 `tools/0002_hammerstein_model.h`，包含：

- `motor_hammerstein_interp_pwm_to_thrust()` — PWM→推力前向查表
- `motor_hammerstein_interp_thrust_to_pwm()` — 推力→PWM 逆查表（前馈用）
- `motor_hammerstein_interp_pct_to_thrust()` — 百分比→推力
- `motor_hammerstein_interp_thrust_to_pct()` — 推力→百分比
- `motor_hammerstein_update_lag()` — 一阶滞后更新

精简版在 `Driver/Inc/drv_motor_model.h`，仅含查表函数，无 lag update。

## 数据集 00001_1 辨识结果

### 实验条件

- 激励方式：21 级阶梯扫描，尾段 10 样本
- 推力范围：-3.2g ~ 1418.7g（接近 0002 数据集）
- 最大稳态噪声：20.4g（50% 油门处，远高于 0002）
- 最大首尾漂移：59.5g（60% 油门处）

### 动态参数

- 拟合过渡段数：20
- 选中用于全局 tau 的过渡段：**0**（所有过渡段 R² 不满足 ≥0.90 条件）
- **推荐全局 tau：不可用**

### 预测误差（仅静态模型，无 tau）

| 指标 | 全响应 (420 样本) | 尾段 (210 样本) |
|---|---|---|
| RMSE | 31.96 g | 14.23 g |
| MAE | 24.93 g | 11.29 g |
| Max Abs Error | 72.08 g | 32.63 g |

00001_1 的噪声显著高于 0002，过渡段拟合质量不足以提取可靠的 tau。**当前以 0002 数据集为准。**

## Hammerstein 模型拟合工具

拟合脚本路径：`tools/fit_motor_hammerstein.py`

```bash
python tools/fit_motor_hammerstein.py tools/0002.csv \
  --source raw --raw-scale 0.1 \
  --tail-samples 25 --trim 0.10 \
  --z-min 0.10 --z-max 0.90 \
  --tau-min-pct 20.0 --tau-max-pct 90.0 \
  --out-dir tools/
```

脚本输出：
- `{stem}_hammerstein_static.csv` — 每级稳态统计
- `{stem}_hammerstein_tau.csv` — 每对过渡段的 tau 拟合
- `{stem}_hammerstein_prediction.csv` — 预测 vs 实测对比
- `{stem}_hammerstein_report.md` — 文本报告
- `{stem}_hammerstein_model.h` — C 头文件
- `{stem}_hammerstein_static.png` — 静态曲线图
- `{stem}_hammerstein_prediction.png` — 动态预测对比图
- `{stem}_hammerstein_tau.png` — tau 随油门变化图

输入 CSV 格式要求字段：`host_time, seq, motor, pct, pulse_us, fc_ms, dwell_ms, sample_index, raw, grams, kind`，`kind` 为 `sample` 的行参与拟合。

---

# 姿态环系统辨识（APP_Ident 模块）

固件内置的舵机→姿态角速度辨识模块，用于采集 pitch/roll 轴的传递函数辨识数据。

源码位置：
- [App/Inc/app_ident.h](App/Inc/app_ident.h)
- [App/Src/app_ident.c](App/Src/app_ident.c)

## 激励信号类型

| 模式 | 命令 | 激励方式 |
|---|---|---|
| Step | `IDENT STEP <axis> <pulse_us> <duration_ms>` | 单方向阶跃 |
| Doublet | `IDENT DOUBLET <axis> <pulse_us> <hold_ms> <repeat>` | 正负交替方波对 |
| PRBS | `IDENT PRBS <axis> <pulse_us> <bit_ms> <duration_ms> <seed>` | 31 位 LFSR 伪随机序列 |

## 轴向映射

- `roll` → 激励 2 号舵机（beta，左右）
- `pitch` → 激励 1 号舵机（alpha，前后）

激励以偏置量 `pulse_offset_us` 叠加在中心脉宽上：

```text
alpha_us = alpha_center_us ± offset_us  (pitch 轴)
beta_us  = beta_center_us  ± offset_us  (roll 轴)
```

## 安全限制

| 参数 | 值 | 说明 |
|---|---|---|
| 最大偏置 | ±80 us | 超出拒绝执行 |
| 最大持续时长 | 10000 ms | 超出拒绝执行 |
| 姿态角限制 | ±25 deg | 超限自动 abort |
| 采样周期 | 20 ms | 50 Hz 输出 |
| alpha 脉宽范围 | 500~2441 us | `1441 +- 1000 us` 后受物理下限裁剪 |
| beta 脉宽范围 | 877~2500 us | `1877 +- 1000 us` 后受物理上限裁剪 |

自动 abort 条件：
- RC 信号丢失（`rc_link_ok == 0`）
- RC disarm（`rc_armed == 0`）
- IMU 数据过期（`imu_valid == 0`）
- 姿态角超过 ±25°

## 典型实验命令

```text
IDENT CENTER alpha_us=1441 beta_us=1877   // 设置当前标定中心
IDENT ARM                                  // 进入待命
IDENT STEP roll pulse_us=20 duration_ms=3000
IDENT STEP roll pulse_us=-20 duration_ms=3000
IDENT STEP pitch pulse_us=20 duration_ms=3000
IDENT STEP pitch pulse_us=-20 duration_ms=3000
IDENT DOUBLET roll pulse_us=30 hold_ms=800 repeat=3
IDENT DOUBLET pitch pulse_us=30 hold_ms=800 repeat=3
IDENT PRBS roll pulse_us=25 bit_ms=250 duration_ms=8000 seed=1
IDENT PRBS pitch pulse_us=25 bit_ms=250 duration_ms=8000 seed=2
IDENT STOP                                 // 终止
IDENT DISARM                               // 退回空闲
IDENT APPLY pitch kp=0.5 kd=-0.12          // 临时覆盖 PID 参数
```

## 数据输出格式

`IDENT ARM` 和 `IDENT START` 后，每个采样周期输出一行：

```text
IDENT sample id=<n> seq=<n> t_ms=<n> axis=<roll|pitch> mode=<step|doublet|prbs>
           alpha_us=<n> beta_us=<n> roll=<deg> pitch=<deg>
           gx=<dps> gy=<dps> rc_arm=<0|1> throttle_us=<n>
```

角度以 mdeg（毫度）精度输出，角速度以 cdps（0.01°/s）精度输出。

## 离线分析方向

采集到的 `IDENT sample` 日志可以用于：

- **时域辨识**：step 响应 → 一阶/二阶传递函数拟合（`roll_deg / pulse_offset_us`）
- **频域辨识**：PRBS 激励 + 互相关 → 频率响应估计
- **子空间辨识**：doublet/PRBS 数据 → 状态空间模型（MATLAB `n4sid` 或 Python `sip`）

辨识得到的 `axis → roll/pitch_deg` 传递函数可用于整定 `coax.roll_angle_kp`、`coax.roll_rate_kd` 等参数。

---

# 辨识结果集成状态

## 已集成在固件中的部分

| 内容 | 位置 | 状态 |
|---|---|---|
| 质量与重心参数 | [Driver/Inc/drv_airframe_model.h](Driver/Inc/drv_airframe_model.h) | 已使用 |
| 舵机几何映射 | `drv_coax_ctrl.h` / `drv_coax_ctrl.c` | 已使用 |
| 悬停油门百分比 (56%) | `DRV_AIRFRAME_HOVER_THRUST_PERCENT` | 已定义 |
| 最大推力 (13.38N) | `DRV_AIRFRAME_MAX_TOTAL_FORCE_N` | 已使用 |
| 姿态辨识激励模块 | [App/Src/app_ident.c](App/Src/app_ident.c) | 已集成，按需使能 |
| 姿态辨识安全监控 | `APP_Ident_Observe()` | 已集成 |

## 尚未集成在固件中的部分

| 内容 | 文件 | 问题 |
|---|---|---|
| Hammerstein 电机模型 | `Driver/Inc/drv_motor_model.h` | **未被任何 .c 文件 include** |
| 推力前馈逆映射 | `motor_hammerstein_interp_thrust_to_pwm()` | 未调用 |
| 一阶动态滞后补偿 | `motor_hammerstein_update_lag()` | 未调用 |
| 姿态辨识离线分析结果 | 原始 IDENT sample 日志 | 尚未进行离线传递函数拟合 |

当前 `drv_motor.c` 和 `drv_coax_ctrl.c` 仍使用线性映射：

- `percent → pulse`（drv_motor.c，`DRV_Motor_PercentToPulse`）
- `omega_rad_s → pulse`（drv_coax_ctrl.c，`DRV_COAX_CTRL_OmegaToMotorPulse`）

两者均未使用辨识出的 21 点非线性推力曲线和 tau=0.177s 的滞后补偿。

## 接入路线

要将 Hammerstein 模型接入控制链路，需要：

1. 在 `drv_motor.c` 或调用侧 include `drv_motor_model.h`
2. 将推力目标值通过 `motor_hammerstein_interp_thrust_to_pwm()` 转换为 PWM
3. 可选：用 `motor_hammerstein_update_lag()` 做前馈动态补偿（输入期望推力，输出滞后补偿后的 PWM）
4. 悬停点附近的线性化前馈：悬停推力约 369g/电机 → PWM 约 1394us

---

# 转动惯量估算

由于左右前后对称且各组件质心均在 z 轴上（x = y = 0），惯量积 I_xy、I_xz、I_yz 均为 0，惯量主轴与机体坐标轴重合。

## I_xx 与 I_yy

将各组件视为点质量，相对于整体 CG 的 z 向偏移：

```text
z_rel_i = z_i − z_cg = z_i + 94.6 mm
```

| 组件 | 质量 (g) | z_rel (mm) | m · z_rel² (g·mm²) |
|---|---:|---:|---:|
| 电路板 | 75 | +94.6 | 671,187 |
| 电池 | 232 | +203.6 | 9,617,087 |
| 底座模块 | 99 | −22.4 | 49,674 |
| 舵机 + 电机 | 348.6 | −149.4 | 7,780,158 |
| **合计** | | | **18,118,106** |

点质量贡献：`I_xx = I_yy ≈ 0.0181 kg·m²`。

加上各组件自身绕质心的局部惯量（量级约 1~5×10⁻⁴ kg·m²，远小于点质量项）：

```text
I_xx ≈ I_yy ≈ 0.019 kg·m²    （绕 CG）
```

此值是可用的一阶近似。更精确的值需要 CAD 模型或三线摆实测。

## I_zz

由于全部组件质心在 z 轴上，`I_zz` 完全由各组件绕自身 z 轴的局部惯量构成，量级远小于 `I_xx/I_yy`。

粗略估计（假设组件为方形板或圆柱）：

| 组件 | 假设形状 | 尺寸 (mm) | I_zz 局部 (g·mm²) |
|---|---:|---|---:|---:|
| 电路板 | 方板 | 60×60 | 45,000 |
| 电池 | 矩形板 | 35×75 | 132,433 |
| 底座模块 | 方板 | 60×60 | 59,400 |
| 舵机 + 电机 | 圆柱 | r≈25 | 108,938 |
| **合计** | | | **≈ 346,000** |

```text
I_zz ≈ 0.00035 kg·m²    （绕 CG，量级估计）
```

**注意**：`I_zz` 的估计非常粗略（组件实际形状未知），且与当前控制器中 `yaw_inertia = 0.52` 差了三个数量级。控制器中的值可能包含单位换算因子或仅为调试占位值。MBD 仿真建模时应使用上述估计值，并计划用实验校核。

## 惯量矩阵汇总

```text
J_CG = diag(I_xx, I_yy, I_zz)

     ≈ diag(0.019, 0.019, 0.00035)    [kg·m²]

IMU 偏移: r_IMU→CG = (0, 0, −94.6 mm) = (0, 0, −0.095 m)
```

在 MBD 仿真中：动力学方程绕 CG 写，用此 J_CG；IMU 模型在 CG 加速度上叠加 ω × r 项以模拟 IMU-CG 偏移。

---

# MBD 仿真模型总结

## 已具备（可直接建模）

| 子系统 | 模型 | 精度 |
|---|---|---|
| 质量 | m = 0.7546 kg | 精确 |
| 重心 | CG = (0, 0, −0.0946) m | 精确 |
| 惯量 I_xx, I_yy | ≈ 0.019 kg·m² | 一阶近似 |
| 惯量 I_zz | ≈ 0.00035 kg·m² | 量级估计，需校核 |
| 电机静推力 | Hammerstein 21 点查表 | RMSE ~1g |
| 电机动态 | 一阶滞后 τ = 0.177 s | 可用 |
| 推力矢量方向 | α/β 倾转角，小角度线性 | 几何精确，sign 待验证 |
| 推力力臂 | d = 0.201 m | 精确 |
| 舵机映射 | 脉宽 ↔ 倾角 | 精确 |

## 尚未具备（影响仿真保真度）

| 缺失项 | 对 MBD 的影响 | 建议 |
|---|---|---|
| I_xx/I_yy 精确值 | 姿态环固有频率偏移 | CAD 或三线摆实测 |
| I_zz 精确值 | 偏航环带宽不准 | 实测或保持鲁棒裕量 |
| 绑绳约束动态模型 | 姿态恢复力矩未知、摆动模态未知 | 先用球铰 + 重力矩近似，用 ident 数据校核 |
| 舵机带宽 / 延迟 | 高频相位滞后 | 查总线舵机协议或做 chirp 测试 |
| 桨叶气动效应 | 阻尼、陀螺力矩 | 先忽略，在线自适应补偿 |
| 传感器噪声模型 | 观测器 / Kalman 设计 | 录一段悬停 IMU 数据做 Allan 方差 |
| 姿态辨识离线分析 | 仿真模型无法验证 | 用已有 IDENT sample 日志做传递函数拟合 |

## 最小可行的 MBD 仿真路线

有了上述数据，可以搭建一个基础 6DOF 仿真：

```
输入: [T_cmd, α_cmd, β_cmd, τ_yaw_cmd]
  │
  ├─ 电机 Hammerstein:  T_cmd → 推力 T (带 τ=0.177s 滞后)
  ├─ 舵机运动学:        α_cmd, β_cmd → 推力方向 F_body
  ├─ 力臂力矩:          F_body × r (d=0.201m)
  ├─ 6DOF 刚体动力学:   绕 CG，J_CG ≈ diag(0.019, 0.019, 0.00035)
  ├─ 绑绳约束（简化）:   球铰 + 重力恢复矩 (pendulum 近似)
  ├─ IMU 模型:          CG 状态 + offset 变换 → IMU 仿真读数
  └─ 输出:              [姿态, 角速度, 加速度]
```

**自适应控制器**可以在此仿真中设计结构和初始参数。实物部署时必须：
- 在线辨识 / 自适应律处理绑绳约束的不确定性
- 用 IDENT step 响应校核仿真 vs 实物的一致性
- 保留参数调节接口，避免完全依赖开环前馈

---

## 后续仍可补充的信息

- 各组件的实际外形尺寸（用于精化 I_zz 和 I_xx/I_yy）
- CAD 模型或三线摆实测转动惯量
- 绑绳材料和松紧程度、是否可视为刚性杆
- 舵机带宽 / 延迟参数
- 离线分析 IDENT sample 日志，拟合 pitch/roll 轴传递函数并与仿真交叉验证
- 更多油门点下的重复推力实验，确认 tau=0.177s 的可复现性

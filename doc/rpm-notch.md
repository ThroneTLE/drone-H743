# 转速陷波（RPM notch）：桨叶振动从控制用陀螺里挖掉

作者 2026-09-27 决定。实现规格见任务卡（`.tmp/rpm_notch_spec.md`），本页只讲怎么用、看什么、有什么限制。

## 解决什么问题

光杆台架悬停时，陀螺 gx/gy 上有一条约 **107.5 Hz** 的桨叶振动线（gy 约 0.08 rad/s rms，两个桨各一条：107.55 / 109.5 Hz）。
它经角速度环的 P 项进入舵机指令，再被 50 Hz 的舵机帧**混叠**到 7～8 Hz——正落在回路 −180° 穿越（约 9.5 Hz）附近，
吃掉增益裕度。

陷波按双向 DShot 回传的电调转速实时跟踪这条线：中心 = eRPM ÷ (60 × 极对数)，每个 IMU 样本（1 kHz）滤一次，
**只给控制环用**。

| 用滤过的陀螺 | 仍用原始陀螺 |
|---|---|
| 同轴控制器（`drv_coax_ctrl` → 角速度环 P/I、角加速度、飞行日志 `ctrl_debug.omega`、遥测 rate_err） | 姿态融合 Fusion、差分角加速度 / 光流导航、姿态零点门 |
| 辨识 RATE / ANGLE 闭环里的 PID | 辨识记录的 gx/gy/gz、杆轴残差门、SERVO 模式 |
| | IMUCAP、`IMU?`、vofa、飞行日志 `imu` / `imu_raw`、验收 |

陷波关（或不可用）时，控制用陀螺与原始陀螺**逐位相同**，行为与没有这个功能时一模一样。

## 怎么用

**上位机**：「系统辨识 → 角速度 / 角度内环 → 高级设置 → 陷波（桨叶振动）」。
- 「读取状态」发 `RPMNOTCH ?`；页面空闲时（新连接后、每轮结束后）也会自动读一次。
- 「打开陷波（ON）」「关闭陷波（OFF）」：只在**已上锁、没有辨识在跑**时能点；灰掉时写明原因。飞控自己也会拒。
- 每轮开跑时飞控回一行 `SYSID NOTCH`，上位机存进该轮 `conditions.json` 的 `rpm_notch`，结果卡片第一行写
  「本轮陷波（桨叶振动）：开/关（开跑时状态，极对数）」。

**命令**（大小写敏感，只改 RAM，重新上电回到默认）：

```
RPMNOTCH | RPMNOTCH ?                 4 行状态
RPMNOTCH ON | OFF                     开 / 关
RPMNOTCH SET POLES <1..30>            极对数（默认 7；Betaflight 的 motor_poles=14 就是 7 对）
RPMNOTCH SET HARM <1..7>              谐波掩码 bit0=1x bit1=2x bit2=3x（默认 1 = 只滤 1x）
RPMNOTCH SET Q <1.5..10>              恒定 Q（默认 3.0）
RPMNOTCH SET MINHZ <20..200>          低于它权重 0（默认 50）
RPMNOTCH SET FADEHZ <5..100>          MINHZ 到 MINHZ+FADEHZ 线性淡入（默认 20）
RPMNOTCH DEFAULTS                     可调量回编译期默认（开关不动）
RPMNOTCH CLEAR                        计数与耗时最大值清零（任何时候都可以）
```

ON/OFF/SET/DEFAULTS 在已解锁时回 `RPMNOTCH event=rejected reason=armed`，辨识占着台架时回 `reason=sysid`；
范围不对回 `reason=range key=<项>`，格式不对回 `reason=usage usage=...`。成功时回完整的 4 行状态。

## 状态字段

```
RPMNOTCH cfg en= state= src= pp= harm= q_x100= min_hz= fade_hz= fs_nom= fs_x10= active=
RPMNOTCH esc ch1_erpm= ch1_hz_x10= ch1_age_ms= ch1_w_x100= ch2_erpm= ch2_hz_x10= ch2_age_ms= ch2_w_x100=
RPMNOTCH count stale= gap1= reset= nonfinite= slewclamp= reject= wdog=
RPMNOTCH time samples= spin= tracked= apply_us_avg_x100= apply_us_max= tick_us_avg_x100= tick_us_max=
```

- `state`：`off` 关；`unavailable` 本固件没有双向 DShot 转速（直通）；`fs_unknown` 陀螺名义 ODR 未知（直通）；
  `fs_bad` 实测采样率偏离名义超过 5%（淡出直通，回到 4% 以内恢复）；`idle` 开着但没有可跟踪的转速；`tracking` 正在滤。
- `src`：`bidir` = 有转速回传；`unavailable` = 没有。
- `fs_nom` 名义 ODR（BMI088 1000、BMI270 800）；`fs_x10` 按 IMU 时间戳估出的实际采样率 ×10。
- `chN_*`：电调通道 N（与桨的上下无关）。`hz_x10` 机械转频 ×10，`age_ms` 回包年龄（9999 = 从未收到），`w_x100` 掩码里最低那个谐波的陷波当前权重 %（默认 `HARM 1` 就是 1x；`HARM 2/4/6` 没有 1x，报 2x 或 3x 的）。
- 计数：`stale` 新鲜→过期次数；`gap1` 丢一个 IMU 样本（补中点）；`reset` 大缺口/时间戳倒退/帧复位；
  `nonfinite` 非有限输入；`slewclamp` 中心变化被限速（>3000 Hz/s，防坏帧）；`reject` eRPM > 150000 被拒；
  `wdog` 控制拍停超过 40 ms、全部淡出。
- `samples`/`spin`/`tracked`：IMU 样本数 / 其中至少一个电机的最低谐波落在全权重频段（`min_hz+fade_hz` 到 0.40·fs）的样本 / 其中所有这样的电机该谐波权重都是 100% 的样本。
  `tracked/spin` 是"转着的时候陷波真的在滤"的比例。
- 耗时：每样本滤波与每拍策略的平均（×100）与最大 µs，包含中断抢占。

## A/B 做法（硬件，需作者授权烧录）

烧 `build/Debug`（`-DESC_PROTOCOL=DSHOT300_BIDIR`），`RPMNOTCH ?` 必须是 `src=bidir` 且两路 `age_ms` 很小。

1. **H1（陷波关）确认极对数与采样率**：SYSID 模式 0、`SYSID RATE 500`，6.0 / 7.4 / 8.5 N 各至少 8 s，一块电池内。
   用 0.5 s 窗口的线峰值对 `erpm/(60·7)`、`erpm_lower/(60·7)`：斜率 1 ± 1%、每桨残差 ≤ 0.5 Hz、eRPM/840 处没有线。
   记下 `fs_x10`；悬停时抓一段 1 kHz IMUCAP 看 2x/3x 大小，决定 HARM。
2. **H2（台架 A/B）**：RATE 模式、双脉冲 100 mrad/s、8 s、目标 740 cN、ψ 1570 mrad（同 rod_035506）。
   关 / 开 / 关 / 开 交替、同一块电池；每个"开"轮前 `RPMNOTCH CLEAR`，轮后 `RPMNOTCH ?`。
   通过：力矩 100–115 Hz rms 降 ≥ 12 dB；7–10 Hz 舵机混叠峰降 ≥ 10 dB；0.3–10 Hz 跟踪误差不劣于轮间离散；
   `tracked/spin` ≥ 99%；计数约为 0；`conditions.json` 的 `rpm_notch.en` 与每轮一致；`apply_us_max` < 10、`tick_us_max` < 20。
3. H2 通过后由主控把 `APP_RPM_NOTCH_DEFAULT_ENABLE` 改成 1；H1 残差 ≤ 0.5 Hz 时可把 Q 提到 5
   （测试会逼着同步更新 `tools/sysid/tune.py` 的 `DEPLOYED_RPM_NOTCH`）。飞行 A/B 另行授权。

## 限制

- **阶段一只在 RAM**：上电默认**关**；没有 Flash 持久化、没有 CFG 版本变化、没有 `coax.*` 参数。
- **非双向构建（CMake 默认 DSHOT300、PWM）没有转速**：状态报 `unavailable`、逐位直通、**不挡解锁**。
- 只滤 1x（默认）。现有 80 Hz 低通在 215 Hz 已有 −19.8 dB；若将来提高低通截止，应同时打开 2x。
- 陷波只能挖掉建了模的线。芯片抽取之前（>500 Hz）折叠下来的混叠，它挖不掉；要靠 1 kHz IMUCAP 核实。
- 控制带内的代价：两个 Q3 陷波在最坏位置（70 Hz）时 10 Hz 处 −5.5° 相位；现行增益的增益裕度 8.66 → 8.42 dB。
  整定器已把它计入（`tools/sysid/tune.py` 的 `DEPLOYED_RPM_NOTCH`），所以新算出的 kp/ki 会比不计陷波时略小（约 −2.7% / −5%）。
- 辨识记录是陷波**之前**的陀螺：拟合出的纯延迟 T 含 80 Hz 低通、不含陷波，所以整定模型单独乘陷波。
- 差分角加速度（光流导航的 IMU 权重）仍被振动驱动；是否也改用滤过的陀螺是后续单独的 A/B。

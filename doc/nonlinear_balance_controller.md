# 非线性平衡式同轴倾转控制器

本文档对应分支 `feat/nonlinear-balance-controller`。该分支与
`fix/controller` 上的“力方向前馈 + 直接角度 P”是两套独立控制器，
不能把两边的姿态增益直接互换。

## 1. 控制目标

控制器采用类似平衡车的优先级：

1. 水平摇杆给速度目标，不直接给舵机角。
2. 速度误差形成期望水平加速度和期望推力方向。
3. 期望推力方向形成期望机体姿态。
4. 姿态误差形成物理力矩。
5. 物理力矩通过推力、效率和实际力臂精确反解倾转角。
6. 稳态无外部阻力时，机体本身保持所需倾角，舵机回到中位。

Z 高度环、偏航控制和双电机推力分配保持原路径。

## 2. 坐标与已知物理量

控制器局部坐标为 X 向前、Y 向右、Z 向下。高度仍使用
`z = -height`。当前 IMU roll 约定只在本控制器构造姿态矩阵时补偿：

```text
roll_force_frame_sign  = -1
pitch_force_frame_sign = +1
```

使用的实测或辨识参数为：

| 参数 | 数值 |
|---|---:|
| 质量 m | 1.367 kg |
| Ixx, Iyy | 0.051 kg*m^2 |
| Izz | 0.00035 kg*m^2（几何粗估，未实测） |
| Pitch 力臂 | 0.145 m |
| Roll 力臂 | 0.105 m |
| Pitch 有效系数 | 0.569 |
| Roll 有效系数 | 0.581 |
| 舵机倾转限制 | 18 deg |

力臂是重心到对应舵机转轴的距离，不是重心到桨盘的 `D=0.250 m`。

## 3. 速度参考模型

定义速度误差为：

$$
e_v=v-v_d
$$

积分状态为：

$$
\dot\xi=e_v
$$

期望加速度为：

$$
a_d=\dot v_d-K_v e_v-K_i\xi
$$

当前默认值：

$$
K_v=2\zeta_v\omega_v=0.50,\qquad
K_i=\omega_v^2=0.0625
$$

对应 $\zeta_v=1$、$\omega_v=0.25\,rad/s$。`reference.ax_m_s2` 和
`reference.ay_m_s2` 是 $\dot v_d$ 前馈；遥控器当前给常值速度时二者为零。
水平加速度矢量限制为 `2.0 m/s^2`。

这里“临界阻尼”指闭环双极点。PI 参考通道还包含一个零点，因此速度阶跃
仍有确定的 $e^{-2}=13.53\%$ 超调；这已由离散模型测试约束，不能把它解释为
无超调参考模型。

## 4. 期望推力方向

局部坐标中的期望推力为：

$$
F_d^N=m
\begin{bmatrix}
a_{d,x}\\
a_{d,y}\\
g-a_{d,z}
\end{bmatrix}
$$

定义期望推力轴：

$$
b_{3T}=\frac{F_d^N}{\lVert F_d^N\rVert}
$$

再使用期望偏航构造航向向量：

$$
b_{1c}=
\begin{bmatrix}
\cos\psi_d & \sin\psi_d & 0
\end{bmatrix}^T
$$

$$
b_{2T}=\frac{b_{3T}\times b_{1c}}
{\lVert b_{3T}\times b_{1c}\rVert},\qquad
b_{1T}=b_{2T}\times b_{3T}
$$

于是期望推力坐标系为：

$$
R_{Td}=\begin{bmatrix}b_{1T}&b_{2T}&b_{3T}\end{bmatrix}
$$

该计算使用归一化、叉乘和三角函数，没有使用小角度近似。

## 5. 舵机与期望机体姿态耦合

倾转机构的精确旋转矩阵为：

$$
R_g=R_y(\alpha)R_x(\beta)=
\begin{bmatrix}
c_\alpha&s_\alpha s_\beta&s_\alpha c_\beta\\
0&c_\beta&-s_\beta\\
-s_\alpha&c_\alpha s_\beta&c_\alpha c_\beta
\end{bmatrix}
$$

推力方向是其第三列：

$$
n_B=
\begin{bmatrix}
\sin\alpha\cos\beta\\
-\sin\beta\\
\cos\alpha\cos\beta
\end{bmatrix}
$$

机体和倾转机构共同满足：

$$
R_d R_g=R_{Td}
$$

所以期望机体姿态为：

$$
R_d=R_{Td}R_g^T
$$

这一步是平衡式控制器的关键。舵机产生姿态力矩时，期望机体姿态会同步
补偿舵机倾角，使最终推力方向仍指向 `F_d`。

## 6. SO(3) 姿态力矩

姿态误差和角速度误差为：

$$
e_R=\frac{1}{2}\operatorname{vee}(R_d^TR-R^TR_d)
$$

$$
e_\omega=\omega-R^TR_d\omega_d
$$

期望物理力矩为：

$$
M_d=-K_Re_R-K_\omega e_\omega+\omega\times J\omega
$$

其中 `Izz=0.00035 kg*m^2` 只是现有几何量级估计，尚未通过实测辨识。
它只进入 $\omega\times J\omega$ 三轴耦合前馈；首次联调必须从低偏航角速度
开始，用 FlightLog 检查该项方向和量级。

当前初始物理增益：

| 轴 | KR [N*m/rad] | Kw [N*m*s/rad] |
|---|---:|---:|
| Roll | 0.0671 | 0.1104 |
| Pitch | 0.0660 | 0.1138 |

这些数值使用 `Ixx=Iyy=0.051 kg*m^2` 形成约 `1.14 rad/s` 的姿态固有频率，
并给出接近临界阻尼的名义阻尼比。它们是有物理依据的初始值，不代表实机
飞行验证结论。

## 7. 物理力矩到舵机角的精确反解

令总推力为 $T=\lVert F_d\rVert$。当前实机校正后的力矩分配极性为
`s_roll=-1`、`s_pitch=-1`；极性只决定恢复方向，不参与下述增益幅值设计：

$$
\beta=\arcsin\left(
\frac{M_x}{s_{roll}\eta_{roll}l_{roll}T}
\right)
$$

$$
\alpha=\arcsin\left(
\frac{M_y}
{s_{pitch}\eta_{pitch}l_{pitch}T\cos\beta}
\right)
$$

反解的正弦输入先限制到 $\pm\sin18^\circ$，因此输出严格限制到
`+-18 deg`。由于分母包含实时推力 $T$，同一姿态误差在不同油门下会自动
得到不同舵机角。这就是此结构需要的动态增益，不需要另外写分段动态 Kp。

控制器每周期执行两次：

```text
Rg -> Rd -> eR/ew -> Md -> alpha/beta
```

用于处理期望机体姿态和舵机倾角之间的耦合。

## 8. 姿态优先保护

以下任一条件会冻结速度积分，并按余量连续缩小水平加速度命令：

| 条件 | 开始削减 | 完全削减 |
|---|---:|---:|
| 光流速度无效 | 立即 | 立即 |
| 水平姿态误差 | 12 deg | 25 deg |
| 倾转力矩使用率 | 75% | 100% |
| 总推力使用率 | 92% | 100% |

保护只削减水平命令，姿态恢复力矩和 Z 高度控制仍继续工作。低油门退出稳定
混控或 IMU 控制失效时，任务层调用 `DRV_COAX_CTRL_ResetState()` 清空速度积分。

## 9. 参数与日志

为保持现有 MCP/VOFA 调参接口，参数名不变：

```text
coax.roll_angle_kp
coax.pitch_angle_kp
coax.roll_rate_kd
coax.pitch_rate_kd
```

但在本分支中它们的单位已经是上表的物理力矩增益。Flash 配置版本提升到
`14`，旧控制器参数不会自动载入。`PARAM SET`、`PID SET` 和 VOFA 滑块均使用
正的物理增益，例如 `PARAM SET coax.roll_angle_kp 0.0671`；App 层会按既有接口
映射为驱动内部负值，参数查询再映射回正值。

FlightLog 版本提升到 `3`，记录新增：

```text
velocity_integral_m
desired_attitude_rpy_rad
attitude_error
rate_error_rad_s
moment_cmd_n_m
horizontal_command_scale
moment_utilization
thrust_utilization
protection_flags
```

这些数据用于检查方向、饱和、积分冻结和模型误差，不能把“固件构建通过”
等同于“实机方向与飞行稳定性已验证”。首次上电必须先拆桨或可靠限位，逐轴
验证恢复方向，再进行低推力系留测试。

## 10. 2026-07-25 全供电舵机模型与 PD 启动值

在已安装机构、双电机停止、舵机全供电状态下，以 `100 Hz` 单轴 `PRAD`
反馈完成 `+-200 us` 阶跃。`500..2500 us` 对应 `0..180 deg`。联合大信号
FOPDT 模型为：

| 控制轴 | 实际舵机 | K | 延迟 L | 上升 tau | 下降 tau | R2 |
|---|---|---:|---:|---:|---:|---:|
| Roll | alpha/index0/ID1 | 0.781794 | 0.016231 s | 0.071236 s | 0.306416 s | 0.963346 |
| Pitch | beta/index1/ID2 | 0.769332 | 0.041320 s | 0.076881 s | 0.057051 s | 0.994138 |

alpha 舵机的下降方向约 `0.306 s`，四次大阶跃都复现，因此 Roll 轴必须按
该最慢方向限制带宽。反馈中位分别约为 `1457.18 us` 和 `1509.53 us`；这是
舵机编码器参考值，不应直接当作机械中位补偿量。

小角度线性化时，令

$$
C=\eta lT
$$

由于期望机体姿态包含指令倾角，力矩反解形成隐式增益：

$$
G_i=\frac{C}{C-K_R}
$$

纳入舵机增益、延迟、一阶极点、`80 Hz` 陀螺滤波和惯量后，开环模型为：

$$
L(s)=\frac{K_a e^{-Ls}}{1+\tau s}\rho
\frac{C}{C-K_R}\frac{K_R+K_\omega sH_g(s)}{Is^2}
$$

稳健筛选使用：`T=5.5..15.644959 N`、`I=0.051+-10% kg*m^2`、物理/分配
效率比 `rho=0.75..1.25`，并覆盖两个方向的舵机时间常数和额外 `20 ms`
延迟。门槛为最坏相位裕度不低于 `50 deg`、增益裕度不低于 `10 dB`。

首轮无支架测试建议使用：

| 轴 | Kp/实机界面 [N*m/rad] | Kd/实机界面 [N*m*s/rad] | 最坏 PM | 最坏 GM |
|---|---:|---:|---:|---:|
| Roll | 0.020 | 0.065 | 51.08 deg | 24.20 dB |
| Pitch | 0.040 | 0.090 | 57.61 deg | 17.79 dB |

对应名义交越频率约为 Roll `1.05 rad/s`、Pitch `1.47 rad/s`。这组数值是
模型验证后的保守调试起点，不是飞行认证值。实机应先逐轴确认恢复方向，再
按 FlightLog 的实际舵机角、力矩利用率和角速度峰值决定是否提高带宽。

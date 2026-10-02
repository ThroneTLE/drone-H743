#ifndef DRV_AIRFRAME_PARAMS_H
#define DRV_AIRFRAME_PARAMS_H

#include <stdint.h>

/*
 * 机体模型：**唯一来源是 Flash**，没有编译期兜底。
 *
 * 这些数是量出来的，会随装配变化（换电池、加挂载、改倾转机构）。以前它们是
 * `drv_airframe_model.h` 里的一堆 `#define`，每改一次都要重新编译烧录；更糟的是
 * 同一个量在代码里存了两份而且**对不上**——四个部件质量加起来 754.6 g，
 * 而 `DRV_AIRFRAME_MASS_KG` 写的是 1.3670 kg；重心 −0.0946 m 按前者算，
 * 重量 13.41 N 按后者算，两套并存，谁也没报错。
 *
 * 所以现在的规矩是：**代码里不再保留任何一份机体数据的副本。** 参数只从 Flash 读，
 * 由上位机写入。没有第二个来源，就不会有两个来源打架。
 *
 * ──────────────── 空参数区怎么办 ────────────────
 *
 * 一块刚烧完固件、还没写过机体数据的板子，`valid` 为 0，所有字段是零。
 * 这种状态下**禁止解锁**（见 DRV_Airframe_IsValid 的调用方）。
 *
 * 这是刻意的取舍：没有质量、没有惯量、没有力臂，控制律算出来的每一个力矩都没有
 * 物理含义。与其塞一组"出厂默认值"让它看起来能飞——那正是上一版埋下数据矛盾的
 * 方式——不如干脆拒绝起飞，并在诊断里说明缺什么。**失效要朝安全的方向倒。**
 *
 * ──────────────── 输入与派生的分界 ────────────────
 *
 *   **输入**  —— 拿尺和秤能直接量到的：各部件质量与其重心 z、几何距离、惯量。
 *   **派生**  —— 由输入算出来的：总质量、整机重心、重量、悬停油门百分比等。
 *
 * `derived_auto` 决定派生值怎么来：
 *   1（推荐）—— 由输入自动重算，写它们会被拒绝。量准之后应当用这一档。
 *   0        —— 派生值保持被写入的样子。留这个口子是因为有些量（例如整机惯量）
 *                可能来自双线摆实测而不是部件表推算，硬要它服从推算反而是错的。
 *                上位机必须把这一档显式标出来，别让人以为数字是算出来的。
 *
 * 上位机用 `DRV_Airframe_ComputeDerived()` 可以在不改变任何状态的前提下算出"若自动
 * 会是多少"，把现值与自动值并排显示——差多少一眼可见。
 */

typedef struct {
    /* ──────── 输入：部件质量 [g] 与各自重心的 z 坐标 [m]（原点为板中心，z+ 向上） */
    float board_mass_g;
    float battery_mass_g;
    float base_mass_g;
    float servo_motor_mass_g;
    float board_cg_z_m;
    float battery_cg_z_m;
    float base_cg_z_m;
    float servo_motor_cg_z_m;

    /* ──────── 输入：几何 [m] */
    float imu_z_m;
    float prop_plane_d_m;
    float roll_axis_to_prop_plane_m;
    float pitch_axis_to_prop_plane_m;

    /*
     * ⚠ 已退役（2026-09-27）。**不要读它，不要写它，不要按它的名字理解它。**
     *
     * 这两个数曾是俯仰/横滚的"推力力臂"（都填 0.145 m），控制律再乘上
     * DRV_COAX_CTRL_PITCH/ROLL_EFFECTIVENESS（0.569/0.581）当倾转力矩的大小。
     * 它们出自 2026-07-25 的辨识，量的是**旧机体（1.367 kg）**上"重心到舵机转轴"
     * 的距离；机体换成 0.7546 kg、重心移到 −0.0946 m 之后没人更新过，于是固件把
     * 俯仰/横滚的倾转权限高估了约 2.3 倍（2026-09-27 光杆辨识 + 直接测量）。
     *
     * 现在力臂不再是一个可以单独填的数，而是两个实测高度相减：
     *     r_z = servoN_axis_z_m − cg_z_m
     * （推力作用线穿过舵机转轴，推导见 drv_coax_ctrl.c 倾转力矩那一段）。
     * 没有经验系数可调，也就不会再有一份和几何对不上的副本。
     *
     * 字段本身保留是因为它是 **Flash ABI**：`DRV_Airframe_Params` 整体嵌在 CFG
     * 记录里，删掉会让 v20 起每一版记录的字节布局变化，迁移读取器再也认不出用户
     * 板子上已经存着的那份配置。旧记录里的值照原样读进来、存回去，但没有任何代码
     * 再读它。名字加了 `retired_` 前缀，好让任何一处漏改的旧代码编译失败，而不是
     * 安静地继续读一个和几何对不上的数。参数表里已经没有这两个名字，
     * `PARAM SET` 写它们会被拒。
     */
    float retired_pitch_thrust_lever_arm_m;
    float retired_roll_thrust_lever_arm_m;

    /*
     * 两个倾转舵机转轴的高度 [m]（原点板中心，z+ 向上，在板下方填负数）。
     *   servo1 = alpha 通道 = PWM ch1，管左右倾 → 横滚力矩；
     *   servo2 = beta  通道 = PWM ch2，管前后倾 → 俯仰力矩。
     *
     * 2026-09-27 起它们是**控制关键量**：倾转力矩的力臂就是
     * r_z = servoN_axis_z_m − cg_z_m，力矩的大小和方向都由它定。为 0（= 没填）、
     * 离重心不到 DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M、或与推力点不在重心同侧时
     * 禁止解锁（见 DRV_Airframe_FirstInvalidName）。
     */
    float servo1_axis_z_m;
    float servo2_axis_z_m;
    float thrust_point_z_m;
    float tether_attach_z_m;
    float tether_rope_m;

    /* ──────── 输入：惯量 [kg·m²]、旋向、环境与执行器标度 */
    float ixx_kgm2;
    float iyy_kgm2;
    float izz_kgm2;

    /*
     * ⚠ 已退役（2026-09-13）。**不要读它，不要写它，不要按它的名字理解它。**
     *
     * 它曾经是下桨旋向，偏航极性由它推导。但那个值不是量出来的，是从调参现象
     * **反推**的：偏航角速度环 Kp 加大会抖振而不是发散 → 闭环是负反馈 →
     * 反推出下桨俯视顺时针 = -1。推理自洽，可它证明的只是"整条链的符号彼此不
     * 矛盾"，不是"桨真的往那边转"——换一套增益、或者把某处符号和它一起翻过来，
     * 现象一模一样，而飞机的偏航方向已经反了。
     *
     * 现在旋向与 ESC 通道归属都由上位机标定，存在 `Driver/Inc/drv_prop_map.h`：
     * 人通电点一下电机，看着它转，把事实填进去。参数表里已经没有这个名字，
     * `PARAM SET` 写它会被拒。
     *
     * 字段本身保留是因为它是 **Flash ABI**：`DRV_Airframe_Params` 整体嵌在 CFG
     * 记录里，删掉会让 v20/v21 记录的字节布局变化，迁移读取器再也认不出用户
     * 板子上已经存着的那份配置。名字加了 `retired_` 前缀，好让任何一处漏改的
     * 旧代码编译失败，而不是安静地继续读一个没人维护的数。
     */
    float retired_lower_rotor_spin_sense;
    float gravity_m_s2;
    float max_total_thrust_g;       /* 双桨合计最大推力 */
    float servo_deg_per_us;

    /* ──────── 派生：由上面算出，见 DRV_Airframe_ComputeDerived() */
    float mass_kg;
    float cg_z_m;
    float weight_n;
    /*
     * 推力点（双桨中点）到重心。2026-09-27 起它**不再**决定倾转力矩——力矩看的
     * 是推力作用线穿过的舵机转轴（见 servo1/2_axis_z_m）。它留在解锁闸门里当
     * 独立的交叉核对：舵机转轴与推力点必须在重心同侧，否则多半是某一个的符号
     * 填反了。
     */
    float thrust_point_to_cg_z_m;
    float tether_attach_to_cg_m;
    float tether_rod_to_cg_m;
    float max_total_force_n;
    float hover_thrust_percent;
    float servo_us_per_deg;

    /* 派生值是否自动重算。1 = 自动（推荐），0 = 保持写入值。 */
    float derived_auto;

    /*
     * ──────── 输入：光流模块安装方向（CFG v26 起，R-FLOWMOUNT-1）
     *
     * 光流"芯片坐标 → 机体 FLU"的换算在 app_optical_flow.c 的 app_flow_fill_sample()
     * 里写死成 FRD→FLU（X 同号、Y 取负），那是按**最初的安装**推出来的。模块换了
     * 安装位置（转了 90° 之类）以后，那条换算得到的已经不是真机体 FLU。
     *
     * 这两项就是把"按旧安装换算出的 FLU 读数"纠正到真实机体 FLU 的变换：
     * 先按 mirror 翻 Y，再绕 +Z（向上）逆时针转 yaw。
     *   flow_mount_yaw_deg：只接受 0 / 90 / 180 / 270；
     *   flow_mount_mirror ：只接受 0 / 1。
     * 默认（及从 v25 及更早迁移来的记录）都是 0/0，即恒等变换——光流链与加入
     * 这两项之前逐位相同。
     *
     * 用 float 存只是为了和整张参数表同一种访问方式（按偏移读写 float），不代表
     * 它是连续量：PARAM SET 写别的值会被拒（见 DRV_Airframe_SetParam）。
     * 它们只影响光流方向，**不进解锁闸门**（DRV_Airframe_FirstInvalidName 不看）。
     * 追加在结构体尾部是 Flash ABI 的要求：前面 36 项原位不动，v20～v25 记录的机体
     * 块就是它的前缀（冻结布局见 app_control_config_compat.h）。
     */
    float flow_mount_yaw_deg;
    float flow_mount_mirror;
} DRV_Airframe_Params;

/*
 * 倾转转轴离重心至少要有多远 [m]。低于它倾转几乎产生不了力矩，力矩反解会
 * 一路顶到倾角限位——那不是"权限小"，是"没有权限还假装有"。1 cm 远小于任何
 * 能飞的构型（2026-09-27 板上部件表重心算出的力臂 3.5 cm；作者随后实测重心
 * −0.01 m，力臂约 12 cm），只用来挡住"量错/填错"那一类情形。控制律的倾转反解也用同一个下限：低于它
 * 或机体模型无效时不驱动舵机（回中），见 drv_coax_ctrl.c。
 */
#define DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M 0.01f

/*
 * 机体模型是否可用。0 = Flash 里没有有效记录，或记录里有物理上不可能的值
 * （质量为零、惯量为零、重力为零、倾转转轴没填或方向矛盾……）。为 0 时必须禁止解锁。
 */
uint8_t DRV_Airframe_IsValid(void);

/*
 * 逐项体检，返回第一个不合格项的名字；全部合格返回 NULL。供诊断如实回报。
 *
 * 单字段判据返回字段名本身（"airframe.mass_kg"）。倾转转轴的两条复合判据
 * 返回"该去改的那个字段名 + 冒号 + 原因"，用户一眼知道改哪一项、为什么：
 *   "airframe.servo1_axis_z_m:near" —— |转轴 z − 重心 z| < DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M
 *   "airframe.servo1_axis_z_m:sign" —— 转轴与推力点不在重心同侧
 * （servo2 同理）。原因词刻意取短：这个名字要塞进 ARM 报文那一行，
 * 行缓冲只有 APP_UART_TX_TEXT_SIZE（256）字节，不能比现有最长的字段名更长
 * （最坏整行现为 250 字符、余量 5，这两个数由 tests/test_arm_status_line_budget.py
 * 按真实格式串算出并核对）。
 * ±Inf 与 NaN 一样判不合格：单字段项报字段名，转轴复合项报 :near。
 */
const char *DRV_Airframe_FirstInvalidName(void);

/*
 * 倾转转轴相对整机重心的 z 偏移 r_z = servoN_axis_z_m − cg_z_m [m]，
 * 即 τ = r × F 里的 r（推导见 drv_coax_ctrl.c）。负值 = 转轴在重心下方。
 * Roll 用 1 号舵机转轴，Pitch 用 2 号舵机转轴。控制律与解锁闸门共用这一处定义。
 */
float DRV_Airframe_RollTiltAxisToCgZ(const DRV_Airframe_Params *p);
float DRV_Airframe_PitchTiltAxisToCgZ(const DRV_Airframe_Params *p);

/* 清空为"未写入"状态。上电时若 Flash 无记录即为此状态。 */
void DRV_Airframe_Clear(void);

/* 只读视图，供控制律取值——热路径上不拷贝整个结构体。永不返回 NULL。 */
const DRV_Airframe_Params *DRV_Airframe_Get(void);

void DRV_Airframe_GetParams(DRV_Airframe_Params *out);

/*
 * 整体写入（持久化层加载 Flash 记录时调用）。写入后自动体检并更新 valid；
 * derived_auto 非零时顺便重算派生值，保证存进去的和算出来的一致。
 * 光流安装两项不在允许集合里时落回 0（恒等），读回的就是实际生效的。
 */
void DRV_Airframe_SetParams(const DRV_Airframe_Params *in);

/*
 * 光流安装方向的规范读取（给热路径用，不查字符串表）。
 * 返回值只可能是 0/90/180/270 与 0/1；结构体里若是别的数（只可能来自绕过
 * DRV_Airframe_SetParam 的整体写入）一律按 0 处理——即恒等变换，与没有这两项
 * 之前的行为相同。p 为 NULL 同样返回 0。
 */
uint16_t DRV_Airframe_FlowMountYawDeg(const DRV_Airframe_Params *p);
uint8_t DRV_Airframe_FlowMountMirror(const DRV_Airframe_Params *p);

/*
 * 具名读写，名字形如 "airframe.battery_mass_g"。
 * 返回 0 表示名字不认识；写派生值且 derived_auto 为 1 时同样返回 0（拒绝）。
 * airframe.flow_mount_yaw_deg 只收 0/90/180/270、airframe.flow_mount_mirror 只收
 * 0/1，其他值返回 0（拒绝），原值不动。
 * 写入后按**新的** derived_auto 决定是否重算：把 derived_auto 从 0 写成 1
 * 当场重算全部派生值（含重心，也就含倾转力臂），不等重启。
 */
uint8_t DRV_Airframe_GetParam(const char *name, float *value);
uint8_t DRV_Airframe_SetParam(const char *name, float value);

/* 遍历用：上位机据此自动生成表单，不必在两边各维护一份名字清单。 */
uint32_t DRV_Airframe_GetParamCount(void);
const char *DRV_Airframe_GetParamNameAt(uint32_t index);

/* 该名字是不是派生值（上位机据此标注"算出来的"并默认置灰）。 */
uint8_t DRV_Airframe_IsDerivedName(const char *name);

/*
 * 按 `in` 的输入字段算出派生字段写进 `out`，**不触碰任何全局状态**。
 * `out` 的输入字段原样复制自 `in`。可以 in == out。
 * 上位机用它做"若切到自动会是多少"的预览。
 */
void DRV_Airframe_ComputeDerived(const DRV_Airframe_Params *in,
                                 DRV_Airframe_Params *out);

#endif /* DRV_AIRFRAME_PARAMS_H */

#ifndef DRV_SYSID_RECORD_H
#define DRV_SYSID_RECORD_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SYSID v3: 16-byte header, seven 30-byte samples (226-byte payload,
 * 235 bytes with $X framing). At 250 Hz: about 8.39 kB/s; use USB.
 * 100 Hz: about 3.36 kB/s, within the 57600-baud default budget.
 * Fields include final pulse-derived torque, angle target and per-sample
 * microsecond offset. v1's nominal dt cannot represent real control jitter.
 * v3 appends lower-rotor eRPM and the scalar servo tilt along the rod axis;
 * the first 13 fields keep v2's order and scale, and `erpm` now carries the
 * real upper-rotor eRPM (v2 always wrote 0).
 * The field table/hash is authoritative; host readers retain v1 offline support.
 *
 * 高度辨识（R-ALTID-1，2026-09-29）在表尾再追加 7 个字段（height / height_raw /
 * height_sp / vz / vz_sp / az / vbat，所有模式都带，非 ALT 轮为 0）：每条 30 → 44 字节，
 * 满批 7 → 5 条（16 + 5*44 = 236 字节净荷，成帧 245）。250 Hz 约 12.3 kB/s（走 USB）；
 * 100 Hz 约 4.9 kB/s——蓝牙 115200 约 43%，数传 57600 约 85%，数传上宜 <= 80 Hz。
 * 版本号**不升**，仍是 3：主机本来就按 SYSID SCHEMA 的字段表逐字段解码，字段表一变
 * schema_hash 就变，旧表硬解会当场被拒；而上位机 tools/sysid/decode.py 只认 1..3，
 * 单升这里会让现有上位机连新表也解不了。要升 4 须与上位机同批改。
 */

typedef enum {
    DRV_SYSID_RECORD_OK = 0,
    DRV_SYSID_RECORD_INVALID,
    DRV_SYSID_RECORD_TOO_SMALL
} DRV_SysIdRecordStatus;

#define DRV_SYSID_RECORD_VERSION      3U
#define DRV_SYSID_RECORD_HEADER_BYTES 16U
#define DRV_SYSID_RECORD_BYTES        44U
/* 16 + 5*44 + 9 = 245 <= APP_UART_TX_TEXT_SIZE (256).
 * A sixth record would require 289 bytes. */
#define DRV_SYSID_RECORD_MAX_COUNT    5U
#define DRV_SYSID_RECORD_MAX_FRAME \
    (DRV_SYSID_RECORD_HEADER_BYTES + \
     (DRV_SYSID_RECORD_BYTES * DRV_SYSID_RECORD_MAX_COUNT))

/* flags 位 */
#define DRV_SYSID_FLAG_FIRST_BATCH  0x0001U  /* 本趟的第一批 */
#define DRV_SYSID_FLAG_LAST_BATCH   0x0002U  /* 本趟的最后一批 */
#define DRV_SYSID_FLAG_ABORTED      0x0004U  /* 本趟是被 abort 结束的 */
/*
 * 本批每一条的上/下桨 eRPM 都来自新鲜有效的电调回包（含"明确未旋转"= 0）。
 * 不带它时 erpm/erpm_lower 里的 0 可能只是"没有数据"，不能当成停转。
 */
#define DRV_SYSID_FLAG_ERPM_VALID   0x0008U
/*
 * 本批之前丢过样本（环满）。
 *
 * 必须显式报出来，不能让主机自己从时间戳里猜：线上时间是 `base + k·dt` 的
 * 压缩形式，丢样之后如果只是悄悄接着发，主机看到的是一段**时间被压缩过**的
 * 波形——而这套辨识测的正是时移。一个没报出来的 GAP 会直接变成一个假的延迟值。
 */
#define DRV_SYSID_FLAG_GAP          0x0010U
#define DRV_SYSID_FLAG_RATE         0x0020U
#define DRV_SYSID_FLAG_ANGLE        0x0040U
/* SERVO 模式（电机不转，只动舵机）：输入是 servo_tilt，torque/thrust 恒为 0，
 * 不能当 FF 数据去拟合惯量。 */
#define DRV_SYSID_FLAG_SERVO        0x0080U
/* 高度辨识（ALT，模式 4）：输入是总推力 / 高度参考，姿态环只把杆轴角度保持在 0。 */
#define DRV_SYSID_FLAG_ALT          0x0100U
/*
 * 本批至少有一条样本的电机脉宽被"最高油门 %"封顶（目前只有 ALT 轮会置）。
 * 封顶时 thrust 记的是封顶后实际下发脉宽对应的推力，不是高度环要的那个值。
 */
#define DRV_SYSID_FLAG_THRUST_CAPPED 0x0200U
/*
 * 水平槽 XY 辨识（SYSID MODE XY，模式 5）：推力恒为托住机体的 target_n，输入是沿槽的倾角偏置；
 * 尾部 7 个字段含义见 doc/sysid-xy-contract.md（height=沿槽位置、vz=沿槽速度等）。
 */
#define DRV_SYSID_FLAG_XY           0x0400U
/*
 * 吊绳偏航辨识（SYSID MODE YAW，模式 6）：推力恒为配置的总推力 F（< 机重），输入是上下桨差速（ΔT）或偏航角速度参考；
 * 尾部 7 个字段含义见 doc/sysid-yaw-contract.md（height=ψ、vz=陀螺 z、vz_sp=r、az=ΔT 等）。
 */
#define DRV_SYSID_FLAG_YAW          0x0800U

/*
 * 物理量形态的一条样本。打包时按各字段的固定缩放转成定点，**饱和而不是回绕**：
 * 回绕会把一个超量程的角速度变成一个反号的小值，那比丢数据危险得多。
 */
typedef struct {
    float gyro_rad_s[3];      /* 规范 FLU 机体角速度 */
    float omega_sp_rad_s;     /* 绕台架杆轴的期望角速度 */
    float alpha_ff_rad_s2;    /* 期望角加速度（前馈用） */
    float tilt_cmd_x_rad;     /* 下发的机体倾转 X */
    float tilt_cmd_y_rad;     /* 下发的机体倾转 Y */
    float thrust_n;           /* 当前总推力估计 */
    float angle_rad;          /* 绕杆轴转角 */
    uint32_t erpm;            /* 上桨电转速；无数据/过期/未旋转时填 0 */
    float torque_n_m;         /* final pulse-derived moment about rod, not measured torque */
    float angle_sp_rad;       /* angle verification target; zero in feedforward/rate */
    uint32_t offset_us;       /* actual acquisition offset from batch base, <=65535 */
    uint32_t erpm_lower;      /* v3：下桨电转速，口径同 erpm */
    /*
     * v3：沿杆轴的标量倾转指令 [rad]，由下发的机体倾转投影得到：
     *     servo_tilt = tilt_x·sinψ·sgn(L_pitch) + tilt_y·cosψ·sgn(L_roll)
     * 正值 = "有推力时会产生绕杆轴正力矩"的那个倾转方向（定义见 app_sysid.h）。
     */
    float servo_tilt_rad;
    /*
     * 高度辨识（R-ALTID-1）追加，所有模式都带、非 ALT 轮为 0（来源见 app_sysid_alt.h）：
     * 控制用高度、TOF 原始测距、高度参考、垂直速度、速度参考（含 v_inj 前馈）、
     * IMU 竖直运动加速度（向上为正、已去 g）、电池电压。
     */
    float height_m;
    float height_raw_m;
    float height_sp_m;
    float vz_m_s;
    float vz_sp_m_s;
    float az_m_s2;
    float vbat_v;
} DRV_SysIdSample;

typedef struct {
    uint16_t run_id;
    uint32_t base_t_us;   /* 本批第一条样本的固件微秒时间戳 */
    uint16_t dt_us;       /* 相邻样本的名义间隔 */
    uint16_t flags;
} DRV_SysIdBatchHeader;

/* 字段表的 32 位散列。字段名/单位/缩放任一改变都会变。 */
uint32_t DRV_SysIdRecord_SchemaHash(void);

/* 字段表遍历，供固件把 schema 报给主机。index 越界返回 NULL。 */
uint32_t    DRV_SysIdRecord_FieldCount(void);
const char *DRV_SysIdRecord_FieldName(uint32_t index);
const char *DRV_SysIdRecord_FieldUnit(uint32_t index);
/* 定点 -> 物理量的乘数。主机拿它还原，不在主机侧写死。 */
float       DRV_SysIdRecord_FieldScale(uint32_t index);
/*
 * 字段的定点类型：0 = int16，1 = uint16。每个字段恒为 2 字节，按表序排列。
 * 有了它，整条记录的布局就是**自描述**的：主机照表逐字段取，
 * 不需要在主机侧再抄一份偏移量——抄的那份迟早和固件对不上。
 */
uint8_t     DRV_SysIdRecord_FieldType(uint32_t index);
#define DRV_SYSID_FIELD_TYPE_I16 0U
#define DRV_SYSID_FIELD_TYPE_U16 1U

/*
 * 打包一批样本。`out` 至少要有 DRV_SYSID_RECORD_MAX_FRAME 字节。
 * 小端。成功时 `*out_len` 为实际字节数。
 */
DRV_SysIdRecordStatus DRV_SysIdRecord_Pack(const DRV_SysIdBatchHeader *header,
                                           const DRV_SysIdSample *samples,
                                           uint32_t count,
                                           uint8_t *out, size_t capacity,
                                           size_t *out_len);

/*
 * 解包，供宿主对拍与固件自检。`samples` 至少要能放 DRV_SYSID_RECORD_MAX_COUNT 条。
 * schema_hash 对不上返回 INVALID——宁可不解，也不要按旧布局解出看着正常的错值。
 */
DRV_SysIdRecordStatus DRV_SysIdRecord_Unpack(const uint8_t *data, size_t length,
                                             DRV_SysIdBatchHeader *header,
                                             DRV_SysIdSample *samples,
                                             uint32_t *out_count);

#ifdef __cplusplus
}
#endif
#endif

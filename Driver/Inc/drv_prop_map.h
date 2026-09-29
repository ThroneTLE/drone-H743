#ifndef DRV_PROP_MAP_H
#define DRV_PROP_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 桨叶与电机接线标定：**偏航极性的唯一来源**。
 *
 * ──────────────── 它替掉了什么 ────────────────
 *
 * 在此之前，偏航力矩极性来自 `airframe.lower_rotor_spin_sense`，而那个值是
 * **从调参现象反推的**：作者观察到偏航角速度环 Kp 加大会抖振而不是发散，抖振
 * 说明闭环是负反馈，于是反推出下桨俯视顺时针。推理链自洽，但它证明的是
 * "整条链的符号彼此不矛盾"，不是"桨真的往那边转"。把一个 PID 现象当成物理事实
 * 有一种特别难查的失效方式：**换一套增益、或者把某处符号和它一起翻过来，
 * 现象一样自洽，而飞机的偏航方向已经反了。**
 *
 * 所以这里改成量出来的事实：哪个 ESC 通道接哪个电机、每个电机往哪边转，
 * 由人在上位机上通电看一眼填进去。偏航极性由它推导，与任何增益的正负无关。
 *
 * ──────────────── 推导链（改这里之前先读完） ────────────────
 *
 * 桨对机体的反作用力矩与自身旋向**相反**。上下桨共轴反转，令 s = 下桨旋向
 * （俯视 +1 逆时针 / -1 顺时针，逆时针与规范 FLU 的 +yaw 机头左转同向）：
 *
 *     Mz = -s_upper*ku*T_upper - s_lower*kl*T_lower      (s_upper = -s)
 *        = -s*(kl*T_lower - ku*T_upper)
 *
 * 因此偏航力矩极性 P = -s（`DRV_PropMap_YawTorquePolarity()`）。
 * s = -1（下桨俯视顺时针）时 P = +1，即正偏航力矩靠**加大下桨**推力获得。
 *
 * 没标定时 P 返回 0：分配式退化成"只按总推力对半分"，偏航通道没有权限，
 * 而不是朝一个猜出来的方向使劲。同时 `DRV_PropMap_IsCalibrated()` 为 0，
 * 解锁被挡住（见 `App/Src/app_stabilizer.c` 的解锁链）。失效朝安全方向倒。
 *
 * ──────────────── 通道归属为什么也要标 ────────────────
 *
 * 控制器算出的是"上桨推力"和"下桨推力"，而 ESC 通道是焊上去的：
 * 通道 1 = M4/PE9，通道 2 = M3/PE11（`BSP/Inc/bsp_pwm.h`）。谁接上谁接下，
 * 装配时可能反过来。接反而代码按固定顺序下发的后果不是"偏航反了"这么简单——
 * 上下桨的偏航力臂 ku/kl 不同，推力分配整体错位，倾转力矩也跟着偏。
 * 所以最终提交那一步必须按角色查通道，不能按数组下标。
 *
 * ──────────────── 本结构体是 Flash ABI ────────────────
 *
 * 它整体嵌在 CFG 配置记录（`App/Src/app_control_config_store.c`）里，和增益、
 * 遥控映射、机体模型、状态灯同住一条记录。字段只能往 `reserved` 里吃，
 * 枚举值只能追加不能重排。
 */

#define DRV_PROP_MAP_MAGIC  0x504F5250UL    /* "PROP" 小端 */
#define DRV_PROP_MAP_SCHEMA 1U

/* ESC 通道数。与 BSP_PWM_ESC_CHANNEL_COUNT 是同一件事，但本层不许包含 BSP 头。
 * 两者不一致时 `tests/test_prop_map_contract.py` 会当场变红。 */
#define DRV_PROP_ESC_CHANNEL_COUNT 2U

typedef enum {
    DRV_PROP_ROLE_UPPER = 0U,   /* 上桨 */
    DRV_PROP_ROLE_LOWER = 1U,   /* 下桨 */
    DRV_PROP_ROLE_COUNT = 2U
} DRV_PropRotorRole;

/*
 * 旋向：**俯视**（从上往下看）。+1 = 逆时针，与规范 FLU 的 +yaw（机头左转）同向；
 * -1 = 顺时针；0 = 还没标定。
 *
 * "俯视"这三个字是契约的一部分。同一个电机从上面看是顺时针、从下面看就是逆时针，
 * 而上位机上填这个值的人手里拿着的是一架正放在桌上的飞机。
 */
#define DRV_PROP_SPIN_CCW  ((int8_t)1)
#define DRV_PROP_SPIN_CW   ((int8_t)-1)
#define DRV_PROP_SPIN_NONE ((int8_t)0)

typedef struct {
    uint8_t role;           /* +0  DRV_PropRotorRole */
    int8_t  spin_sense;     /* +1  DRV_PROP_SPIN_* */
    uint8_t reserved[2];    /* +2 */
} DRV_PropChannel;          /* 4 B */

typedef struct {
    uint32_t magic;                                       /* +0  */
    uint16_t schema;                                      /* +4  */
    uint16_t size;                                        /* +6  = sizeof(DRV_PropMap) */
    /* 下标 0 = ESC 通道 1（M4/PE9），下标 1 = ESC 通道 2（M3/PE11）。 */
    DRV_PropChannel channel[DRV_PROP_ESC_CHANNEL_COUNT];  /* +8 .. +15 */
    uint8_t  calibrated;                                  /* +16 0 = 从没标定过 */
    uint8_t  reserved0[3];                                /* +17 */
    uint32_t generation;                                  /* +20 上位机据此判"是否已生效" */
    uint32_t reserved1[2];                                /* +24 .. +31 */
} DRV_PropMap;                                            /* 32 B */

/* 未标定状态：两路旋向都是 0，角色按"通道1=上、通道2=下"摆着但不算数。 */
void DRV_PropMap_Defaults(DRV_PropMap *out);

/*
 * 校验的是"这份标定在物理上可不可能"，不是"填没填"。
 *
 *   calibrated = 0 → 两路旋向必须都是 0（半份标定比没有标定更危险：
 *                    它看起来像标过了）。
 *   calibrated = 1 → 两路角色一上一下（不能都是上桨）；两路旋向都是 ±1
 *                    且**互为相反数**（共轴必反转，同向转的两个桨不是这架飞机）。
 */
uint8_t DRV_PropMap_Validate(const DRV_PropMap *map);

void     DRV_PropMap_ResetActive(void);
uint8_t  DRV_PropMap_PublishActive(const DRV_PropMap *map);
uint8_t  DRV_PropMap_ReadActive(DRV_PropMap *out);
uint32_t DRV_PropMap_GetActiveGeneration(void);

/*
 * ↓↓↓ 以下三个跑在 500 Hz 提交点上 ↓↓↓
 *
 * 它们读的是一个**单独的 32 位派生字**，不是上面那份 32 字节结构体：控制环
 * 拷 32 字节再解析，既慢又要防撕裂。发布时先把派生字清成"未标定"，写完整份
 * 再一次性写回——控制环任何一拍看到的要么是旧的一致状态，要么是"未标定"，
 * 不可能看到半新半旧的组合。
 */

/* 0 = Flash 里没有标定过，或标定值不自洽。为 0 时必须禁止解锁。 */
uint8_t DRV_PropMap_IsCalibrated(void);

/* 下桨旋向（俯视，±1），未标定返回 0。 */
float DRV_PropMap_LowerSpinSense(void);

/*
 * 偏航力矩极性 = -下桨旋向。未标定返回 0——那不是"正方向"，是"没有方向"，
 * 分配式因此给不出偏航力矩。别在调用点把 0 当成 +1 兜底。
 */
float DRV_PropMap_YawTorquePolarity(void);

/*
 * 该角色接在哪个 ESC 通道上，返回 1..DRV_PROP_ESC_CHANNEL_COUNT。
 * 未标定或角色非法返回 0，调用方必须当成"不知道该发给谁"而禁止输出，
 * 不要退回下标顺序——退回去正好是接反时最危险的那种行为。
 */
uint8_t DRV_PropMap_EscChannelForRole(uint8_t role);

/* 名字 ↔ 值。全仓库只此一份：命令族、诊断、上位机都引用它。 */
const char *DRV_PropMap_RoleName(uint8_t role);
uint8_t     DRV_PropMap_RoleFromName(const char *name, uint8_t *role);
const char *DRV_PropMap_SpinName(int8_t spin_sense);
uint8_t     DRV_PropMap_SpinFromName(const char *name, int8_t *spin_sense);

#ifdef __cplusplus
}
#endif

#endif /* DRV_PROP_MAP_H */

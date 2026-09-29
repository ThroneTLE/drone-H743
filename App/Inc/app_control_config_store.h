#ifndef APP_CONTROL_CONFIG_STORE_H
#define APP_CONTROL_CONFIG_STORE_H

#include "app_control.h"
#include "app_control_config_compat.h"
#include "app_flash_service.h"

#define APP_CONTROL_CFG_MAGIC       0x44524346UL
/*
 * v20：记录里新增机体模型块（DRV_Airframe_Params）。
 * 机体数据以前是 drv_airframe_model.h 里的编译期常量，现在唯一来源是 Flash，
 * 由上位机写入——代码里不再保留副本，就不会出现两份数据打架。
 * 从 v19 迁移过来时记录里**没有**机体块，那就是真的没有：DRV_Airframe_Clear()
 * 之后 valid 为 0，解锁被挡住，直到有人把量出来的数填进去。
 */
/*
 * v21：记录里新增状态灯颜色绑定表（APP_LedConfig）。
 * 从 v20 及更早迁移过来时记录里**没有**这一块，读取器显式把它落回出厂默认色，
 * 而不是留着 RAM 里上一份——否则 LEDMAP? 报的和 Flash 里存的不是一回事。
 */
/*
 * v22：记录里新增桨叶与电机接线标定块（DRV_PropMap）。
 * 它是偏航极性的唯一来源，取代了 `airframe.lower_rotor_spin_sense`——那个值是
 * 从调参现象反推的，见 `Driver/Inc/drv_prop_map.h`。
 * 从 v21 及更早迁移过来时记录里**没有**这一块，读取器把它落回"未标定"，
 * 于是飞控拒绝解锁，直到有人真的通电看一眼把旋向填进来。
 * **不从旧的 airframe 字段迁移**：那等于让反推值顶着"已标定"的名义继续生效。
 */
/*
 * v23：记录里新增磁力计硬磁/软磁校准 + 轴向验证块（DRV_MAG_Calibration，
 * Driver/Inc/drv_mag_calibration.h）。磁力计此前完全没有校准，也没有持久化，
 * 这一块是它第一次拥有存盘位置。
 * 从 v22 及更早迁移过来时记录里**没有**这一块，读取器把它落回出厂未校准/
 * 未验证状态（app_cmd_magcal.c 的 app_cmd_magcal_apply_config(NULL)）——这本来
 * 就是磁力计从未接入姿态融合以前的真实状态，不是降级。
 */
/*
 * v24：记录里新增横滚/俯仰指令整形与出口陷波块（APP_ControlCoaxShapingParams，
 * coax.rate_out_notch_* / coax.att_ref_*）。从 v23 及更早迁移过来时记录里**没有**
 * 这一块，读取器显式落回默认：两个开关为 0，控制律与加入它们之前逐位相同。
 */
/*
 * v25：整形块尾部追加第二级出口陷波（coax.rate_out_notch2_hz / _q），前四个字段原位不动。
 * 从 v24 迁移过来时按冻结的 v24 块读前四项，第二级落回默认（notch2_hz = 0 关、
 * notch2_q = 1.0，APP_ControlConfigCompat_ShapingV24ToCurrent）；v23 及更早整块落回默认。
 */
#define APP_CONTROL_CFG_VERSION     25U
#define APP_CONTROL_CFG_VERSION_V24 24U
#define APP_CONTROL_CFG_VERSION_V23 23U
#define APP_CONTROL_CFG_VERSION_V22 22U
#define APP_CONTROL_CFG_VERSION_V21 21U
#define APP_CONTROL_CFG_VERSION_V20 20U
#define APP_CONTROL_CFG_VERSION_V19 19U
#define APP_CONTROL_CFG_VERSION_V18 18U
#define APP_CONTROL_CFG_VERSION_V17 17U
#define APP_CONTROL_CFG_VERSION_V16 16U
#define APP_CONTROL_CFG_VERSION_V15 15U
/*
 * 配置记录改为 A/B 双槽（2026-09-11）。
 *
 * 以前它是单槽：NOR 上写一半掉电，记录就坏了，落回默认值。那时的代价是
 * "增益和遥控映射要重设"，忍得了。现在这条记录里还装着**机体模型**，
 * 而没有机体模型飞控禁止解锁——单槽意味着一次掉电就能让飞机在野外起不来。
 *
 * 两个槽各自独占一个片内 Flash 扇区（擦除粒度 128 KB，不能共用）。
 * `APP_CONTROL_CFG_ADDRESS` 保留为槽 A 的别名，诊断报文仍按它显示。
 */
#define APP_CONTROL_CFG_SLOT_A      APP_FLASH_SERVICE_CFG_SLOT_A_OFFSET
#define APP_CONTROL_CFG_SLOT_B      APP_FLASH_SERVICE_CFG_SLOT_B_OFFSET
#define APP_CONTROL_CFG_ADDRESS     APP_CONTROL_CFG_SLOT_A

/*
 * 提交字：独占一个 32 字节 flash word，写在记录主体**之后**。
 *
 * 片内 Flash 一次擦除后每个 word 只能编程一次，所以不能像 NOR 那样"先写记录、
 * 回头把状态位改成 VALID"。两阶段落在两个不同的 word 上：先写主体并回读校验，
 * 再单独写提交字。没有提交字的槽 = 写到一半掉电，读取时整槽作废。
 *
 * sequence 用来在两个都有效时选新的那个。旧的单槽记录没有提交字，
 * 按 sequence 0 处理——迁移链因此不受影响。
 */
#define APP_CONTROL_CFG_COMMIT_MAGIC 0x43464743UL

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t body_checksum;
    uint32_t reserved;
} APP_ControlConfigCommit;

APP_FlashService_Status APP_ControlConfigStore_Load(APP_ControlConfig *config);
APP_FlashService_Status APP_ControlConfigStore_Save(const APP_ControlConfig *config);
void APP_ControlConfigStore_CaptureTunables(APP_ControlCoaxTunableParams *out);

#endif /* APP_CONTROL_CONFIG_STORE_H */

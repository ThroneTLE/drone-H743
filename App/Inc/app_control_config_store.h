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
#define APP_CONTROL_CFG_VERSION     20U
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

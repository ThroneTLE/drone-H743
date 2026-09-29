#ifndef APP_MAGCAL_H
#define APP_MAGCAL_H

#include "drv_mag_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 磁力计校准 + 轴向验证的持久化存取入口（App 层）。
 *
 * 状态直接用 Driver/Inc/drv_mag_calibration.h 的 DRV_MAG_Calibration 表达——
 * 它已经带齐 C3/C6 需要的全部字段（calibrated、axis_verified、
 * frame_contract_version、hard_iron_bias_mgauss、soft_iron_matrix），不必
 * 另起一套同构的类型。命令解析（MAGCAL/MAGFRAME 命令族）与 Flash 落盘接口
 * 都在 App/Src/app_cmd_magcal.c；本头文件只声明跨模块需要的入口：
 *   - App/Src/app_mag.c：读"有效"校准去纠正 20Hz 原始快照。
 *   - App/Src/app_control_config_store.c：存取 CFG 记录里的这一块
 *     （app_control_internal.h 里的 app_cmd_magcal_apply_config /
 *     app_cmd_magcal_config，与 LEDMAP / PROPCAL 同构）。
 */

/*
 * 返回"有效"校准：在持久化的工作副本基础上应用 C6 的失效规则——
 * 若 frame_contract_version 与当前 DRV_FRAME_CONTRACT_VERSION 不一致
 * （坐标契约已经变了），把 axis_verified 强制视为
 * DRV_MAG_CAL_AXIS_UNVERIFIED，不管持久化记录里存的是什么。**不**改动持久化
 * 的工作副本本身（那份仍然原样保留，供 MAGCAL?/MAGFRAME?/COMMIT 读取）。
 *
 * 融合门控第 2 条（轴向已验证）必须只读这个"有效"视图，不能直接读存储值——
 * 否则坐标契约版本一变，旧的验证记录会继续冒充"已验证"喂进姿态融合。
 *
 * out == NULL 时安全地什么也不做。
 */
void APP_MagCal_GetEffective(DRV_MAG_Calibration *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAGCAL_H */

#ifndef DRV_IMU_IFACE_H
#define DRV_IMU_IFACE_H

/*
 * 多型号 IMU 的统一接口。
 *
 * 背景：本仓库原本只支持一颗 ICM-42688，`drv_imu.h` 里的类型名虽然是中性的
 * （DRV_IMU_ScaledData 等），实现却是该芯片专用的。移植到 MicoAir743v2 之后板上
 * 换成了 BMI088 + BMI270 两颗，需要在运行期探测到哪颗就用哪颗。
 *
 * 分工（decoupling-spec D1-4）：
 *   Driver/  —— 每颗芯片一个驱动，只说"芯片方言"：寄存器、片选、量程换算。
 *   BSP/     —— 拥有设备实例与总线句柄，按探测顺序选中一颗（碰 HAL）。
 *   Services/—— svc_imu 负责芯片轴向 → 机体 FLU 的装配变换与健康判定（不碰 HAL）。
 *
 * 值类型（DRV_IMU_Status / RawData / ScaledData / Config / Odr / Range）继续由
 * `drv_imu.h` 拥有，本文件不重复定义，只加一层函数表。这样既统一了接口，
 * 又不需要把已经在用的类型名全仓库改一遍。
 *
 * 契约：
 *   - ReadScaled 输出仍是**芯片自身轴向**的 g 与 dps，不做任何机体变换。
 *     变换是 svc_imu 的职责，驱动不得替下游猜符号（D4-2）。
 *   - 所有 op 都必须有超时出口，不允许无界死等（D1-3）。
 *   - ctx 是各驱动自己的设备结构体指针；接口层不解释其内容。
 */

#include "drv_imu.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DRV_IMU_ChipKind 定义在 drv_imu_types.h，Services 层要用它但不能碰 HAL。 */

typedef struct {
    DRV_IMU_ChipKind kind;
    const char      *name;

    /*
     * 只读 WHO_AM_I / CHIP_ID，不改变芯片状态。探测阶段用它决定板上是哪颗，
     * 失败必须快速返回（BAD_ID 或 TIMEOUT），不能阻塞后续候选的探测。
     */
    DRV_IMU_Status (*probe)(void *ctx, uint8_t *chip_id);

    /* 复位 + 量程/ODR/滤波配置 + 使能 DRDY 中断。bus 由调用者预先填进 ctx。 */
    DRV_IMU_Status (*init)(void *ctx, const DRV_IMU_Config *config);

    DRV_IMU_Status (*read_raw)(void *ctx, DRV_IMU_RawData *raw);
    DRV_IMU_Status (*read_scaled)(void *ctx, DRV_IMU_ScaledData *scaled);
    DRV_IMU_Status (*is_data_ready)(void *ctx, bool *ready);
} DRV_IMU_Ops;

/*
 * ICM-42688 的函数表。ctx 是 DRV_IMU_Device *，且 ctx->bus 必须由调用者预先填好
 * （BMI088/BMI270 的对应 getter 在各自的头文件里）。
 * 老板子仍然用这颗，所以它留在候选列表里，不是死代码。
 */
const DRV_IMU_Ops *DRV_IMU_GetOps(void);

#ifdef __cplusplus
}
#endif

#endif

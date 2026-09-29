#ifndef DRV_THRUST_LUT_H
#define DRV_THRUST_LUT_H

/*
 * 推力查补表：推力台实测数据拟合出的固定网格表 + 双线性插值（纯算法，无硬件依赖）。
 *
 * 表数据由上位机生成（Driver/Src/drv_thrust_lut_table.inc，勿手改）：
 *   python -m tools.thrust_bench.thrust_lut_export
 * 与上位机 tools/thrust_bench/thrust_lut.py 同一套网格和插值，逐点可对拍。
 *
 * 两张表，推力单位均为克（双桨合计）：
 *   转速表：T(上桨 eRPM, 下桨 eRPM)。同一转速下电池电压几乎不影响推力。
 *   油门表：T(上桨 e, 下桨 e)，e = 油门% × 电量电压 / v_ref（等效电机电压）。
 *           电压折进坐标，所以同一张表覆盖不同电量。
 * 同油门（上下桨同一油门）的正查/反查用生成器预先算好的对角线（单调包络）。
 *
 * 电量电压 = 带载电压 + sag_k × Σ(eRPM/1e4)³，与推力台同一口径。
 */

#include <stdint.h>

typedef struct {
    float x0;
    float dx;
    uint16_t nx;
    float y0;
    float dy;
    uint16_t ny;
    const float *values; /* [iy * nx + ix]，克 */
} DRV_ThrustLutGrid;

typedef struct {
    const char *model_id;
    float v_ref;
    float sag_k;
    float charge_measured_v[2];
    float charge_usable_v[2]; /* 补偿只在此范围内生效，超出按边界算 */
    DRV_ThrustLutGrid speed;
    DRV_ThrustLutGrid effective;
    uint16_t diag_count;
    const float *diag_effective; /* 升序，等效油门 % */
    const float *diag_thrust_g;  /* 非减，双桨合计克 */
} DRV_ThrustLutTable;

extern const DRV_ThrustLutTable DRV_ThrustLut_Table;

/* 网格双线性插值；坐标先夹到网格范围内（飞控必须总有输出，不返回无效）。 */
float DRV_ThrustLut_ReadGrid(const DRV_ThrustLutGrid *grid, float x, float y);
float DRV_ThrustLut_ThrustFromSpeed(float upper_erpm, float lower_erpm);
float DRV_ThrustLut_ThrustFromEffective(float upper_effective, float lower_effective);

/* 同油门对角线：等效油门 → 合推力，及其反查。超出实测上限时饱和在上限。 */
float DRV_ThrustLut_BalancedThrustForEffective(float effective);
float DRV_ThrustLut_BalancedEffectiveForThrust(float total_g);

/*
 * 上下桨同时平移的量（等效油门 %），使二维油门表的合推力等于 total_g；油门差保持不变。
 * 限幅 ±DRV_THRUST_LUT_PAIR_SHIFT_LIMIT：实测最多只需 3.5%，表值再错也推不远。
 * 上限 e_max = 100% 油门对应的等效油门；够不到时返回能达到的边界。任一桨已饱和
 * （超过 e_max）或低于实测最低油门时返回 0，即照逐桨换算。
 */
#define DRV_THRUST_LUT_PAIR_SHIFT_LIMIT 10.0f
float DRV_ThrustLut_PairShift(float e_upper, float e_lower, float total_g, float e_max);

float DRV_ThrustLut_ChargeVoltage(float loaded_v, float erpm_a, float erpm_b);
/* charge_v <= 0 表示未知：不做电压补偿（按 v_ref 计）。结果夹在 0..100。 */
float DRV_ThrustLut_PercentForEffective(float effective, float charge_v);
float DRV_ThrustLut_EffectiveForPercent(float percent, float charge_v);

#endif /* DRV_THRUST_LUT_H */

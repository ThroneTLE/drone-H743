/*
 * 参数名路由：把 `PARAM` 命令同时接到控制参数（coax.*）与机体模型（airframe.*）两张表。
 *
 * 为什么要分成两张表而不是合并：`DEFAULTS` 命令会把控制增益恢复默认。机体模型是
 * **量出来的物理事实**，不是调参结果——混在一起的话，一次"恢复默认增益"就会顺手
 * 抹掉你拿秤和尺量了半天的数据，而且不会有任何提示。
 *
 * 为什么路由函数放在这个新模块里而不是直接写进 app_control.c：那个文件是硬约束的
 * "只减不增"。这里提供同形态的替代函数，调用点做等量替换即可。
 *
 * 顺序固定为"先 coax 后 airframe"。两张表的名字前缀不同，本来不会撞；固定顺序是为了
 * 让 `PARAM?` 的输出顺序稳定——上位机按行号对齐显示时，顺序漂移会让人以为参数丢了。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_proto.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

/*
 * 六位小数的定点格式化，与 app_control.c 里那个 static 的同名函数逐字符一致。
 * 复制一份而不是把那个改成外部可见：那个文件是"只减不增"的硬约束，
 * 而上位机按固定小数位解析，两处输出格式一旦不同就会静默解析错。
 */
static void airframe_format_float(float value, char *buffer, uint32_t size)
{
    int scaled;

    if ((buffer == NULL) || (size == 0U)) {
        return;
    }

    scaled = (int)((value * 1000000.0f) +
                   ((value >= 0.0f) ? 0.5f : -0.5f));
    (void)snprintf(buffer,
                   size,
                   "%s%d.%06u",
                   (scaled < 0) ? "-" : "",
                   abs(scaled / 1000000),
                   (unsigned int)abs(scaled % 1000000));
}

uint32_t app_control_param_count_any(void)
{
    return DRV_COAX_CTRL_ParamCount() + DRV_Airframe_GetParamCount();
}

const char *app_control_param_name_any(uint32_t index)
{
    const uint32_t coax_count = DRV_COAX_CTRL_ParamCount();

    if (index < coax_count) {
        return DRV_COAX_CTRL_ParamName(index);
    }
    return DRV_Airframe_GetParamNameAt(index - coax_count);
}

uint8_t app_control_param_get_any(const char *name, float *value)
{
    if (DRV_COAX_CTRL_GetParam(name, value) != 0U) {
        return 1U;
    }
    return DRV_Airframe_GetParam(name, value);
}

uint8_t app_control_param_set_any(const char *name, float value)
{
    if (DRV_COAX_CTRL_SetParam(name, value) != 0U) {
        return 1U;
    }
    return DRV_Airframe_SetParam(name, value);
}

/*
 * 推力表口径。它是一个**字符串**，说明 max_total_thrust_g 记的是"双桨合计"
 * 而不是单桨——这个口径搞错，推重比会差一倍。因为不是数，它进不了 float
 * 参数表，所以留在代码里：它描述的是这份数据怎么读，不是这架飞机量出来多少。
 */
#define AIRFRAME_THRUST_TABLE_SCOPE "dual_motor_total"

void app_control_report_airframe_record(void)
{
    const DRV_Airframe_Params *p = DRV_Airframe_Get();
    char mass_kg[24];
    char cg_z_m[24];
    char imu_z_m[24];
    char attach_z_m[24];
    char attach_to_cg_m[24];
    char rope_m[24];
    char rod_to_cg_m[24];
    char servo_deg_per_us[24];
    char servo_us_per_deg[24];
    char max_force_n[24];
    char hover_pct[24];

    airframe_format_float(p->mass_kg, mass_kg, (uint32_t)sizeof(mass_kg));
    airframe_format_float(p->cg_z_m, cg_z_m, (uint32_t)sizeof(cg_z_m));
    airframe_format_float(p->imu_z_m, imu_z_m, (uint32_t)sizeof(imu_z_m));
    airframe_format_float(p->tether_attach_z_m, attach_z_m, (uint32_t)sizeof(attach_z_m));
    airframe_format_float(p->tether_attach_to_cg_m, attach_to_cg_m, (uint32_t)sizeof(attach_to_cg_m));
    airframe_format_float(p->tether_rope_m, rope_m, (uint32_t)sizeof(rope_m));
    airframe_format_float(p->tether_rod_to_cg_m, rod_to_cg_m, (uint32_t)sizeof(rod_to_cg_m));
    airframe_format_float(p->servo_deg_per_us, servo_deg_per_us, (uint32_t)sizeof(servo_deg_per_us));
    airframe_format_float(p->servo_us_per_deg, servo_us_per_deg, (uint32_t)sizeof(servo_us_per_deg));
    airframe_format_float(p->max_total_force_n, max_force_n, (uint32_t)sizeof(max_force_n));
    airframe_format_float(p->hover_thrust_percent, hover_pct, (uint32_t)sizeof(hover_pct));

    /*
     * 报文形状与改造前逐字段一致——上位机的证据快照按这些键名存档，改键名会让
     * 历史存档和新存档对不上。变的只是数据来源：以前是编译期常量，现在是 Flash。
     *
     * "模型是否有效、缺哪一项"不放在这条线上：那是**解锁闸门**的事实，归 ARM
     * 报文（app_cmd_arm.c）。放两处会分叉，而分叉的诊断迟早互相矛盾。
     */
    app_control_queue_proto_text(APP_PROTO_MSG_AIRFRAME_RECORD,
                                 "AIRFRAME mass_kg=%s cg_z_m=%s imu_z_m=%s tether_attach_z_m=%s tether_attach_to_cg_m=%s rope_m=%s rod_to_cg_m=%s servo_deg_per_us=%s servo_us_per_deg=%s thrust_scope=%s max_total_force_n=%s hover_thrust_pct=%s\r\n",
                                 mass_kg,
                                 cg_z_m,
                                 imu_z_m,
                                 attach_z_m,
                                 attach_to_cg_m,
                                 rope_m,
                                 rod_to_cg_m,
                                 servo_deg_per_us,
                                 servo_us_per_deg,
                                 AIRFRAME_THRUST_TABLE_SCOPE,
                                 max_force_n,
                                 hover_pct);
}

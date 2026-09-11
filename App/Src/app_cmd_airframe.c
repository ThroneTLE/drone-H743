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

void app_control_report_airframe_model(void)
{
    const DRV_Airframe_Params *p = DRV_Airframe_Get();
    const char *bad = DRV_Airframe_FirstInvalidName();
    char total[24];
    char cg[24];
    char hover[24];
    char thrust_arm[24];

    /*
     * 先报"能不能用"，再报数值。顺序是有意的：模型无效时下面那些数字全是零，
     * 单看数字会以为是"还没量"，而实际后果是**解锁被挡住**——这件事必须第一行就说清。
     */
    APP_Control_QueueText(
        "AIRFRAME valid=%u missing=%s derived_auto=%u params=%lu\r\n",
        (unsigned int)DRV_Airframe_IsValid(),
        (bad != NULL) ? bad : "-",
        (unsigned int)(((p->derived_auto > 0.5f) || (p->derived_auto < -0.5f)) ? 1U : 0U),
        (unsigned long)DRV_Airframe_GetParamCount());

    airframe_format_float(p->mass_kg, total, (uint32_t)sizeof(total));
    airframe_format_float(p->cg_z_m, cg, (uint32_t)sizeof(cg));
    airframe_format_float(p->hover_thrust_percent, hover,
                                    (uint32_t)sizeof(hover));
    airframe_format_float(p->thrust_point_to_cg_z_m, thrust_arm,
                                    (uint32_t)sizeof(thrust_arm));

    APP_Control_QueueText(
        "AIRFRAME mass_kg=%s cg_z_m=%s thrust_point_to_cg_z_m=%s hover_pct=%s\r\n",
        total, cg, thrust_arm, hover);
}

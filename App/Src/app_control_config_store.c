#include "app_control_config_store.h"

#include "app_control_config_compat.h"
#include "app_control_internal.h"
#include "app_rc_config.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define APP_CONTROL_CONFIG_PI 3.14159265358979323846f
#define APP_CONTROL_TILT_LIMIT_DEFAULT_RAD 0.4886922f
#define APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD 0.31415927f
#define APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD 0.43633231f
#define APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD 0.001f

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
} APP_ControlFlashHeader;

/*
 * `has_airframe` 是 v20 加的：机体模型块只存在于当前版本的记录里。
 * 它必须排在 checksum **之前**、其余块之后——旧版本记录的字节布局因此原封不动，
 * 迁移读取器仍能按各自的 offsetof 校验通过。
 */
#define APP_CONTROL_RECORD_TYPE(name, tunable_type, has_rc, has_airframe) \
    typedef struct { \
        uint32_t magic; \
        uint16_t version; \
        uint16_t size; \
        APP_ControlConfig config; \
        tunable_type coax_tunables; \
        has_rc \
        has_airframe \
        uint32_t checksum; \
    } name

APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecord,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        DRV_Airframe_Params airframe;);
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV19,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;, );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV18,
                        APP_ControlCoaxTunableParamsV18,
                        APP_RcConfig rc_config;, );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV17,
                        APP_ControlCoaxTunableParamsV17,
                        APP_RcConfig rc_config;, );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV16,
                        APP_ControlCoaxTunableParamsV17, , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV15,
                        APP_ControlCoaxTunableParamsV15, , );

static uint32_t config_checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;

    for (uint32_t index = 0U; index < length; ++index) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[index];
        sum += 0x9E3779B9UL;
    }
    return sum;
}

void APP_ControlConfigStore_CaptureTunables(APP_ControlCoaxTunableParams *out)
{
    DRV_COAX_CTRL_Params params;

    if (out == NULL) {
        return;
    }
    DRV_COAX_CTRL_GetParams(&params);
    out->pos_x_kp = params.position.pos_kp[0];
    out->pos_y_kp = params.position.pos_kp[1];
    out->pos_z_kp = params.position.pos_kp[2];
    out->pos_xy_vel_max_m_s = params.position.xy_speed_limit_m_s;
    out->pos_z_vel_up_max_m_s = params.position.z_speed_limit_up_m_s;
    out->pos_z_vel_down_max_m_s = params.position.z_speed_limit_down_m_s;
    out->vel_x_kp = params.position.vel_kp[0];
    out->vel_y_kp = params.position.vel_kp[1];
    out->vel_z_kp = params.position.vel_kp[2];
    out->vel_x_ki = params.position.vel_ki[0];
    out->vel_y_ki = params.position.vel_ki[1];
    out->vel_z_ki = params.position.vel_ki[2];
    out->vel_x_kd = params.position.vel_kd[0];
    out->vel_y_kd = params.position.vel_kd[1];
    out->vel_z_kd = params.position.vel_kd[2];
    out->vel_x_i_limit_m_s2 = params.position.vel_integrator_limit[0];
    out->vel_y_i_limit_m_s2 = params.position.vel_integrator_limit[1];
    out->vel_z_i_limit_m_s2 = params.position.vel_integrator_limit[2];
    out->accel_lpf_cutoff_hz = params.position.accel_lpf_cutoff_hz;
    out->accel_xy_max_m_s2 = params.position.xy_accel_limit_m_s2;
    out->accel_z_up_max_m_s2 = params.position.z_accel_limit_up_m_s2;
    out->accel_z_down_max_m_s2 = params.position.z_accel_limit_down_m_s2;
    out->att_roll_kp = params.attitude.att_kp[0];
    out->att_pitch_kp = params.attitude.att_kp[1];
    out->att_yaw_kp = params.attitude.att_kp[2];
    out->roll_rate_limit_rad_s = params.attitude.rate_limit_rad_s[0];
    out->pitch_rate_limit_rad_s = params.attitude.rate_limit_rad_s[1];
    out->yaw_rate_limit_rad_s = params.attitude.rate_limit_rad_s[2];
    out->rate_roll_kp = params.rate.kp[0];
    out->rate_pitch_kp = params.rate.kp[1];
    out->rate_yaw_kp = params.rate.kp[2];
    out->rate_roll_ki = params.rate.ki[0];
    out->rate_pitch_ki = params.rate.ki[1];
    out->rate_yaw_ki = params.rate.ki[2];
    out->rate_roll_kd = params.rate.kd[0];
    out->rate_pitch_kd = params.rate.kd[1];
    out->rate_yaw_kd = params.rate.kd[2];
    out->rate_roll_i_limit_n_m = params.rate.integrator_limit[0];
    out->rate_pitch_i_limit_n_m = params.rate.integrator_limit[1];
    out->rate_yaw_i_limit_n_m = params.rate.integrator_limit[2];
    out->angular_accel_lpf_cutoff_hz =
        params.rate.alpha_lpf_cutoff_rad_s / (2.0f * APP_CONTROL_CONFIG_PI);
    out->rate_roll_ff = params.rate.ff_gain[0];
    out->rate_pitch_ff = params.rate.ff_gain[1];
    out->rate_yaw_ff = params.rate.ff_gain[2];
    out->tilt_limit_rad = params.tilt_limit_rad;
    out->vel_loop_enable = params.vel_loop_enable;
}

static void config_apply_tunables(const APP_ControlCoaxTunableParams *in)
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.position.pos_kp[0] = in->pos_x_kp;
    params.position.pos_kp[1] = in->pos_y_kp;
    params.position.pos_kp[2] = in->pos_z_kp;
    params.position.xy_speed_limit_m_s = in->pos_xy_vel_max_m_s;
    params.position.z_speed_limit_up_m_s = in->pos_z_vel_up_max_m_s;
    params.position.z_speed_limit_down_m_s = in->pos_z_vel_down_max_m_s;
    params.position.vel_kp[0] = in->vel_x_kp;
    params.position.vel_kp[1] = in->vel_y_kp;
    params.position.vel_kp[2] = in->vel_z_kp;
    params.position.vel_ki[0] = in->vel_x_ki;
    params.position.vel_ki[1] = in->vel_y_ki;
    params.position.vel_ki[2] = in->vel_z_ki;
    params.position.vel_kd[0] = in->vel_x_kd;
    params.position.vel_kd[1] = in->vel_y_kd;
    params.position.vel_kd[2] = in->vel_z_kd;
    params.position.vel_integrator_limit[0] = in->vel_x_i_limit_m_s2;
    params.position.vel_integrator_limit[1] = in->vel_y_i_limit_m_s2;
    params.position.vel_integrator_limit[2] = in->vel_z_i_limit_m_s2;
    params.position.accel_lpf_cutoff_hz = in->accel_lpf_cutoff_hz;
    params.position.xy_accel_limit_m_s2 = in->accel_xy_max_m_s2;
    params.position.z_accel_limit_up_m_s2 = in->accel_z_up_max_m_s2;
    params.position.z_accel_limit_down_m_s2 = in->accel_z_down_max_m_s2;
    params.attitude.att_kp[0] = in->att_roll_kp;
    params.attitude.att_kp[1] = in->att_pitch_kp;
    params.attitude.att_kp[2] = in->att_yaw_kp;
    params.attitude.rate_limit_rad_s[0] = in->roll_rate_limit_rad_s;
    params.attitude.rate_limit_rad_s[1] = in->pitch_rate_limit_rad_s;
    params.attitude.rate_limit_rad_s[2] = in->yaw_rate_limit_rad_s;
    params.rate.kp[0] = in->rate_roll_kp;
    params.rate.kp[1] = in->rate_pitch_kp;
    params.rate.kp[2] = in->rate_yaw_kp;
    params.rate.ki[0] = in->rate_roll_ki;
    params.rate.ki[1] = in->rate_pitch_ki;
    params.rate.ki[2] = in->rate_yaw_ki;
    params.rate.kd[0] = in->rate_roll_kd;
    params.rate.kd[1] = in->rate_pitch_kd;
    params.rate.kd[2] = in->rate_yaw_kd;
    params.rate.integrator_limit[0] = in->rate_roll_i_limit_n_m;
    params.rate.integrator_limit[1] = in->rate_pitch_i_limit_n_m;
    params.rate.integrator_limit[2] = in->rate_yaw_i_limit_n_m;
    params.rate.alpha_lpf_cutoff_rad_s =
        in->angular_accel_lpf_cutoff_hz * 2.0f * APP_CONTROL_CONFIG_PI;
    params.rate.ff_gain[0] = in->rate_roll_ff;
    params.rate.ff_gain[1] = in->rate_pitch_ff;
    params.rate.ff_gain[2] = in->rate_yaw_ff;
    params.tilt_limit_rad = in->tilt_limit_rad;
    if ((fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD) ||
        (fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD)) {
        params.tilt_limit_rad = APP_CONTROL_TILT_LIMIT_DEFAULT_RAD;
    }
    params.vel_loop_enable = in->vel_loop_enable;
    DRV_COAX_CTRL_SetParams(&params);
}

static uint8_t config_read_current(APP_ControlConfig *config)
{
    APP_ControlFlashRecord record;
    if (APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS, (uint8_t *)&record,
                                  sizeof(record)) != APP_FLASH_SERVICE_OK) {
        return 0U;
    }
    if ((record.magic != APP_CONTROL_CFG_MAGIC) ||
        (record.version != APP_CONTROL_CFG_VERSION) ||
        (record.size != sizeof(record.config) + sizeof(record.coax_tunables) +
                        sizeof(record.rc_config)) ||
        (config_checksum((const uint8_t *)&record.config, record.size) !=
         record.checksum)) {
        return 0U;
    }
    *config = record.config;
    config_apply_tunables(&record.coax_tunables);
    app_cmd_rcmap_apply_config(&record.rc_config);
    DRV_Airframe_SetParams(&record.airframe);
    return 1U;
}

#define APP_CONTROL_DEFINE_LEGACY_READER(function_name, type, expected_version, \
                                         convert_fn, apply_rc_statement) \
    static uint8_t function_name(APP_ControlConfig *config) \
    { \
        type record; \
        APP_ControlCoaxTunableParams migrated; \
        if (APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS, \
                                      (uint8_t *)&record, sizeof(record)) != \
            APP_FLASH_SERVICE_OK) return 0U; \
        if ((record.magic != APP_CONTROL_CFG_MAGIC) || \
            (record.version != (expected_version)) || \
            (record.size != (uint16_t)(offsetof(type, checksum) - \
                                       offsetof(type, config))) || \
            (config_checksum((const uint8_t *)&record.config, record.size) != \
             record.checksum) || \
            ((convert_fn)(&record.coax_tunables, &migrated) == 0U)) return 0U; \
        *config = record.config; \
        config_apply_tunables(&migrated); \
        apply_rc_statement; \
        return 1U; \
    }

APP_CONTROL_DEFINE_LEGACY_READER(config_read_v19, APP_ControlFlashRecordV19,
                                 APP_CONTROL_CFG_VERSION_V19,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 app_cmd_rcmap_apply_config(&record.rc_config))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v18, APP_ControlFlashRecordV18,
                                 APP_CONTROL_CFG_VERSION_V18,
                                 APP_ControlConfigCompat_V18ToCurrent,
                                 app_cmd_rcmap_apply_config(&record.rc_config))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v17, APP_ControlFlashRecordV17,
                                 APP_CONTROL_CFG_VERSION_V17,
                                 APP_ControlConfigCompat_V17ToCurrent,
                                 app_cmd_rcmap_apply_config(&record.rc_config))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v16, APP_ControlFlashRecordV16,
                                 APP_CONTROL_CFG_VERSION_V16,
                                 APP_ControlConfigCompat_V17ToCurrent,
                                 app_cmd_rcmap_apply_config(NULL))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v15, APP_ControlFlashRecordV15,
                                 APP_CONTROL_CFG_VERSION_V15,
                                 APP_ControlConfigCompat_V15ToCurrent,
                                 app_cmd_rcmap_apply_config(NULL))

APP_FlashService_Status APP_ControlConfigStore_Load(APP_ControlConfig *config)
{
    APP_ControlFlashHeader header;
    APP_FlashService_Status status;

    if (config == NULL) {
        return APP_FLASH_SERVICE_ERROR;
    }
    status = APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS,
                                       (uint8_t *)&header, sizeof(header));
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }
    if (header.magic != APP_CONTROL_CFG_MAGIC) {
        return APP_FLASH_SERVICE_BAD_ID;
    }
    switch (header.version) {
    case APP_CONTROL_CFG_VERSION:
        if (config_read_current(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V19:
        if (config_read_v19(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V18:
        if (config_read_v18(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V17:
        if (config_read_v17(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V16:
        if (config_read_v16(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V15:
        if (config_read_v15(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    default:
        return APP_FLASH_SERVICE_BAD_ID;
    }
    config->loaded_from_flash = 1U;
    config->flash_valid = 1U;
    return APP_FLASH_SERVICE_OK;
}

APP_FlashService_Status APP_ControlConfigStore_Save(const APP_ControlConfig *config)
{
    APP_ControlFlashRecord record;
    APP_FlashService_Status status;

    if (config == NULL) {
        return APP_FLASH_SERVICE_ERROR;
    }
    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = (uint16_t)(sizeof(record.config) +
                             sizeof(record.coax_tunables) +
                             sizeof(record.rc_config) +
                             sizeof(record.airframe));
    record.config = *config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    APP_ControlConfigStore_CaptureTunables(&record.coax_tunables);
    record.rc_config = *(const APP_RcConfig *)app_cmd_rcmap_config();
    DRV_Airframe_GetParams(&record.airframe);
    record.checksum = config_checksum((const uint8_t *)&record.config,
                                      record.size);
    status = APP_FlashService_EraseSector(APP_CONTROL_CFG_ADDRESS);
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }
    return APP_FlashService_WriteData(APP_CONTROL_CFG_ADDRESS,
                                      (const uint8_t *)&record,
                                      sizeof(record));
}

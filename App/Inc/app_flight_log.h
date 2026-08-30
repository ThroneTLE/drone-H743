#ifndef APP_FLIGHT_LOG_H
#define APP_FLIGHT_LOG_H

#include "drv_coax_ctrl.h"
#include "drv_imu.h"
#include "app_ident.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FLIGHT_LOG_REGION_START       0x00002000UL
#define APP_FLIGHT_LOG_REGION_END_EXCL    0x003FC000UL
#define APP_FLIGHT_LOG_RATE_HZ            250U
#define APP_FLIGHT_LOG_SECTOR_HEADER_SIZE 256U
#define APP_FLIGHT_LOG_EXPORT_BAUD        57600U
#define APP_FLIGHT_LOG_BACKGROUND_IDLE_MS 5U

typedef enum {
    APP_FLIGHT_LOG_CMD_OK = 0,
    APP_FLIGHT_LOG_CMD_BUSY,
    APP_FLIGHT_LOG_CMD_RECORDING,
    APP_FLIGHT_LOG_CMD_NOT_EXPORTING,
    APP_FLIGHT_LOG_CMD_ERROR,
} APP_FlightLogCommandStatus;

typedef enum {
    APP_FLIGHT_LOG_MOTOR_REASON_UNKNOWN = 0,
    APP_FLIGHT_LOG_MOTOR_REASON_STABILIZED_MIX = 1,
    APP_FLIGHT_LOG_MOTOR_REASON_DIRECT_THROTTLE = 2,
    APP_FLIGHT_LOG_MOTOR_REASON_DISARMED_MIN = 3,
    APP_FLIGHT_LOG_MOTOR_REASON_NO_RC_SEEN_MIN = 4,
    APP_FLIGHT_LOG_MOTOR_REASON_RC_LOSS_DISABLE = 5,
    APP_FLIGHT_LOG_MOTOR_REASON_IDENT_DIRECT = 6,
    APP_FLIGHT_LOG_MOTOR_REASON_IMU_INVALID_DIRECT = 7,
    APP_FLIGHT_LOG_MOTOR_REASON_ATTITUDE_DEBUG = 8,
} APP_FlightLogMotorOutputReason;

typedef struct {
    /*
     * 坐标溯源（R-F5b）。由 app_stabilizer.c 从控制环里本就现成的源填入，
     * 经 APP_FlightLog_Observe 缓存后写进扇区头（每会话一次，不进每条记录）。
     * frame_orientation_code 为 V0 码：0..23 = 已发布规范 FLU，
     * 255 = legacy 中间轴。firmware_crc32 不在此处取——首次计算会遍历整个
     * 镜像，250Hz 控制环不得阻塞（spec §5），改由扇区头填充路径在后台采样。
     */
    uint8_t frame_orientation_code;
    uint8_t frame_contract_version;
    uint32_t calibration_generation;
    uint64_t timestamp_us;
    uint32_t tick_ms;
    uint32_t imu_sequence;
    DRV_IMU_RawData imu_raw;
    DRV_IMU_ScaledData imu;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint16_t rc_channels[8];
    uint16_t throttle_us;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    uint16_t servo_alpha_feedback_us;
    uint16_t servo_beta_feedback_us;
    uint16_t servo_alpha_feedback_age_ms;
    uint16_t servo_beta_feedback_age_ms;
    uint16_t servo_alpha_feedback_sequence;
    uint16_t servo_beta_feedback_sequence;
    uint8_t servo_feedback_valid_mask;
    uint16_t motor_upper_us;
    uint16_t motor_lower_us;
    uint8_t rc_armed;
    uint8_t rc_link_ok;
    uint8_t throttle_over_20;
    uint8_t imu_valid;
    uint8_t motor_output_reason;
    uint8_t rc_link_seen;
    uint8_t arm_switch_high;
    uint8_t arm_throttle_low;
    uint8_t arm_switch_seen_low;
    uint8_t arm_switch_prev_high;
    uint8_t imu_fault_latched;
    uint8_t imu_fault_reason;
    float acc_nav_m_s2[3];
    float vel_est_m_s[3];
    float vel_ref_m_s[2];
    float vel_err_m_s[2];
    float vel_pid_out_m_s2[2];
    float vel_pid_p_m_s2[2];
    float vel_pid_i_m_s2[2];
    float vel_pid_d_m_s2[2];
    float vel_loop_active;
    DRV_COAX_CTRL_Debug ctrl_debug;
    float z_ref_m;
    APP_IdentAttLog ident_att;
    uint16_t servo_alpha_sent_us;
    uint16_t servo_beta_sent_us;
    int16_t flow_raw_x;
    int16_t flow_raw_y;
    uint16_t flow_sample_age_ms;
    uint16_t flow_height_age_ms;
    uint8_t flow_quality;
    uint8_t flow_valid;
    uint8_t flow_velocity_valid;
    uint8_t flow_height_valid;
    float flow_height_raw_m;
    float flow_height_m;
    float flow_sensor_velocity_m_s[2];
    float flow_optical_rot_comp_m_s[2];
    float flow_offset_rot_comp_m_s[2];
    float flow_corrected_velocity_m_s[2];
    uint32_t servo_move_attempt_count;
    uint32_t servo_move_sent_count;
    uint32_t servo_move_busy_count;
    uint32_t servo_move_error_count;
    uint32_t servo_feedback_request_count;
    uint32_t servo_feedback_response_count;
    uint32_t servo_feedback_timeout_count;
    uint32_t servo_feedback_parse_error_count;
    uint32_t servo_feedback_uart_error_count;
    uint32_t servo_feedback_busy_count;
} APP_FlightLogSnapshot;

typedef struct {
    uint8_t initialized;
    uint8_t recording;
    uint8_t export_active;
    uint8_t export_pending;
    uint8_t sector_open;
    uint32_t used_sectors;
    uint32_t used_bytes;
    uint32_t total_records;
    uint32_t dropped_records;
    uint32_t buffered_records;
    uint32_t session_id;
    uint32_t sector_seq;
    uint32_t export_bytes_sent;
    uint32_t export_total_bytes;
    uint32_t last_flash_status;
} APP_FlightLogStatus;

void APP_FlightLog_Init(void);
void APP_FlightLog_BackgroundStep(void);
void APP_FlightLog_Observe(const APP_FlightLogSnapshot *snapshot,
                           uint8_t should_record);
void APP_FlightLog_GetStatus(APP_FlightLogStatus *status);
APP_FlightLogCommandStatus APP_FlightLog_StartDump(void);
APP_FlightLogCommandStatus APP_FlightLog_CancelDump(void);
APP_FlightLogCommandStatus APP_FlightLog_TestFill(uint32_t sectors);
uint8_t APP_FlightLog_IsExportActive(void);
const char *APP_FlightLog_CommandStatusText(APP_FlightLogCommandStatus status);

#ifdef __cplusplus
}
#endif

#endif

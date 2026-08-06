#include "drv_attitude_fusion.h"

#include "FusionAhrs.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * x-io Fusion, upstream commit 015d68494274b479b5996bff2530ecbcfdc266f2.
 * The vendored implementation and MIT license are in ThirdParty/Fusion.
 */
#define DRV_ATTITUDE_FUSION_SAMPLE_RATE_HZ 1000.0f
#define DRV_ATTITUDE_FUSION_GAIN 0.5f
#define DRV_ATTITUDE_FUSION_GYRO_RANGE_DPS 1000.0f
/*
 * Coaxial-rotor vibration makes the instantaneous gravity innovation large even
 * when the airframe is level, so the innovation gate must not be tight. Offline
 * replay of log/flightlog_20260726_*.csv and log/flightlog_20260727_045138.csv
 * showed the previous 10 deg / [0.85, 1.15] g pair ignoring the accelerometer
 * for 75-85 % of powered flight, leaving attitude on free-running gyro
 * integration. Widening both gates and shortening the timeout drops that to
 * roughly 0 % and cuts median tilt error from 9.6-25.6 deg to 2.6-4.9 deg.
 */
#define DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG 45.0f
#define DRV_ATTITUDE_FUSION_REJECTION_TIMEOUT_S 0.5f
/* Reject only physically implausible specific force (impact, free fall). */
#define DRV_ATTITUDE_FUSION_ACCEL_NORM_MIN_G 0.40f
#define DRV_ATTITUDE_FUSION_ACCEL_NORM_MAX_G 1.80f
#define DRV_ATTITUDE_FUSION_DT_MIN_S 0.0001f
#define DRV_ATTITUDE_FUSION_DT_MAX_S 0.030f

typedef struct {
    FusionAhrs ahrs;
    DRV_AttitudeFusionOutput output;
} DRV_AttitudeFusionState;

#if defined(__GNUC__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static DRV_AttitudeFusionState attitude_fusion_state;

static uint8_t finite_vector3(const float value[3])
{
    return (isfinite(value[0]) && isfinite(value[1]) &&
            isfinite(value[2])) ? 1U : 0U;
}

void DRV_AttitudeFusion_Init(void)
{
    FusionAhrsSettings settings = fusionAhrsDefaultSettings;

    memset(&attitude_fusion_state, 0, sizeof(attitude_fusion_state));
    FusionAhrsInitialise(&attitude_fusion_state.ahrs);
    settings.sampleRate = DRV_ATTITUDE_FUSION_SAMPLE_RATE_HZ;
    settings.convention = FusionConventionNed;
    settings.gain = DRV_ATTITUDE_FUSION_GAIN;
    settings.gyroscopeRange = DRV_ATTITUDE_FUSION_GYRO_RANGE_DPS;
    settings.accelerationRejection =
        DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG;
    settings.magneticRejection = 0.0f;
    settings.rejectionTimeout =
        DRV_ATTITUDE_FUSION_REJECTION_TIMEOUT_S;
    FusionAhrsSetSettings(&attitude_fusion_state.ahrs, &settings);
    attitude_fusion_state.output.quaternion[0] = 1.0f;
}

uint8_t DRV_AttitudeFusion_Update(
    const DRV_AttitudeFusionInput *input,
    DRV_AttitudeFusionOutput *output)
{
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionQuaternion quaternion;
    FusionEuler euler;
    FusionAhrsInternalStates internal_states;
    FusionAhrsFlags flags;
    float accel_norm;

    if ((input == NULL) || !isfinite(input->dt_s) ||
        (input->dt_s < DRV_ATTITUDE_FUSION_DT_MIN_S) ||
        (input->dt_s > DRV_ATTITUDE_FUSION_DT_MAX_S) ||
        (finite_vector3(input->gyroscope_dps) == 0U) ||
        (finite_vector3(input->accelerometer_g) == 0U)) {
        if (output != NULL) {
            *output = attitude_fusion_state.output;
        }
        return 0U;
    }

    gyroscope.axis.x = input->gyroscope_dps[0];
    gyroscope.axis.y = input->gyroscope_dps[1];
    gyroscope.axis.z = input->gyroscope_dps[2];
    accelerometer.axis.x = input->accelerometer_g[0];
    accelerometer.axis.y = input->accelerometer_g[1];
    accelerometer.axis.z = input->accelerometer_g[2];
    accel_norm = sqrtf((accelerometer.axis.x * accelerometer.axis.x) +
                       (accelerometer.axis.y * accelerometer.axis.y) +
                       (accelerometer.axis.z * accelerometer.axis.z));

    attitude_fusion_state.output.accel_norm_rejected = 0U;
    if ((accel_norm < DRV_ATTITUDE_FUSION_ACCEL_NORM_MIN_G) ||
        (accel_norm > DRV_ATTITUDE_FUSION_ACCEL_NORM_MAX_G)) {
        accelerometer = FUSION_VECTOR_ZERO;
        attitude_fusion_state.output.accel_norm_rejected = 1U;

        /*
         * Feeding a zero vector makes FusionAhrsUpdate() skip its whole
         * rejection/recovery block, so norm-gated samples would otherwise be
         * invisible to accelerationRecoveryTrigger. The counter could then never
         * reach the timeout and the built-in recovery never fired — attitude
         * stayed on gyro integration indefinitely. Advance the counter here so a
         * sustained gated stretch still latches recovery on the first sample
         * that gets through, mirroring the library's own rejected-sample branch.
         */
        if (attitude_fusion_state.ahrs.rejectionTimeout > 0) {
            int32_t trigger =
                attitude_fusion_state.ahrs.accelerationRecoveryTrigger + 1;
            if (trigger > attitude_fusion_state.ahrs.rejectionTimeout) {
                trigger = attitude_fusion_state.ahrs.rejectionTimeout;
            }
            attitude_fusion_state.ahrs.accelerationRecoveryTrigger = trigger;
        }
    }

    FusionAhrsSetSamplePeriod(&attitude_fusion_state.ahrs, input->dt_s);
    FusionAhrsUpdateNoMagnetometer(&attitude_fusion_state.ahrs,
                                   gyroscope,
                                   accelerometer);

    quaternion = FusionAhrsGetQuaternion(&attitude_fusion_state.ahrs);
    euler = FusionQuaternionToEuler(quaternion);
    internal_states =
        FusionAhrsGetInternalStates(&attitude_fusion_state.ahrs);
    flags = FusionAhrsGetFlags(&attitude_fusion_state.ahrs);

    attitude_fusion_state.output.time_us = input->time_us;
    attitude_fusion_state.output.quaternion[0] = quaternion.element.w;
    attitude_fusion_state.output.quaternion[1] = quaternion.element.x;
    attitude_fusion_state.output.quaternion[2] = quaternion.element.y;
    attitude_fusion_state.output.quaternion[3] = quaternion.element.z;
    attitude_fusion_state.output.roll_deg = euler.angle.roll;
    attitude_fusion_state.output.pitch_deg = euler.angle.pitch;
    attitude_fusion_state.output.yaw_deg = euler.angle.yaw;
    attitude_fusion_state.output.acceleration_error_deg =
        internal_states.accelerationError;
    attitude_fusion_state.output.acceleration_recovery_trigger =
        internal_states.accelerationRecoveryTrigger;
    attitude_fusion_state.output.sample_period_s = input->dt_s;
    ++attitude_fusion_state.output.sample_count;
    attitude_fusion_state.output.initialized = 1U;
    attitude_fusion_state.output.startup = flags.startup ? 1U : 0U;
    attitude_fusion_state.output.accelerometer_ignored =
        internal_states.accelerometerIgnored ? 1U : 0U;
    attitude_fusion_state.output.acceleration_recovery =
        flags.accelerationRecovery ? 1U : 0U;
    attitude_fusion_state.output.angular_rate_recovery =
        flags.angularRateRecovery ? 1U : 0U;
    if (attitude_fusion_state.output.accelerometer_ignored == 0U) {
        ++attitude_fusion_state.output.accel_correction_count;
    }

    if (output != NULL) {
        *output = attitude_fusion_state.output;
    }
    return 1U;
}

void DRV_AttitudeFusion_GetOutput(DRV_AttitudeFusionOutput *output)
{
    if (output != NULL) {
        *output = attitude_fusion_state.output;
    }
}

#include "drv_attitude_fusion.h"

#include "FusionAhrs.h"
#include "drv_mag_calibration.h"

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
 * Conservatively reject accelerometer directions that disagree with the
 * predicted gravity direction.  Impact/free-fall rejection is handled
 * separately by the acceleration-norm gate below.  Suitability during powered
 * flight remains an M7 same-data A/B validation item.
 */
#define DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG 10.0f
/*
 * Directional innovation gate for the magnetometer, mirroring
 * DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG above. This complements (does not
 * replace) the field-magnitude plausibility gate in
 * DRV_AttitudeFusion_Update(): magnitude alone cannot catch interference
 * that happens to sit inside [DRV_MAG_FIELD_MIN_MGAUSS,
 * DRV_MAG_FIELD_MAX_MGAUSS] but points the wrong way (nearby ferrous
 * hardware, motor phase current). Setting this unconditionally is safe even
 * before any magnetometer is calibrated: the library's own magnetic block
 * is skipped whenever the fed magnetometer vector is exactly zero (see the
 * magnetometer_valid handling below), so this threshold is inert until a
 * real magnetometer sample is actually fed.
 */
#define DRV_ATTITUDE_FUSION_MAG_REJECTION_DEG 10.0f
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
    DRV_AttitudeFusion_InitForConvention(
        DRV_ATTITUDE_FUSION_CONVENTION_NED);
}

void DRV_AttitudeFusion_InitForConvention(
    DRV_AttitudeFusionConvention convention)
{
    FusionAhrsSettings settings = fusionAhrsDefaultSettings;

    memset(&attitude_fusion_state, 0, sizeof(attitude_fusion_state));
    FusionAhrsInitialise(&attitude_fusion_state.ahrs);
    settings.sampleRate = DRV_ATTITUDE_FUSION_SAMPLE_RATE_HZ;
    settings.convention =
        (convention == DRV_ATTITUDE_FUSION_CONVENTION_NWU) ?
        FusionConventionNwu : FusionConventionNed;
    settings.gain = DRV_ATTITUDE_FUSION_GAIN;
    settings.gyroscopeRange = DRV_ATTITUDE_FUSION_GYRO_RANGE_DPS;
    settings.accelerationRejection =
        DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG;
    settings.magneticRejection = DRV_ATTITUDE_FUSION_MAG_REJECTION_DEG;
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

    {
        FusionVector magnetometer = FUSION_VECTOR_ZERO;
        uint8_t mag_subsystem_enabled = (input->magnetometer_valid != 0U) ?
            1U : 0U;
        uint8_t mag_field_rejected = 0U;
        uint8_t mag_used = 0U;

        if (mag_subsystem_enabled != 0U) {
            if (DRV_MAG_FieldMagnitude_InRange(input->magnetometer_mgauss) ==
                0U) {
                mag_field_rejected = 1U;

                /*
                 * Mirrors the accelerometer norm-gate fix above: advance the
                 * library's own magnetic recovery trigger manually. Feeding
                 * a zero vector makes FusionAhrsUpdate() skip its whole
                 * magnetic rejection/recovery block, so a sustained run of
                 * field-magnitude-gated samples (motor-current interference,
                 * a spike) would otherwise leave magneticRecoveryTrigger
                 * frozen instead of climbing -- the first good sample after
                 * a long gated stretch would then face the ordinary (not
                 * recovery) admission threshold even though heading may have
                 * drifted through the whole gated interval on gyro
                 * integration alone. Only advance while the subsystem is
                 * actually enabled (calibrated + axis-verified): a vehicle
                 * with no magnetometer configured at all has no recovery
                 * concept to bootstrap.
                 */
                if (attitude_fusion_state.ahrs.rejectionTimeout > 0) {
                    int32_t trigger =
                        attitude_fusion_state.ahrs.magneticRecoveryTrigger +
                        1;
                    if (trigger > attitude_fusion_state.ahrs.rejectionTimeout) {
                        trigger = attitude_fusion_state.ahrs.rejectionTimeout;
                    }
                    attitude_fusion_state.ahrs.magneticRecoveryTrigger =
                        trigger;
                }
            } else {
                magnetometer.axis.x = input->magnetometer_mgauss[0];
                magnetometer.axis.y = input->magnetometer_mgauss[1];
                magnetometer.axis.z = input->magnetometer_mgauss[2];
                mag_used = 1U;
            }
        }

        attitude_fusion_state.output.magnetometer_subsystem_enabled =
            mag_subsystem_enabled;
        attitude_fusion_state.output.magnetometer_field_rejected =
            mag_field_rejected;
        attitude_fusion_state.output.magnetometer_used = mag_used;

        FusionAhrsSetSamplePeriod(&attitude_fusion_state.ahrs, input->dt_s);
        if (mag_used != 0U) {
            FusionAhrsUpdate(&attitude_fusion_state.ahrs, gyroscope,
                             accelerometer, magnetometer);
        } else {
            /*
             * Identical call to every prior build whenever mag_used is 0
             * (subsystem disabled, or this sample's field magnitude was
             * rejected) -- this is the C4 safety requirement: default state
             * must produce attitude output bit-identical to a
             * magnetometer-free build.
             */
            FusionAhrsUpdateNoMagnetometer(&attitude_fusion_state.ahrs,
                                           gyroscope,
                                           accelerometer);
        }
    }

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
    attitude_fusion_state.output.magnetometer_ignored =
        internal_states.magnetometerIgnored ? 1U : 0U;
    attitude_fusion_state.output.magnetic_recovery =
        flags.magneticRecovery ? 1U : 0U;
    attitude_fusion_state.output.magnetic_error_deg =
        internal_states.magneticError;
    attitude_fusion_state.output.magnetic_recovery_trigger =
        internal_states.magneticRecoveryTrigger;
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

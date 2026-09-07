#ifndef DRV_FRAME_CONTRACT_H
#define DRV_FRAME_CONTRACT_H

#include <stdint.h>

/*
 * Canonical airframe body-frame contract for drone-H743.
 *
 * This header is the sole normative source for body-axis semantics.  Comments,
 * documents, estimator-library conventions, RC mappings, and actuator wiring
 * are adapters or descriptions; they must not redefine the body frame.
 *
 * Body frame: FLU, right-handed
 *   +X: forward
 *   +Y: left
 *   +Z: up
 *   +roll  about +X: right wing moves down
 *   +pitch about +Y: nose moves down
 *   +yaw   about +Z: nose turns left
 *
 * Body angular rates use the same signs: p=+roll, q=+pitch, r=+yaw.
 * Accelerometer values crossing the canonical boundary are specific force;
 * a stationary, level airframe therefore measures approximately [0, 0, +1] g.
 * Navigation/world frames are separate contracts and must be named explicitly.
 */

#define DRV_FRAME_CONTRACT_VERSION                         1U
#define DRV_FRAME_CANONICAL_BODY_IS_FLU                    1U
#define DRV_FRAME_BODY_IS_RIGHT_HANDED                     1U
#define DRV_FRAME_CANONICAL_ANGLE_UNIT_IS_RADIAN            1U
#define DRV_FRAME_CANONICAL_RATE_UNIT_IS_RAD_PER_SECOND     1U

#define DRV_FRAME_POSITIVE_ROLL_IS_RIGHT_WING_DOWN         1U
#define DRV_FRAME_POSITIVE_PITCH_IS_NOSE_DOWN              1U
#define DRV_FRAME_POSITIVE_YAW_IS_NOSE_LEFT                1U

#define DRV_FRAME_LEVEL_SPECIFIC_FORCE_X_G                 0.0f
#define DRV_FRAME_LEVEL_SPECIFIC_FORCE_Y_G                 0.0f
#define DRV_FRAME_LEVEL_SPECIFIC_FORCE_Z_G                 1.0f

/* Runtime migration is tracked per seam so completion cannot be hand-waved. */
#define DRV_FRAME_MIGRATION_SENSOR_TO_FLU_BIT               (1U << 0)
#define DRV_FRAME_MIGRATION_ESTIMATOR_ADAPTER_BIT           (1U << 1)
#define DRV_FRAME_MIGRATION_NAVIGATION_BIT                  (1U << 2)
#define DRV_FRAME_MIGRATION_CONTROLLER_BIT                  (1U << 3)
#define DRV_FRAME_MIGRATION_RC_ACTUATOR_BIT                 (1U << 4)
#define DRV_FRAME_MIGRATION_TELEMETRY_LOG_BIT               (1U << 5)
#define DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK           0x3FU

/*
 * The canonical contract is established, but most runtime seams above remain
 * legacy. Set bits only with a matching end-to-end executable/physical test.
 *
 * Author-directed validation override (2026-09-06): all bits are set before
 * the props-off V2 physical pass so the author can arm and manually validate
 * the chain. This only removes the migration arm lock; it is not flight
 * release, and the pending physical evidence remains tracked in PIPELINE.md.
 */
#define DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK               0x3FU
#define DRV_FRAME_RUNTIME_MIGRATION_COMPLETE \
    (((DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK & \
       DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK) == \
      DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK) ? 1U : 0U)

typedef enum {
    DRV_FRAME_AXIS_X_FORWARD = 0,
    DRV_FRAME_AXIS_Y_LEFT = 1,
    DRV_FRAME_AXIS_Z_UP = 2,
} DRV_FRAME_BodyAxis;

typedef struct {
    float x;
    float y;
    float z;
} DRV_FRAME_Vector3f;

static inline DRV_FRAME_Vector3f DRV_FRAME_Cross(
    DRV_FRAME_Vector3f lhs,
    DRV_FRAME_Vector3f rhs)
{
    DRV_FRAME_Vector3f result = {
        (lhs.y * rhs.z) - (lhs.z * rhs.y),
        (lhs.z * rhs.x) - (lhs.x * rhs.z),
        (lhs.x * rhs.y) - (lhs.y * rhs.x),
    };
    return result;
}

/*
 * Adapter for libraries or algorithms whose body axes are FRD
 * (X forward, Y right, Z down).  This is a proper 180-degree rotation about X,
 * so polar vectors and axial vectors use the same mapping.
 */
static inline DRV_FRAME_Vector3f DRV_FRAME_FluToFrd(
    DRV_FRAME_Vector3f value_flu)
{
    DRV_FRAME_Vector3f value_frd = {
        value_flu.x,
        -value_flu.y,
        -value_flu.z,
    };
    return value_frd;
}

static inline DRV_FRAME_Vector3f DRV_FRAME_FrdToFlu(
    DRV_FRAME_Vector3f value_frd)
{
    return DRV_FRAME_FluToFrd(value_frd);
}

#endif /* DRV_FRAME_CONTRACT_H */

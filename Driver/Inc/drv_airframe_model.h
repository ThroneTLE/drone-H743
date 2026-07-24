#ifndef DRV_AIRFRAME_MODEL_H
#define DRV_AIRFRAME_MODEL_H

#define DRV_AIRFRAME_BOARD_MASS_G                 75.0f
#define DRV_AIRFRAME_BATTERY_MASS_G              232.0f
#define DRV_AIRFRAME_BASE_MASS_G                  99.0f
#define DRV_AIRFRAME_SERVO_MOTOR_MASS_G          348.6f

#define DRV_AIRFRAME_BOARD_CG_Z_M                  0.0f
#define DRV_AIRFRAME_BATTERY_CG_Z_M                0.109f
#define DRV_AIRFRAME_BASE_CG_Z_M                  -0.117f
#define DRV_AIRFRAME_SERVO_MOTOR_CG_Z_M           -0.244f

#define DRV_AIRFRAME_MASS_KG                       1.2000f
#define DRV_AIRFRAME_CG_Z_M                       -0.0946f
#define DRV_AIRFRAME_IMU_Z_M                       0.0f

#define DRV_AIRFRAME_TETHER_ATTACH_Z_M             0.1563f
#define DRV_AIRFRAME_TETHER_ATTACH_TO_CG_M         0.2509f
#define DRV_AIRFRAME_TETHER_ROPE_M                 0.6400f
#define DRV_AIRFRAME_TETHER_ROD_TO_CG_M            0.8909f

#define DRV_AIRFRAME_SERVO_DEG_PER_US              0.090f
#define DRV_AIRFRAME_SERVO_US_PER_DEG             11.111111f

#define DRV_AIRFRAME_GRAVITY_M_S2                  9.81f
#define DRV_AIRFRAME_WEIGHT_N                     11.772000f
#define DRV_AIRFRAME_THRUST_TABLE_SCOPE            "dual_motor_total"
#define DRV_AIRFRAME_MAX_TOTAL_THRUST_G         1595.342f
#define DRV_AIRFRAME_MAX_TOTAL_FORCE_N            15.644959f
#define DRV_AIRFRAME_HOVER_THRUST_PERCENT         78.013457f

/* Servo axis z-positions (origin = board center, z+ up) */
#define DRV_AIRFRAME_SERVO1_AXIS_Z_M              -0.161f
#define DRV_AIRFRAME_SERVO2_AXIS_Z_M              -0.215f

/* Effective thrust application point (coax twin-prop midpoint) */
#define DRV_AIRFRAME_THRUST_POINT_Z_M             -0.2955f
#define DRV_AIRFRAME_THRUST_LEVER_ARM_M            0.201f

/* Estimated moments of inertia about CG (kg*m^2) */
#define DRV_AIRFRAME_IXX_KGM2                      0.019f
#define DRV_AIRFRAME_IYY_KGM2                      0.019f
#define DRV_AIRFRAME_IZZ_KGM2                      0.00035f

#endif /* DRV_AIRFRAME_MODEL_H */

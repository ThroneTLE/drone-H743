#ifndef MOTOR_HAMMERSTEIN_MODEL_H
#define MOTOR_HAMMERSTEIN_MODEL_H

#include <math.h>
#include <stdint.h>

#define MOTOR_HAMMERSTEIN_POINTS 21U

static const uint16_t motor_hammerstein_pwm_us[MOTOR_HAMMERSTEIN_POINTS] = {
    1100U, 1142U, 1184U, 1226U, 1268U, 1310U, 1352U, 1394U,
    1436U, 1478U, 1520U, 1562U, 1604U, 1646U, 1688U, 1730U,
    1772U, 1814U, 1856U, 1898U, 1940U,
};

static const float motor_hammerstein_pct[MOTOR_HAMMERSTEIN_POINTS] = {
    0.000f, 5.000f, 10.000f, 15.000f, 20.000f, 25.000f,
    30.000f, 35.000f, 40.000f, 45.000f, 50.000f, 55.000f,
    60.000f, 65.000f, 70.000f, 75.000f, 80.000f, 85.000f,
    90.000f, 95.000f, 100.000f,
};

static const float motor_hammerstein_thrust_g[MOTOR_HAMMERSTEIN_POINTS] = {
    -3.150f, 1.863f, 33.425f, 70.675f, 121.575f, 178.037f,
    247.025f, 316.838f, 402.713f, 489.463f, 575.925f, 673.675f,
    777.475f, 870.825f, 955.525f, 1046.763f, 1134.875f, 1217.062f,
    1303.075f, 1379.288f, 1418.675f,
};

static const float motor_hammerstein_tau_s = 0.000000f;

static inline float motor_hammerstein_interp_f32(
    const float *xs, const float *ys, uint32_t count, float x)
{
    if (count == 0U) {
        return 0.0f;
    }
    if (x <= xs[0]) {
        return ys[0];
    }
    for (uint32_t i = 1U; i < count; ++i) {
        if (x <= xs[i]) {
            const float dx = xs[i] - xs[i - 1U];
            const float ratio = (dx > 0.0f) ? ((x - xs[i - 1U]) / dx) : 0.0f;
            return ys[i - 1U] + ratio * (ys[i] - ys[i - 1U]);
        }
    }
    return ys[count - 1U];
}

static inline float motor_hammerstein_interp_pwm_to_thrust(uint16_t pwm_us)
{
    if (pwm_us <= motor_hammerstein_pwm_us[0]) {
        return motor_hammerstein_thrust_g[0];
    }
    for (uint32_t i = 1U; i < MOTOR_HAMMERSTEIN_POINTS; ++i) {
        if (pwm_us <= motor_hammerstein_pwm_us[i]) {
            const float left = (float)motor_hammerstein_pwm_us[i - 1U];
            const float right = (float)motor_hammerstein_pwm_us[i];
            const float ratio = (((float)pwm_us) - left) / (right - left);
            return motor_hammerstein_thrust_g[i - 1U] +
                   ratio * (motor_hammerstein_thrust_g[i] - motor_hammerstein_thrust_g[i - 1U]);
        }
    }
    return motor_hammerstein_thrust_g[MOTOR_HAMMERSTEIN_POINTS - 1U];
}

static inline float motor_hammerstein_interp_pct_to_thrust(float pct)
{
    return motor_hammerstein_interp_f32(
        motor_hammerstein_pct, motor_hammerstein_thrust_g, MOTOR_HAMMERSTEIN_POINTS, pct);
}

static inline float motor_hammerstein_interp_thrust_to_pct(float thrust_g)
{
    return motor_hammerstein_interp_f32(
        motor_hammerstein_thrust_g, motor_hammerstein_pct, MOTOR_HAMMERSTEIN_POINTS, thrust_g);
}

static inline uint16_t motor_hammerstein_interp_thrust_to_pwm(float thrust_g)
{
    if (thrust_g <= motor_hammerstein_thrust_g[0]) {
        return motor_hammerstein_pwm_us[0];
    }
    for (uint32_t i = 1U; i < MOTOR_HAMMERSTEIN_POINTS; ++i) {
        if (thrust_g <= motor_hammerstein_thrust_g[i]) {
            const float left = motor_hammerstein_thrust_g[i - 1U];
            const float right = motor_hammerstein_thrust_g[i];
            const float ratio = (right > left) ? ((thrust_g - left) / (right - left)) : 0.0f;
            const float pwm = (float)motor_hammerstein_pwm_us[i - 1U] +
                              ratio * ((float)motor_hammerstein_pwm_us[i] -
                                       (float)motor_hammerstein_pwm_us[i - 1U]);
            return (uint16_t)(pwm + 0.5f);
        }
    }
    return motor_hammerstein_pwm_us[MOTOR_HAMMERSTEIN_POINTS - 1U];
}

static inline float motor_hammerstein_update_lag(
    float thrust_est_g, float thrust_ss_g, float dt_s)
{
    if ((dt_s <= 0.0f) || (motor_hammerstein_tau_s <= 0.0f)) {
        return thrust_ss_g;
    }
    const float beta = 1.0f - expf(-dt_s / motor_hammerstein_tau_s);
    return thrust_est_g + beta * (thrust_ss_g - thrust_est_g);
}

#endif

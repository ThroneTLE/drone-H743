#ifndef DRV_MOTOR_MODEL_H
#define DRV_MOTOR_MODEL_H

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
    -0.048f, 19.510f, 51.971f, 98.257f, 154.719f, 218.567f,
    289.319f, 378.271f, 457.081f, 545.305f, 647.957f, 732.914f,
    833.371f, 919.348f, 1003.343f, 1079.143f, 1156.662f, 1226.329f,
    1297.967f, 1349.614f, 1363.410f,
};

static const float motor_hammerstein_tau_s = 0.176624f;

static inline float motor_hammerstein_interp_f32(
    const float *xs, const float *ys, uint32_t count, float x)
{
    if (count == 0U) return 0.0f;
    if (x <= xs[0]) return ys[0];
    for (uint32_t i = 1U; i < count; ++i) {
        if (x <= xs[i]) {
            const float dx = xs[i] - xs[i - 1U];
            const float ratio = (dx > 0.0f) ? ((x - xs[i - 1U]) / dx) : 0.0f;
            return ys[i - 1U] + ratio * (ys[i] - ys[i - 1U]);
        }
    }
    return ys[count - 1U];
}

static inline float motor_hammerstein_interp_thrust_to_pct(float thrust_g)
{
    return motor_hammerstein_interp_f32(
        motor_hammerstein_thrust_g, motor_hammerstein_pct,
        MOTOR_HAMMERSTEIN_POINTS, thrust_g);
}

static inline uint16_t motor_hammerstein_interp_thrust_to_pwm(float thrust_g)
{
    if (thrust_g <= motor_hammerstein_thrust_g[0])
        return motor_hammerstein_pwm_us[0];
    for (uint32_t i = 1U; i < MOTOR_HAMMERSTEIN_POINTS; ++i) {
        if (thrust_g <= motor_hammerstein_thrust_g[i]) {
            const float left  = motor_hammerstein_thrust_g[i - 1U];
            const float right = motor_hammerstein_thrust_g[i];
            const float ratio = (right > left) ? ((thrust_g - left) / (right - left)) : 0.0f;
            const float pwm   = (float)motor_hammerstein_pwm_us[i - 1U] +
                                ratio * ((float)motor_hammerstein_pwm_us[i] -
                                         (float)motor_hammerstein_pwm_us[i - 1U]);
            return (uint16_t)(pwm + 0.5f);
        }
    }
    return motor_hammerstein_pwm_us[MOTOR_HAMMERSTEIN_POINTS - 1U];
}

#endif /* DRV_MOTOR_MODEL_H */

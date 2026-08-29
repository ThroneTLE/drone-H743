# Motor Hammerstein Fit Report

Source: `D:\stm32hal\drone-H743\tools\0002.csv`
Tail samples for steady state: 25
Tail trim fraction: 0.100
Tau fit z window: 0.10..0.90

## Static Lookup

Points: 21
Throttle range: 0.0%..100.0%
Pulse range: 1100us..1940us
Thrust range: -0.0g..1363.4g
Max tail std: 2.7g at 95.0%
Max first-to-tail drift: 53.5g at 50.0%

Use the `steady_mean_g` column as the static nonlinearity `f(u)`.

## Dynamic Lag

Fitted transitions: 20
Selected transitions for global tau: 10
Recommended global tau: 0.1766s

## Prediction Error

Full-response samples: 2100
Full-response RMSE: 23.084g
Full-response MAE: 10.133g
Full-response max abs error: 104.959g
Tail samples: 525
Tail RMSE: 1.047g
Tail MAE: 0.662g
Tail max abs error: 4.014g

## C Lookup Snippet

```c
#define MOTOR_HAMMERSTEIN_POINTS 21U

static const uint16_t motor_pwm_us_table[MOTOR_HAMMERSTEIN_POINTS] = {
    1100U, 1142U, 1184U, 1226U, 1268U, 1310U, 1352U, 1394U,
    1436U, 1478U, 1520U, 1562U, 1604U, 1646U, 1688U, 1730U,
    1772U, 1814U, 1856U, 1898U, 1940U,
};

static const float motor_pct_table[MOTOR_HAMMERSTEIN_POINTS] = {
    0.000f, 5.000f, 10.000f, 15.000f, 20.000f, 25.000f,
    30.000f, 35.000f, 40.000f, 45.000f, 50.000f, 55.000f,
    60.000f, 65.000f, 70.000f, 75.000f, 80.000f, 85.000f,
    90.000f, 95.000f, 100.000f,
};

static const float motor_thrust_g_table[MOTOR_HAMMERSTEIN_POINTS] = {
    -0.048f, 19.510f, 51.971f, 98.257f, 154.719f, 218.567f,
    289.319f, 378.271f, 457.081f, 545.305f, 647.957f, 732.914f,
    833.371f, 919.348f, 1003.343f, 1079.143f, 1156.662f, 1226.329f,
    1297.967f, 1349.614f, 1363.410f,
};

static const float motor_tau_s = 0.176624f;
```

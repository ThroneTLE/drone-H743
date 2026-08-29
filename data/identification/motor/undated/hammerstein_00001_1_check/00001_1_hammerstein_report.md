# Motor Hammerstein Fit Report

Source: `D:\stm32hal\drone-H743\tools\00001_1.csv`
Tail samples for steady state: 10
Tail trim fraction: 0.100
Tau fit z window: 0.10..0.90

## Static Lookup

Points: 21
Throttle range: 0.0%..100.0%
Pulse range: 1100us..1940us
Thrust range: -3.2g..1418.7g
Max tail std: 20.4g at 50.0%
Max first-to-tail drift: 59.5g at 60.0%

Use the `steady_mean_g` column as the static nonlinearity `f(u)`.

## Dynamic Lag

Fitted transitions: 20
Selected transitions for global tau: 0
Recommended global tau: unavailable

## Prediction Error

Full-response samples: 420
Full-response RMSE: 31.960g
Full-response MAE: 24.925g
Full-response max abs error: 72.075g
Tail samples: 210
Tail RMSE: 14.230g
Tail MAE: 11.287g
Tail max abs error: 32.625g

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
    -3.150f, 1.863f, 33.425f, 70.675f, 121.575f, 178.037f,
    247.025f, 316.838f, 402.713f, 489.463f, 575.925f, 673.675f,
    777.475f, 870.825f, 955.525f, 1046.763f, 1134.875f, 1217.062f,
    1303.075f, 1379.288f, 1418.675f,
};
```

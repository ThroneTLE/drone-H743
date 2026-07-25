# Flight Log System Identification Summary

- CSV: `D:\stm32hal\drone-H743\flightlog_20260725_134804.csv`
- rows: 9158
- columns: 131
- wall-clock span: 231.300 s
- nominal log rate: 250.0 Hz
- observed contiguous rate: 113.6 Hz
- sequence missing: 11024
- dropped_records delta: 11024
- export complete: True

## Flags
- log is discontinuous: 10 time segments
- flight recorder dropped 11024 records before export
- tilt saturation is significant; avoid linear PID fits in saturated samples
- no direct range/height channel found; Z position-loop identification is limited

## Segments
|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|
|---:|---:|---:|---:|---:|---:|---:|---|---|
|0|366|3.483|104.8|11054|11925|506|stabilized_mix|direct_throttle|
|1|831|7.236|114.7|11926|13735|979|stabilized_mix|direct_throttle|
|2|626|5.344|117.0|13736|15072|711|stabilized_mix|disarmed_min|
|3|567|4.792|118.1|15076|16274|632|stabilized_mix|direct_throttle|
|4|2119|18.881|112.2|16284|21004|2602|stabilized_mix|direct_throttle|
|5|1202|10.552|113.8|21005|23643|1437|stabilized_mix|direct_throttle|
|6|524|4.472|116.9|23650|24768|595|stabilized_mix|direct_throttle|
|7|1114|9.907|112.3|24769|27246|1364|stabilized_mix|direct_throttle|
|8|999|8.896|112.2|27249|29473|1226|stabilized_mix|disarmed_min|
|9|810|7.012|115.4|29482|31235|944|stabilized_mix|direct_throttle|

## Gain Groups
|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|roll_angle_kp=-0.8, roll_rate_kd=0.8, pitch_angle_kp=-1, pitch_rate_kd=0.3, pos_z_kp=3.8, yaw_rate_kd=0.5|369|3.491|5.591|4.964|33.073|11.538|24.5|0.0|15.4|
|roll_angle_kp=-0.6, roll_rate_kd=0.4, pitch_angle_kp=-0.6, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|828|7.224|6.722|4.043|23.993|21.828|2.0|0.1|7.1|
|roll_angle_kp=-0.6, roll_rate_kd=0.4, pitch_angle_kp=-0.2, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|3312|29.016|4.337|2.585|15.526|13.565|0.8|0.0|3.6|
|roll_angle_kp=-0.1, roll_rate_kd=0.4, pitch_angle_kp=-0.1, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|4649|40.840|5.814|3.685|21.062|17.623|1.1|0.7|5.3|

## Actuator Fits
|label|input|output|slope|intercept|r2|corr|count|
|---|---|---|---:|---:|---:|---:|---:|
|body_x_tilt_to_servo_beta|ctrl_tilt_out_rad_0|servo_beta_us|-636.664|1500|0.9999|-1.0000|9158|
|body_y_tilt_to_servo_alpha|ctrl_tilt_out_rad_1|servo_alpha_us|-636.637|1500|1.0000|-1.0000|9158|
|total_force_to_motor_upper|ctrl_total_force_n|motor_upper_us|53.4649|1110.61|0.7976|0.8931|9158|
|total_force_to_motor_lower|ctrl_total_force_n|motor_lower_us|51.0284|1105.09|0.8024|0.8958|9158|


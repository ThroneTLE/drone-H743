# Flight Log System Identification Summary

- CSV: `D:\stm32hal\drone-H743\flightlog_20260725_141334.csv`
- rows: 9154
- columns: 131
- wall-clock span: -359.228 s
- nominal log rate: 250.0 Hz
- observed contiguous rate: 112.7 Hz
- sequence missing: 11184
- dropped_records delta: -5898
- export complete: True

## Flags
- log is discontinuous: 10 time segments
- flight recorder dropped 11184 records before export
- no direct range/height channel found; Z position-loop identification is limited

## Segments
|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|
|---:|---:|---:|---:|---:|---:|---:|---|---|
|0|504|4.517|111.4|19875|21004|626|stabilized_mix|direct_throttle|
|1|1202|10.552|113.8|21005|23643|1437|stabilized_mix|direct_throttle|
|2|524|4.472|116.9|23650|24768|595|stabilized_mix|direct_throttle|
|3|1114|9.907|112.3|24769|27246|1364|stabilized_mix|direct_throttle|
|4|999|8.896|112.2|27249|29473|1226|stabilized_mix|disarmed_min|
|5|810|7.012|115.4|29482|31235|944|stabilized_mix|direct_throttle|
|6|1212|10.820|111.9|1|2706|1494|stabilized_mix|direct_throttle|
|7|1061|9.500|111.6|2707|5082|1315|stabilized_mix|direct_throttle|
|8|866|7.784|111.1|5083|7029|1081|stabilized_mix|direct_throttle|
|9|862|7.787|110.6|7030|8977|1086|stabilized_mix|direct_throttle|

## Gain Groups
|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|roll_angle_kp=-0.6, roll_rate_kd=0.4, pitch_angle_kp=-0.2, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|2781|24.848|4.925|1.806|14.346|9.622|0.1|0.0|6.7|
|roll_angle_kp=-0.6, roll_rate_kd=0.4, pitch_angle_kp=-0.6, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|860|7.780|2.895|3.281|17.644|13.309|0.0|0.0|7.3|
|roll_angle_kp=-0.3, roll_rate_kd=0.4, pitch_angle_kp=-0.2, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|864|7.772|4.638|3.377|16.778|11.668|0.2|0.7|6.9|
|roll_angle_kp=-0.1, roll_rate_kd=0.4, pitch_angle_kp=-0.1, pitch_rate_kd=0.4, pos_z_kp=3.8, yaw_rate_kd=0.5|4649|40.840|5.814|3.685|21.062|17.623|1.1|0.7|5.3|

## Actuator Fits
|label|input|output|slope|intercept|r2|corr|count|
|---|---|---|---:|---:|---:|---:|---:|
|body_x_tilt_to_servo_beta|ctrl_tilt_out_rad_0|servo_beta_us|-636.61|1500|0.9999|-1.0000|9154|
|body_y_tilt_to_servo_alpha|ctrl_tilt_out_rad_1|servo_alpha_us|-636.627|1500|1.0000|-1.0000|9154|
|total_force_to_motor_upper|ctrl_total_force_n|motor_upper_us|52.9166|1119.91|0.8939|0.9455|9154|
|total_force_to_motor_lower|ctrl_total_force_n|motor_lower_us|50.2973|1119.25|0.8944|0.9457|9154|


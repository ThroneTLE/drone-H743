# Flight Log System Identification Summary

- CSV: `D:\stm32hal\drone-H743\log\flightlog_20260725_161555.csv`
- rows: 8139
- columns: 139
- wall-clock span: -155.580 s
- nominal log rate: 250.0 Hz
- observed contiguous rate: 111.2 Hz
- sequence missing: 10160
- dropped_records delta: -1130
- export complete: True

## Flags
- log is discontinuous: 2 time segments
- flight recorder dropped 10160 records before export
- no direct range/height channel found; Z position-loop identification is limited

## Segments
|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|
|---:|---:|---:|---:|---:|---:|---:|---|---|
|0|4320|38.916|111.0|11049|20778|5410|stabilized_mix|direct_throttle|
|1|3819|34.273|111.4|1|8569|4750|stabilized_mix|direct_throttle|

## Gain Groups
|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|roll_angle_kp=-0.8, roll_rate_kd=0.299, pitch_angle_kp=-0.8, pitch_rate_kd=0.3, pos_z_kp=3.8, yaw_rate_kd=0.5|8139|73.189|4.091|2.077|15.762|12.412|0.4|1.3|1.5|

## Actuator Fits
|label|input|output|slope|intercept|r2|corr|count|
|---|---|---|---:|---:|---:|---:|---:|
|body_x_tilt_to_servo_beta|ctrl_tilt_out_rad_0|servo_beta_us|-636.521|1500.01|0.9999|-0.9999|8139|
|body_y_tilt_to_servo_alpha|ctrl_tilt_out_rad_1|servo_alpha_us|-636.657|1499.99|1.0000|-1.0000|8139|
|total_force_to_motor_upper|ctrl_total_force_n|motor_upper_us|53.4224|1120.03|0.9530|0.9762|8139|
|total_force_to_motor_lower|ctrl_total_force_n|motor_lower_us|50.6791|1111.27|0.9558|0.9777|8139|


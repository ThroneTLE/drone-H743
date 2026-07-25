# Flight Log System Identification Summary

- CSV: `flightlog_20260724_200832.csv`
- rows: 12207
- columns: 102
- wall-clock span: 1066.992 s
- nominal log rate: 250.0 Hz
- observed contiguous rate: 170.8 Hz
- sequence missing: 5677
- dropped_records delta: 5677
- export complete: True

## Flags
- log is discontinuous: 10 time segments
- flight recorder dropped 5677 records before export
- tilt saturation is significant; avoid linear PID fits in saturated samples
- motor high saturation is significant; thrust authority is a limiting factor
- velocity loop is disabled in this log
- angle-P tilt contribution is zero; angle KP is acting through force terms or disabled
- no direct range/height channel found; Z position-loop identification is limited

## Segments
|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|
|---:|---:|---:|---:|---:|---:|---:|---|---|
|0|2276|13.656|166.6|27421|30835|1139|stabilized_mix|direct_throttle|
|1|2016|11.892|169.4|30842|33815|958|stabilized_mix|direct_throttle|
|2|1102|6.393|172.2|33822|35420|497|stabilized_mix|direct_throttle|
|3|1043|6.048|172.3|35421|36933|470|stabilized_mix|direct_throttle|
|4|495|2.788|177.2|36934|37631|203|stabilized_mix|direct_throttle|
|5|893|5.184|172.1|37632|38928|404|stabilized_mix|direct_throttle|
|6|799|4.604|173.3|38929|40080|353|stabilized_mix|direct_throttle|
|7|1356|7.908|171.3|40081|42058|622|stabilized_mix|direct_throttle|
|8|1153|6.723|171.3|42059|43740|529|stabilized_mix|direct_throttle|
|9|1074|6.252|171.6|43741|45304|490|stabilized_mix|direct_throttle|

## Gain Groups
|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|roll_angle_kp=-10, roll_rate_kd=-5, pitch_angle_kp=-10, pitch_rate_kd=-5, pos_z_kp=0, yaw_rate_kd=0.15|1044|6.048|3.188|4.344|21.129|46.068|63.5|33.6|8.0|
|roll_angle_kp=-10, roll_rate_kd=-3.2, pitch_angle_kp=-10, pitch_rate_kd=-3.2, pos_z_kp=0, yaw_rate_kd=0.15|1104|6.397|3.112|2.715|18.129|23.640|45.9|52.0|7.6|
|roll_angle_kp=-10, roll_rate_kd=-1.5, pitch_angle_kp=-10, pitch_rate_kd=-1.5, pos_z_kp=0, yaw_rate_kd=0.15|900|5.208|1.969|3.068|12.342|41.101|0.1|59.7|9.3|
|roll_angle_kp=-10, roll_rate_kd=-0.5, pitch_angle_kp=-10, pitch_rate_kd=-0.5, pos_z_kp=0, yaw_rate_kd=0.15|492|2.772|3.579|7.569|31.608|54.824|0.0|64.0|17.1|
|roll_angle_kp=-8, roll_rate_kd=-1, pitch_angle_kp=-8, pitch_rate_kd=-1, pos_z_kp=1, yaw_rate_kd=0.15|2280|13.668|2.662|3.815|14.888|29.864|0.5|7.5|3.7|
|roll_angle_kp=-8, roll_rate_kd=-1, pitch_angle_kp=-8, pitch_rate_kd=-1, pos_z_kp=0, yaw_rate_kd=0.15|2016|11.888|3.078|5.856|19.723|68.143|0.3|37.2|4.1|
|roll_angle_kp=-5, roll_rate_kd=-1.5, pitch_angle_kp=-5, pitch_rate_kd=-1.5, pos_z_kp=0, yaw_rate_kd=0.15|792|4.572|1.928|3.201|15.730|34.020|8.0|25.5|10.6|
|roll_angle_kp=-1.5, roll_rate_kd=-1, pitch_angle_kp=-1.5, pitch_rate_kd=-1, pos_z_kp=0, yaw_rate_kd=0.5|1071|6.240|4.414|2.081|14.124|15.662|0.0|27.4|7.8|
|roll_angle_kp=-1, roll_rate_kd=-1.5, pitch_angle_kp=-1, pitch_rate_kd=-1.5, pos_z_kp=0, yaw_rate_kd=0.15|1356|7.904|1.943|1.629|8.724|18.289|1.2|3.9|6.3|
|roll_angle_kp=-0, roll_rate_kd=-1, pitch_angle_kp=-0, pitch_rate_kd=-1, pos_z_kp=0, yaw_rate_kd=0.5|1152|6.716|2.992|4.397|13.839|30.401|0.0|28.4|7.3|

## Actuator Fits
|label|input|output|slope|intercept|r2|corr|count|
|---|---|---|---:|---:|---:|---:|---:|
|body_x_tilt_to_servo_beta|ctrl_tilt_out_rad_0|servo_beta_us|-570.624|1501.04|0.8934|-0.9452|12207|
|body_y_tilt_to_servo_alpha|ctrl_tilt_out_rad_1|servo_alpha_us|-548.679|1505.05|0.8709|-0.9332|12207|
|total_force_to_motor_upper|ctrl_total_force_n|motor_upper_us|49.5082|1123.11|0.4145|0.6438|12207|
|total_force_to_motor_lower|ctrl_total_force_n|motor_lower_us|52.2112|1073.72|0.4545|0.6742|12207|


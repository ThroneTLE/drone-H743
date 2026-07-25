# Flight Log System Identification Summary

- CSV: `log\flightlog_20260725_075151.csv`
- rows: 9154
- columns: 120
- wall-clock span: 289.344 s
- nominal log rate: 250.0 Hz
- observed contiguous rate: 124.3 Hz
- sequence missing: 9288
- dropped_records delta: 9284
- export complete: False

## Flags
- log is discontinuous: 10 time segments
- flight recorder dropped 9288 records before export
- export has missing byte ranges
- motor high saturation is significant; thrust authority is a limiting factor
- velocity loop is disabled in this log
- no direct range/height channel found; Z position-loop identification is limited

## Segments
|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|
|---:|---:|---:|---:|---:|---:|---:|---|---|
|0|1637|13.748|119.0|10889|14326|1801|stabilized_mix|direct_throttle|
|1|1482|12.176|121.6|14327|17371|1563|stabilized_mix|direct_throttle|
|2|1864|15.132|123.1|17374|21157|1920|stabilized_mix|direct_throttle|
|3|472|3.616|130.3|21158|22062|433|stabilized_mix|direct_throttle|
|4|487|3.760|129.3|22063|23003|454|stabilized_mix|direct_throttle|
|5|593|4.681|126.5|23004|24174|578|stabilized_mix|direct_throttle|
|6|166|1.056|156.3|24175|24439|99|stabilized_mix|direct_throttle|
|7|1258|10.176|123.5|24448|26992|1287|stabilized_mix|direct_throttle|
|8|412|3.128|131.4|26993|27775|371|stabilized_mix|direct_throttle|
|9|783|6.184|126.5|27784|29330|764|stabilized_mix|direct_throttle|

## Gain Groups
|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|roll_angle_kp=-1.5, roll_rate_kd=0.3, pitch_angle_kp=-0.5, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|513|3.776|1.273|2.734|12.170|10.968|5.0|0.0|12.9|
|roll_angle_kp=-1.5, roll_rate_kd=0.3, pitch_angle_kp=-1, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|720|5.616|1.460|1.831|9.455|10.567|3.6|1.2|8.8|
|roll_angle_kp=-0.7, roll_rate_kd=0.3, pitch_angle_kp=-1, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|324|2.672|2.144|6.846|25.823|33.189|2.6|17.6|19.4|
|roll_angle_kp=-0.7, roll_rate_kd=0.3, pitch_angle_kp=-0, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|2611|20.512|4.228|5.859|12.912|11.088|1.5|0.0|9.7|
|roll_angle_kp=-0.5, roll_rate_kd=0.11, pitch_angle_kp=-0.5, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|1638|13.748|2.389|3.099|13.938|13.756|0.0|0.0|3.9|
|roll_angle_kp=-0.5, roll_rate_kd=0.3, pitch_angle_kp=-0.5, pitch_rate_kd=0.11, pos_z_kp=3.8, yaw_rate_kd=0.5|3348|27.312|1.433|3.149|11.962|17.320|0.0|1.3|3.8|

## Actuator Fits
|label|input|output|slope|intercept|r2|corr|count|
|---|---|---|---:|---:|---:|---:|---:|
|body_x_tilt_to_servo_beta|ctrl_tilt_out_rad_0|servo_beta_us|-636.614|1500|0.9998|-0.9999|9154|
|body_y_tilt_to_servo_alpha|ctrl_tilt_out_rad_1|servo_alpha_us|-636.647|1500|1.0000|-1.0000|9154|
|total_force_to_motor_upper|ctrl_total_force_n|motor_upper_us|53.3619|1130.41|0.9657|0.9827|9154|
|total_force_to_motor_lower|ctrl_total_force_n|motor_lower_us|49.8979|1132.45|0.9672|0.9834|9154|


# IMU attitude fusion data

This directory stores VOFA or optional read-only OpenOCD IMU captures and
x-io Fusion diagnostic reports produced by `tools/imu_attitude_tuner.py`.

`imu_vertical_shake_20260727_130109.csv` is the first real vertical-shake
regression sample. It was captured read-only from the running STM32H743 at
approximately 100 Hz. The removed estimator retained a 4.46 degree tilt error
after the aircraft was put down. It predates Fusion diagnostic channels and
cannot validate the replacement algorithm.

The matching files contain:

- `_meta.json`: capture provenance and immutable SHA-256.
- `_fusion_analysis.json`: data quality, Fusion diagnostic availability,
  landing residuals, and whether the current settings need review.

Generated captures should use timestamped names so existing evidence is never
overwritten.

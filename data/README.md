# Project Data Directory

All project-owned captures, logs, calibration data, identification datasets, and derived reports live under this directory. Host tools obtain these paths from `tools/project_paths.py`; do not add new `tools/data`, root-level `captures*`, `saleae_capture*`, or `log` directories.

Run artifacts use one sortable date bucket below their category:

```text
category/YYYY-MM-DD/files-for-that-day
```

Readers search category roots recursively. Files with no trustworthy date token belong in `undated/`. Stable files such as calibration state, manuals, and README files remain at the category root. `python tools/organize_data.py` checks the structure; add `--apply` to organize newly imported legacy files.

| Directory | Purpose | Default consumers |
|---|---|---|
| `flight_logs/YYYY-MM-DD/` | FlightLog BIN/CSV/metadata exports | Flight-log receiver, waveform, system-identification, workbench, Rerun tools |
| `captures/imu_attitude/YYYY-MM-DD/` | VOFA/OpenOCD attitude and Fusion captures | VOFA capture and IMU attitude tuner |
| `captures/imu_vibration/YYYY-MM-DD/` | Full-rate raw IMU captures and filter reports | IMU vibration capture/UI/report tools |
| `captures/vofa/YYYY-MM-DD/` | General VOFA telemetry captures | Offline telemetry analysis |
| `captures/usb_flight_logs/YYYY-MM-DD/` | Curated USB FlightLog captures | Flight-log parser tests and manual analysis |
| `captures/saleae_*/YYYY-MM-DD/` | Saleae bus captures grouped by purpose | Saleae capture/decoder tools |
| `identification/attitude/YYYY-MM-DD/` | Attitude excitation runs | Ground-station panel and attitude fitting tools |
| `identification/motor/YYYY-MM-DD/` | Motor Hammerstein input data and fitted outputs | Motor model fitting tool |
| `identification/thrust/YYYY-MM-DD/` | Thrust and dual-prop identification data | Pressure GUI and thrust viewer |
| `calibration/pressure/` | Pressure sensor calibration and reference material | Pressure GUI/test tools |
| `calibration/airframe/YYYY-MM-DD/` | Read-only airframe sensor-validation sessions and reports | Ground-station V0 validation page |
| `calibration/imu_metrology/YYYY-MM-DD/<session>/` | V1 IMUCAP raw CSV/meta, resumable manifests and evidence-only candidates | Ground-station V1 metrology page |
| `calibration/flight_acceptance_v2/YYYY-MM-DD/<session>/` | V2A snapshots, physical confirmations, and strict reports | Ground-station V2A page and offline acceptance engine |
| `calibration/servo_mechanical/YYYY-MM-DD/` | 舵机机械中心、方向与安全行程的人工确认记录 | Ground-station 舵机机械校准页 |
| `calibration/flow_range/YYYY-MM-DD/` | 光流/组合测距坐标、比例、零偏与旋转补偿的地面采样证据 | Ground-station 光流/测距校准页 |
| `telemetry/YYYY-MM-DD/` | Ad-hoc barometer/GPS exports | Ground-station panel |
| `analysis/.../YYYY-MM-DD/` | Derived reports and Rerun artifacts | Offline analysis tools |
| `logs/.../YYYY-MM-DD/` | Miscellaneous local runtime logs | Manual diagnostics |
| `firmware_updates/YYYY-MM-DD/` | USB ROM DFU programming/verification logs | Ground-station firmware-update page |

Generated runtime logs and analysis output are ignored by Git where appropriate. Curated datasets already under version control remain tracked. `.tmp/` stays separate because it contains disposable environments, external reference trees, and scratch work rather than project data.

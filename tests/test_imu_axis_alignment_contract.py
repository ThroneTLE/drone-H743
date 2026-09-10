"""Legacy IMU intermediate-axis adapter checks.

The canonical body-frame contract is Driver/Inc/drv_frame_contract.h.  These
checks preserve the current pre-migration adapter as evidence; they do not
define a second body frame.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_legacy_imu_mount_axis_adapter_is_documented() -> None:
    source = read("App/Src/app_sensor.c")
    header = read("App/Inc/app_sensor.h")

    # Mounting corrected by R-F0 to match the physically measured, persisted V0
    # result (orientation code 3).  The pre-V0 text claimed +Y down and +Z aft,
    # which is a 180 deg error about Y; see
    # tests/test_flu_seam0_sensor_frame.py.
    assert "IMU +X 朝飞机左方" in source
    assert "IMU +Y 朝飞机上方" in source
    assert "IMU +Z 朝飞机前方" in source
    assert "本机标定中间轴" in source
    # The legacy compensation must stay documented as the unmigrated fallback.
    assert "roll rate = -gyro X, pitch rate = +gyro Y, yaw rate = +gyro Z" in source
    assert "specific force = [-accel X, +accel Y, -accel Z]" in source
    assert "姿态最终正负号以实机补偿后的输出为准" in header


def test_imu_axes_are_rotated_to_legacy_intermediate_frame() -> None:
    source = read("App/Src/app_sensor.c")

    # R-F0 relabelled these from "body" to the intermediate frame's real name:
    # the mapping's output is legacy_intermediate_v1, not the body frame.
    assert "legacy_intermediate_v1 X = -imu Z" in source
    assert "legacy_intermediate_v1 Y = -imu X" in source
    assert "legacy_intermediate_v1 Z =  imu Y" in source
    # 这三行本身搬到了 Services/svc_imu.c：MicoAir743v2 上板载 IMU 换成了
    # BMI088/BMI270，芯片轴 → 中间轴的映射必须随芯片走，而不再是一份写死的。
    # ICM-42688 那一支必须逐字节保持原样，老板子的 V0 实测证据依赖它。
    service = read("Services/Src/svc_imu.c")
    assert "out[0] = -in[2];" in service
    assert "out[1] = -in[0];" in service
    assert "out[2] =  in[1];" in service
    assert "SVC_IMU_ChipToIntermediate(BSP_IMU_GetChipKind(), in, out);" in source


def test_legacy_intermediate_alignment_is_right_handed_and_forward_positive() -> None:
    intermediate_x_in_imu = (0.0, 0.0, -1.0)
    intermediate_y_in_imu = (-1.0, 0.0, 0.0)
    intermediate_z_in_imu = (0.0, 1.0, 0.0)

    cross_xy = (
        intermediate_x_in_imu[1] * intermediate_y_in_imu[2] -
        intermediate_x_in_imu[2] * intermediate_y_in_imu[1],
        intermediate_x_in_imu[2] * intermediate_y_in_imu[0] -
        intermediate_x_in_imu[0] * intermediate_y_in_imu[2],
        intermediate_x_in_imu[0] * intermediate_y_in_imu[1] -
        intermediate_x_in_imu[1] * intermediate_y_in_imu[0],
    )

    assert cross_xy == intermediate_z_in_imu

    imu_forward_accel = (0.0, 0.0, -1.0)
    intermediate_forward_accel = -imu_forward_accel[2]

    assert intermediate_forward_accel > 0.0

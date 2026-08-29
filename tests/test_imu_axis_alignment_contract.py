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

    assert "IMU +Y 朝飞机下方" in source
    assert "IMU +Z 朝飞机后方" in source
    assert "IMU +X 朝飞机左方" in source
    assert "本机标定中间轴" in source
    assert "roll rate = -gyro X, pitch rate = +gyro Y, yaw rate = +gyro Z" in source
    assert "specific force = [-accel X, +accel Y, -accel Z]" in source
    assert "姿态最终正负号以实机补偿后的输出为准" in header


def test_imu_axes_are_rotated_to_legacy_intermediate_frame() -> None:
    source = read("App/Src/app_sensor.c")

    assert "body X = -imu Z" in source
    assert "body Y = -imu X" in source
    assert "body Z =  imu Y" in source
    assert "out[0] = -in[2];" in source
    assert "out[1] = -in[0];" in source
    assert "out[2] =  in[1];" in source


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

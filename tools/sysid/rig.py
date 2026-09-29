"""台架几何 —— 与固件 `Driver/Inc/drv_sysid_rig.h` 同一套约定。

这份是主机侧的镜像，用来在**不连飞控**的情况下预演与复核。两边的判据必须一致，
由 `tests/test_sysid_rig_geometry.py` 对拍；不一致时以固件为准，因为实际发出去的
力矩是它算的。

坐标一律是规范 FLU 机体系（+X 前、+Y 左、+Z 上）。ψ 是杆轴在 XY 平面内相对 +X
的方位角，绕 +Z 为正。当前光杆台架 ψ = 45°。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

AZIMUTH_45_RAD = math.pi / 4.0


@dataclass(frozen=True)
class Rig:
    """一套台架的完整物理条件。存进辨识档案的就是这几个数。"""

    azimuth_rad: float = AZIMUTH_45_RAD
    #: 杆轴相对整机质心的垂直偏移 [m]，+ 为杆在质心上方。名义 0。
    axis_offset_above_cg_m: float = 0.0
    #: 飞控/IMU 相对整机质心的垂直偏移 [m]，取自 `airframe.imu_z_m - airframe.cg_z_m`。
    imu_above_cg_m: float = 0.0
    name: str = "光杆架-45度"

    @property
    def azimuth_deg(self) -> float:
        return math.degrees(self.azimuth_rad)

    def axis(self) -> tuple[float, float, float]:
        """杆轴单位向量 n = (cos ψ, sin ψ, 0)。"""
        return (math.cos(self.azimuth_rad), math.sin(self.azimuth_rad), 0.0)

    def effective_inertia(self, inertia_diag: tuple[float, float, float]) -> float:
        """绕杆轴的有效惯量 nᵀJn。

        对称机体（Jxx≈Jyy）且 n_z=0 时结果恒等于 Jxx——**绕 45° 轴辨出来的惯量
        直接就是 Ixx/Iyy**，不必分两个轴做。这是"XY 耦合、惯量当一致"这个前提
        在数学上的出口。
        """
        nx, ny, nz = self.axis()
        jxx, jyy, jzz = inertia_diag
        return jxx * nx * nx + jyy * ny * ny + jzz * nz * nz

    def project_rate(self, omega: tuple[float, float, float]) -> float:
        """实测角速度在杆轴上的投影 ω·n —— 这套系统唯一的自由度。"""
        nx, ny, nz = self.axis()
        return omega[0] * nx + omega[1] * ny + omega[2] * nz

    def rate_residual(self, omega: tuple[float, float, float]) -> float:
        """角速度垂直于杆轴的残差范数 ‖ω − (ω·n)n‖。

        理想台架恒为 0。超过门限说明杆不在 ψ 方向、机体没夹紧、或轴向填错了——
        该趟数据必须作废而不是拿去拟合。这是"执行结果符不符合"的第一道机检。
        """
        nx, ny, nz = self.axis()
        along = self.project_rate(omega)
        rx = omega[0] - along * nx
        ry = omega[1] - along * ny
        rz = omega[2] - along * nz
        return math.sqrt(rx * rx + ry * ry + rz * rz)

    def project_attitude(self, roll_rad: float, pitch_rad: float) -> float:
        """小角度下姿态矢量在杆轴上的投影，即绕杆轴的转角 θ。

        固件里限位判据用的就是它（陀螺积分会漂，几秒下来限位就不可信了）。
        """
        nx, ny, _nz = self.axis()
        return roll_rad * nx + pitch_rad * ny

    def moment_about_axis(self, moment_n_m: float) -> tuple[float, float, float]:
        """绕杆轴的力矩 τ_n -> 机体系力矩矢量 τ_n·n。

        垂直于 n 的分量会被光杆的约束力全部吃掉：不产生运动，只产生轴承载荷。
        所以激励必须打在 n 上，**两个舵机因此是联动的**。
        """
        nx, ny, nz = self.axis()
        return (moment_n_m * nx, moment_n_m * ny, moment_n_m * nz)

    def gravity_moment(self, mass_kg: float, gravity_m_s2: float,
                       angle_rad: float) -> float:
        """单摆恢复力矩 −m·g·d·sin θ。杆过质心（d=0）时恒为 0。

        拟合出来的 d 若接近 0，那就是"杆确实过质心"的证据；不为 0 则给出偏心量。
        这个量不在这里假定，由数据说话。
        """
        d = self.axis_offset_above_cg_m
        if d == 0.0:
            return 0.0
        return -mass_kg * gravity_m_s2 * d * math.sin(angle_rad)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "azimuth_deg": self.azimuth_deg,
            "axis_offset_above_cg_m": self.axis_offset_above_cg_m,
            "imu_above_cg_m": self.imu_above_cg_m,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Rig":
        if "azimuth_deg" in data:
            azimuth = math.radians(float(data["azimuth_deg"]))
        else:
            azimuth = float(data.get("azimuth_rad", AZIMUTH_45_RAD))
        return cls(
            azimuth_rad=azimuth,
            axis_offset_above_cg_m=float(data.get("axis_offset_above_cg_m", 0.0)),
            imu_above_cg_m=float(data.get("imu_above_cg_m", 0.0)),
            name=str(data.get("name", "光杆架-45度")),
        )


def imu_lever_arm_error(rig: Rig, omega: tuple[float, float, float],
                        alpha: tuple[float, float, float]) -> tuple[float, float, float]:
    """IMU 因为不在质心上而多读到的比力 ω̇×r + ω×(ω×r)，r=(0,0,imu_above_cg)。

    杆过质心时质心的线加速度为零，所以这两项就是**姿态角估计误差的主要来源**。
    角速度是刚体不变量、不受影响——这正是"内环辨识以陀螺为准、姿态只做交叉校验
    与限位"的理由。

    返回值单位 m/s²。用它扣掉之后仍然对不上，说明 `airframe.imu_z_m` 或
    `airframe.cg_z_m` 填错了。
    """
    r = (0.0, 0.0, rig.imu_above_cg_m)

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    tangential = cross(alpha, r)
    centripetal = cross(omega, cross(omega, r))
    return tuple(tangential[i] + centripetal[i] for i in range(3))

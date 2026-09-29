"""一键开始要下发的那一串配置怎么组装：激励、采样、限位、模式、台架几何、程序油门。

台架几何和目标推力都依赖机体参数（PARAM? 回显），所以做成"轮到它时才生成"的
可调用项，由 `Workflow.advance()` 在发送前调用；生成时顺手把要核对的回显值写进
`expected`。缺数据时抛中文 ValueError，事务会整体取消，不发 START。

**舵机单独（SERVO）。** 电机不转，所以全程要**保持上锁**（解锁会被固件立即停止，
原因 rc_arm）：不下发程序油门、不做油门/解锁检查；激励命令末尾追加
`servo_tilt_mrad=<舵机摆幅>`，回显逐项核对。

**高度（ALT，槽式台架）。** 要程序油门（手动油门直接拒绝）；激励幅值按注入类型的单位与上限
检查（`alt_config.check_amplitude`），不套用按角速度给的幅值建议。`SYSID MODE ALT` 之后多发一条
`SYSID ALT inject=.. mass_g=.. win_mm=.. lift_mm=..`：mass_g = 读回的机体质量 + 台架随动附加质量，
轮到它时才生成（要等机体参数回来），回显只有一行，由 `AltWorkflow.alt_handle_line` 逐项核对。
非 ALT 轮不发这条。
"""
from __future__ import annotations

import math

from .geometry import fc_above_cg_m, metres
from .settings_store import MODES

#: 舵机摆幅（固件 EXC 键 servo_tilt_mrad）的默认值与范围 [mrad]。
SERVO_TILT_DEFAULT_MRAD = 87
SERVO_TILT_RANGE_MRAD = (10, 262)


def servo_tilt_mrad(text: str) -> int:
    """「舵机摆幅 [deg]」→ 固件的 mrad 整数；越界抛中文 ValueError。"""
    try:
        degrees = float(text)
    except (TypeError, ValueError):
        raise ValueError("舵机摆幅要填数字（单位 °）") from None
    mrad = int(round(math.radians(degrees) * 1000.0)) if math.isfinite(degrees) else -1
    low, high = SERVO_TILT_RANGE_MRAD
    if not low <= mrad <= high:
        raise ValueError(f"舵机摆幅要在 {math.degrees(low / 1000):.1f}°～{math.degrees(high / 1000):.1f}°"
                         f"（{low}～{high} mrad）之间")
    return mrad

GRAVITY_M_S2 = 9.80665


class StartConfig:
    def start(self):
        p = self.page
        if self.run_id is not None and self.end is None:
            p.status_var.set("当前轮次尚未结束；请先停止并等待结束报告")
            return
        transport = getattr(p.panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            p.status_var.set("未连接飞控：先连接，再点「开始辨识」")
            p.refresh_banner()
            return
        try:
            spec = p.excitation()
            if spec is None:
                raise ValueError("激励设置无效：到「高级设置」检查激励参数")
            values = [float(v.get()) for v in (p.psi_var, p.angle_limit_var,
                      p.resid_limit_var, p.angle_amp_var)]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("物理条件必须是有限数字")
            rate = int(p.rate_var.get())
            inertia = float(p.inertia_var.get() or "0")
            if not math.isfinite(inertia) or inertia < 0:
                raise ValueError("假定惯量无效")
            mode = p.mode_var.get()
            if mode not in MODES:
                raise ValueError("辨识模式无效")
            servo = mode == "SERVO"
            if servo:
                thr = p._current_thr()
                if thr is not None and str(thr.get("armed", "")).strip() == "1":
                    raise ValueError("舵机单独模式要保持上锁：先用遥控器上锁，再点「开始辨识」"
                                     "（电机不转；解锁会立即停止）。")
                tilt_mrad = servo_tilt_mrad(p.servo_tilt_var.get())
            psi, angle, resid, amp = values
            if not 0 < amp <= 15 or not 0 < angle <= 30 or amp >= angle:
                raise ValueError("目标角度应在 0～15°，且小于保护角度")
            if not 50 <= rate <= 500:
                raise ValueError("采样率应在 50～500Hz；无线链路最多 100Hz")
            self.run_note = ""
            if rate > 100 and not p.link_is_usb():
                # 固件在非 USB 链路上拒绝 >100 Hz；本轮自动降，不改你在高级设置里填的值。
                rate = 100
                self.run_note = "（当前不是 USB 连接，本轮线上采样率自动降到 100 Hz）"
            self.rig_request = (psi,) + p.geometry_inputs()
            self.pivot_request = p.pivot_inputs()
            self.throttle = (True, None, 75.0) if servo else p.throttle_settings()
            alt = self.alt_prepare(p, spec)
            self.expected = dict(mode=MODES.index(mode), rate_hz=rate,
                psi_mrad=int(math.radians(psi)*1000),
                angle_mrad=int(math.radians(angle)*1000),
                resid_mrad_s=int(math.radians(resid)*1000), profile=spec.profile,
                amp_mrad_s=int(spec.amplitude_rad_s*1000), ramp_ms=spec.ramp_ms,
                hold_ms=spec.hold_ms, repeat=spec.repeat, seed=spec.prbs_seed,
                bit_ms=spec.prbs_bit_ms, dur_ms=spec.duration_ms,
                f0_mhz=int(spec.chirp_f0_hz*1000), f1_mhz=int(spec.chirp_f1_hz*1000),
                angle_amp_mrad=int(math.radians(amp)*1000))
            excitation = spec.command()
            if servo:
                excitation += f" servo_tilt_mrad={tilt_mrad}"
                self.expected["servo_tilt_mrad"] = tilt_mrad
            p.save_rig_settings()      # 输入都过了检查、马上开始发命令：此刻记住台架设置
            p.schema = None
            p.schema_lines = []
            # 参数快照要是这块板子此刻的回显：先扔掉上次读到的机体参数（可能是换固件之前的
            # 旧力臂键），PARAM? 再整份读回来。
            self._forget_airframe_params()
            # RIG 排在 PARAM? 之后的第一份状态报告之后：到那时机体参数一定已经收齐。
            # 舵机单独：电机不转，不下发程序油门。高度：MODE 之后发 SYSID ALT（同样等机体质量）。
            self.submit(["SYSID SCHEMA", "PARAM?", excitation,
                f"SYSID RATE {rate}", f"SYSID INERTIA {inertia:g}",
                f"SYSID LIMIT angle_deg={angle:g} resid_dps={resid:g}",
                f"SYSID MODE {mode} {amp:g}"] + ([self._alt_command] if alt else [])
                + [self._rig_command] + ([] if servo else [self._throttle_command]), "start")
        except (ValueError, OverflowError) as error:
            self.notice = str(error)
            self.fail(str(error))

    def send_rig(self, psi, rod_to_fc, override):
        """单独下发台架几何：先取机体参数和一份状态报告，再组装 RIG 并核对回显。"""
        if self.awaiting or self.job:
            self.page.status_var.set("正在等待飞控回显或分析结束，请稍等")
            return
        self.rig_request = (psi, rod_to_fc, override)
        self.expected = dict(psi_mrad=int(math.radians(psi)*1000))
        self._forget_airframe_params()
        self.submit(["PARAM?", "SYSID?", self._rig_command], "rig")

    def rig_numbers(self, rod_to_fc, override):
        """`(d, 飞控到质心, 来源)`；缺机体参数时抛中文 ValueError。"""
        found = fc_above_cg_m(self.params, self.ready)
        if found is None:
            raise ValueError("读不到飞控到质心的距离（机体参数 airframe.imu_z_m − airframe.cg_z_m）："
                             "先到「机体模型」页确认参数已下发到飞控，再重试。")
        fc, source = found
        return (override if override is not None else rod_to_fc + fc), fc, source

    def _rig_command(self):
        psi, rod_to_fc, override = self.rig_request
        d, fc, _source = self.rig_numbers(rod_to_fc, override)
        self.expected.update(axis_off_um=round(d*1e6), imu_off_um=round(fc*1e6))
        # 倾转轴到质心 = 倾转轴到飞控板（尺量）+ 飞控到质心，与杆的算法相同；没填的留 None。
        self.pivot_to_cg = tuple(None if value is None else value + fc
                                 for value in self.pivot_request)
        self.page.refresh_geometry_hint()
        return f"SYSID RIG psi_deg={psi:g} axis_off_m={metres(d)} imu_off_m={metres(fc)}"

    def weight_n(self):
        """机重 [N]：优先用参数回显的 airframe.weight_n，退而用状态报告里的质量。"""
        for source in (lambda: float(self.params["airframe.weight_n"]),
                       lambda: float(self.status["mass_mg"])*1e-6*GRAVITY_M_S2):
            try:
                value = source()
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value) and value > 0:
                return value
        return None

    def _throttle_command(self):
        manual, target, max_pct = self.throttle
        if manual:
            target = 0.0
        elif target is None:
            weight = self.weight_n()
            if weight is None:
                raise ValueError("读不到机重：请在「准备」页的「目标合推力」里手填一个数（单位 N）")
            target = round(weight, 1)
        self.target_n = target
        self.expected.update(auto=0 if manual else 1, target_cn=int(round(target*100)),
                             max_pct_x10=int(round(max_pct*10)))
        return f"SYSID THROTTLE target_n={target:g} max_pct={max_pct:g}"

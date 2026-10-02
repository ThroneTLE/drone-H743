"""光流安装方向：由 +X/+Y 两步推荐、写入飞控、读回确认（R-FLOWMOUNT-1）。

**为什么要有它。** 光流模块换了安装位置（例如转了 90°）以后，固件按最初安装写死的
"芯片坐标 → 机体 FLU"换算就不对了：往左推，飞控以为往后走。以前标定页只能诊断
"正向符号错误"，没有任何参数能改。现在固件多了两项 airframe.flow_mount_yaw_deg /
airframe.flow_mount_mirror（只在 app_optical_flow.c 的方言边界生效），本模块负责：

  1. 两步都采完后，用 ``recommend_flow_mount`` 算出该写的安装参数并讲清依据；
  2. 「写入飞控（需上锁）」：先现读一次 ARM 状态确认上锁 → 两条 PARAM SET 逐条核对
     回显 → PARAM? 读回两项都对上才算成功；任何一步不对就停，已写的那条如实报告；
  3. 写入前后值、依据写进证据 JSON（target_parameters_written=true）；
  4. 写完提示重采两步，两步都显示正向符号正确才算验证通过。

**为什么是单独模块。** flow_ranging.py 只留五个挂钩（建页、开始/结束一步、存证据、
实时行显示），状态机和文字都在这里；drone_tcp_panel.py 一行不改——飞控回包从
board_line_hooks 取整行文本。

**采样时的安装参数从哪来。** 固件在 ``FLOW ok`` 那一行行尾报 ``mount_yaw=``/
``mount_mirror=``（实际生效值）。每一步采样期间看到的值都记下来：推荐必须以"那两步
是在什么参数下采的"为准，不能拿"飞控现在是什么"去套旧数据——写入之后两者就不同了，
这时旧的两步只能作为依据留档，推荐按钮灰掉，提示重采。
"""

from __future__ import annotations

import copy
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

from ..board_line_hooks import register_board_line_hook
from ..proto import (
    PROTO_REQ_PARAM_SET,
    PROTO_REQ_PARAMS,
    PROTO_REQ_STATUS,
    parse_kv,
    safe_float,
)

try:
    from ...ground_calibration import (
        GroundCalibrationError,
        describe_flow_mount,
        recommend_flow_mount,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.ground_calibration import (
            GroundCalibrationError,
            describe_flow_mount,
            recommend_flow_mount,
        )
    except ImportError:
        from ground_calibration import (
            GroundCalibrationError,
            describe_flow_mount,
            recommend_flow_mount,
        )


FLOW_MOUNT_YAW_PARAM = "airframe.flow_mount_yaw_deg"
FLOW_MOUNT_MIRROR_PARAM = "airframe.flow_mount_mirror"
FLOW_MOUNT_STAGES = ("forward_x", "left_y")

# 与主页解锁横幅同一种读法（arm_banner.py），id 不同只是为了在日志里分得清是谁问的；
# 飞控的回包固定 id=0（app_cmd_arm.c），所以按 mod=ARM 认。
FLOW_MOUNT_ARM_REQUEST = "REQ id=9102 mod=ARM op=STATUS"

# 每一步等回包的上限。PARAM? 一次回七十来行，给得宽一些。
FLOW_MOUNT_ARM_TIMEOUT_MS = 2000
FLOW_MOUNT_SET_TIMEOUT_MS = 3000
FLOW_MOUNT_READBACK_TIMEOUT_MS = 5000

_STEP_TEXT = {
    "arm": "读取解锁状态",
    "set_yaw": "写入安装转角",
    "set_mirror": "写入安装镜像",
    "readback": "PARAM? 读回",
}


def _mount_tuple(values: dict[str, str]) -> tuple[int, int] | None:
    """FLOW 行里的 (mount_yaw, mount_mirror)；老固件没有这两个键时返回 None。"""
    yaw = values.get("mount_yaw")
    mirror = values.get("mount_mirror")
    if yaw is None or mirror is None:
        return None
    try:
        return int(yaw), int(mirror)
    except ValueError:
        return None


def _same_number(text: str | None, expected: int) -> bool:
    return text is not None and safe_float(text, float("nan")) == float(expected)


class FlowMountPanelMixin:
    """挂在 FlowRangingPageMixin 上；所有状态都以 ``flow_mount_`` 开头，懒创建。"""

    # ── 状态 ──────────────────────────────────────────────────────────
    def _flow_mount_init_state(self) -> None:
        if getattr(self, "_flow_mount_state_ready", False):
            return
        self._flow_mount_state_ready = True
        self.flow_mount_live: tuple[int, int] | None = None
        # 每一步采样期间看到过的安装参数；多于一个 = 采到一半参数被改了。
        self.flow_mount_stage_mounts: dict[str, set[tuple[int, int]]] = {
            stage: set() for stage in FLOW_MOUNT_STAGES
        }
        self.flow_mount_recommendation: dict[str, object] | None = None
        self.flow_mount_reason = "等待 +X、+Y 两步都采完"
        self.flow_mount_write_record: dict[str, object] | None = None
        self.flow_mount_verification: dict[str, object] | None = None
        self.flow_mount_txn: dict[str, object] | None = None
        self.flow_mount_txn_token = 0
        self.flow_mount_timer = None
        if not hasattr(self, "flow_mount_text_var"):
            self.flow_mount_text_var = tk.StringVar(value=self.flow_mount_reason)
            self.flow_mount_status_var = tk.StringVar(value="")

    # ── 页面 ──────────────────────────────────────────────────────────
    def _build_flow_mount_panel(self, parent: ttk.Frame) -> None:
        self._flow_mount_init_state()
        box = ttk.LabelFrame(parent, text="光流安装方向（由 +X / +Y 两步推荐）", padding=8)
        box.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(
            box, textvariable=self.flow_mount_text_var,
            wraplength=1080, justify=tk.LEFT,
        ).pack(fill=tk.X)
        row = ttk.Frame(box)
        row.pack(fill=tk.X, pady=(6, 0))
        self.flow_mount_write_button = ttk.Button(
            row, text="写入飞控（需上锁）", command=self._flow_mount_write,
            style="Warning.TButton", state=tk.DISABLED,
        )
        self.flow_mount_write_button.pack(side=tk.LEFT)
        ttk.Label(
            row, textvariable=self.flow_mount_status_var,
            style="Guide.TLabel", wraplength=900, justify=tk.LEFT,
        ).pack(side=tk.LEFT, padx=(12, 0))
        register_board_line_hook(self, self._flow_mount_handle_board_line)
        self._flow_mount_refresh()

    # ── flow_ranging.py 的挂钩 ────────────────────────────────────────
    def _flow_mount_on_stage_start(self, stage: str) -> None:
        self._flow_mount_init_state()
        if stage in self.flow_mount_stage_mounts:
            # 只记"开始之后"看到的值，不拿开始前的旧值起头：两步之间参数可能被改过，
            # 旧值会让这一步被误判成"采样期间变过"。每次 FLOW? 回包的状态行都先于补偿行
            # 到达，一步至少 5 个样本，漏不掉。
            self.flow_mount_stage_mounts[stage] = set()
        self._flow_mount_refresh()

    def _flow_mount_on_stage_result(self, stage: str, result: dict[str, object]) -> None:
        """把这一步采样期间的安装参数记进结果本身（随证据 JSON 落盘），再重算推荐。"""
        self._flow_mount_init_state()
        if stage in self.flow_mount_stage_mounts:
            seen = sorted(self.flow_mount_stage_mounts[stage])
            if len(seen) == 1:
                result["flow_mount_yaw_deg"], result["flow_mount_mirror"] = seen[0]
            elif seen:
                result["flow_mount_mixed"] = [list(item) for item in seen]
        self._flow_mount_refresh()

    def _flow_mount_evidence(self) -> dict[str, object]:
        self._flow_mount_init_state()
        written = self.flow_mount_write_record is not None
        return {
            "target_parameters_written": written,
            "flow_mount": {
                "live_yaw_deg": None if self.flow_mount_live is None else self.flow_mount_live[0],
                "live_mirror": None if self.flow_mount_live is None else self.flow_mount_live[1],
                "recommendation": copy.deepcopy(self.flow_mount_recommendation),
                "recommendation_unavailable_reason": (
                    None if self.flow_mount_recommendation is not None else self.flow_mount_reason
                ),
                "write": copy.deepcopy(self.flow_mount_write_record),
                "verification": copy.deepcopy(self.flow_mount_verification),
            },
        }

    # ── 推荐 ──────────────────────────────────────────────────────────
    def _flow_mount_capture_mount(self) -> tuple[tuple[int, int] | None, str]:
        """两步采样时的安装参数；拿不到时返回 (None, 原因)。"""
        results = getattr(self, "flow_cal_results", {})
        mounts = []
        for stage, label in (("forward_x", "+X"), ("left_y", "+Y")):
            result = results.get(stage)
            if result is None:
                return None, "等待 +X、+Y 两步都采完"
            if "flow_mount_mixed" in result:
                return None, f"{label} 步采样期间安装参数变过，请重采这一步"
            if "flow_mount_yaw_deg" not in result:
                return None, (
                    f"{label} 步采样时固件没有上报安装参数（FLOW 行缺 mount_yaw/mount_mirror），"
                    "请先更新固件再采"
                )
            mounts.append((int(result["flow_mount_yaw_deg"]), int(result["flow_mount_mirror"])))
        if mounts[0] != mounts[1]:
            return None, (
                f"两步是在不同的安装参数下采的（+X：{describe_flow_mount(*mounts[0])}；"
                f"+Y：{describe_flow_mount(*mounts[1])}），请在同一组参数下重采两步"
            )
        return mounts[0], ""

    def _flow_mount_refresh(self) -> None:
        self._flow_mount_init_state()
        self.flow_mount_recommendation = None
        self.flow_mount_verification = None
        capture, reason = self._flow_mount_capture_mount()
        write = self.flow_mount_write_record
        lines: list[str] = []
        if write is not None:
            after = write["after"]
            lines.append(
                f"本次已写入：{describe_flow_mount(after['yaw_deg'], after['mirror'])}"
                f"（写入前：{describe_flow_mount(write['before']['yaw_deg'], write['before']['mirror'])}）"
            )
        if capture is None:
            self.flow_mount_reason = reason
            lines.append(f"暂无推荐：{reason}")
            self._flow_mount_set_text(lines, enable=False)
            return

        if self.flow_mount_live is not None and self.flow_mount_live != capture:
            self.flow_mount_reason = (
                f"飞控现在是{describe_flow_mount(*self.flow_mount_live)}，而这两步是在"
                f"{describe_flow_mount(*capture)}下采的：请重采 +X/+Y 两步验证"
            )
            lines.append(self.flow_mount_reason)
            self._flow_mount_set_text(lines, enable=False)
            return

        results = self.flow_cal_results
        forward, left = results["forward_x"], results["left_y"]
        if write is not None and capture == (write["after"]["yaw_deg"], write["after"]["mirror"]):
            passed = bool(forward.get("positive_sign_ok")) and bool(left.get("positive_sign_ok"))
            self.flow_mount_verification = {
                "yaw_deg": capture[0],
                "mirror": capture[1],
                "forward_positive_sign_ok": bool(forward.get("positive_sign_ok")),
                "left_positive_sign_ok": bool(left.get("positive_sign_ok")),
                "passed": passed,
            }
            lines.append(
                "写入后重采验证：" + ("通过——+X、+Y 两步正向符号都正确" if passed else
                                    "未通过——两步都显示正向符号正确才算通过")
            )
        try:
            recommendation = recommend_flow_mount(
                (float(forward["observed_distance_m"]), float(forward["cross_axis_distance_m"])),
                (float(left["cross_axis_distance_m"]), float(left["observed_distance_m"])),
                capture[0], capture[1],
            )
        except (GroundCalibrationError, KeyError, TypeError, ValueError) as exc:
            self.flow_mount_reason = str(exc)
            lines.append(f"暂无推荐：{exc}")
            self._flow_mount_set_text(lines, enable=False)
            return

        self.flow_mount_recommendation = recommendation
        basis = (
            f"依据：+X 步位移 {recommendation['forward_distance_m']:.3f} m，纠正后余弦 "
            f"{recommendation['forward_cosine']:.2f}、主/串轴比 {recommendation['forward_axis_ratio']:.1f}；"
            f"+Y 步位移 {recommendation['left_distance_m']:.3f} m，纠正后余弦 "
            f"{recommendation['left_cosine']:.2f}、主/串轴比 {recommendation['left_axis_ratio']:.1f}"
        )
        if recommendation["changed"]:
            self.flow_mount_reason = ""
            lines.append(
                f"建议写入：{recommendation['description']}"
                f"（当前：{recommendation['current_description']}）"
            )
            lines.append(basis)
            self._flow_mount_set_text(lines, enable=True)
        else:
            self.flow_mount_reason = "当前安装参数已与两步采样一致，无需写入"
            lines.append(f"当前安装参数已正确：{recommendation['description']}，无需写入")
            lines.append(basis)
            self._flow_mount_set_text(lines, enable=False)

    def _flow_mount_set_text(self, lines: list[str], *, enable: bool) -> None:
        self.flow_mount_text_var.set("\n".join(lines))
        self._flow_mount_set_button(enable)

    def _flow_mount_set_button(self, enable: bool) -> None:
        """写入进行中一律灰掉，免得连点两次发出两组命令。"""
        button = getattr(self, "flow_mount_write_button", None)
        if button is None:
            return
        allowed = enable and self.flow_mount_txn is None
        try:
            button.configure(state=tk.NORMAL if allowed else tk.DISABLED)
        except tk.TclError:
            pass

    # ── 写入事务 ──────────────────────────────────────────────────────
    def _flow_mount_write(self) -> None:
        self._flow_mount_init_state()
        recommendation = self.flow_mount_recommendation
        if self.flow_mount_txn is not None:
            return
        if recommendation is None or not recommendation.get("changed"):
            self.flow_mount_status_var.set(f"没有可写入的推荐：{self.flow_mount_reason}")
            return
        if not self._transport_connected():
            self.flow_mount_status_var.set("请先连接飞控")
            return
        before = (int(recommendation["current_yaw_deg"]), int(recommendation["current_mirror"]))
        after = (int(recommendation["yaw_deg"]), int(recommendation["mirror"]))
        confirmed = messagebox.askokcancel(
            "写入光流安装方向",
            f"将把光流安装参数从「{describe_flow_mount(*before)}」改为"
            f"「{describe_flow_mount(*after)}」。\n\n"
            "它决定光流速度的方向，写错会让定点/定速往反方向修正。飞控会先确认处于上锁"
            "状态，写入后自动保存到 Flash。\n\n确定写入吗？",
            icon=messagebox.WARNING,
        )
        if not confirmed:
            self.flow_mount_status_var.set("已取消写入")
            return
        self.flow_mount_txn_token += 1
        self.flow_mount_txn = {
            "token": self.flow_mount_txn_token,
            "step": "",
            "before": before,
            "after": after,
            "recommendation": copy.deepcopy(recommendation),
            "readback": {},
            "set_ok": [],
        }
        self._flow_mount_set_button(False)
        self._flow_mount_advance("arm")

    def _flow_mount_advance(self, step: str) -> bool:
        txn = self.flow_mount_txn
        if txn is None:
            return False
        txn["step"] = step
        self._flow_mount_cancel_timer()
        if step == "arm":
            sent = self._send_proto_silent(PROTO_REQ_STATUS, FLOW_MOUNT_ARM_REQUEST)
            timeout = FLOW_MOUNT_ARM_TIMEOUT_MS
        elif step in {"set_yaw", "set_mirror"}:
            name = FLOW_MOUNT_YAW_PARAM if step == "set_yaw" else FLOW_MOUNT_MIRROR_PARAM
            value = txn["after"][0] if step == "set_yaw" else txn["after"][1]
            payload = f"PARAM SET {name} {value}"
            sent = self._flow_mount_send(PROTO_REQ_PARAM_SET, payload)
            timeout = FLOW_MOUNT_SET_TIMEOUT_MS
        else:
            txn["readback"] = {}
            sent = self._flow_mount_send(PROTO_REQ_PARAMS, "PARAM?")
            timeout = FLOW_MOUNT_READBACK_TIMEOUT_MS
        if not sent:
            self._flow_mount_fail(f"{_STEP_TEXT[step]}的命令没有发出去（连接断开或被验收会话拦下）")
            return False
        self.flow_mount_status_var.set(f"正在{_STEP_TEXT[step]}…")
        token = txn["token"]
        self.flow_mount_timer = self.after(
            timeout, lambda t=token, s=step: self._flow_mount_timeout(t, s)
        )
        return True

    def _flow_mount_send(self, function: int, payload: str) -> bool:
        if not self._transport_connected() or not self._validation_command_allowed(payload):
            return False
        if hasattr(self, "_mark_param_pending") and payload.startswith("PARAM SET "):
            _, _, name, value = payload.split(" ", 3)
            self._mark_param_pending(name, value)
        self._send_proto(function, payload)
        return True

    def _flow_mount_timeout(self, token: int, step: str) -> None:
        txn = self.flow_mount_txn
        if txn is None or txn["token"] != token or txn["step"] != step:
            return
        self.flow_mount_timer = None
        self._flow_mount_fail(f"{_STEP_TEXT[step]}超时，没有等到飞控回包")

    def _flow_mount_cancel_timer(self) -> None:
        timer, self.flow_mount_timer = self.flow_mount_timer, None
        if timer is not None:
            try:
                self.after_cancel(timer)
            except (tk.TclError, ValueError):
                pass

    def _flow_mount_fail(self, reason: str) -> None:
        txn = self.flow_mount_txn
        written = [] if txn is None else list(txn.get("set_ok", []))
        self.flow_mount_txn = None
        self._flow_mount_cancel_timer()
        suffix = ""
        if written:
            suffix = f"；已被飞控接受的：{'、'.join(written)}——请读 FLOW? 核对当前值后再决定"
        self.flow_mount_status_var.set(f"写入未完成：{reason}{suffix}")
        self._flow_mount_refresh()

    def _flow_mount_succeed(self) -> None:
        txn = self.flow_mount_txn
        assert txn is not None
        self.flow_mount_txn = None
        self._flow_mount_cancel_timer()
        before, after = txn["before"], txn["after"]
        results = getattr(self, "flow_cal_results", {})
        self.flow_mount_write_record = {
            "written_at": datetime.now().astimezone().isoformat(),
            "parameters": [FLOW_MOUNT_YAW_PARAM, FLOW_MOUNT_MIRROR_PARAM],
            "before": {"yaw_deg": before[0], "mirror": before[1]},
            "after": {"yaw_deg": after[0], "mirror": after[1]},
            "readback": dict(txn["readback"]),
            "recommendation": txn["recommendation"],
            "basis_stages": {
                stage: copy.deepcopy(results[stage]) for stage in FLOW_MOUNT_STAGES
                if stage in results
            },
        }
        # 飞控已经是新值（读回对上了）：推荐与"采样时的参数"对比时按它来。
        self.flow_mount_live = after
        self.flow_mount_status_var.set(
            f"已写入并读回确认：{describe_flow_mount(*after)}（飞控会自动存 Flash）。"
            "请重采 +X/+Y 两步验证，两步都显示正向符号正确才算通过"
        )
        self._flow_mount_refresh()

    # ── 回包 ──────────────────────────────────────────────────────────
    def _flow_mount_handle_board_line(self, line: str) -> None:
        self._flow_mount_init_state()
        if line.startswith("FLOW "):
            self._flow_mount_absorb_flow_line(line)
            return
        txn = self.flow_mount_txn
        if txn is None:
            return
        step = txn["step"]
        if step == "arm":
            if line.startswith("RSP ") and parse_kv(line).get("mod", "").upper() == "ARM":
                self._flow_mount_absorb_arm(parse_kv(line))
            return
        if step in {"set_yaw", "set_mirror"}:
            self._flow_mount_absorb_set(step, line)
            return
        if step == "readback" and line.startswith("PARAM "):
            self._flow_mount_absorb_readback(parse_kv(line))

    def _flow_mount_absorb_flow_line(self, line: str) -> None:
        mount = _mount_tuple(parse_kv(line))
        if mount is None:
            return
        # 采样期间看到的每一个值都记到这一步名下（FLOW ok 行先于 FLOW comp 到达）。
        stage = getattr(self, "flow_cal_active_stage", None)
        if getattr(self, "flow_cal_collecting", False) and stage in self.flow_mount_stage_mounts:
            self.flow_mount_stage_mounts[stage].add(mount)
        if mount != self.flow_mount_live:
            self.flow_mount_live = mount
            self._flow_mount_refresh()

    def _flow_mount_absorb_arm(self, values: dict[str, str]) -> None:
        armed = values.get("armed")
        if armed == "0":
            self._flow_mount_advance("set_yaw")
        elif armed is None:
            self._flow_mount_fail("ARM 状态回包里没有 armed 字段，无法确认已上锁")
        else:
            self._flow_mount_fail("飞机处于解锁状态（armed=1），请先用遥控器上锁再写入")

    def _flow_mount_absorb_set(self, step: str, line: str) -> None:
        txn = self.flow_mount_txn
        assert txn is not None
        name = FLOW_MOUNT_YAW_PARAM if step == "set_yaw" else FLOW_MOUNT_MIRROR_PARAM
        expected = txn["after"][0] if step == "set_yaw" else txn["after"][1]
        if line.startswith("ERR param target") and name in line.split():
            self._flow_mount_fail(f"飞控拒绝 {name}={expected}")
            return
        if line.startswith("ERR usage PARAM SET"):
            self._flow_mount_fail("飞控不认这条 PARAM SET（固件可能太旧，没有安装方向参数）")
            return
        if not line.startswith("OK param "):
            return
        values = parse_kv(line)
        if values.get("name") != name:
            return
        if not _same_number(values.get("value"), expected):
            self._flow_mount_fail(f"{name} 回显 {values.get('value')}，与要写的 {expected} 不符")
            return
        txn["set_ok"].append(f"{name}={expected}")
        self._flow_mount_advance("set_mirror" if step == "set_yaw" else "readback")

    def _flow_mount_absorb_readback(self, values: dict[str, str]) -> None:
        txn = self.flow_mount_txn
        assert txn is not None
        name = values.get("name")
        if name not in {FLOW_MOUNT_YAW_PARAM, FLOW_MOUNT_MIRROR_PARAM}:
            return
        txn["readback"][name] = values.get("value")
        readback = txn["readback"]
        if len(readback) < 2:
            return
        yaw_ok = _same_number(readback.get(FLOW_MOUNT_YAW_PARAM), txn["after"][0])
        mirror_ok = _same_number(readback.get(FLOW_MOUNT_MIRROR_PARAM), txn["after"][1])
        if yaw_ok and mirror_ok:
            self._flow_mount_succeed()
        else:
            self._flow_mount_fail(
                f"PARAM? 读回 转角={readback.get(FLOW_MOUNT_YAW_PARAM)}、"
                f"镜像={readback.get(FLOW_MOUNT_MIRROR_PARAM)}，与写入值不符"
            )


__all__ = [
    "FLOW_MOUNT_ARM_REQUEST",
    "FLOW_MOUNT_MIRROR_PARAM",
    "FLOW_MOUNT_YAW_PARAM",
    "FlowMountPanelMixin",
]

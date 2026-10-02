"""吊绳偏航辨识 YAW（「本轮做」选 YAW）：参数、下发命令、回显核对与溯源。

台架：机体用绳从上方吊住，**推力始终小于机重**（绳子一直绷紧、升不起来）；横滚/俯仰由吊挂摆约束，
本模式固件把舵机锁中位、不跑横滚俯仰环，只用上下桨差速产生偏航力矩。

与固件的契约见 `doc/sysid-yaw-contract.md`（两侧共同以它为准），这里只对接、不改：

* `SYSID MODE YAW`（固件编号 6）。固件不读 `SYSID THROTTLE` 的 target_n（总推力就是 thrust_mn），
  所以 YAW 轮**不下发** `SYSID THROTTLE`；开始前只发 EXC / RATE / INERTIA / LIMIT / MODE / YAW / RIG。
* `SYSID YAW inject=diff|rate thrust_mn=<总推力 mN> twist_deg=<90..1440>`：只在空闲时收、只存 RAM，
  回**一行** `SYSID YAW inject=.. thrust_mn=.. twist_deg=.. control=openloop|closed_loop`（diff 开环，
  rate 闭环）；拒绝回 `SYSID YAW event=rejected reason=running|range|lift|usage`
  （lift：thrust_mn > 0.8×机重，会把机体提起来；range：< 2000 mN 或参数越界）。
* 幅值沿用「激励」设置（单位随注入类型）：diff 是差速推力 ΔT [N]，上限 min(0.8·F, 2·(T单max − F/2))；
  rate 是偏航角速度 [rad/s]，上限 2.0。越界开跑前被固件拒 `ERR sysid yaw amp over limit (inject=.. max=..)`，
  页面在开跑前就按同一公式先拦。lift 判据用机重 mass×g（`lift_weight_n`），不是悬停学习值。
* 溯源：开跑后紧跟 `SYSID YAWSTART run=.. yaw_inject=.. yaw_thrust_mn=.. yaw_k_um_per_n=.. yaw_izz_ugm2=..
  yaw_single_max_mn=.. yaw_twist_deg=..`，存进 conditions["yaw"]；页面下发并核对过的配置存进
  conditions["yaw_request"]。
* 记录：schema 自描述解码，尾部 7 个字段在 YAW 模式下的含义见 `tools/sysid/yaw_analysis.py`。
"""
from __future__ import annotations

import math
import time

from .alt_config import GRAVITY_M_S2, airframe_mass_kg

YAW_MODE = "YAW"
#: 固件里的模式编号（READY/快照里的 mode=6）。
YAW_MODE_CODE = 6
#: 别处发的 `SYSID YAW` 的回复最多等这么久（与 workflow.STALE_REPORT_S 同量级）[s]。
STRAY_EXPIRY_S = 2.5

INJECTS = ("diff", "rate")
INJECT_LABELS = {
    "diff": "开环差速：给上下桨一个固定的推力差 ΔT（M = k·ΔT），测偏航对象：b = 1/Izz、阻尼、绳扭转刚度、延迟",
    "rate": "闭环偏航角速度：生产偏航角速度环（coax.rate_yaw_*）跟随参考 r，验证整定（没有参考模型前馈，rate_yaw_ff 不起作用）",
}
CONTROL_BY_INJECT = {"diff": "openloop", "rate": "closed_loop"}
#: 注入类型 -> 幅值单位。
AMP_UNITS = {"diff": "N", "rate": "rad/s"}
#: rate 的幅值上限 [rad/s]；diff 的上限随总推力与单桨最大推力而变（`amplitude_limit`）。
RATE_AMP_LIMIT_RAD_S = 2.0
#: 注入类型 -> 默认激励（键同 sections.EXPERIMENTS 的预设）。总时长 = 2 × 保持 × 对数。
INJECT_PRESETS = {
    "diff": dict(profile="doublet", amp="0.5", hold="800", repeat="3", dur="4800", ramp="100"),  # 首轮实测效能约模型 3.7 倍
    "rate": dict(profile="doublet", amp="0.8", hold="1500", repeat="2", dur="6000", ramp="150"),
}
DEFAULT_INJECT = "diff"
DEFAULT_TWIST_DEG = "720"
TWIST_RANGE_DEG = (90, 1440)
#: 总推力的下限 [N]（固件 range：< 2000 mN）与上限比例（lift：> 0.8×机重）。
THRUST_MIN_N = 2.0
LIFT_FRACTION = 0.8
#: 默认总推力 = 这个比例 × 悬停推力。
DEFAULT_THRUST_FRACTION = 0.5
#: 拿不到单桨最大推力（THRMODE? 的 tmax_mn、YAWSTART 的 yaw_single_max_mn）时开跑前检查用的保守值 [N]。
CONSERVATIVE_T_SINGLE_MAX_N = 6.0
#: 程序油门的最高油门 [%]：偏航辨识总推力远低于上限，固定一个值、不另设输入框。
YAW_MAX_THROTTLE_PCT = 90.0
#: 溯源键（`SYSID YAWSTART` 行）。
PROVENANCE_KEYS = ("yaw_inject", "yaw_thrust_mn", "yaw_k_um_per_n", "yaw_izz_ugm2",
                   "yaw_single_max_mn", "yaw_twist_deg")
#: 开跑前检查字段表里有没有这些（YAW 沿用 ALT 追加的尾字段，另需偏航力矩 torque）。
REQUIRED_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "torque")
#: 批头标志里明确属于别的模式的位（RATE/ANGLE/SERVO）；YAW 自己的位（0x0800）页面不要求，只拒绝别人的。
YAW_FOREIGN_FLAGS = (0x20, 0x40, 0x80)

REJECT_TEXT = {
    "running": "飞控拒绝偏航设置：辨识正在进行，只能在空闲时设置。等这一轮结束后再点开始。",
    "range": (f"飞控拒绝偏航设置：数值超出范围（总推力要 ≥ {THRUST_MIN_N * 1000:.0f} mN，"
              f"绞绳上限 {TWIST_RANGE_DEG[0]}～{TWIST_RANGE_DEG[1]}°）。到「偏航（吊绳）」页检查。"),
    "lift": (f"飞控拒绝偏航设置：总推力超过 {LIFT_FRACTION:g}×机重，会把机体从绳上提起来。"
             "到「偏航（吊绳）」页把总推力改小（默认 0.5×悬停推力）。"),
    "usage": "飞控拒绝偏航设置：命令格式不对（面板和固件版本可能不一致），请更新固件。",
}
UNSUPPORTED_SCHEMA = ("吊绳偏航辨识（YAW）需要记录里带 height/height_raw/height_sp/vz/vz_sp/az 尾字段"
                      "与 torque 的新固件：请更新固件后重试。")


def _number(text, what: str, low: float, high: float, unit: str) -> float:
    try:
        value = float(str(text).strip())
    except (TypeError, ValueError):
        raise ValueError(f"{what}要填数字（单位 {unit}）") from None
    if not (math.isfinite(value) and low <= value <= high):
        raise ValueError(f"{what}要在 {low:g}～{high:g} {unit} 之间")
    return value


def hover_param_n(params: dict) -> float | None:
    """飞控参数 coax.hover_thrust_n（悬停推力，0 = 未设）；读不到或没设返回 None。"""
    try:
        value = float(params["coax.hover_thrust_n"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def default_thrust_n(params: dict) -> float | None:
    """默认总推力 = 0.5×悬停推力 [N]；悬停推力读不到返回 None。"""
    hover = hover_param_n(params)
    return None if hover is None else DEFAULT_THRUST_FRACTION * hover


def lift_weight_n(params: dict, status: dict) -> float | None:
    """机重 mass×g [N]（固件 lift 判据同口径：机体质量参数 × g，不是悬停学习值）；质量读不到返回 None。"""
    mass = airframe_mass_kg(params, status)
    return None if mass is None else mass * GRAVITY_M_S2


def lift_limit_n(weight_n: float | None) -> float | None:
    """总推力上限 0.8×机重 [N]（超过它固件拒 lift）；机重未知返回 None。"""
    return None if weight_n is None else LIFT_FRACTION * weight_n


def amplitude_limit(inject: str, thrust_n: float | None, t_single_max_n: float | None) -> tuple[str, float | None]:
    """`(单位, 上限)`。diff：min(0.8·F, 2·(T单max − F/2))；rate：2.0；总推力未知时 diff 返回 None。"""
    unit = AMP_UNITS[inject]
    if inject == "rate":
        return unit, RATE_AMP_LIMIT_RAD_S
    if thrust_n is None:
        return unit, None
    t_max = CONSERVATIVE_T_SINGLE_MAX_N if t_single_max_n is None else t_single_max_n
    return unit, max(0.0, min(LIFT_FRACTION * thrust_n, 2.0 * (t_max - thrust_n / 2.0)))


def check_amplitude(inject: str, amplitude: float, thrust_n: float, t_single_max_n: float | None) -> None:
    """激励幅值的单位与上限随注入类型变；超限抛中文 ValueError。"""
    unit, limit = amplitude_limit(inject, thrust_n, t_single_max_n)
    if not (math.isfinite(amplitude) and 0.0 < amplitude <= limit):
        guess = "" if t_single_max_n is not None else "（还没读到单桨最大推力，按保守值估的）"
        what = ("差速推力 ΔT 上限 = min(0.8·总推力, 2·(单桨最大推力 − 总推力/2))" if inject == "diff"
                else "偏航角速度参考上限 2 rad/s")
        raise ValueError(f"吊绳偏航辨识（{inject}）的激励幅值单位是 {unit}，上限 {limit:.3g} {unit}{guess}"
                         f"（{what}）；现在是 {amplitude:g}。到本页「激励」改小。")


def check_thrust(thrust_n: float, weight_n: float | None, *, require_weight: bool = True) -> None:
    """总推力：不低于 2 N，不高于 0.8×机重（绳子要一直绷紧）；机重未知默认也不开跑
    （`require_weight=False` 只做能做的检查，留给机体参数收齐后再查）。"""
    if not (math.isfinite(thrust_n) and thrust_n >= THRUST_MIN_N):
        raise ValueError(f"总推力 {thrust_n:g} N 太小：至少 {THRUST_MIN_N:g} N（固件要 ≥ {THRUST_MIN_N * 1000:.0f} mN）。")
    if weight_n is None:
        if not require_weight:
            return
        raise ValueError("读不到机重（机体参数 airframe.weight_n / airframe.mass_kg）：吊绳偏航要用它核对"
                         f"总推力 ≤ {LIFT_FRACTION:g}×机重。先到「机体模型」页确认参数已下发到飞控，再重试。")
    limit = LIFT_FRACTION * weight_n
    if thrust_n > limit:
        raise ValueError(f"总推力 {thrust_n:g} N 超过 {LIFT_FRACTION:g}×机重 = {limit:.2f} N（机重 {weight_n:.2f} N）："
                         "会把机体从绳上提起来；已取消开始。")


def parse_twist(text) -> int:
    value = _number(text, "绞绳上限", *TWIST_RANGE_DEG, "°")
    if value != int(value):
        raise ValueError("绞绳上限要填整数度")
    return int(value)


def yaw_command(inject: str, thrust_mn: int, twist_deg: int) -> str:
    return f"SYSID YAW inject={inject} thrust_mn={thrust_mn} twist_deg={twist_deg}"


def schema_has_yaw(schema) -> bool:
    try:
        names = set(schema.field_names())
    except AttributeError:
        return False
    return all(name in names for name in REQUIRED_FIELDS)


def reject_text(reason: str | None) -> str:
    return REJECT_TEXT.get((reason or "").strip(), "飞控拒绝了偏航设置。")


def provenance(conditions: dict | None) -> dict:
    """本轮偏航设置的溯源：`SYSID YAWSTART` 行（conditions["yaw"]）。"""
    yaw = (conditions or {}).get("yaw")
    return {key: yaw[key] for key in PROVENANCE_KEYS if key in yaw} if isinstance(yaw, dict) else {}


def provenance_text(conditions: dict | None) -> str:
    """结果抬头那句：这一轮飞控实际用的偏航设置（没有溯源就写页面下发的请求）。"""
    conditions = conditions or {}
    found = provenance(conditions)
    if found:
        parts = [f"注入 {found.get('yaw_inject', '?')}"]
        for key, label, scale, unit in (("yaw_thrust_mn", "总推力", 1e-3, "N"),
                                        ("yaw_single_max_mn", "单桨最大推力", 1e-3, "N"),
                                        ("yaw_k_um_per_n", "k（模型）", 1e-3, "mN·m/N"),
                                        ("yaw_izz_ugm2", "Izz（模型）", 1e-6, "kg·m²")):
            if key in found:
                try:
                    parts.append(f"{label} {float(found[key]) * scale:g} {unit}")
                except (TypeError, ValueError):
                    parts.append(f"{label} {found[key]}（格式异常）")
        if "yaw_twist_deg" in found:
            parts.append(f"绞绳上限 {found['yaw_twist_deg']}°")
        return "本轮偏航设置（飞控开跑时报告）：" + " · ".join(parts)
    request = conditions.get("yaw_request")
    if isinstance(request, dict) and request:
        return ("本轮偏航设置（页面下发并核对过；固件开跑时没报溯源）："
                f"注入 {request.get('inject', '?')} · 总推力 {request.get('thrust_mn', '?')} mN · "
                f"绞绳上限 {request.get('twist_deg', '?')}° · 控制 {request.get('control', '?')}")
    return "本轮偏航设置：固件没报溯源。"


OFFLINE_NOTE = ("原始记录与本轮条件已按采集情况存档；存档里尾部 7 列沿用 ALT 字段表的旧列名，"
                "YAW 里各列的含义与分析层的新名字见 doc/sysid-yaw-contract.md。")


class YawWorkflow:
    """Workflow 的 YAW 部分：开跑前的输入检查、`SYSID YAW` 的生成与回显核对、开跑溯源。"""

    yaw_request: dict | None = None
    yaw_expected: dict = {}
    #: 别处（AI 接口、旧按钮）发出、回复还没回来的 `SYSID YAW` 查询/设置的过期时刻。
    yaw_strays: list = []
    #: 页面自己的 `SYSID YAW ...` 发出时排在它前面、回复会先到的别人的回复数。
    yaw_skip: int = 0

    def yaw_prepare(self, page, spec) -> bool:
        """开始前（发任何命令之前）检查 YAW 输入；不是 YAW 就清掉上一轮的 YAW 状态。"""
        self.yaw_request = None
        self.yaw_expected = {}
        if page.mode_var.get() != YAW_MODE:
            return False
        inject = page.yaw_inject_var.get()
        if inject not in INJECTS:
            raise ValueError("偏航注入类型要选 diff / rate")
        twist = parse_twist(page.yaw_twist_var.get())
        t_max = page.yaw_t_single_max_n
        self.yaw_request = dict(inject=inject, twist_deg=twist, amplitude=spec.amplitude_rad_s,
                                t_single_max_n=t_max,
                                t_single_max_source="THRMODE?" if t_max is not None else "保守值")
        # 总推力 / 机重此刻读得到（页面一连上就读过 PARAM?）就先拦一道，免得白发一串配置；
        # 读不到的（刚连上、参数还没回来）留到 `_yaw_command` 再查，那时机体参数一定收齐了。
        thrust_n = page.yaw_effective_thrust_n()
        if thrust_n is not None:
            self._yaw_check(thrust_n, lift_weight_n(self.params, self.status), strict=False)
        return True

    def _yaw_check(self, thrust_n: float, weight_n: float | None, *, strict: bool) -> None:
        request = self.yaw_request
        check_thrust(thrust_n, weight_n, require_weight=strict)
        check_amplitude(request["inject"], request["amplitude"], thrust_n, request["t_single_max_n"])

    def _yaw_command(self):
        """轮到它时才生成（机重、悬停推力要等 PARAM? 回来）。"""
        request = self.yaw_request
        thrust_n = self.page.yaw_effective_thrust_n()
        if thrust_n is None:
            raise ValueError("总推力没填，也读不到飞控的悬停推力（coax.hover_thrust_n 为 0 或没读到）："
                             "在「偏航（吊绳）」页填总推力（默认取 0.5×悬停推力）。")
        self._yaw_check(thrust_n, lift_weight_n(self.params, self.status), strict=True)
        request.update(thrust_n=thrust_n, thrust_mn=int(round(thrust_n * 1000.0)))
        self.yaw_expected = dict(inject=request["inject"], thrust_mn=request["thrust_mn"],
                                 twist_deg=request["twist_deg"], control=CONTROL_BY_INJECT[request["inject"]])
        return yaw_command(request["inject"], request["thrust_mn"], request["twist_deg"])

    def yaw_handle_line(self, line: str, values: dict) -> bool:
        """`SYSID YAW ...`（回显/拒绝）与 `SYSID YAWSTART ...`（溯源）；返回 True = 已处理。"""
        if line.startswith("SYSID YAWSTART "):
            run = values.get("run")
            open_run = self.run_id is not None and self.end is None
            if self.snapshot is not None and (str(self.run_id) == run or (run is None and open_run)):
                self.snapshot["yaw"] = dict(values)
                problem = self._yaw_provenance_problem(values)
                if problem:
                    self.mark_data_error(problem)
                    self.page.status_var.set(problem + "；已请求停止，本轮仅留作诊断")
                    self.page.send("SYSID STOP")
            return True
        if not line.startswith("SYSID YAW "):
            return False
        if not (self.awaiting and self.awaiting.startswith("SYSID YAW ")):
            self._yaw_stray_answered()
            return True                  # 不在等它：旧回复或别处发的，不当成本次回显
        if self.yaw_skip > 0:
            self.yaw_skip -= 1           # 排在页面命令前面的别人的回复（回的是旧配置），不是本次回显
            return True
        if values.get("event") == "rejected":
            self.fail(f"{reject_text(values.get('reason'))}（飞控原话：{line}）")
            return True
        for key, expected in self.yaw_expected.items():
            if str(values.get(key, "")) != str(expected):
                self.fail(f"配置回读不符：偏航 {key}（发 {expected}，回 {values.get(key, '缺')}），取消开始")
                return True
        if not self.pending and not self._verify():
            return True
        self.advance()
        return True

    def yaw_note_sent(self, text: str) -> None:
        """每条发出的 `SYSID YAW ...` 都来记账（`Workflow.note_sent` 调）；道理同 `xy_note_sent`。"""
        if not text.startswith("SYSID YAW"):
            return
        now = time.monotonic()
        self.yaw_strays = [t for t in self.yaw_strays if t > now]
        if text == self.awaiting:
            self.yaw_skip = len(self.yaw_strays)
            self.yaw_strays = []
        else:
            self.yaw_strays.append(now + STRAY_EXPIRY_S)

    def _yaw_stray_answered(self) -> None:
        now = time.monotonic()
        self.yaw_strays = [t for t in self.yaw_strays if t > now]
        if self.yaw_strays:
            self.yaw_strays.pop(0)

    def yaw_snapshot(self) -> None:
        """开跑时把页面这次下发的偏航请求（含总推力换算与单桨最大推力来源）记进快照。"""
        if self.snapshot is not None and self.yaw_request and self.yaw_expected:
            self.snapshot["yaw_request"] = dict(self.yaw_request, control=self.yaw_expected["control"])

    def _yaw_provenance_problem(self, values: dict) -> str:
        expected = {"yaw_inject": self.yaw_expected.get("inject"),
                    "yaw_thrust_mn": self.yaw_expected.get("thrust_mn"),
                    "yaw_twist_deg": self.yaw_expected.get("twist_deg")}
        for key, wanted in expected.items():
            if key == "yaw_thrust_mn":
                # 固件按 float 取整，页面按毫牛四舍五入：差 1 mN 内算一致。
                try:
                    if abs(int(values.get(key, "")) - int(wanted)) <= 1:
                        continue
                except (TypeError, ValueError):
                    pass
            elif str(values.get(key, "")) == str(wanted):
                continue
            return f"飞控开跑溯源不符：{key}（期望 {wanted}，回 {values.get(key, '缺')}）"
        return ""

    def yaw_finish_validation(self, end: dict, gap_count: int, batches: list) -> None:
        """YAW 原始数据可存档；中止、缺包与溯源缺失必须显式标为不能分析。"""
        if not self.snapshot or str(self.snapshot.get("mode")) != str(YAW_MODE_CODE):
            return
        problems = [self.data_error] if self.data_error else []
        yaw = self.snapshot.get("yaw")
        problem = (self._yaw_provenance_problem(yaw) if isinstance(yaw, dict)
                   else "缺少飞控开跑溯源（SYSID YAWSTART）")
        if problem:
            problems.append(problem)
        if end.get("state") != "done":
            problems.append(f"固件中止本轮：{end.get('reason', '未知原因')}")
        try:
            dropped = int(end.get("dropped", ""))
        except (TypeError, ValueError):
            dropped = -1
        if gap_count or dropped:
            problems.append(f"采样有断点 {gap_count}、固件丢弃计数 {end.get('dropped', '缺')}")
        if not batches or not batches[0].first or not batches[-1].last:
            problems.append("采样首尾不完整")
        if problems:
            self.mark_data_error("；".join(dict.fromkeys(problems)))

    def yaw_quality_note(self) -> str:
        if self.data_error:
            return f"本轮仅供诊断，不可用于分析：{self.data_error}。收到的原始记录仍按情况存档。"
        return "本轮采集完整，原始记录已存档；分析结果只反映这一轮，多轮对比再下结论。"


__all__ = [
    "AMP_UNITS", "CONSERVATIVE_T_SINGLE_MAX_N", "CONTROL_BY_INJECT", "DEFAULT_INJECT",
    "DEFAULT_TWIST_DEG", "GRAVITY_M_S2", "INJECTS", "INJECT_LABELS", "INJECT_PRESETS", "LIFT_FRACTION",
    "OFFLINE_NOTE", "REQUIRED_FIELDS", "UNSUPPORTED_SCHEMA", "YAW_FOREIGN_FLAGS", "YAW_MAX_THROTTLE_PCT",
    "lift_weight_n",
    "YAW_MODE", "YAW_MODE_CODE", "YawWorkflow", "amplitude_limit", "check_amplitude", "check_thrust",
    "default_thrust_n", "hover_param_n", "lift_limit_n", "parse_twist", "provenance",
    "provenance_text", "reject_text", "schema_has_yaw", "yaw_command",
]

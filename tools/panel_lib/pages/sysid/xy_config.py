"""水平槽 XY 速度/位置辨识（「本轮做」选 XY，R-XYID-1）：参数、下发命令、回显核对与溯源。

台架：碳杆穿过机体、杆两端落在**水平**槽里；推力托住机体自重，绕杆倾斜时推力水平分量推着机体沿槽平移。
平移方向 u = 水平且垂直于杆 = (−sin ψ, cos ψ)，ψ 沿用内环页台架设置里的「杆轴方位角」，本页不另设。

与固件的契约见 `doc/sysid-xy-contract.md`（两侧共同以它为准），这里只对接、不改：

* `SYSID MODE XY`（固件编号 5）。
* `SYSID XY inject=tilt|vel|pos win_mm=<30..400> mass_g=<0|500..3000>`：只在空闲时收、只存 RAM，
  回**一行** `SYSID XY inject=.. win_mm=.. mass_g=.. control=openloop|closed_loop`（tilt 开环，vel/pos 闭环）；
  拒绝回 `SYSID XY event=rejected reason=running|range|usage`。开始前的配置事务把它排在 `SYSID MODE`
  之后，逐项核对这一行再发下一条。
* 开跑前检（`ERR sysid xy <理由>`）：程序油门（target_n = 托住机体的推力，≤ 整机最大推力）、幅值上限
  tilt ≤ 0.10 rad、vel ≤ 0.30 m/s、pos ≤ 0.15 m 且 ≤ win、光流速度/位置有效且新鲜、测距有效。
* 溯源：开跑后紧跟一行 `SYSID XYSTART run=.. xy_inject=.. xy_win_mm=.. xy_mass_g=.. xy_psi_mrad=..
  xy_target_cn=.. xy_x0_mm=.. xy_y0_mm=..`，存进 conditions["xy"]；页面下发并核对过的配置存进
  conditions["xy_request"]。
* `SYSID THR?` 行末尾追加实时量 `xy_pos_mm xy_vel_mms xy_ok`。
* 记录：schema 自描述解码，尾部 7 个字段在 XY 模式下的含义见 `tools/sysid/xy_analysis.py`
  （分析层改名，页面上不出现 "height"）。

激励的剖面/幅值/平台沿用「激励」设置，幅值单位随注入类型（tilt rad、vel m/s、pos m）。
"""
from __future__ import annotations

import math
import time

from .alt_config import (EXTRA_MASS_RANGE_G, GRAVITY_M_S2, MASS_RANGE_G, WIN_RANGE_MM,
                         airframe_mass_kg, airframe_max_force_n, total_mass_g)

XY_MODE = "XY"
#: 别处发的 `SYSID XY` 的回复最多等这么久（与 workflow.STALE_REPORT_S 同量级）[s]。
STRAY_EXPIRY_S = 2.5
#: 固件里的模式编号（READY/快照里的 mode=5）。
XY_MODE_CODE = 5

INJECTS = ("tilt", "vel", "pos")
INJECT_LABELS = {
    "tilt": "开环倾角：直接给沿 u 的倾角、不跑位置/速度环，测倾角→加速度增益、静摩擦门槛角与光流滞后",
    "vel": "闭环速度参考：生产位置/速度环（x 通道参数）沿 u 一维，验证速度环",
    "pos": "闭环位置参考：生产位置/速度环沿 u 一维，验证位置环",
}
#: 注入类型 -> (幅值单位, 幅值上限)。
AMP_UNITS = {"tilt": ("rad", 0.10), "vel": ("m/s", 0.30), "pos": ("m", 0.15)}
#: 注入类型 -> 默认激励（键同 sections.EXPERIMENTS 的预设）。总时长 = 2 × 保持 × 对数。
#: 槽里能走的距离有限：默认幅值按默认出窗余量 150 mm 定，tilt 每半周期约走 60 mm。
#: 2026-09-30 上台前端到端彩排（tests/test_sysid_xy_e2e.py，静/动摩擦 0.3/0.2 m/s²）定的默认：
#: tilt 0.05 rad 时滑动点太少、动摩擦散布 0.14~0.36，改 0.07 rad×400 ms×3 对（仍在 150 mm 窗内）；
#: vel/pos 原幅值推不动静摩擦（跟上 0/4），要配合 SYSID PARAM 试用 x 通道 1.2/3.0/3.0；
#: vel 0.07 配该增益在无摩擦时会出窗，只取 0.06；pos 0.10 离窗太近，取 0.08、平台 3 s。
INJECT_PRESETS = {
    "tilt": dict(profile="doublet", amp="0.07", hold="400", repeat="3", dur="2400", ramp="100"),
    "vel": dict(profile="doublet", amp="0.06", hold="1500", repeat="2", dur="6000", ramp="150"),
    "pos": dict(profile="doublet", amp="0.08", hold="3000", repeat="2", dur="12000", ramp="150"),
}
DEFAULT_INJECT = "tilt"
DEFAULT_EXTRA_MASS_G = "38.8"
DEFAULT_WIN_MM = "150"
#: pos 幅值 ≤ 该比例 × 出窗余量（app_sysid_xy.c 开跑前检 `amp over window: inject=pos amp<=0.7*win`）。
POS_AMP_WIN_FRACTION = 0.7
CONTROL_BY_INJECT = {"tilt": "openloop", "vel": "closed_loop", "pos": "closed_loop"}
#: 托住推力的下限（SYSID THROTTLE 的下限，飞控低于它直接拒收）[N]。
HOLD_THRUST_MIN_N = 2.0
#: 溯源键（`SYSID XYSTART` 行）。
PROVENANCE_KEYS = ("xy_inject", "xy_win_mm", "xy_mass_g", "xy_psi_mrad", "xy_target_cn",
                   "xy_x0_mm", "xy_y0_mm")
#: 开跑前检查字段表里有没有这些（XY 沿用 ALT 追加的尾字段）。
REQUIRED_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az")
#: 批头标志里明确属于别的模式的位（RATE/ANGLE/SERVO）；XY 自己的位不在契约里写死，页面不要求它。
XY_FOREIGN_FLAGS = (0x20, 0x40, 0x80)

REJECT_TEXT = {
    "running": "飞控拒绝水平槽设置：辨识正在进行，只能在空闲时设置。等这一轮结束后再点开始。",
    "range": (f"飞控拒绝水平槽设置：数值超出范围（出窗余量 {WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm、"
              f"质量 {MASS_RANGE_G[0]}～{MASS_RANGE_G[1]} g）。到「XY 速度 / 位置环」页检查。"),
    "usage": "飞控拒绝水平槽设置：命令格式不对（面板和固件版本可能不一致），请更新固件。",
}
UNSUPPORTED_SCHEMA = ("水平槽辨识（XY）需要记录里带 height/height_raw/height_sp/vz/vz_sp/az 尾字段的"
                      "新固件：请更新固件后重试。")


def _number(text, what: str, low: float, high: float, unit: str) -> float:
    try:
        value = float(str(text).strip())
    except (TypeError, ValueError):
        raise ValueError(f"{what}要填数字（单位 {unit}）") from None
    if not (math.isfinite(value) and low <= value <= high):
        raise ValueError(f"{what}要在 {low:g}～{high:g} {unit} 之间")
    return value


def parse_inputs(inject: str, extra_mass: str, win: str) -> dict:
    """设置区三项 → `dict(inject, extra_mass_g, win_mm)`；填错抛中文 ValueError。"""
    if inject not in INJECTS:
        raise ValueError("水平槽注入类型要选 tilt / vel / pos")
    extra = _number(extra_mass, "台架随动附加质量", *EXTRA_MASS_RANGE_G, "g")
    win_mm = _number(win, "出窗余量", *WIN_RANGE_MM, "mm")
    if win_mm != int(win_mm):
        raise ValueError("出窗余量要填整数毫米")
    return dict(inject=inject, extra_mass_g=extra, win_mm=int(win_mm))


def amplitude_limit(inject: str, win_mm: int | None = None) -> tuple[str, float]:
    """`(单位, 上限)`；pos 的上限还要不超过 0.7×出窗余量（固件开跑前检同一口径，留超调余地）。"""
    unit, limit = AMP_UNITS[inject]
    if inject == "pos" and win_mm is not None:
        limit = min(limit, POS_AMP_WIN_FRACTION * win_mm * 0.001)
    return unit, limit


def check_amplitude(inject: str, amplitude: float, win_mm: int | None = None) -> None:
    """激励幅值的单位与上限随注入类型变；超限抛中文 ValueError。"""
    unit, limit = amplitude_limit(inject, win_mm)
    if not (math.isfinite(amplitude) and 0.0 < amplitude <= limit):
        extra = "（pos 幅值还不能超过 0.7×出窗余量）" if inject == "pos" else ""
        raise ValueError(f"水平槽辨识（{inject}）的激励幅值单位是 {unit}，上限 {limit:g} {unit}{extra}；"
                         f"现在是 {amplitude:g}。到本页「激励」改小，或重选一次注入类型换回默认值。")


def check_hold_thrust(target_n: float, max_force_n: float | None = None) -> None:
    """托住推力：不低于 2 N，不高于飞控整机最大推力（飞控会拒收）；否则抛中文 ValueError。"""
    if not (math.isfinite(target_n) and target_n >= HOLD_THRUST_MIN_N):
        raise ValueError(f"托住推力 {target_n:g} N 太小：至少 {HOLD_THRUST_MIN_N:g} N。"
                         "填机体悬停所需的合推力，或点「读取学到的悬停推力」。")
    if max_force_n is not None and target_n > max_force_n:
        raise ValueError(f"托住推力 {target_n:g} N 超过飞控整机最大推力 {max_force_n:.2f} N"
                         "（机体参数 airframe.max_total_force_n，飞控会拒收）；已取消开始。")


def hover_param_n(params: dict) -> float | None:
    """飞控参数 coax.hover_thrust_n（悬停推力，0 = 未设）；读不到或没设返回 None。"""
    try:
        value = float(params["coax.hover_thrust_n"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def xy_command(inject: str, win_mm: int, mass_g: int) -> str:
    return f"SYSID XY inject={inject} win_mm={win_mm} mass_g={mass_g}"


def schema_has_xy(schema) -> bool:
    try:
        names = set(schema.field_names())
    except AttributeError:
        return False
    return all(name in names for name in REQUIRED_FIELDS)


def reject_text(reason: str | None) -> str:
    return REJECT_TEXT.get((reason or "").strip(), "飞控拒绝了水平槽设置。")


def direction_text(psi_deg: float) -> str:
    """平移方向 u = (−sin ψ, cos ψ) 的一句话（FLU 机体系，x 前 y 左）。"""
    psi = math.radians(psi_deg)
    ux, uy = -math.sin(psi), math.cos(psi)
    return f"杆轴方位角 ψ = {psi_deg:g}° → 平移方向 u = (x {ux:+.3f}, y {uy:+.3f})，沿 +u 为正"


def provenance(conditions: dict | None) -> dict:
    """本轮水平槽设置的溯源：`SYSID XYSTART` 行（conditions["xy"]）。"""
    xy = (conditions or {}).get("xy")
    return {key: xy[key] for key in PROVENANCE_KEYS if key in xy} if isinstance(xy, dict) else {}


def provenance_text(conditions: dict | None) -> str:
    """结果抬头那句：这一轮飞控实际用的水平槽设置（没有溯源就写页面下发的请求）。"""
    conditions = conditions or {}
    found = provenance(conditions)
    if found:
        parts = [f"注入 {found.get('xy_inject', '?')}", f"质量 {found.get('xy_mass_g', '?')} g",
                 f"出窗余量 {found.get('xy_win_mm', '?')} mm"]
        if "xy_psi_mrad" in found:
            try:
                parts.append(f"杆轴方位角 {int(found['xy_psi_mrad']) / 1000.0 * 180.0 / math.pi:.1f}°")
            except (TypeError, ValueError):
                parts.append(f"杆轴方位角 {found['xy_psi_mrad']} mrad（格式异常）")
        if "xy_target_cn" in found:
            try:
                parts.append(f"托住推力 {int(found['xy_target_cn']) / 100:g} N")
            except (TypeError, ValueError):
                parts.append(f"托住推力 {found['xy_target_cn']} cN（格式异常）")
        if "xy_x0_mm" in found:
            parts.append(f"起点 ({found['xy_x0_mm']}, {found.get('xy_y0_mm', '?')}) mm")
        return "本轮水平槽设置（飞控开跑时报告）：" + " · ".join(parts)
    request = conditions.get("xy_request")
    if isinstance(request, dict) and request:
        return ("本轮水平槽设置（页面下发并核对过；固件开跑时没报溯源）："
                f"注入 {request.get('inject', '?')} · 质量 {request.get('mass_g', '?')} g · "
                f"出窗余量 {request.get('win_mm', '?')} mm · 控制 {request.get('control', '?')}")
    return "本轮水平槽设置：固件没报溯源。"


OFFLINE_NOTE = ("原始记录与本轮条件已按采集情况存档；存档里尾部 7 列沿用 ALT 字段表的旧列名，"
                "XY 里各列的含义与分析层的新名字见 doc/sysid-xy-contract.md。")


class XyWorkflow:
    """Workflow 的 XY 部分：开跑前的输入检查、`SYSID XY` 的生成与回显核对、开跑溯源。"""

    xy_request: dict | None = None
    xy_expected: dict = {}
    #: 别处（AI 接口、旧按钮）发出、回复还没回来的 `SYSID XY` 查询/设置的过期时刻。
    xy_strays: list = []
    #: 页面自己的 `SYSID XY ...` 发出时排在它前面、回复会先到的别人的回复数。
    xy_skip: int = 0

    def xy_prepare(self, page, spec) -> bool:
        """开始前（发任何命令之前）检查 XY 输入；不是 XY 就清掉上一轮的 XY 状态。"""
        self.xy_request = None
        self.xy_expected = {}
        if page.mode_var.get() != XY_MODE:
            return False
        if self.throttle[0]:
            raise ValueError("水平槽辨识（XY）要程序油门：先取消勾选「遥控器手动给油门」"
                             "（固件接管合推力并保持杆轴姿态环）。")
        request = parse_inputs(page.xy_inject_var.get(), page.xy_extra_mass_var.get(),
                               page.xy_win_var.get())
        check_amplitude(request["inject"], spec.amplitude_rad_s, request["win_mm"])
        self.xy_request = request
        return True

    def _xy_command(self):
        """轮到它时才生成：机体质量、悬停推力要等 PARAM? / 状态报告回来。"""
        request = self.xy_request
        mass = airframe_mass_kg(self.params, self.status)
        if mass is None:
            raise ValueError("读不到机体质量（机体参数 airframe.mass_kg）：水平槽辨识要用它算移动"
                             "质量。先到「机体模型」页确认参数已下发到飞控，再重试。")
        mass_g = total_mass_g(mass, request["extra_mass_g"])
        manual, target, max_pct = self.throttle
        if target is None:
            target = hover_param_n(self.params)
            if target is None:
                raise ValueError("托住推力没填，也读不到飞控的悬停推力（coax.hover_thrust_n 为 0 或没读到）："
                                 "在「XY 速度 / 位置环」页填托住推力，或点「读取学到的悬停推力」。")
        check_hold_thrust(target, airframe_max_force_n(self.params))
        self.throttle = (manual, target, max_pct)
        self.xy_expected = dict(inject=request["inject"], win_mm=request["win_mm"], mass_g=mass_g,
                                control=CONTROL_BY_INJECT[request["inject"]])
        request.update(airframe_mass_g=round(mass * 1000.0, 1), mass_g=mass_g, hold_thrust_n=target)
        return xy_command(request["inject"], request["win_mm"], mass_g)

    def xy_handle_line(self, line: str, values: dict) -> bool:
        """`SYSID XY ...`（回显/拒绝）与 `SYSID XYSTART ...`（溯源）；返回 True = 已处理。"""
        if line.startswith("SYSID XYSTART "):
            run = values.get("run")
            open_run = self.run_id is not None and self.end is None
            if self.snapshot is not None and (str(self.run_id) == run or (run is None and open_run)):
                self.snapshot["xy"] = dict(values)
                problem = self._xy_provenance_problem(values)
                if problem:
                    self.mark_data_error(problem)
                    self.page.status_var.set(problem + "；已请求停止，本轮仅留作诊断")
                    self.page.send("SYSID STOP")
            return True
        if not line.startswith("SYSID XY "):
            return False
        if not (self.awaiting and self.awaiting.startswith("SYSID XY ")):
            self._xy_stray_answered()
            return True                  # 不在等它：旧回复或别处发的，不当成本次回显
        if self.xy_skip > 0:
            self.xy_skip -= 1            # 排在页面命令前面的别人的回复（回的是旧配置），不是本次回显
            return True
        if values.get("event") == "rejected":
            self.fail(f"{reject_text(values.get('reason'))}（飞控原话：{line}）")
            return True
        for key, expected in self.xy_expected.items():
            if str(values.get(key, "")) != str(expected):
                self.fail(f"配置回读不符：水平槽 {key}（发 {expected}，回 {values.get(key, '缺')}），取消开始")
                return True
        if not self.pending and not self._verify():
            return True
        self.advance()
        return True

    def xy_note_sent(self, text: str) -> None:
        """每条发出的 `SYSID XY ...` 都来记账（`Workflow.note_sent` 调）。

        飞控对每条 `SYSID XY` 只回一行、按发送顺序。页面自己那条发出的那一刻，前面还没回的别人的
        （AI 接口查询等）回复会先到：记下个数，先到的不当回显核对，否则拿旧配置核对出「配置回读不符」。
        """
        if not text.startswith("SYSID XY"):
            return
        now = time.monotonic()
        self.xy_strays = [t for t in self.xy_strays if t > now]
        if text == self.awaiting:
            self.xy_skip = len(self.xy_strays)
            self.xy_strays = []
        else:
            self.xy_strays.append(now + STRAY_EXPIRY_S)

    def _xy_stray_answered(self) -> None:
        now = time.monotonic()
        self.xy_strays = [t for t in self.xy_strays if t > now]
        if self.xy_strays:
            self.xy_strays.pop(0)

    def xy_snapshot(self) -> None:
        """开跑时把页面这次下发的水平槽请求（含质量拆分与托住推力）记进快照。"""
        if self.snapshot is not None and self.xy_request and self.xy_expected:
            self.snapshot["xy_request"] = dict(self.xy_request,
                                               control=self.xy_expected["control"],
                                               target_cn=self.expected.get("target_cn"),
                                               psi_mrad=self.expected.get("psi_mrad"))

    def _xy_provenance_problem(self, values: dict) -> str:
        expected = {"xy_inject": self.xy_expected.get("inject"),
                    "xy_win_mm": self.xy_expected.get("win_mm"),
                    "xy_mass_g": self.xy_expected.get("mass_g"),
                    "xy_psi_mrad": self.expected.get("psi_mrad"),
                    "xy_target_cn": self.expected.get("target_cn")}
        for key, wanted in expected.items():
            if key in ("xy_psi_mrad", "xy_target_cn"):
                # psi：页面 int() 截断、固件四舍五入，差 1 mrad 内算一致。target_cn：页面是 Python
                # round()（银行家舍入、双精度），固件是 lroundf（半数远离零、float32）；托住推力填到
                # 0.001 N（如 12.345）时恰好落在 x.5 cN 上会差 1 cN，不能因此把刚转起来的电机停掉。
                try:
                    if abs(int(values.get(key, "")) - int(wanted)) <= 1:
                        continue
                except (TypeError, ValueError):
                    pass
            elif str(values.get(key, "")) == str(wanted):
                continue
            return f"飞控开跑溯源不符：{key}（期望 {wanted}，回 {values.get(key, '缺')}）"
        return ""

    def xy_finish_validation(self, end: dict, gap_count: int, batches: list) -> None:
        """XY 原始数据可存档；中止、缺包与溯源缺失必须显式标为不能分析。"""
        if not self.snapshot or str(self.snapshot.get("mode")) != str(XY_MODE_CODE):
            return
        problems = [self.data_error] if self.data_error else []
        xy = self.snapshot.get("xy")
        problem = (self._xy_provenance_problem(xy) if isinstance(xy, dict)
                   else "缺少飞控开跑溯源（SYSID XYSTART）")
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

    def xy_quality_note(self) -> str:
        if self.data_error:
            return f"本轮仅供诊断，不可用于分析：{self.data_error}。收到的原始记录仍按情况存档。"
        return "本轮采集完整，原始记录已存档；分析结果只反映这一轮，多轮对比再下结论。"


__all__ = [
    "AMP_UNITS", "CONTROL_BY_INJECT", "DEFAULT_EXTRA_MASS_G", "DEFAULT_INJECT", "DEFAULT_WIN_MM",
    "GRAVITY_M_S2", "INJECTS", "INJECT_LABELS", "INJECT_PRESETS", "OFFLINE_NOTE",
    "REQUIRED_FIELDS", "UNSUPPORTED_SCHEMA", "XY_FOREIGN_FLAGS", "XY_MODE", "XY_MODE_CODE",
    "XyWorkflow", "amplitude_limit", "check_amplitude", "check_hold_thrust", "direction_text",
    "hover_param_n", "parse_inputs", "provenance", "provenance_text", "reject_text",
    "schema_has_xy", "xy_command",
]

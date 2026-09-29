"""光杆台架高度辨识（「本轮做」选 ALT，R-ALTID-1）：参数、下发命令、回显核对与溯源。

台架：机体装在碳杆上（杆轴 −45°），杆的轴装在竖直槽里，机体连杆一起沿槽上下移动。
垂直移动质量 = 机体质量（飞控机体参数 airframe.mass_kg）+ 台架随动附加质量（碳杆等，默认 38.8 g）。

与固件的契约（固件子任务同批实现，这里只对接、不改）：

* `SYSID MODE ALT`（固件编号 4）。
* `SYSID ALT inject=force|vel|pos mass_g=<g> win_mm=<mm> lift_mm=<mm>`：只在空闲时收、只存 RAM。
  回的是**一行** `SYSID ALT inject=.. mass_g=.. win_mm=.. lift_mm=..`（不是整份状态报告），
  拒绝回 `SYSID ALT event=rejected reason=running|range|usage`。开始前的配置事务把它排在
  `SYSID MODE` 之后，逐项核对这一行再发下一条。
* 激励沿用 `SYSID EXC`，幅值单位随注入类型：force → N（≤3）、vel → m/s（≤0.3）、pos → m（≤0.15）。
* 溯源：`SYSID start` 行末尾带 alt_*（自然进 conditions["start"]），或紧跟一行
  `SYSID ALTSTART`（存进 conditions["alt"]）；两种都认。
* 记录字段表末尾追加 height/height_raw/height_sp/vz/vz_sp/az/vbat：页面按 SCHEMA 自描述解码，
  这里只在开跑前检查字段表里有没有高度，不写死字段个数。

页面不拟合 ALT 轮：数据存档后离线分析。高度环参数（coax.pos_z_kp、coax.vel_z_*）的试用由
分析脚本经 `SYSID PARAM` 下发；页面「临时应用」的放行名单（ram_params._ALLOWED）仍只有姿态增益。
"""
from __future__ import annotations

import math

ALT_MODE = "ALT"
#: 固件里的模式编号（READY/快照里的 mode=4）。
ALT_MODE_CODE = 4

INJECTS = ("force", "vel", "pos")
INJECT_LABELS = {
    "force": "推力叠加激励：测推力与电机动态，配合挂砝码测质量",
    "vel": "速度参考（慢速三角/双脉冲）：匀速升降测摩擦",
    "pos": "高度阶跃：验证高度环参数",
}
#: 注入类型 -> (幅值单位, 幅值上限)。
AMP_UNITS = {"force": ("N", 3.0), "vel": ("m/s", 0.3), "pos": ("m", 0.15)}
#: 注入类型 -> 默认激励，写进「高级设置 → 激励编排」（键同 sections.EXPERIMENTS 的预设）。
#: 总时长按 2 × 保持 × 对数给足，免得双脉冲被总时长截掉；作者仍可改。
#: 按作者的槽式台架定（2026-09-29：槽行程 160 mm）：槽底起、抬升 50 mm 悬停、超过 120 mm 中止（离槽顶留 40 mm）。
#: 50/70 而不是 60/60：推力表悬停点误差 ±5% 时起升会冲高，仿真最坏约 120 mm（scripts/alt_hold_sim.py，summary §12）。
#: 幅值都让高度留在约 10–90 mm：阶跃 ±40 mm；匀速 0.04 m/s × 1 s 每段走 40 mm；推力 0.4 N 每个脉冲约 15 mm。
INJECT_PRESETS = {
    "force": dict(profile="doublet", amp="0.4", hold="300", repeat="8", dur="4800", ramp="20"),
    "vel": dict(profile="doublet", amp="0.04", hold="1000", repeat="3", dur="6000", ramp="150"),
    "pos": dict(profile="doublet", amp="0.04", hold="2000", repeat="2", dur="8000", ramp="150"),
}

DEFAULT_INJECT = "force"
DEFAULT_EXTRA_MASS_G = "38.8"
DEFAULT_WIN_MM = "70"
DEFAULT_LIFT_MM = "50"
#: 固件接受的范围（mass_g 另可为 0 = 用机体质量；页面总是下发合计，不发 0）。
MASS_RANGE_G = (500, 3000)
WIN_RANGE_MM = (30, 400)
LIFT_RANGE_MM = (30, 300)
#: 台架随动附加质量的页面检查范围 [g]（只挡明显填错的）。
EXTRA_MASS_RANGE_G = (0.0, 500.0)

#: 记录字段表末尾追加的高度相关字段（非 ALT 轮为 0）。开跑前只要求有 height 与 height_sp。
HEIGHT_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")
REQUIRED_FIELDS = ("height", "height_sp")
#: 溯源键（`SYSID start` 行末尾或 `SYSID ALTSTART` 行）。
PROVENANCE_KEYS = ("alt_inject", "alt_mass_g", "alt_win_mm", "alt_lift_mm", "alt_h0_mm")

REJECT_TEXT = {
    "running": "飞控拒绝高度设置：辨识正在进行，只能在空闲时设置。等这一轮结束后再点开始。",
    "range": (f"飞控拒绝高度设置：数值超出范围（质量 {MASS_RANGE_G[0]}～{MASS_RANGE_G[1]} g、"
              f"出窗余量 {WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm、抬升 {LIFT_RANGE_MM[0]}～"
              f"{LIFT_RANGE_MM[1]} mm）。到「准备」页「高度」分区检查。"),
    "usage": "飞控拒绝高度设置：命令格式不对（面板和固件版本可能不一致），请更新固件。",
}
UNSUPPORTED_SCHEMA = ("高度辨识（ALT）需要记录里带高度字段（height、height_sp）的新固件："
                      "请更新固件后重试。")


def _number(text, what: str, low: float, high: float, unit: str) -> float:
    try:
        value = float(str(text).strip())
    except (TypeError, ValueError):
        raise ValueError(f"{what}要填数字（单位 {unit}）") from None
    if not (math.isfinite(value) and low <= value <= high):
        raise ValueError(f"{what}要在 {low:g}～{high:g} {unit} 之间")
    return value


def parse_inputs(inject: str, extra_mass: str, win: str, lift: str) -> dict:
    """「高度」分区的四项 → `dict(inject, extra_mass_g, win_mm, lift_mm)`；填错抛中文 ValueError。"""
    if inject not in INJECTS:
        raise ValueError("高度注入类型要选 force / vel / pos")
    extra = _number(extra_mass, "台架随动附加质量", *EXTRA_MASS_RANGE_G, "g")
    win_mm = _number(win, "出窗余量", *WIN_RANGE_MM, "mm")
    lift_mm = _number(lift, "抬升高度", *LIFT_RANGE_MM, "mm")
    if win_mm != int(win_mm) or lift_mm != int(lift_mm):
        raise ValueError("出窗余量和抬升高度要填整数毫米")
    return dict(inject=inject, extra_mass_g=extra, win_mm=int(win_mm), lift_mm=int(lift_mm))


def check_amplitude(inject: str, amplitude: float) -> None:
    """ALT 下激励幅值的单位与上限随注入类型变；超限抛中文 ValueError。"""
    unit, limit = AMP_UNITS[inject]
    if not (math.isfinite(amplitude) and 0.0 < amplitude <= limit):
        raise ValueError(f"高度辨识（{inject}）的激励幅值单位是 {unit}，上限 {limit:g} {unit}；"
                         f"现在是 {amplitude:g}。到「高级设置 → 激励编排」改小，"
                         "或在「准备」页重选一次注入类型换回默认激励。")


def airframe_mass_kg(params: dict, status: dict) -> float | None:
    """机体质量 [kg]：优先参数回显的 airframe.mass_kg，退而用状态报告 READY 的 mass_mg。"""
    for source in (lambda: float(params["airframe.mass_kg"]),
                   lambda: float(status["mass_mg"]) * 1e-6):
        try:
            value = source()
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            return value
    return None


def total_mass_g(airframe_kg: float, extra_g: float) -> int:
    """下发的 mass_g = 机体质量 + 附加质量，四舍五入到整数克；超出固件范围抛中文 ValueError。"""
    total = int(round(airframe_kg * 1000.0 + extra_g))
    low, high = MASS_RANGE_G
    if not low <= total <= high:
        raise ValueError(f"垂直移动质量合计 {total} g 超出飞控接受的 {low}～{high} g："
                         "核对机体参数的质量和「台架随动附加质量」。")
    return total


def alt_command(inject: str, mass_g: int, win_mm: int, lift_mm: int) -> str:
    return f"SYSID ALT inject={inject} mass_g={mass_g} win_mm={win_mm} lift_mm={lift_mm}"


def schema_has_height(schema) -> bool:
    try:
        names = set(schema.field_names())
    except AttributeError:
        return False
    return all(name in names for name in REQUIRED_FIELDS)


def reject_text(reason: str | None) -> str:
    return REJECT_TEXT.get((reason or "").strip(), "飞控拒绝了高度设置。")


def provenance(conditions: dict | None) -> dict:
    """本轮高度设置的溯源：`SYSID ALTSTART` 行（conditions["alt"]）优先，其次 start 行的 alt_*。"""
    conditions = conditions or {}
    found = {}
    start = conditions.get("start")
    if isinstance(start, dict):
        found.update({key: start[key] for key in PROVENANCE_KEYS if key in start})
    alt = conditions.get("alt")
    if isinstance(alt, dict):
        found.update({key: alt[key] for key in PROVENANCE_KEYS if key in alt})
    return found


def provenance_text(conditions: dict | None) -> str:
    """结果抬头那句：这一轮飞控实际用的高度设置（没有溯源就写页面下发的请求）。"""
    conditions = conditions or {}
    found = provenance(conditions)
    if found:
        parts = [f"注入 {found.get('alt_inject', '?')}", f"质量 {found.get('alt_mass_g', '?')} g",
                 f"出窗余量 {found.get('alt_win_mm', '?')} mm", f"抬升 {found.get('alt_lift_mm', '?')} mm"]
        if "alt_h0_mm" in found:
            parts.append(f"起始高度 {found['alt_h0_mm']} mm")
        return "本轮高度设置（飞控开跑时报告）：" + " · ".join(parts)
    request = conditions.get("alt_request")
    if isinstance(request, dict) and request:
        return ("本轮高度设置（页面下发并核对过；固件开跑时没报溯源）："
                f"注入 {request.get('inject', '?')} · 质量 {request.get('mass_g', '?')} g · "
                f"出窗余量 {request.get('win_mm', '?')} mm · 抬升 {request.get('lift_mm', '?')} mm")
    return "本轮高度设置：固件没报溯源。"


OFFLINE_NOTE = ("高度辨识：页面不做拟合，离线分析（原始记录与本轮条件已存档，"
                "高度环参数的试用由分析脚本经 SYSID PARAM 下发）。")


class AltWorkflow:
    """Workflow 的 ALT 部分：开跑前的输入检查、`SYSID ALT` 的生成与回显核对、开跑溯源。"""

    alt_request: dict | None = None
    alt_expected: dict = {}

    def alt_prepare(self, page, spec) -> bool:
        """开始前（发任何命令之前）检查 ALT 输入；不是 ALT 就清掉上一轮的 ALT 状态。"""
        self.alt_request = None
        self.alt_expected = {}
        if page.mode_var.get() != ALT_MODE:
            return False
        if self.throttle[0]:
            raise ValueError("高度辨识（ALT）要程序油门：先取消勾选「遥控器手动给油门」"
                             "（固件自己升推力、用高度环把机体抬起来）。")
        request = parse_inputs(page.alt_inject_var.get(), page.alt_extra_mass_var.get(),
                               page.alt_win_var.get(), page.alt_lift_var.get())
        check_amplitude(request["inject"], spec.amplitude_rad_s)
        self.alt_request = request
        return True

    def _alt_command(self):
        """轮到它时才生成：机体质量要等 PARAM? / 状态报告回来。"""
        request = self.alt_request
        mass = airframe_mass_kg(self.params, self.status)
        if mass is None:
            raise ValueError("读不到机体质量（机体参数 airframe.mass_kg）：高度辨识要用它算垂直移动"
                             "质量。先到「机体模型」页确认参数已下发到飞控，再重试。")
        mass_g = total_mass_g(mass, request["extra_mass_g"])
        self.alt_expected = dict(inject=request["inject"], mass_g=mass_g,
                                 win_mm=request["win_mm"], lift_mm=request["lift_mm"])
        request.update(airframe_mass_g=round(mass * 1000.0, 1), mass_g=mass_g)
        return alt_command(request["inject"], mass_g, request["win_mm"], request["lift_mm"])

    def alt_handle_line(self, line: str, values: dict) -> bool:
        """`SYSID ALT ...`（回显/拒绝）与 `SYSID ALTSTART ...`（溯源）；返回 True = 已处理。"""
        if line.startswith("SYSID ALTSTART "):
            run = values.get("run")
            open_run = self.run_id is not None and self.end is None
            if self.snapshot is not None and (str(self.run_id) == run or (run is None and open_run)):
                self.snapshot["alt"] = dict(values)
            return True
        if not line.startswith("SYSID ALT "):
            return False
        if not (self.awaiting and self.awaiting.startswith("SYSID ALT ")):
            return True                  # 不在等它：旧回复或别处发的，不当成本次回显
        if values.get("event") == "rejected":
            self.fail(f"{reject_text(values.get('reason'))}（飞控原话：{line}）")
            return True
        for key, expected in self.alt_expected.items():
            if str(values.get(key, "")) != str(expected):
                self.fail(f"配置回读不符：高度 {key}（发 {expected}，回 {values.get(key, '缺')}），取消开始")
                return True
        if not self.pending and not self._verify():
            return True
        self.advance()
        return True

    def alt_snapshot(self) -> None:
        """开跑时把页面这次下发的高度请求（含机体质量与附加质量的拆分）记进快照。"""
        if self.snapshot is not None and self.alt_request and self.alt_expected:
            self.snapshot["alt_request"] = dict(self.alt_request)


__all__ = [
    "ALT_MODE", "ALT_MODE_CODE", "AMP_UNITS", "AltWorkflow", "HEIGHT_FIELDS", "INJECTS",
    "INJECT_LABELS", "INJECT_PRESETS", "OFFLINE_NOTE", "airframe_mass_kg", "alt_command",
    "check_amplitude", "parse_inputs", "provenance", "provenance_text", "reject_text",
    "schema_has_height", "total_mass_g",
]

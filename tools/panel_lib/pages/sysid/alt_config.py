"""光杆台架高度辨识（「本轮做」选 ALT，R-ALTID-1）：参数、下发命令、回显核对与溯源。

台架：机体装在碳杆上（杆轴 −45°），杆的轴装在竖直槽里，机体连杆一起沿槽上下移动。
垂直移动质量 = 机体质量（飞控机体参数 airframe.mass_kg）+ 台架随动附加质量（碳杆等，默认 38.8 g）。

与固件的契约（跨侧契约 2026-09-30，固件同批实现，这里只对接、不改）：

* `SYSID MODE ALT`（固件编号 4）。
* `SYSID ALT inject=break|vel|pos mass_g=<g> win_mm=<mm> lift_mm=<mm> bottom_mm=<mm> top_mm=<mm>`：
  只在空闲时收、只存 RAM。回的是**一行** `SYSID ALT inject=.. mass_g=.. win_mm=.. lift_mm=..
  bottom_mm=.. top_mm=.. control=breakaway|closed_loop`（不是整份状态报告），拒绝回
  `SYSID ALT event=rejected reason=running|range|usage`。开始前的配置事务把它排在 `SYSID MODE`
  之后，逐项核对这一行再发下一条。
* bottom_mm/top_mm 是测距（TOF）在物理槽底/槽顶的**绝对读数**；行程 top − bottom 要在 135～185 mm。
  上端硬窗：vel/pos = min(槽底 + 抬升 + 余量, 槽顶 − 40 mm)，break = 槽顶 − 10 mm；低于槽底 30 mm
  也中止。固件流程自己的中止先软着陆（慢降回槽底、再 1 s 到怠速）才报。
* break（离地/滑落阈值）：慢升推力找离地、制停、慢降找滑落，不经高度 PID。激励沿用 `SYSID EXC`，
  但 amp 是慢升/慢降**速率** [N/s]（≤1），剖面不用；合推力是**离地搜索上限**，要在本轮移动质量的
  重力到重力 + 5 N 之间。vel/pos 仍是生产高度环闭环，幅值 m/s（≤0.3）、m（≤0.15）。
* 溯源：`SYSID start` 行末尾带 alt_*（自然进 conditions["start"]），或紧跟一行
  `SYSID ALTSTART`（存进 conditions["alt"]）；两种都认。本轮的 `SYSID PHASE` 回报按顺序记进
  conditions["alt_phases"]（break 的阈值分析要用各阶段进入时的推力）。
* 记录字段表末尾追加 height/height_raw/height_sp/vz/vz_sp/az/vbat：页面按 SCHEMA 自描述解码，
  这里只在开跑前检查字段表里有没有高度，不写死字段个数。

break 轮结束后页面按内存里的记录算离地/滑落阈值（`alt_breakaway.py` → `tools/sysid/breakaway.py`），
不拟合模型。高度环参数（coax.pos_z_kp、coax.vel_z_*）按模型离线设计，经 `SYSID PARAM` 试用；
页面「临时应用」的放行名单（ram_params._ALLOWED）仍只有姿态增益。
"""
from __future__ import annotations

import math

ALT_MODE = "ALT"
#: 固件里的模式编号（READY/快照里的 mode=4）。
ALT_MODE_CODE = 4

INJECTS = ("break", "vel", "pos")
INJECT_LABELS = {
    "break": "离地/滑落阈值：慢升找离地、慢降找滑落，测推力表偏差与槽摩擦（不经高度 PID）",
    "vel": "闭环速度参考：使用当前生产高度 PID，供对象辨识后的候选参数验证",
    "pos": "闭环高度参考：使用当前生产高度 PID，验证候选高度环参数",
}
#: 注入类型 -> (幅值单位, 幅值上限)。
#: break 的「幅值」是慢升/慢降速率 [N/s]。
AMP_UNITS = {"break": ("N/s", 1.0), "vel": ("m/s", 0.3), "pos": ("m", 0.15)}
#: 注入类型 -> 默认激励，写进「高级设置 → 激励编排」（键同 sections.EXPERIMENTS 的预设）。
#: vel/pos 的总时长按 2 × 保持 × 对数给足，免得双脉冲被总时长截掉；作者仍可改。
#: 按作者的槽式台架定（2026-09-29：槽行程 160 mm）：槽底 + lift 50 mm + win 70 mm =
#: 上端硬窗 120 mm（离槽顶留 40 mm）。break 固件只用 amp（速率），剖面给一个能过体检的短阶跃。
INJECT_PRESETS = {
    "break": dict(profile="step", amp="0.5", hold="100", repeat="1", dur="300", ramp="100"),
    "vel": dict(profile="doublet", amp="0.04", hold="1000", repeat="3", dur="6000", ramp="150"),
    "pos": dict(profile="doublet", amp="0.04", hold="2000", repeat="2", dur="8000", ramp="150"),
}

DEFAULT_INJECT = "break"
DEFAULT_EXTRA_MASS_G = "38.8"
DEFAULT_WIN_MM = "70"
DEFAULT_LIFT_MM = "50"
#: 固件接受的范围（mass_g 另可为 0 = 用机体质量；页面总是下发合计，不发 0）。
MASS_RANGE_G = (500, 3000)
WIN_RANGE_MM = (30, 400)
LIFT_RANGE_MM = (30, 300)
#: 槽底/槽顶测距读数 [mm]（0 = 没配置，固件拒绝开跑）与槽行程的范围。
SLOT_RANGE_MM = (1, 4000)
TRAVEL_RANGE_MM = (135, 185)
#: 上端硬窗离槽顶至少留这么多 [mm]（break 没有抬升/余量，只留 10 mm：顶到止挡不危险）。
TOP_MARGIN_MM = 40
BREAK_TOP_MARGIN_MM = 10
#: break 的离地搜索上限最多比本轮移动质量的重力高这么多 [N]。
BREAK_EXCESS_MAX_N = 5.0   # 2026-09-30：实测要约 14.5～15 N 才滑起来，+3 N 不够
GRAVITY_M_S2 = 9.80665
#: 台架随动附加质量的页面检查范围 [g]（只挡明显填错的）。
EXTRA_MASS_RANGE_G = (0.0, 500.0)

#: 记录字段表末尾追加的高度相关字段（非 ALT 轮为 0）。开跑前只要求有 height 与 height_sp。
HEIGHT_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")
REQUIRED_FIELDS = ("height", "height_sp")
#: 溯源键（`SYSID start` 行末尾或 `SYSID ALTSTART` 行）。
PROVENANCE_KEYS = ("alt_inject", "alt_mass_g", "alt_win_mm", "alt_lift_mm", "alt_h0_mm",
                   "alt_control", "alt_target_cn", "alt_bottom_mm", "alt_top_mm")

CONTROL_BY_INJECT = {"break": "breakaway", "vel": "closed_loop", "pos": "closed_loop"}
#: drv_sysid_record.h：本批至少有一次合推力被油门/分配上限裁剪。
ALT_THRUST_CAPPED_FLAG = 0x0200

REJECT_TEXT = {
    "running": "飞控拒绝高度设置：辨识正在进行，只能在空闲时设置。等这一轮结束后再点开始。",
    "range": (f"飞控拒绝高度设置：数值超出范围（质量 {MASS_RANGE_G[0]}～{MASS_RANGE_G[1]} g、"
              f"出窗余量 {WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm、抬升 {LIFT_RANGE_MM[0]}～"
              f"{LIFT_RANGE_MM[1]} mm、槽底/槽顶读数 {SLOT_RANGE_MM[0]}～{SLOT_RANGE_MM[1]} mm）。"
              "到「Z 高度」页「高度设置」检查。"),
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


def _slot_mm(text, what: str, button: str) -> int:
    if not str(text).strip():
        raise ValueError(f"{what}还没填：机体压在对应位置时点「{button}」取当前测距，"
                         "或手填测距（TOF）读数（mm）")
    value = _number(text, what, *SLOT_RANGE_MM, "mm")
    if value != int(value):
        raise ValueError(f"{what}要填整数毫米")
    return int(value)


def parse_inputs(inject: str, extra_mass: str, win: str, lift: str, bottom: str, top: str) -> dict:
    """「高度设置」分区六项 → `dict(inject, extra_mass_g, win_mm, lift_mm, bottom_mm, top_mm)`；
    填错抛中文 ValueError。"""
    if inject not in INJECTS:
        raise ValueError("高度注入类型要选 break / vel / pos")
    extra = _number(extra_mass, "台架随动附加质量", *EXTRA_MASS_RANGE_G, "g")
    win_mm = _number(win, "出窗余量", *WIN_RANGE_MM, "mm")
    lift_mm = _number(lift, "抬升高度", *LIFT_RANGE_MM, "mm")
    if win_mm != int(win_mm) or lift_mm != int(lift_mm):
        raise ValueError("出窗余量和抬升高度要填整数毫米")
    bottom_mm = _slot_mm(bottom, "槽底测距读数", "记为槽底")
    top_mm = _slot_mm(top, "槽顶测距读数", "记为槽顶")
    travel = top_mm - bottom_mm
    low, high = TRAVEL_RANGE_MM
    if not low <= travel <= high:
        raise ValueError(f"槽行程（槽顶 − 槽底）= {travel} mm，要在 {low}～{high} mm 之间：核对两个读数"
                         "（槽顶读数要比槽底大），或把机体压到位后重新点「记为槽底」「记为槽顶」")
    return dict(inject=inject, extra_mass_g=extra, win_mm=int(win_mm), lift_mm=int(lift_mm),
                bottom_mm=bottom_mm, top_mm=top_mm)


def upper_window_mm(bottom_mm: int, top_mm: int, lift_mm: int, win_mm: int, inject: str) -> int:
    """固件的上端硬窗：vel/pos = min(槽底 + 抬升 + 余量, 槽顶 − 40 mm)，break = 槽顶 − 10 mm。"""
    if inject == "break":
        return top_mm - BREAK_TOP_MARGIN_MM
    return min(bottom_mm + lift_mm + win_mm, top_mm - TOP_MARGIN_MM)


def check_amplitude(inject: str, amplitude: float) -> None:
    """ALT 下激励幅值的单位与上限随注入类型变（break 是慢升/慢降速率）；超限抛中文 ValueError。"""
    unit, limit = AMP_UNITS[inject]
    if not (math.isfinite(amplitude) and 0.0 < amplitude <= limit):
        what = "慢升/慢降速率" if inject == "break" else "激励幅值"
        raise ValueError(f"高度辨识（{inject}）的{what}单位是 {unit}，上限 {limit:g} {unit}；"
                         f"现在是 {amplitude:g}。到本页「激励」改小，或重选一次注入类型换回默认值。")


def break_target_limit(mass_g: int, max_force_n: float | None = None) -> float:
    """离地搜索上限最多填多少：重力 + 5 N，且不超过飞控整机最大推力（airframe.max_total_force_n，
    SYSID THROTTLE 的上限，超了飞控直接拒收）。"""
    high = mass_g * 0.001 * GRAVITY_M_S2 + BREAK_EXCESS_MAX_N
    if max_force_n is not None and math.isfinite(max_force_n) and max_force_n > 0.0:
        high = min(high, max_force_n)
    return high


def airframe_max_force_n(params: dict) -> float | None:
    try:
        value = float(params["airframe.max_total_force_n"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def check_break_target(target_n: float, mass_g: int, max_force_n: float | None = None) -> None:
    """break 的离地搜索上限：不低于本轮移动质量的重力（否则离不了地），不高于重力 + 5 N（离地后冲太猛），
    也不高于飞控整机最大推力。"""
    weight = mass_g * 0.001 * GRAVITY_M_S2
    limit = break_target_limit(mass_g, max_force_n)
    low = math.ceil(weight * 10.0) / 10.0
    high = math.floor(limit * 10.0) / 10.0
    if max_force_n is not None and target_n > limit and limit < weight + BREAK_EXCESS_MAX_N:
        raise ValueError(f"离地搜索上限 {target_n:g} N 超过飞控整机最大推力 {limit:.2f} N"
                         f"（机体参数 airframe.max_total_force_n，飞控会拒收）。改成 {low:g}～{high:g} N；已取消开始。")
    if target_n < weight:
        raise ValueError(f"离地搜索上限 {target_n:g} N 低于本轮移动质量（{mass_g} g）的重力 "
                         f"{weight:.2f} N：慢升到上限也离不了地。改成 {low:g}～{high:g} N；已取消开始。")
    if target_n > weight + BREAK_EXCESS_MAX_N:
        raise ValueError(f"离地搜索上限 {target_n:g} N 超过本轮移动质量（{mass_g} g）的重力 + 5 N"
                         f"（{weight + BREAK_EXCESS_MAX_N:.2f} N）：一旦离地会冲得太猛。"
                         f"改成 {low:g}～{high:g} N；已取消开始。")


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


def alt_command(inject: str, mass_g: int, win_mm: int, lift_mm: int, bottom_mm: int,
                top_mm: int) -> str:
    return (f"SYSID ALT inject={inject} mass_g={mass_g} win_mm={win_mm} lift_mm={lift_mm} "
            f"bottom_mm={bottom_mm} top_mm={top_mm}")


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
        if "alt_bottom_mm" in found or "alt_top_mm" in found:
            parts.append(f"槽底 {found.get('alt_bottom_mm', '?')} mm / 槽顶 "
                         f"{found.get('alt_top_mm', '?')} mm")
        if "alt_control" in found:
            parts.append(f"控制 {found['alt_control']}")
        if "alt_target_cn" in found:
            try:
                label = "离地搜索上限" if found.get("alt_control") == "breakaway" else "目标合推力"
                parts.append(f"{label} {int(found['alt_target_cn']) / 100:g} N")
            except (TypeError, ValueError):
                parts.append(f"合推力设置 {found['alt_target_cn']} cN（格式异常）")
        if "alt_h0_mm" in found:
            parts.append(f"起始高度 {found['alt_h0_mm']} mm")
        if "alt_thrust_capped_batches" in conditions:
            parts.append(f"推力封顶批次 {conditions['alt_thrust_capped_batches']}")
        return "本轮高度设置（飞控开跑时报告）：" + " · ".join(parts)
    request = conditions.get("alt_request")
    if isinstance(request, dict) and request:
        return ("本轮高度设置（页面下发并核对过；固件开跑时没报溯源）："
                f"注入 {request.get('inject', '?')} · 质量 {request.get('mass_g', '?')} g · "
                f"出窗余量 {request.get('win_mm', '?')} mm · 抬升 {request.get('lift_mm', '?')} mm · "
                f"槽底 {request.get('bottom_mm', '?')} mm / 槽顶 {request.get('top_mm', '?')} mm · "
                f"控制 {request.get('control', '?')}")
    return "本轮高度设置：固件没报溯源。"


OFFLINE_NOTE = ("高度辨识：页面不做拟合，离线分析（收到的原始记录与本轮条件按采集情况存档，"
                "高度环参数的试用由分析脚本经 SYSID PARAM 下发）。")
BREAK_NOTE = ("离地/滑落阈值由页面按本轮记录算出（上一行）；原始记录与本轮条件已存档，"
              "多轮复核用 python -m tools.sysid.breakaway <alt_* 目录…>（给均值与离散）。")


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
                             "（固件接管合推力并保持杆轴姿态环）。")
        request = parse_inputs(page.alt_inject_var.get(), page.alt_extra_mass_var.get(),
                               page.alt_win_var.get(), page.alt_lift_var.get(),
                               page.alt_bottom_var.get(), page.alt_top_var.get())
        check_amplitude(request["inject"], spec.amplitude_rad_s)
        if request["inject"] == "break":
            if self.throttle[1] is None:
                raise ValueError("离地/滑落阈值（break）须明确填写「离地搜索上限 [N]」：它限定慢升找离地"
                                 "的推力，不能留空自动用机重。")
            if self.throttle[1] < 2.0:
                raise ValueError("离地搜索上限至少 2 N；到「Z 高度」页检查所填合推力。")
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
        if request["inject"] == "break":
            check_break_target(self.throttle[1], mass_g, airframe_max_force_n(self.params))
        self.alt_expected = dict(inject=request["inject"], mass_g=mass_g,
                                 win_mm=request["win_mm"], lift_mm=request["lift_mm"],
                                 bottom_mm=request["bottom_mm"], top_mm=request["top_mm"],
                                 control=CONTROL_BY_INJECT[request["inject"]])
        request.update(airframe_mass_g=round(mass * 1000.0, 1), mass_g=mass_g)
        return alt_command(request["inject"], mass_g, request["win_mm"], request["lift_mm"],
                           request["bottom_mm"], request["top_mm"])

    def alt_handle_line(self, line: str, values: dict) -> bool:
        """`SYSID ALT ...`（回显/拒绝）与 `SYSID ALTSTART ...`（溯源）；返回 True = 已处理。
        本轮 ALT 的 `SYSID PHASE` 回报按顺序记进快照（返回 False：别处照常用它刷新阶段）。"""
        if line.startswith("SYSID PHASE "):
            if (self.snapshot is not None and self.run_id is not None and self.end is None
                    and str(self.run_id) == values.get("run")
                    and str(self.snapshot.get("mode")) == str(ALT_MODE_CODE)):
                self.snapshot.setdefault("alt_phases", []).append(dict(values))
            return False
        if line.startswith("SYSID ALTSTART "):
            run = values.get("run")
            open_run = self.run_id is not None and self.end is None
            if self.snapshot is not None and (str(self.run_id) == run or (run is None and open_run)):
                self.snapshot["alt"] = dict(values)
                problem = self._alt_provenance_problem(values)
                if problem:
                    self.mark_data_error(problem)
                    self.page.status_var.set(problem + "；已请求停止，本轮仅留作诊断")
                    self.page.send("SYSID STOP")
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
            self.snapshot["alt_request"] = dict(self.alt_request,
                                                control=self.alt_expected["control"],
                                                target_cn=self.expected.get("target_cn"))

    def _alt_provenance_problem(self, values: dict) -> str:
        expected = {"alt_inject": self.alt_expected.get("inject"),
                    "alt_mass_g": self.alt_expected.get("mass_g"),
                    "alt_win_mm": self.alt_expected.get("win_mm"),
                    "alt_lift_mm": self.alt_expected.get("lift_mm"),
                    "alt_control": self.alt_expected.get("control"),
                    "alt_target_cn": self.expected.get("target_cn"),
                    "alt_bottom_mm": self.alt_expected.get("bottom_mm"),
                    "alt_top_mm": self.alt_expected.get("top_mm")}
        for key, wanted in expected.items():
            if str(values.get(key, "")) != str(wanted):
                return f"飞控开跑溯源不符：{key}（期望 {wanted}，回 {values.get(key, '缺')}）"
        return ""

    def alt_finish_validation(self, end: dict, gap_count: int, batches: list) -> None:
        """ALT 原始数据可存档；中止、缺包与溯源缺失必须显式标为不能拟合。"""
        if not self.snapshot or str(self.snapshot.get("mode")) != str(ALT_MODE_CODE):
            return
        capped = sum(bool(batch.flags & ALT_THRUST_CAPPED_FLAG) for batch in batches)
        self.snapshot["alt_thrust_capped_batches"] = capped
        self.snapshot["alt_thrust_capped_flag"] = bool(capped)
        problems = [self.data_error] if self.data_error else []
        alt = self.snapshot.get("alt")
        problem = (self._alt_provenance_problem(alt) if isinstance(alt, dict)
                   else "缺少飞控开跑溯源（SYSID ALTSTART）")
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
        if ((self.snapshot.get("alt_request") or {}).get("control") == "breakaway"
                and not any(entry.get("phase") == "settle"
                            for entry in self.snapshot.get("alt_phases", []))):
            problems.append("未离地（没收到飞控 settle 阶段回报 SYSID PHASE phase=settle）")
        if capped:
            problems.append(f"推力封顶 {capped} 批（THRUST_CAPPED 0x0200），激励已被裁剪")
        if problems:
            self.mark_data_error("；".join(dict.fromkeys(problems)))

    def alt_quality_note(self) -> str:
        if self.data_error:
            return f"本轮仅供诊断，不可用于拟合：{self.data_error}。收到的原始记录仍按情况存档。"
        return "本轮采集完整，原始记录已存档；离线拟合前仍需核对激励和测高质量。"


__all__ = [
    "ALT_MODE", "ALT_MODE_CODE", "AMP_UNITS", "AltWorkflow", "BREAK_NOTE", "HEIGHT_FIELDS",
    "INJECTS", "INJECT_LABELS", "INJECT_PRESETS", "OFFLINE_NOTE", "SLOT_RANGE_MM",
    "TRAVEL_RANGE_MM", "airframe_mass_kg", "airframe_max_force_n", "alt_command", "break_target_limit",
    "check_amplitude", "check_break_target",
    "parse_inputs", "provenance", "provenance_text", "reject_text", "schema_has_height",
    "total_mass_g", "upper_window_mm",
]

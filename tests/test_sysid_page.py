"""「系统辨识」三个子页的契约。

钉的都是这次重写要解决的具体问题：

* 旧页整个住在 `tools/drone_tcp_panel.py` 里（`_build_ident_page` + 19 个
  `_ident_*` 方法），而那个文件是**只减不增**的。新页不许在里面留一个字。
* 旧页发的是**舵机脉宽偏置**（`IDENT STEP … pulse_us=…`）。新页发的是**期望角
  速度**，反解由固件用在飞的那个分配器做。
* 旧页的 `_ident_apply_fit` 是个空壳，自己在界面上写着"不写入四环控制器"。
  新页写 RAM 且写的是真实存在的参数名。
* 新页**没有**写 Flash 的路径。
* 未实现的两页不放假按钮——点了没反应比"还没做"危险。
"""
from __future__ import annotations

import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from panel_lib.pages.sysid import (  # noqa: E402
    SYSID_ALTITUDE_TAB_TEXT, SYSID_HORIZONTAL_TAB_TEXT,
    SYSID_INNER_TAB_TEXT, SYSID_TAB_TEXT,
)
from sysid.decode import FLAG_FIRST_BATCH, FLAG_LAST_BATCH  # noqa: E402
sys.path.pop(0)


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ---------------------------------------------------------------- 迁出


def test_the_old_page_left_no_trace_in_the_append_banned_file():
    panel = read("tools/drone_tcp_panel.py")
    for symbol in ("_build_ident_page", "_ident_handle_line", "_ident_fit",
                   "_ident_apply_fit", "ident_record_from_line",
                   "fit_ident_step", "MAX_IDENT_SAMPLES", "ident_samples",
                   "ident_axis_var", "ident_figure"):
        assert symbol not in panel, symbol


def test_the_new_page_does_not_live_in_the_append_banned_file():
    panel = read("tools/drone_tcp_panel.py")
    assert "sysid" not in panel.lower()
    assert "SYSID" not in panel


def test_the_one_kept_variable_is_the_one_still_in_use():
    """`ident_link_var` 留着是因为 rx_dispatch 还在写它（UDP 裸帧计数）。

    删了它 UDP 那条路径会在运行时炸，而单测未必覆盖得到。
    """
    assert "self.ident_link_var" in read("tools/drone_tcp_panel.py")
    assert "panel.ident_link_var" in read("tools/panel_lib/rx_dispatch.py")


def test_the_shell_registers_the_group_and_its_three_sub_pages():
    shell = read("tools/panel_lib/shell.py")
    assert "mount_sysid(self, sysid_group)" in shell
    assert "self.notebook.add(sysid_group, text=SYSID_TAB_TEXT)" in shell
    assert "self._build_ident_page" not in shell

    package = read("tools/panel_lib/pages/sysid/__init__.py")
    for label in (SYSID_INNER_TAB_TEXT, SYSID_HORIZONTAL_TAB_TEXT,
                  SYSID_ALTITUDE_TAB_TEXT):
        assert f'text={label!r}'.replace("'", '"') in package or label in package
    assert SYSID_TAB_TEXT == "系统辨识"


def test_the_geometry_gate_walks_into_the_sub_pages():
    """不登记分组的话几何遍历只到分组框为止，三个子页一个都不检查。

    那种漏检在测试结果里表现为"全绿"——所以必须钉死。
    """
    harness = read("tools/panel_qa/harness.py")
    assert 'getattr(panel, "sysid_tab", None)' in harness
    assert 'getattr(\n                panel, "sysid_notebook", None)' in harness \
        or 'panel, "sysid_notebook", None' in harness


# ---------------------------------------------------------------- 命令面


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []

    def send_line(self, text: str) -> bool:
        self.lines.append(text)
        return True


class FakePanel:
    def __init__(self) -> None:
        self.transport = FakeTransport()
        self.logged: list[str] = []

    def _append(self, text: str) -> None:
        self.logged.append(text)


@pytest.fixture
def page(monkeypatch, tmp_path):
    """台架设置文件指向本测试的临时目录：读、写都不碰真实 data/。"""
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:  # pragma: no cover - 无显示环境
        pytest.skip("Tk display unavailable")
    root.withdraw()
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.inner_loop import SysIdInnerLoopPage
        from panel_lib.pages.sysid import settings_store
        from tkinter import ttk

        monkeypatch.setattr(settings_store, "settings_path",
                            lambda: tmp_path / "rig_settings.json")
        panel = FakePanel()
        frame = ttk.Frame(root)
        instance = SysIdInnerLoopPage(panel, frame)
        instance.panel = panel
        yield instance
    finally:
        sys.path.pop(0)
        root.destroy()


def test_the_page_commands_an_angular_rate_not_a_servo_pulse(page):
    """旧页发 `IDENT STEP roll pulse_us=20`——直接给舵机脉宽偏置。

    新页发期望角速度，力矩反解交给固件里在飞的那个分配器。辨识自己再实现一遍
    力学的话，辨出来的是"辨识程序的模型"而不是"飞控的模型"。
    """
    page.send_excitation()
    sent = page.panel.transport.lines
    assert len(sent) == 1
    command = sent[0]
    assert command.startswith("SYSID EXC ")
    assert "amp=" in command
    assert "pulse_us" not in command


#: 机体参数里飞控在质心上方 0.035 m（imu_z − cg_z），照固件 PARAM 行格式。
AIRFRAME_GEOMETRY = ["PARAM name=airframe.imu_z_m value=0.080000",
                     "PARAM name=airframe.cg_z_m value=0.045000"]


def rig_command_after_readback(page, *, params=AIRFRAME_GEOMETRY, **report):
    """「下发台架几何」：先 PARAM? + SYSID?，等机体参数回来才组装 RIG。"""
    page.send_rig()
    assert page.panel.transport.lines[-2:] == ["PARAM?", "SYSID?"]
    feed(page, params)
    feed(page, status_report({}, **report))
    return page.panel.transport.lines[-1]


def test_rod_to_cg_is_measured_rod_to_board_plus_airframe_board_to_cg(page):
    """d 是拟合的输入：你量杆到飞控板，飞控到质心取机体参数，两段相加。"""
    page.psi_var.set("45")
    page.rod_to_fc_var.set("0.15")
    command = rig_command_after_readback(page)
    assert command == "SYSID RIG psi_deg=45 axis_off_m=0.185 imu_off_m=0.035"
    hint = page.geometry_hint_var.get()
    assert "+0.035" in hint and "0.185" in hint


def test_board_to_cg_falls_back_to_the_status_report(page):
    page.rod_to_fc_var.set("0.1")
    command = rig_command_after_readback(page, params=(), airframe_imu_off_um=20000)
    assert "axis_off_m=0.12 imu_off_m=0.02" in command


def test_direct_rod_to_cg_override_wins(page):
    page.axis_override_var.set("0.2")
    command = rig_command_after_readback(page)
    assert "axis_off_m=0.2 imu_off_m=0.035" in command


def test_unmeasured_rod_distance_is_refused_before_sending(page):
    page.send_rig()
    page.start_run()
    assert page.panel.transport.lines == []
    assert "杆到飞控板的垂直距离" in page.banner_detail_var.get()


def test_missing_airframe_geometry_is_a_chinese_error(page):
    page.rod_to_fc_var.set("0.15")
    rig_command_after_readback(page, params=(), airframe_imu_off_um=None)
    assert not any(line.startswith("SYSID RIG ") for line in page.panel.transport.lines)
    assert "读不到飞控到质心的距离" in page.status_var.get()
    assert page.workflow.awaiting is None


def test_a_broken_excitation_is_not_sent_at_all(page):
    """半套参数跑出来的数据看起来完全正常，只是它不是你以为的那条激励。"""
    page.ramp_var.set("0")  # 方波的导数是冲激，前馈会要求无穷大倾角
    page.send_excitation()
    assert page.panel.transport.lines == []
    assert "未下发" in page.status_var.get()


def test_rig_send_waits_for_verified_reply_and_reports_next_to_button(page):
    page.psi_var.set("90")
    page.rod_to_fc_var.set("0.465")
    rig_command_after_readback(page)
    assert "等待" in page.status_var.get()
    assert str(page.rig_status_label.cget("textvariable")) == str(page.status_var)
    feed(page, status_report(page.workflow.expected))
    assert page.workflow.expected["axis_off_um"] == 500000
    assert "台架几何已回读确认" in page.status_var.get()
    assert page.workflow.awaiting is None


@pytest.mark.parametrize("outcome", ["timeout", "reject", "mismatch"])
def test_rig_send_failure_is_visible(page, outcome):
    page.rod_to_fc_var.set("0.15")
    assert rig_command_after_readback(page).startswith("SYSID RIG ")
    if outcome == "timeout":
        page.workflow.timeout(page.workflow.ticket)
        assert "超时" in page.status_var.get()
    elif outcome == "reject":
        page.handle_line("ERR sysid rig range")
        assert "ERR sysid rig range" in page.status_var.get()
    else:
        feed(page, status_report({}))            # 回显里 axis_off_um=0，不是 185000
        assert "回读不符" in page.status_var.get()
    assert page.workflow.awaiting is None


@pytest.mark.parametrize("index,angle", [(0, 90), (1, 45), (2, -45)])
def test_axis_preset_updates_draft_then_sends_existing_rig_command(page, index, angle):
    page.axis_preset_combo.current(index)
    page.axis_preset_combo.event_generate("<<ComboboxSelected>>")
    assert float(page.psi_var.get()) == angle
    assert page.panel.transport.lines == []
    page.rod_to_fc_var.set("0.15")
    assert f"psi_deg={angle}" in rig_command_after_readback(page)


def test_manual_axis_edit_and_custom_keep_single_angle_source(page):
    page.psi_var.set("90")
    assert "Pitch" in page.axis_preset_var.get()
    page.psi_var.set("12.5")
    assert page.axis_preset_var.get() == "自定义"
    page.axis_preset_combo.current(3)
    page.axis_preset_combo.event_generate("<<ComboboxSelected>>")
    assert page.psi_var.get() == "12.5"
    page.psi_var.set("")
    assert page.axis_preset_var.get() == "自定义"
    assert page.panel.transport.lines == []


@pytest.mark.parametrize("scale", [1.0, 1.25, 1.5])
@pytest.mark.parametrize("size", [(1080, 700), (1366, 768), (1500, 900)])
def test_axis_selector_in_real_panel(size, scale):
    from tools.panel_qa import OfflinePanel
    with OfflinePanel.launch(size=size, scale=scale, connected=False) as session:
        panel = session.panel
        panel.notebook.select(panel.sysid_tab)
        panel.sysid_notebook.select(0)
        panel.update_idletasks()
        combo = panel.sysid_page.axis_preset_combo
        assert combo.winfo_width() > 100
        assert combo.winfo_rootx() >= panel.winfo_rootx()
        assert combo.winfo_rootx() + combo.winfo_width() <= panel.winfo_rootx() + panel.winfo_width()
        combo.current(0)
        combo.event_generate("<<ComboboxSelected>>")
        assert panel.sysid_page.psi_var.get() == "90"
        page = panel.sysid_page
        assert len(page.steps_notebook.tabs()) == 4
        for tab in page.steps_notebook.tabs():
            page.steps_notebook.select(tab)
            panel.update_idletasks()
            for button in (page.start_button, page.stop_button):
                assert button.winfo_ismapped()
                assert button.winfo_rooty() + button.winfo_height() <= panel.winfo_rooty() + panel.winfo_height()
        page.steps_notebook.select(0)
        assert not session.callback_errors


def test_schema_timeout_explains_that_acquisition_has_not_started(page):
    page.workflow.submit(["SYSID SCHEMA"], "start")
    page.workflow.timeout(page.workflow.ticket)
    assert "尚未开始辨识" in page.status_var.get()
    assert "准备页" in page.status_var.get()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]


def test_the_page_never_offers_to_write_flash():
    """本页没有写 Flash 的路径。

    按**字符串字面量**查而不是全文查：文件里到处在解释"为什么没有 SAVE"，
    全文查会被自己的说明绊倒。真正危险的是某处真的发出一条 `SAVE`。
    页面拆成了多个模块，每个都查。
    """
    import ast

    package = ROOT / "tools/panel_lib/pages/sysid"
    modules = sorted(path for path in package.glob("*.py") if path.name != "_core.py")
    assert {"inner_loop.py", "workflow.py", "live_status.py"} <= {m.name for m in modules}
    for module in modules:
        tree = ast.parse(module.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                text = node.value.strip()
                first = text.split()[0] if text.split() else ""
                assert first not in {"SAVE", "COMMIT"}, (module.name, text)
                assert "COMMIT" not in first, (module.name, text)


def ram_ok(name, value):
    """固件 `SYSID PARAM` 成功回显（App/Src/app_cmd_sysid.c，6 位小数）。"""
    return f"OK sysid param name={name} value={float(value):.6f} ram=1"


#: 几何力矩模型的舵机转轴（飞控下方 0.13 m）；配 START_PARAMS 的重心 0.045 → 力臂 0.095 m。
GEOMETRIC_AXES = (("airframe.servo1_axis_z_m", "-0.050000"), ("airframe.servo2_axis_z_m", "-0.050000"))
#: 「临时应用/恢复」时重读到的飞控机体参数（与上面同一套力臂）。
BOARD = (("airframe.cg_z_m", "0.045000"),) + GEOMETRIC_AXES
#: 候选参数录制时的力矩单位：与 BOARD 相同 → 不用换算。
SAME_UNITS = {"signature": "geometric", "levers_m": [0.095, 0.095]}


def answer_param_barrier(page, board=BOARD):
    """应用/恢复先重读飞控参数：`PARAM?` + 一份状态报告作顺序屏障，之后才发 SYSID PARAM。"""
    assert page.panel.transport.lines[-2:] == ["PARAM?", "SYSID?"]
    feed(page, [f"PARAM name={name} value={value}" for name, value in board])
    feed(page, status_report({}))


def test_applying_gains_writes_ram_only_and_says_so(page):
    """普通 PARAM SET 成功后固件 1.5 s 自动存 Flash；试用必须走只写 RAM 的 SYSID PARAM。"""
    page.apply_scale_var.set("100%")
    page.workflow.end = {"state": "done"}
    page.workflow.params = {"coax.rate_roll_kp": "0.1", "coax.rate_pitch_kp": "0.1"}
    page._commands = ["SYSID PARAM coax.rate_roll_kp 0.5", "SYSID PARAM coax.rate_pitch_kp 0.5"]
    page._commands_source = SAME_UNITS
    page.apply_to_ram()
    assert page.panel.transport.lines == ["PARAM?", "SYSID?"], "先重读飞控参数，确认力矩单位"
    answer_param_barrier(page)
    assert page.panel.transport.lines[2:] == page._commands[:1]
    assert "RAM" not in page.status_var.get()  # enqueue is not application
    page.handle_line("OK param name=coax.rate_roll_kp value=0.5")   # 普通回显不算数
    assert page.panel.transport.lines[2:] == page._commands[:1]
    page.handle_line(ram_ok("coax.rate_roll_kp", 0.5))
    assert page.panel.transport.lines[2:] == page._commands
    page.handle_line(ram_ok("coax.rate_pitch_kp", 0.5))
    assert "已按 100% 临时应用" in page.status_var.get() and "RAM" in page.status_var.get()
    assert "换成 RATE" in page.banner_detail_var.get()
    assert not any(c.startswith(("SAVE", "PARAM SET")) for c in page.panel.transport.lines)


def test_legacy_param_set_candidates_are_sent_as_ram_only(page):
    page.apply_scale_var.set("100%")
    page.workflow.end = {"state": "done"}
    page.workflow.params = {"coax.att_roll_kp": "4"}
    page._commands = ["PARAM SET coax.att_roll_kp 5"]
    page._commands_source = SAME_UNITS
    page.apply_to_ram()
    answer_param_barrier(page)
    assert page.panel.transport.lines[2:] == ["SYSID PARAM coax.att_roll_kp 5"]


def test_non_gain_candidates_are_refused(page):
    page.workflow.end = {"state": "done"}
    page.workflow.params = {"airframe.mass_kg": "1"}
    page._commands = ["SYSID PARAM airframe.mass_kg 2"]
    page.apply_to_ram()
    assert page.panel.transport.lines == []
    assert "只允许试用控制增益" in page.status_var.get()


@pytest.mark.parametrize("reply,message", [
    ("ERR unknown sysid subcmd PARAM", "飞控固件不支持只写 RAM 的参数试用，未应用，请更新固件"),
    ("ERR sysid param coax.rate_pitch_kp", "飞控固件不支持只写 RAM 的参数试用"),
    ("OK sysid param name=coax.rate_pitch_kp value=0.500000 ram=0", "没确认只写 RAM"),
    ("OK sysid param name=coax.rate_pitch_kp value=0.700000 ram=1", "回显不对"),
])
def test_ram_only_trial_fails_closed(page, reply, message):
    page.apply_scale_var.set("100%")
    page.workflow.end = {"state": "done"}
    page.workflow.params = {"coax.rate_roll_kp": "0.1", "coax.rate_pitch_kp": "0.1"}
    page._commands = ["SYSID PARAM coax.rate_roll_kp 0.5", "SYSID PARAM coax.rate_pitch_kp 0.5"]
    page._commands_source = SAME_UNITS
    page.apply_to_ram()
    answer_param_barrier(page)
    page.handle_line(ram_ok("coax.rate_roll_kp", 0.5))
    page.handle_line(reply)
    assert message in page.status_var.get()
    assert "前面 1 项已写入 RAM" in page.status_var.get()
    assert page.workflow.awaiting is None
    assert len(page.panel.transport.lines) == 4, "失败后不重试、不继续"


def test_nothing_is_sent_when_the_panel_is_not_connected(page):
    page.panel.transport.is_connected = False
    page.rod_to_fc_var.set("0.15")
    page.send_rig()
    assert page.panel.transport.lines == []
    assert "未连接" in page.status_var.get()


def test_the_panel_safety_gate_can_block_a_command(page):
    """验收会话和固件升级期间不该往飞控发东西。LEDMAP 页上真出过这个口子。"""
    page.panel._validation_command_allowed = lambda _text: False
    page.rod_to_fc_var.set("0.15")
    page.send_rig()
    assert page.panel.transport.lines == []
    assert "挡下" in page.status_var.get()


# ---------------------------------------------------------------- 收包


SCHEMA_LINES = [
    "SYSID SCHEMA ver=1 n=3 hash=DEADBEEF rec=6",
    "SYSID FIELD idx=0 name=gx unit=rad/s scale=0.001 type=i16",
    "SYSID FIELD idx=1 name=gy unit=rad/s scale=0.001 type=i16",
    "SYSID FIELD idx=2 name=omega_sp unit=rad/s scale=0.001 type=i16",
]


def make_frame(schema_hash: int, values: list[tuple[int, int, int]],
               *, flags: int = FLAG_FIRST_BATCH, base_us: int = 1000) -> bytes:
    header = struct.pack("<BBHIIHH", 1, len(values), 7, schema_hash,
                         base_us, 2000, flags)
    body = b"".join(struct.pack("<hhh", *row) for row in values)
    return header + body


def test_the_page_decodes_using_the_schema_the_firmware_reported(page):
    for line in SCHEMA_LINES:
        page.handle_line(line)
    assert page.schema is not None
    assert page.schema.record_bytes == 6

    page.accept(make_frame(0xDEADBEEF, [(700, 700, 1000), (710, 705, 1000)]))
    assert len(page.samples) == 2
    assert page.samples[0]["gx"] == pytest.approx(0.7, abs=1e-6)
    assert "样本 2" in page.sample_count_var.get()


def test_data_arriving_before_the_schema_is_refused_with_an_instruction(page):
    page.accept(make_frame(0xDEADBEEF, [(1, 2, 3)]))
    assert page.samples == []
    assert "字段表" in page.status_var.get()


def test_a_stale_schema_is_dropped_instead_of_decoding_plausible_garbage(page):
    """主机拿旧表硬解会得到一组看着正常的错值——那种错查不出来。"""
    for line in SCHEMA_LINES:
        page.handle_line(line)
    page.accept(make_frame(0x12345678, [(1, 2, 3)]))
    assert page.samples == []
    assert page.schema is None, "对不上之后必须重新取表，不能继续用旧的"
    assert "拒绝解码" in page.status_var.get()


def test_the_last_batch_closes_the_run_visibly(page):
    for line in SCHEMA_LINES:
        page.handle_line(line)
    page.accept(make_frame(0xDEADBEEF, [(1, 2, 3)], flags=FLAG_FIRST_BATCH))
    page.accept(make_frame(0xDEADBEEF, [(4, 5, 6)], flags=FLAG_LAST_BATCH,
                           base_us=3000))
    assert "本趟结束" in page.status_var.get()


def test_the_firmware_assumed_inertia_is_taken_from_the_status_reply(page):
    """拟合必须用飞控**实际**在用的那个假定惯量。

    界面输入框留空时固件会自己去取 `airframe.ixx_kgm2`，两边对不上会让
    「惯量比值」整体偏一个常数——而那个比值正是判断还要不要再跑一轮的依据。
    """
    page.handle_line("SYSID LIMITS rate_hz=250 I_ugm2=19000 angle_mrad=349 "
                     "resid_mrad_s=523 min_thrust_cn=200")
    assert page.firmware_inertia_kg_m2 == pytest.approx(0.019)


def test_fitting_without_the_assumed_inertia_refuses_instead_of_guessing(page):
    page.samples = [{"gx": 0.0, "gy": 0.0, "alpha_ff": 0.0, "angle": 0.0}] * 200
    page.run_fit()
    assert "正常结束" in page.fit_var.get()


# ---------------------------------------------------------------- 占位页


@pytest.mark.parametrize("module,marker", [
    ("tools/panel_lib/pages/sysid/horizontal.py", "动不了位置"),
    ("tools/panel_lib/pages/sysid/altitude.py", "总推力"),
])
def test_the_unimplemented_pages_explain_themselves(module, marker):
    source = read(module)
    assert "本页尚未实现" in read("tools/panel_lib/pages/sysid/common.py")
    assert marker in source
    # 不放假按钮：点了没反应会让人以为是飞控的问题。
    assert "ttk.Button" not in source
    assert "send" not in source


def test_the_altitude_page_says_why_it_cannot_share_the_horizontal_model():
    """作者的要求原话是「高度由于和水平的物理特性不一样需要单独出来辨识」。"""
    source = read("tools/panel_lib/pages/sysid/altitude.py")
    for reason in ("非线性", "悬停", "电压", "不对称"):
        assert reason in source, reason


# ---------------------------------------------------------------- 一键开始（程序油门，SYSID ver=3）
#
# 报文键名照 App/Src/app_sysid.c 的 APP_SysId_ReportStatus / SYSID start / SYSID PHASE /
# SYSID end 格式串写；THR 行排在 EXC 与 LIMITS 之间（LIMITS 仍是每份报告的最后一行）。

V2_FIELDS = ("gx", "gy", "omega_sp", "angle", "angle_sp", "torque")
V2_HASH = 0xCAFEF00D
SCHEMA_V2_LINES = [f"SYSID SCHEMA ver=2 n={len(V2_FIELDS)} hash={V2_HASH:08X} rec={2*len(V2_FIELDS)}"] + [
    f"SYSID FIELD idx={index} name={name} unit=x scale=0.001000000 type=i16"
    for index, name in enumerate(V2_FIELDS)]


def thr_line(*, auto=0, target_cn=0, max_pct_x10=750, phase="idle", armed=1, thr_low=1):
    """`SYSID THR?` 的回复，也是整份报告里的那一行。"""
    return (f"SYSID THR auto={auto} target_cn={target_cn} max_pct_x10={max_pct_x10} "
            f"phase={phase} pulse_us=0 armed={armed} thr_low={thr_low} capped=0")


def status_report(expected, *, ver=3, mass_mg=1000000, armed=1, thr_low=1, phase="idle",
                  thr=True, state="idle", run=0, airframe_imu_off_um=35000, **override):
    e = {"mode": 0, "angle_amp_mrad": 52, "imu_off_um": 0, "psi_mrad": 785,
         "axis_off_um": 0, "profile": 1, "amp_mrad_s": 150, "dur_ms": 4000,
         "hold_ms": 250, "repeat": 8, "ramp_ms": 150, "f0_mhz": 300, "f1_mhz": 6000,
         "bit_ms": 40, "seed": 1, "auto": 0, "target_cn": 0, "max_pct_x10": 750,
         "rate_hz": 250, "angle_mrad": 349, "resid_mrad_s": 523}
    e.update(expected)
    e.update(override)
    ready_imu = "" if airframe_imu_off_um is None else f" imu_off_um={airframe_imu_off_um}"
    lines = [
        f"SYSID READY ver={ver} mode={e['mode']} angle_amp_mrad={e['angle_amp_mrad']} "
        f"mass_mg={mass_mg}{ready_imu} thrust=lut engaged=0",
        f"SYSID state={state} run={run} reason=init queued=0 dropped=0",
        f"SYSID RIG psi_mrad={e['psi_mrad']} axis_off_um={e['axis_off_um']} imu_off_um={e['imu_off_um']}",
        f"SYSID EXC profile={e['profile']} amp_mrad_s={e['amp_mrad_s']} dur_ms={e['dur_ms']} "
        f"hold_ms={e['hold_ms']} repeat={e['repeat']} ramp_ms={e['ramp_ms']} f0_mhz={e['f0_mhz']} "
        f"f1_mhz={e['f1_mhz']} bit_ms={e['bit_ms']} seed={e['seed']} total_ms=4000",
    ]
    if thr:
        lines.append(thr_line(auto=e["auto"], target_cn=e["target_cn"],
                              max_pct_x10=e["max_pct_x10"], phase=phase, armed=armed,
                              thr_low=thr_low))
    lines.append(f"SYSID LIMITS rate_hz={e['rate_hz']} I_ugm2=20000 angle_mrad={e['angle_mrad']} "
                 f"resid_mrad_s={e['resid_mrad_s']} min_thrust_cn=200")
    return lines


def feed(page, lines):
    for line in lines:
        page.handle_line(line)


START_PARAMS = (("airframe.weight_n", "9.806650"),
                ("airframe.thrust_point_to_cg_z_m", "-0.030000"),
                ("airframe.imu_z_m", "0.080000"), ("airframe.cg_z_m", "0.045000"),
                ("coax.rate_roll_kp", "0.010000"), ("coax.rate_pitch_kp", "0.010000"),
                ("coax.rate_roll_ki", "0.020000"), ("coax.rate_pitch_ki", "0.020000"),
                ("coax.rate_roll_kd", "0.000000"), ("coax.rate_pitch_kd", "0.000000"),
                ("coax.att_roll_kp", "4.000000"), ("coax.att_pitch_kp", "4.000000"))
#: 带舵机转轴的机体参数：录制的轮次带着几何力矩单位（力臂 0.095 m），候选参数才能换算写入。
GEOMETRIC_START = START_PARAMS + GEOMETRIC_AXES


def drive_config(page, *, params=START_PARAMS, rod="0.15", pivots=("-0.2", "-0.2"),
                 schema=None, **report):
    """点开始 → 回字段表 → 回参数 → 每条配置命令回一整份状态报告。"""
    if rod is not None:
        page.rod_to_fc_var.set(rod)
    if pivots is not None:
        page.roll_pivot_var.set(pivots[0])
        page.pitch_pivot_var.set(pivots[1])
    page.start_run()
    assert page.panel.transport.lines[-1] == "SYSID SCHEMA"
    feed(page, schema or SCHEMA_V2_LINES)
    feed(page, [f"PARAM name={name} value={value}" for name, value in params])
    for _ in range(12):
        awaiting = page.workflow.awaiting
        if not awaiting or awaiting == "SYSID START":
            break
        feed(page, status_report(page.workflow.expected, **report))


def start_run(page, run=3, **config):
    drive_config(page, **config)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.handle_line(f"SYSID start run={run} profile=1 amp_mrad_s=150 dur_ms=4000 rate_hz=250 "
                     f"I=20000 ugm2 psi_mrad=785 auto=1 target_cn=980")
    assert page.workflow.awaiting is None


def v2_frame(run, rows, *, flags, base_us, mode_flag=0, dt_us=4000):
    header = struct.pack("<BBHIIHH", 2, len(rows), run, V2_HASH, base_us, dt_us,
                         flags | mode_flag)
    return header + b"".join(struct.pack("<6h", *row) for row in rows)


def feed_full_run(page, run=3, n=100, mode_flag=0):
    rows = [(int(300*math.sin(i/6)), int(300*math.sin(i/6)), int(400*math.sin(i/6)),
             0, 0, int(200*math.cos(i/6))) for i in range(n)]
    half = n // 2
    page.accept(v2_frame(run, rows[:half], flags=FLAG_FIRST_BATCH, base_us=1000,
                         mode_flag=mode_flag))
    page.accept(v2_frame(run, rows[half:], flags=FLAG_LAST_BATCH, base_us=1000 + half*4000,
                         mode_flag=mode_flag))


def test_one_click_start_configures_program_throttle_from_airframe_weight(page):
    drive_config(page)
    sent = page.panel.transport.lines
    assert sent[:2] == ["SYSID SCHEMA", "PARAM?"]
    assert sent[2].startswith("SYSID EXC profile=doublet ") and "repeat=16" in sent[2]
    assert sent[3:7] == ["SYSID RATE 250", "SYSID INERTIA 0",
                         "SYSID LIMIT angle_deg=20 resid_dps=30", "SYSID MODE FF 3"]
    # RIG 等机体参数回来才组装：d = 0.15（量的）+ 0.035（imu_z − cg_z）。
    assert sent[7:] == ["SYSID RIG psi_deg=45 axis_off_m=0.185 imu_off_m=0.035",
                        "SYSID THROTTLE target_n=9.8 max_pct=75", "SYSID START"]
    expected = page.workflow.expected
    assert (expected["auto"], expected["target_cn"], expected["max_pct_x10"]) == (1, 980, 750)
    assert "SYSID HOLD" not in sent
    assert page.banner_var.get() == "正在开始…"


def test_manual_throttle_sends_target_zero(page):
    page.manual_throttle_var.set(True)
    drive_config(page)
    assert "SYSID THROTTLE target_n=0 max_pct=75" in page.panel.transport.lines
    assert page.panel.transport.lines[-1] == "SYSID START"
    assert page.workflow.expected["auto"] == 0


def test_typed_target_thrust_wins_over_weight(page):
    page.target_thrust_var.set("12.5")
    drive_config(page)
    assert "SYSID THROTTLE target_n=12.5 max_pct=75" in page.panel.transport.lines


def test_default_excitation_is_sixteen_doublets_in_eight_seconds(page):
    spec = page.excitation()
    assert (spec.repeat, spec.total_ms()) == (16, 8000)


def test_weight_falls_back_to_reported_mass(page):
    drive_config(page, params=(), mass_mg=1200000)   # 1.2 kg -> 11.77 N -> 11.8
    assert "SYSID THROTTLE target_n=11.8 max_pct=75" in page.panel.transport.lines
    assert "SYSID RIG psi_deg=45 axis_off_m=0.185 imu_off_m=0.035" in page.panel.transport.lines
    assert "11.8" in page.thrust_hint_var.get()


@pytest.mark.parametrize("override,message", [
    ({"auto": 0}, "回读不符：auto"),
    ({"max_pct_x10": 600}, "回读不符：max_pct_x10"),
    ({"thr": False}, "固件太旧"),
])
def test_throttle_echo_is_verified_before_start(page, override, message):
    drive_config(page, **override)
    assert "SYSID START" not in page.panel.transport.lines
    assert message in page.status_var.get()
    assert page.banner_var.get() == "没能开始"


def test_bad_throttle_entry_is_refused_before_anything_is_sent(page):
    page.rod_to_fc_var.set("0.15")
    page.max_pct_var.set("150")
    page.start_run()
    assert page.panel.transport.lines == []
    assert page.banner_var.get() == "没能开始"
    assert "10%～95%" in page.banner_detail_var.get()


@pytest.mark.parametrize("line,hint", [
    ("ERR sysid not armed: arm with throttle stick low first", "请先用遥控器解锁"),
    ("ERR sysid throttle stick not low", "油门杆没在最低"),
])
def test_start_refused_by_firmware_is_translated(page, line, hint):
    drive_config(page)
    assert page.workflow.awaiting == "SYSID START"
    page.handle_line(line)
    assert page.workflow.awaiting is None
    assert page.banner_var.get() == "没能开始"
    assert hint in page.banner_detail_var.get()
    assert line in page.status_var.get()   # 原话留给开发排查


def test_throttle_config_rejection_is_translated(page):
    page.rod_to_fc_var.set("0.15")
    page.target_thrust_var.set("99")
    page.start_run()
    feed(page, SCHEMA_V2_LINES)
    while page.workflow.awaiting and not page.workflow.awaiting.startswith("SYSID THROTTLE"):
        feed(page, status_report(page.workflow.expected))
    page.handle_line("ERR sysid throttle: target_n=0 (manual) or 2..max_total_force_n, max_pct 10..95")
    assert "目标合推力" in page.banner_detail_var.get()
    assert "SYSID START" not in page.panel.transport.lines


@pytest.mark.parametrize("armed,low,title", [
    (0, 1, "请先解锁（油门杆最低）"),
    (1, 0, "请把油门杆拉到最低"),
    (1, 1, "就绪，可以开始"),
])
def test_banner_follows_rc_state_from_status_report(page, armed, low, title):
    feed(page, status_report({}, armed=armed, thr_low=low))
    assert page.banner_var.get() == title
    if not armed:
        assert "请先用遥控器解锁（油门杆拉到最低）" in page.banner_detail_var.get()
    assert ("已解锁" if armed else "未解锁") in page.rc_var.get()


def test_stale_rc_state_is_not_shown_as_ready(page):
    feed(page, status_report({}, armed=1, thr_low=1))
    assert page.banner_var.get() == "就绪，可以开始"
    page.thr_time -= 10.0                  # 十秒没刷新（例如页面被切走、飞控不回话）
    page.refresh_banner()
    assert page.banner_var.get() == "正在读取飞控状态…"
    assert "未知" in page.rc_var.get()


def test_banner_says_not_connected_and_old_firmware(page):
    feed(page, status_report({}, ver=2))
    assert page.banner_var.get() == "飞控固件需要更新"
    page.panel.transport.is_connected = False
    page.refresh_banner()
    assert page.banner_var.get() == "未连接"


def test_banner_walks_through_the_program_throttle_phases(page):
    start_run(page)
    assert page.banner_var.get() == "升油门…"
    page.handle_line("SYSID PHASE run=3 phase=settle pulse_us=1400 thrust_cn=980")
    assert page.banner_var.get() == "稳定中…"
    page.handle_line("SYSID PHASE run=3 phase=excite pulse_us=1400 thrust_cn=980")
    feed_full_run(page, n=40)
    assert page.banner_var.get() == "激励中（样本 40）"
    page.handle_line("SYSID PHASE run=3 phase=ramp_down pulse_us=1300 thrust_cn=700")
    assert page.banner_var.get() == "降油门…"


@pytest.mark.parametrize("token,what", [
    ("actuator_saturated", "舵机行程或推力不够产生所需力矩"),
    ("rc_throttle_override", "你推了油门杆，程序已把油门交还遥控器"),
    ("axis_residual", "机体没有只绕杆转"),
    ("test-reset", "test-reset"),
])
def test_aborted_run_explains_reason_and_refuses_analysis(page, token, what, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    feed_full_run(page, n=40)
    page.handle_line(f"SYSID end run=3 state=aborted reason={token} dropped=0")
    assert page.banner_var.get().startswith("中止：")
    assert what in page.banner_var.get()
    assert page.banner_detail_var.get().startswith("下一步：")
    assert page.steps_notebook.index("current") == page.steps_notebook.index(page.observe_tab)
    page.run_fit()
    assert "中止" in page.fit_var.get() and "不能分析" in page.fit_var.get()


@dataclass
class FakeFit:
    inertia_kg_m2: float = 0.018
    inertia_rod_kg_m2: float = 0.021
    pivot_above_cg_m: float = 0.012
    torque_scale: float = 1.0
    damping_n_m_s: float = 0.01
    delay_s: float = 0.03
    fit_percent: float = 88.0
    natural_hz: float = 1.3
    separable: bool = True
    samples: int = 100
    tuning_inertia_kg_m2: float = 0.018
    tuning_damping_n_m_s: float = 0.01
    inertia_uncertainty_pct: float = 30.0
    tuning_inertia_uncertainty_pct: float = 6.0
    fit_percent_15hz: float = 61.0
    torque_model_scale: float = 1.05
    tuning_blockers: tuple = ()
    fit_percent_runs: tuple = ()

    @property
    def warnings(self):
        return ["示例提醒"]


def wait_for(page, predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        page.workflow.pump()
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_done_ff_run_is_saved_then_fitted_then_gains_are_proposed(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis as workflow_module
    calls = []

    def fake_fit(times, samples, **kwargs):
        calls.append((len(times), len(samples), kwargs))
        return FakeFit()

    monkeypatch.setattr(workflow_module, "fit_inner_loop", fake_fit)
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert page.banner_var.get() == "完成"
    assert wait_for(page, lambda: page._commands is not None)

    (_n_times, n_samples, kwargs), = calls
    assert n_samples == 100
    assert kwargs["mass_kg"] == pytest.approx(1.0)
    assert kwargs["assumed_inertia_kg_m2"] == pytest.approx(0.02)
    assert kwargs["thrust_point_to_cg_z_m"] == pytest.approx(-0.03)
    assert kwargs["pivot_above_cg_m"] == pytest.approx(0.185)   # 量的 d，是输入
    card = page.fit_var.get() + page.plant_var.get() + page.fit_ref_var.get()
    for label in ("整定惯量", "±6%", "杆上惯量", "飞行惯量", "力矩模型标定系数 κ", "1.050",
                  "杆到质心距离（你量的输入）", "185.0 mm", "1 Hz 等效延迟：30.0 ms",
                  "摆动固有频率", "拟合度（4 Hz 以下，去掉开头 0.5 s）：88.0%",
                  "15 Hz 全频段 61.0%（含碳杆约 9 Hz 台架抖动，只作参考）", "示例提醒"):
        assert label in card, label
    assert "逐轮" not in card
    assert "kp=" in page.gain_var.get()
    assert "分析完成" in page.banner_detail_var.get()
    assert page.steps_notebook.index("current") == page.steps_notebook.index(page.results_tab)
    saved = {path.name for path in (tmp_path / "run").iterdir()}
    assert {"samples.csv", "conditions.json", "fit.json"} <= saved
    # 写 RAM 仍然要人点；整个流程没有发过 SAVE 或 PARAM SET。
    assert not any(line.startswith(("SAVE", "PARAM SET")) for line in page.panel.transport.lines)


JITTER_FIELDS = V2_FIELDS + ("offset_us",)
JITTER_HASH = 0xB0A7F00D
SCHEMA_JITTER_LINES = [
    f"SYSID SCHEMA ver=2 n={len(JITTER_FIELDS)} hash={JITTER_HASH:08X} rec={2*len(JITTER_FIELDS)}"] + [
    f"SYSID FIELD idx={index} name={name} unit=x scale="
    f"{'1.000000000 type=u16' if name == 'offset_us' else '0.001000000 type=i16'}"
    for index, name in enumerate(JITTER_FIELDS)]


def jittered_sample_times(ticks_us, n, period_us=4000):
    """250 Hz 采样跟着控制拍走：每个名义时刻取它之后的第一拍。"""
    times, t, k = [], 0, 0
    for i in range(n):
        while t < i*period_us:
            t += ticks_us[k % len(ticks_us)]
            k += 1
        times.append(t)
    return times


@pytest.mark.parametrize("ticks", [(2000, 3100), (3100, 3000)])
def test_jittered_control_ticks_without_loss_are_not_gaps(page, monkeypatch, tmp_path, ticks):
    from panel_lib.pages.sysid import analysis as workflow_module
    calls = []
    monkeypatch.setattr(workflow_module, "fit_inner_loop",
                        lambda *a, **k: calls.append(a) or FakeFit())
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page, schema=SCHEMA_JITTER_LINES)
    times = jittered_sample_times(ticks, 100)
    steps = [b - a for a, b in zip(times, times[1:])]
    assert max(steps) > 1.5*4000 or ticks == (2000, 3100)   # 第二组确实超过旧的 1.5×dt 门
    for start in range(0, 100, 10):
        chunk = times[start:start+10]
        flags = (FLAG_FIRST_BATCH if start == 0 else 0) | (FLAG_LAST_BATCH if start == 90 else 0)
        rows = [(0, 0, 0, 0, 0, 10, t - chunk[0]) for t in chunk]
        header = struct.pack("<BBHIIHH", 2, len(rows), 3, JITTER_HASH, 1000 + chunk[0], 4000, flags)
        page.accept(header + b"".join(struct.pack("<6hH", *row) for row in rows))
    assert page.gap_count == 0 and not page.workflow.data_error
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: calls and page.workflow.job is None)
    times_fed = calls[0][0]
    assert len(times_fed) == 100


def test_only_the_firmware_gap_flag_or_a_huge_step_counts_as_a_gap(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    rows = [(0, 0, 0, 0, 0, 10)]*10
    page.accept(v2_frame(3, rows, flags=FLAG_FIRST_BATCH, base_us=1000))
    page.accept(v2_frame(3, rows, flags=0x0010, base_us=1000 + 10*4000))        # GAP 标志
    assert page.gap_count == 1
    page.accept(v2_frame(3, rows, flags=FLAG_LAST_BATCH, base_us=1000 + 40*4000))  # 跨了 20 个样本
    assert page.gap_count == 2
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving)
    assert "缺口" in page.fit_var.get()


def test_a_failed_ram_write_does_not_poison_the_run(page, monkeypatch, tmp_path):
    """写 RAM 超时是通信问题，不是数据问题：之后仍能重新分析、重新写。"""
    from panel_lib.pages.sysid import analysis as workflow_module
    monkeypatch.setattr(workflow_module, "fit_inner_loop", lambda *a, **k: FakeFit())
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page, params=GEOMETRIC_START)
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page._commands is not None)
    page.apply_to_ram()
    answer_param_barrier(page)
    page.workflow.timeout(page.workflow.ticket)
    assert "超时" in page.status_var.get()
    page.run_fit()
    assert page.workflow.job is not None, "重新分析不该被一次通信超时挡住"
    assert wait_for(page, lambda: page.workflow.job is None)
    sent = len(page.panel.transport.lines)
    page.apply_to_ram()
    answer_param_barrier(page)
    assert len(page.panel.transport.lines) == sent + 3


@pytest.mark.parametrize("bad,why,advice", [
    ({"tuning_blockers": ("带内拟合优度 62% 低于 85%",)}, "带内拟合优度",
     "检查台架夹紧，或加长激励后重跑"),
    ({"tuning_blockers": ("整定惯量不确定度 31% 超过 15%",)}, "不确定度",
     "检查台架夹紧，或加长激励后重跑"),
    ({"tuning_blockers": ("力矩模型比例 κ = 2.10 不在 0.5–1.5：力矩模型或杆距测量可能有误，先核对",)},
     "κ = 2.10", "核对杆和倾转轴的尺量值"),
    ({"tuning_blockers": ("缺倾转轴高度（俯仰），κ 与整定惯量无法换算",)}, "缺倾转轴高度",
     "到「1 · 准备」填俯仰倾转轴到飞控板的距离，再点「重新分析」，不用重跑"),
    ({"delay_s": 1e-12}, "没辨出延迟", "检查台架夹紧"),     # 兜底：近零延迟会算出天文数字的增益
    ({"tuning_inertia_kg_m2": -0.001}, "整定用惯量不为正", "核对杆和倾转轴的尺量值"),
])
def test_blockers_stop_gains_and_say_what_to_do(page, monkeypatch, tmp_path, bad, why, advice):
    from panel_lib.pages.sysid import analysis as workflow_module
    monkeypatch.setattr(workflow_module, "fit_inner_loop",
                        lambda *a, **k: FakeFit(**bad))
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    assert page._commands is None
    assert "不给建议参数" in page.gain_var.get() and why in page.gain_var.get()
    assert advice in page.gain_var.get() and "下一步" in page.banner_detail_var.get()
    page.synthesise()          # 手动再点也不给
    assert page._commands is None


def test_low_band_fit_alone_is_not_a_page_side_gate(page, monkeypatch, tmp_path):
    """门槛以拟合给的 blocker 为准：页面不再自己写死拟合度。"""
    finished = finished_ff_run(page, monkeypatch, tmp_path, fake=lambda k: FakeFit(fit_percent=60.0))
    assert finished and page._commands


def test_rate_verification_run_reports_tracking_error(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.mode_var.set("RATE")
    start_run(page, mode=1)
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert "跟踪误差（15 Hz 以下" in page.fit_var.get()
    assert "振动噪声（15 Hz 以上" in page.fit_var.get()
    assert page.banner_var.get() == "完成"
    assert "验证轮完成" in page.banner_detail_var.get()


def test_polling_never_hijacks_the_echo_check(page):
    page.parent.winfo_viewable = lambda: True
    page.rod_to_fc_var.set("0.15")
    assert page.poll_status()
    assert not page.poll_status(), "上一次轮询的那一行没回来前不再发"
    page.request_status()                    # 「读回飞控状态」：一整份报告也在路上
    assert page.panel.transport.lines == ["SYSID THR?", "SYSID?"]
    assert "> SYSID THR?" not in page.panel.logged
    # 点开始的那一刻，这两份回复都还在路上。
    page.start_run()
    assert page.panel.transport.lines == ["SYSID THR?", "SYSID?", "SYSID SCHEMA"]
    page.handle_line(thr_line())
    feed(page, status_report({}, auto=0))
    assert page.workflow.awaiting == "SYSID SCHEMA", "轮询/读回的回复不能当成配置回显"
    feed(page, SCHEMA_V2_LINES)
    assert page.workflow.awaiting.startswith("SYSID EXC ")
    assert not page.poll_status(), "事务等回显期间不轮询"
    feed(page, [f"PARAM name={n} value={v}" for n, v in START_PARAMS])
    while page.workflow.awaiting and page.workflow.awaiting != "SYSID START":
        feed(page, status_report(page.workflow.expected))
    assert page.panel.transport.lines[-2:] == ["SYSID THROTTLE target_n=9.8 max_pct=75",
                                               "SYSID START"]
    assert page.panel.transport.lines.count("SYSID THR?") == 1


def test_poll_reply_refreshes_rc_state_with_one_line(page):
    page.parent.winfo_viewable = lambda: True
    assert page.poll_status()
    page.handle_line(thr_line(armed=0))
    assert page.banner_var.get() == "请先解锁（油门杆最低）"
    assert page.poll_status(), "回复到了就能发下一次"


def test_old_firmware_refusing_the_poll_does_not_break_a_start(page):
    page.parent.winfo_viewable = lambda: True
    page.rod_to_fc_var.set("0.15")
    assert page.poll_status()
    page.start_run()
    page.handle_line("ERR unknown sysid subcmd THR?")
    assert page.workflow.awaiting == "SYSID SCHEMA"
    page.stop_run()
    assert not page.poll_status(), "这次连接里不再发它不认识的轮询"


def test_polling_stays_quiet_when_hidden_or_disconnected(page):
    assert not page.poll_status()          # 夹具里的根窗口是隐藏的
    page.parent.winfo_viewable = lambda: True
    page.panel.transport.is_connected = False
    assert not page.poll_status()
    assert page.panel.transport.lines == []


def test_stop_label_follows_throttle_mode(page):
    assert page.stop_button.cget("text") == "停止（电机回到遥控器油门）"
    page.manual_throttle_var.set(True)
    assert page.stop_button.cget("text") == "停止辨识"
    assert "不会停电机" in page.stop_hint_var.get()


def test_stop_during_run_shows_stopping_then_reason(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    page.stop_run()
    assert page.panel.transport.lines[-1] == "SYSID STOP"
    assert page.banner_var.get() == "正在停止…"
    feed(page, status_report({}, state="aborted", run=3))   # STOP 回的状态报告
    page.handle_line("SYSID end run=3 state=aborted reason=command dropped=0")
    assert page.banner_var.get() == "中止：你点了停止"


def test_polling_in_the_real_panel_only_when_the_page_is_visible():
    from tools.panel_qa import OfflinePanel
    with OfflinePanel.launch(connected=True) as session:
        panel = session.panel
        page = panel.sysid_page
        panel.notebook.select(0)
        panel.update_idletasks()
        assert not page.poll_status()
        panel.notebook.select(panel.sysid_tab)
        panel.sysid_notebook.select(0)
        panel.update_idletasks()
        assert page.poll_status()
        assert session.transport.lines.count("SYSID THR?") == 1
        assert not session.callback_errors


# ---------------------------------------------------------------- 台架设置持久化


def fresh_page(page):
    """同一个根窗口里再开一页：等于下次打开面板。设置文件仍指向本测试的临时目录。"""
    from panel_lib.pages.sysid.inner_loop import SysIdInnerLoopPage
    from tkinter import ttk
    panel = FakePanel()
    return SysIdInnerLoopPage(panel, ttk.Frame(page.parent.winfo_toplevel()))


def settings_file():
    from panel_lib.pages.sysid import settings_store
    return settings_store.settings_path()


def test_rig_settings_are_remembered_when_a_run_starts(page):
    assert "首次使用" in page.status_var.get()
    page.axis_preset_combo.current(0)                        # 绕 Y · Pitch 90°
    page.axis_preset_combo.event_generate("<<ComboboxSelected>>")
    page.rod_to_fc_var.set("0.162")
    page.axis_override_var.set("0.2")
    page.target_thrust_var.set("11.5")
    page.max_pct_var.set("70")
    page.manual_throttle_var.set(True)
    page.mode_var.set("ANGLE")
    page.angle_amp_var.set("4")
    assert not settings_file().exists(), "按键不写盘"
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]
    data = json.loads(settings_file().read_text(encoding="utf-8"))
    assert data["version"] == 1
    assert not list(settings_file().parent.glob("*.tmp-*")), "临时文件必须被 os.replace 掉"

    again = fresh_page(page)
    assert (again.psi_var.get(), again.axis_preset_var.get()) == ("90", "绕 Y 轴 · Pitch（90°）")
    assert again.rod_to_fc_var.get() == "0.162"
    assert again.axis_override_var.get() == "0.2"
    assert again.target_thrust_var.get() == "11.5"
    assert again.max_pct_var.get() == "70"
    assert again.manual_throttle_var.get() is True
    assert again.stop_button.cget("text") == "停止辨识"
    assert (again.mode_var.get(), again.angle_amp_var.get()) == ("ANGLE", "4")
    assert again.status_var.get() == ""


def test_refused_start_does_not_overwrite_saved_settings(page):
    page.max_pct_var.set("150")
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert not settings_file().exists()


def test_corrupt_settings_file_falls_back_to_defaults_quietly(page):
    settings_file().write_text("{not json", encoding="utf-8")
    again = fresh_page(page)
    assert (again.psi_var.get(), again.rod_to_fc_var.get(), again.max_pct_var.get()) == ("45.0", "", "75")
    assert "损坏" in again.status_var.get()


def test_invalid_saved_values_are_not_loaded(page):
    settings_file().write_text(json.dumps({
        "version": 1, "psi_deg": "abc", "rod_to_fc_m": 0.12, "axis_override_m": float("nan"),
        "target_thrust_n": -3, "max_throttle_pct": 150, "manual_throttle": "yes",
        "mode": "XX", "angle_amp_deg": 40}), encoding="utf-8")
    again = fresh_page(page)
    assert again.rod_to_fc_var.get() == "0.12"                 # 唯一合法的一项照读
    assert again.psi_var.get() == "45.0" and again.axis_override_var.get() == ""
    assert again.target_thrust_var.get() == "" and again.max_pct_var.get() == "75"
    assert again.manual_throttle_var.get() is False and again.mode_var.get() == "FF"
    assert again.angle_amp_var.get() == "3"
    assert "7 项无效" in again.status_var.get()


def test_wrong_version_is_treated_as_corrupt(page):
    settings_file().write_text(json.dumps({"version": 2, "rod_to_fc_m": 0.3}), encoding="utf-8")
    again = fresh_page(page)
    assert again.rod_to_fc_var.get() == ""
    assert "版本" in again.status_var.get()


def test_a_failed_settings_write_does_not_block_the_start(page, monkeypatch):
    from panel_lib.pages.sysid import settings_store

    def broken(*_args, **_kwargs):
        raise OSError("磁盘只读")

    monkeypatch.setattr(settings_store, "save", broken)
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]
    assert any("台架设置没能保存" in line for line in page.panel.logged)


# ---------------------------------------------------------------- 第四轮：原参数存档、断线、链路、倾转轴


def pivot_aware_fit(kwargs):
    """模仿拟合核心：斜杆两个倾转轴高度都要，缺了就给 blocker。"""
    missing = [axis for axis, key in (("横滚", "roll_pivot_to_cg_z_m"), ("俯仰", "pitch_pivot_to_cg_z_m"))
               if kwargs.get(key) is None]
    blockers = (f"缺倾转轴高度（{'、'.join(missing)}），κ 与整定惯量无法换算",) if missing else ()
    return FakeFit(tuning_blockers=blockers)


def finished_ff_run(page, monkeypatch, tmp_path, fake=pivot_aware_fit, **config):
    from panel_lib.pages.sysid import analysis as workflow_module
    calls = []
    monkeypatch.setattr(workflow_module, "fit_inner_loop",
                        lambda *a, **k: calls.append(k) or fake(k))
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page, **config)
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    return calls


def apply_all(page, board=BOARD):
    page.apply_to_ram()
    answer_param_barrier(page, board)
    while page.workflow.awaiting and page.workflow.awaiting.startswith("SYSID PARAM "):
        _, _, name, value = page.workflow.awaiting.split()
        page.handle_line(ram_ok(name, value))


def reconnect(page):
    transport = page.panel.transport
    transport.connection_generation = getattr(transport, "connection_generation", 0) + 1


def test_original_gains_survive_a_reconnect_via_the_run_archive(page, monkeypatch, tmp_path):
    finished_ff_run(page, monkeypatch, tmp_path, params=GEOMETRIC_START)
    assert page._commands
    apply_all(page)
    record = json.loads((tmp_path / "run" / "original_gains.json").read_text(encoding="utf-8"))
    assert record["gains"]["coax.rate_roll_kp"] == "0.010000"
    assert record["torque_model"] == {"signature": "geometric",
                                      "levers_m": [pytest.approx(0.095), pytest.approx(0.095)]}
    reconnect(page)
    feed(page, status_report({}))                  # 新连接上的第一份报告
    assert page.workflow.original_gains is None
    before = len(page.panel.transport.lines)
    page.workflow.restore()
    answer_param_barrier(page)
    sent = page.panel.transport.lines[before + 2:]
    assert sent == [f"SYSID PARAM {page.workflow.awaiting.split()[2]} 0.010000"]
    while page.workflow.awaiting:
        _, _, name, value = page.workflow.awaiting.split()
        page.handle_line(ram_ok(name, value))
    assert "已恢复原参数" in page.status_var.get()
    assert "候选" not in page.status_var.get()
    assert all(line.startswith("SYSID PARAM ") for line in page.panel.transport.lines[before + 2:])


def test_a_new_panel_can_restore_from_the_latest_archive(page, monkeypatch, tmp_path):
    finished_ff_run(page, monkeypatch, tmp_path, params=GEOMETRIC_START)
    apply_all(page)
    again = fresh_page(page)
    again.workflow.restore()
    answer_param_barrier(again)
    assert again.panel.transport.lines[2].startswith("SYSID PARAM coax.")
    assert "存档 run" in again.workflow.restore_source


def test_restore_without_any_record_says_so(page):
    page.workflow.restore()
    assert page.panel.transport.lines == []
    assert "找不到应用前的原参数记录" in page.status_var.get()


def test_link_loss_closes_the_run_and_still_handles_the_line(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    feed_full_run(page, n=20)
    reconnect(page)
    page.handle_line("SYSID PHASE run=9 phase=idle pulse_us=0 thrust_cn=0")
    assert page.workflow.end["reason"] == "link_lost"
    assert page.banner_var.get() == "中止：连接中断或飞控复位，本轮作废"
    # 真正的结束报告晚到：不被吞掉，替换本地合成的那份。
    page.handle_line("SYSID end run=3 state=aborted reason=rc_disarm dropped=0")
    assert page.workflow.end["reason"] == "rc_disarm"
    assert "上锁" in page.banner_var.get()


def test_end_report_arriving_as_the_first_line_of_a_new_connection_is_not_swallowed(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    reconnect(page)
    page.handle_line("SYSID end run=3 state=aborted reason=rc_lost dropped=0")
    assert page.workflow.end["reason"] == "rc_lost"
    assert not page.workflow.end.get("synthetic")


def test_firmware_state_releases_a_page_stuck_in_running(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    page.handle_line("SYSID state=idle run=0 reason=init queued=0 dropped=0")   # 飞控复位过
    assert page.workflow.end["reason"] == "state_mismatch"
    assert page.banner_var.get().startswith("中止：飞控报告这一轮已经不在运行")


def test_finished_run_waits_briefly_for_the_real_end_report(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    w = page.workflow
    page.handle_line("SYSID state=aborted run=3 reason=command queued=5 dropped=0")
    assert w.end is None and w._state_grace is None, "还有数据在上传：等结束报告"
    page.handle_line("SYSID state=aborted run=3 reason=command queued=0 dropped=0")
    assert w.end is None and w._state_grace is not None
    w.close_if_still_open(w._state_grace, {"state": "aborted", "reason": "command", "dropped": "0"})
    assert w.end["reason"] == "command" and w.end["synthetic"] == "1"
    assert page.banner_var.get() == "中止：你点了停止"


class Var:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


@pytest.mark.parametrize("setup,usb", [
    ({"transport_var": Var("蓝牙")}, False),
    ({"transport_var": Var("tcp")}, False),
    ({"transport_var": Var("serial"), "identity": (0x10C4, 0xEA60)}, False),   # CP210 数传
    ({"transport_var": Var("serial"), "identity": (0x0483, 0x5740)}, True),
    ({}, True),                                                                   # 判断不了按串口
])
def test_non_usb_links_run_at_100_hz(page, setup, usb):
    panel = page.panel
    if "transport_var" in setup:
        panel.transport_var = setup["transport_var"]
    if "identity" in setup:
        panel.serial_transport = panel.transport
        panel.transport.active_port = "COM7"
        vid, pid = setup["identity"]
        panel._serial_port_identity = {"COM7": {"vid": vid, "pid": pid}}
    drive_config(page)
    rate = "250" if usb else "100"
    assert f"SYSID RATE {rate}" in panel.transport.lines
    assert page.workflow.expected["rate_hz"] == int(rate)
    assert page.rate_var.get() == "250", "只降本轮，不改高级设置里的值"
    page.handle_line("SYSID start run=3 profile=1 amp_mrad_s=150 dur_ms=4000 rate_hz=100 "
                     "I=20000 ugm2 psi_mrad=785 auto=1 target_cn=980")
    assert page.status_var.get().startswith("飞控已确认开始")
    assert ("自动降到 100 Hz" in page.status_var.get()) is (not usb)


@pytest.mark.parametrize("preset,roll,pitch", [(0, False, True), (1, True, True), (2, True, True)])
def test_only_the_needed_pivot_heights_are_asked(page, preset, roll, pitch):
    page.axis_preset_combo.current(preset)
    page.axis_preset_combo.event_generate("<<ComboboxSelected>>")
    shown = {axis: all(w.grid_info() for w in widgets) for axis, widgets in page.pivot_rows.items()}
    assert shown == {"roll": roll, "pitch": pitch}


def test_roll_rig_asks_only_for_the_roll_pivot(page):
    page.psi_var.set("0")
    assert all(w.grid_info() for w in page.pivot_rows["roll"])
    assert not any(w.grid_info() for w in page.pivot_rows["pitch"])


def test_pivot_heights_are_prefilled_from_airframe_params_but_stay_editable(page):
    page.roll_pivot_var.set("-0.3")
    feed(page, ["PARAM name=airframe.thrust_point_z_m value=0.020000",
                "PARAM name=airframe.pitch_axis_to_prop_plane_m value=-0.150000",
                "PARAM name=airframe.roll_axis_to_prop_plane_m value=-0.100000",
                "PARAM name=airframe.imu_z_m value=0.080000"])
    assert page.pitch_pivot_var.get() == "-0.210"       # 0.02 − 0.15 − 0.08
    assert page.roll_pivot_var.get() == "-0.3", "已经填了的不覆盖"


def test_pivot_heights_reach_the_fit_as_pivot_to_cg(page, monkeypatch, tmp_path):
    calls = finished_ff_run(page, monkeypatch, tmp_path, pivots=("-0.2", "-0.18"))
    kwargs = calls[0]
    assert kwargs["roll_pivot_to_cg_z_m"] == pytest.approx(-0.165)    # −0.2 + 0.035
    assert kwargs["pitch_pivot_to_cg_z_m"] == pytest.approx(-0.145)
    assert kwargs["pivot_above_cg_m"] == pytest.approx(0.185)


def test_missing_needed_pivot_still_runs_but_gives_no_gains(page, monkeypatch, tmp_path):
    calls = finished_ff_run(page, monkeypatch, tmp_path, pivots=("", "-0.2"))
    assert calls and calls[0]["roll_pivot_to_cg_z_m"] is None
    assert page._commands is None
    assert "缺倾转轴高度（横滚）" in page.gain_var.get()
    assert "填横滚倾转轴到飞控板的距离，再点「重新分析」，不用重跑" in page.gain_var.get()


def test_reanalysis_uses_the_geometry_filled_in_now(page, monkeypatch, tmp_path):
    """作者补量了倾转轴（顺手改了杆距）：已采的这一轮直接重算，不用重跑。"""
    calls = finished_ff_run(page, monkeypatch, tmp_path, pivots=("-0.2", ""))
    assert page._commands is None
    page.pitch_pivot_var.set("-0.18")
    page.rod_to_fc_var.set("0.16")
    page.run_fit()
    assert wait_for(page, lambda: page.workflow.job is None and len(calls) == 2)
    kwargs = calls[-1]
    assert kwargs["pitch_pivot_to_cg_z_m"] == pytest.approx(-0.145)   # −0.18 + 0.035（本轮快照）
    assert kwargs["roll_pivot_to_cg_z_m"] == pytest.approx(-0.165)
    assert kwargs["pivot_above_cg_m"] == pytest.approx(0.195)          # 0.16 + 0.035
    assert page.fit_var.get().startswith("用当前填写的几何重新计算")
    assert "d = 0.195 m" in page.fit_var.get()
    assert page._commands, "补齐之后给出参数"
    assert page.panel.transport.lines.count("SYSID START") == 1, "没有重跑"


# ---------------------------------------------------------------- 多轮联合分析

ARCHIVE_FIELDS = ("gx", "gy", "omega_sp", "angle", "angle_sp", "torque")


def write_archive(root, day, name, *, state="done", mode="0", axis_off="185000", gap_at=None, n=100,
                  profile="1", amp="150", echo=None, fit=None):
    """按页面存档的格式造一轮：conditions.json + samples.csv（t_us 是固件微秒）。"""
    folder = root / day / name
    folder.mkdir(parents=True)
    conditions = {
        "mode": mode, "psi_mrad": "785", "axis_off_um": axis_off, "imu_off_um": "35000",
        "mass_mg": "1000000", "I_ugm2": "20000", "profile": profile, "amp_mrad_s": amp,
        "hold_ms": "250", "ramp_ms": "150", "target_cn": "980",
        "start": {"run": "1", "target_cn": "980"},
        "parameter_echo": {"airframe.thrust_point_to_cg_z_m": "-0.030000",
                           **(dict(BOARD) if echo is None else echo)},
        "end": {"run": "1", "state": state, "reason": "complete" if state == "done" else "rc_disarm",
                "dropped": "0"}}
    (folder / "conditions.json").write_text(json.dumps(conditions), encoding="utf-8")
    lines = ["t_us,run_id,gap," + ",".join(ARCHIVE_FIELDS)]
    for i in range(n):
        gap = 1 if gap_at == i else 0
        values = (0.3*math.sin(i/6), 0.3*math.sin(i/6), 0.4*math.sin(i/6), 0.0, 0.0, 0.02*math.cos(i/6))
        lines.append(f"{4294960000 + i*4000 & 0xffffffff},1,{gap}," + ",".join(f"{v:.6f}" for v in values))
    (folder / "samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if fit is not None:
        (folder / "fit.json").write_text(json.dumps(fit), encoding="utf-8")
    return folder


def joint_archive(tmp_path):
    day = "2026-09-27"
    return {
        "a": write_archive(tmp_path, day, "rod_012332_aaaa1111"),
        "b": write_archive(tmp_path, day, "rod_012409_bbbb2222"),
        "other": write_archive(tmp_path, day, "rod_013000_cccc3333", axis_off="120000"),
        "aborted": write_archive(tmp_path, day, "rod_013500_dddd4444", state="aborted"),
        "rate": write_archive(tmp_path, day, "rod_014000_eeee5555", mode="1"),
    }


def test_joint_list_shows_finished_ff_runs_and_picks_same_conditions(page, tmp_path):
    folders = joint_archive(tmp_path)
    page.scan_joint_runs()
    listed = [run.folder for run, _ in page.joint_choices]
    assert folders["aborted"] not in listed and folders["rate"] not in listed
    assert listed[0] == folders["other"], "最新的在前"
    picked = {run.folder for run, var in page.joint_choices if var.get()}
    assert picked == {folders["other"]}, "默认勾与最近一轮同条件的"
    # 最近一轮换成 a/b 那组条件时，两轮都默认勾上。
    import shutil
    shutil.rmtree(folders["other"])
    page.scan_joint_runs()
    picked = {run.folder for run, var in page.joint_choices if var.get()}
    assert picked == {folders["a"], folders["b"]}


def test_joint_fit_uses_current_geometry_and_writes_fit_joint(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis
    import shutil
    folders = joint_archive(tmp_path)
    shutil.rmtree(folders["other"])
    calls = []

    def fake_multi(runs, **kwargs):
        calls.append((runs, kwargs))
        return FakeFit(fit_percent=87.0, fit_percent_runs=(87.0, 91.5))

    monkeypatch.setattr(analysis, "fit_inner_loop_multi", fake_multi)
    page.rod_to_fc_var.set("0.15")
    page.roll_pivot_var.set("-0.2")
    page.pitch_pivot_var.set("-0.18")
    page.scan_joint_runs()
    page.run_joint_fit()
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    (runs, kwargs), = calls
    assert len(runs) == 2
    times, samples = runs[0]
    assert times[0] == 0.0 and times[1] == pytest.approx(0.004), "跨 32 位回绕也连续"
    assert set(samples[0]) == set(ARCHIVE_FIELDS)
    assert kwargs["azimuth_rad"] == pytest.approx(0.785)
    assert kwargs["pivot_above_cg_m"] == pytest.approx(0.185)
    assert kwargs["pitch_pivot_to_cg_z_m"] == pytest.approx(-0.145)
    assert kwargs["roll_pivot_to_cg_z_m"] == pytest.approx(-0.165)
    assert kwargs["mass_kg"] == pytest.approx(1.0) and kwargs["assumed_inertia_kg_m2"] == pytest.approx(0.02)
    assert page.fit_var.get().startswith("联合 2 轮")
    assert "逐轮拟合度：87.0% / 91.5%" in page.fit_ref_var.get()
    record = json.loads((folders["b"] / "fit_joint.json").read_text(encoding="utf-8"))
    assert len(record["runs"]) == 2 and (folders["b"] / "report_joint.md").exists()
    # 联合结果同样按轴出参数，也能临时应用（原参数记在最近一轮目录里）。
    assert page._commands
    assert page._commands_source == SAME_UNITS, "力矩单位取联合里最新一轮的参数回显"
    assert json.loads((folders["b"] / "fit_joint.json").read_text(encoding="utf-8"))[
        "torque_model"] == SAME_UNITS
    page.workflow.params.update({c.split()[2]: "0.1" for c in page._commands})
    page.apply_to_ram()
    answer_param_barrier(page)
    assert page.panel.transport.lines[-1].startswith("SYSID PARAM ")
    assert (folders["b"] / "original_gains.json").exists()


def test_joint_fit_problems_are_reported_in_chinese(page, tmp_path):
    write_archive(tmp_path, "2026-09-27", "rod_012332_aaaa1111", gap_at=40)
    write_archive(tmp_path, "2026-09-27", "rod_012409_bbbb2222")
    page.rod_to_fc_var.set("0.15")
    page.scan_joint_runs()
    page.run_joint_fit()
    assert wait_for(page, lambda: page.workflow.job is None and "分析失败" in page.fit_var.get())
    assert "丢过样" in page.fit_var.get()


def test_a_single_archived_run_can_be_reanalysed(page, monkeypatch, tmp_path):
    """重开地面站后没有"本轮"：勾一轮就对存档里的那一轮单独重新分析（2026-09-27 作者要的），
    结果照样能临时应用。"""
    from panel_lib.pages.sysid import analysis
    import shutil
    folders = joint_archive(tmp_path)
    shutil.rmtree(folders["other"])
    shutil.rmtree(folders["a"])
    calls = []

    def fake_multi(runs, **kwargs):
        calls.append(runs)
        return FakeFit(fit_percent=87.0, fit_percent_runs=(87.0,))

    monkeypatch.setattr(analysis, "fit_inner_loop_multi", fake_multi)
    page.rod_to_fc_var.set("0.15")
    page.scan_joint_runs()
    assert len(page.selected_joint_runs()) == 1
    page.run_joint_fit()
    assert "至少勾" not in page.joint_hint_var.get()
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    (runs,) = calls
    assert len(runs) == 1
    assert page.fit_var.get().startswith("存档单轮重新分析：")
    assert page._commands, "单轮存档分析同样给出可临时应用的参数"


def test_joint_fit_needs_at_least_one_run(page, tmp_path):
    write_archive(tmp_path, "2026-09-27", "rod_012332_aaaa1111")
    page.scan_joint_runs()
    for _run, variable in page.joint_choices:
        variable.set(False)
    page.run_joint_fit()
    assert "至少勾一轮" in page.joint_hint_var.get()


def test_lengths_over_one_metre_are_refused(page):
    page.rod_to_fc_var.set("15")
    page.start_run()
    assert page.panel.transport.lines == []
    assert "单位是米，15 cm 填 0.15" in page.banner_detail_var.get()
    settings_file().write_text(json.dumps({"version": 1, "rod_to_fc_m": 1.5,
                                           "pitch_pivot_to_fc_m": -1.2}), encoding="utf-8")
    again = fresh_page(page)
    assert again.rod_to_fc_var.get() == "" and again.pitch_pivot_var.get() == ""


def test_pivot_heights_are_remembered(page):
    page.rod_to_fc_var.set("0.15")
    page.roll_pivot_var.set("-0.21")
    page.pitch_pivot_var.set("-0.19")
    page.start_run()
    again = fresh_page(page)
    assert (again.roll_pivot_var.get(), again.pitch_pivot_var.get()) == ("-0.21", "-0.19")


def test_firmware_version_is_forgotten_on_a_new_connection(page):
    feed(page, status_report({}, ver=2))
    assert page.banner_var.get() == "飞控固件需要更新"
    reconnect(page)
    page.refresh_banner()
    assert page.banner_var.get() == "正在读取飞控状态…"
    page.parent.winfo_viewable = lambda: True
    assert page.poll_status(), "新连接先试 THR? 轮询"


def test_poll_refused_by_old_firmware_shows_update_banner(page):
    page.parent.winfo_viewable = lambda: True
    assert page.poll_status()
    page.handle_line("ERR unknown sysid subcmd THR?")
    assert page.banner_var.get() == "飞控固件需要更新"


def test_angle_limit_advice_depends_on_the_mode():
    from panel_lib.pages.sysid.reasons import explain_end
    assert "ANGLE 角度幅值" in explain_end("angle_limit", "ANGLE")[1]
    assert "自然下垂" in explain_end("angle_limit", "2")[1]
    assert "激励幅值" in explain_end("angle_limit", "FF")[1]
    assert "激励幅值" in explain_end("angle_limit", "1")[1]


def test_capped_throttle_is_reported_in_the_rc_line(page, monkeypatch, tmp_path):
    start_run(page)
    page.handle_line(thr_line(auto=1, target_cn=980, phase="excite") .replace("capped=0", "capped=1"))
    assert "已按最高油门封顶，实际推力低于目标" in page.rc_var.get()
    page.handle_line(thr_line(auto=1, target_cn=980, phase="idle").replace("capped=0", "capped=1"))
    assert "封顶" not in page.rc_var.get()


def test_first_visible_idle_page_primes_status_and_params_quietly(page):
    page.parent.winfo_viewable = lambda: True
    assert page.prime_status()
    assert page.panel.transport.lines == ["SYSID?", "PARAM?"]
    assert page.panel.logged == []
    assert not page.prime_status(), "每个连接只要一次"
    feed(page, ["PARAM name=airframe.weight_n value=9.806650"] + AIRFRAME_GEOMETRY)
    assert "9.8" in page.thrust_hint_var.get() and "+0.035" in page.geometry_hint_var.get()
    # 预读的整份报告还在路上时点开始：不许被当成配置回显。
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    feed(page, status_report({}))
    assert page.workflow.awaiting == "SYSID SCHEMA"
    reconnect(page)
    page.handle_line("SYSID READY ver=3 mode=0")
    page.stop_run()
    assert page.prime_status(), "新连接再要一次"


# ---------------------------------------------------------------- 第六轮：尾巴摇狗卡片、按比例试用、扫频


@dataclass
class TwdFit(FakeFit):
    dead_time_s: float = 0.041
    servo_wn_rad_s: float = 31.4
    servo_zeta: float = 0.62
    reaction_couple_s2: float = 0.0021
    rig_zero_hz: float = 3.4
    flight_zero_hz: float = 3.1
    equivalent_delay_1hz_s: float = 0.066
    gain_margin_db: float = 6.4
    phase_margin_deg: float = 52.0
    crossover_hz: float = 1.35
    rigid_tuning_inertia_kg_m2: float = 0.021
    rigid_delay_s: float = 0.067
    rigid_fit_percent: float = 92.2
    fit_band_hz: float = 12.0
    delay_s: float = 0.066


def test_result_card_has_three_plain_blocks(page, monkeypatch, tmp_path):
    finished_ff_run(page, monkeypatch, tmp_path, fake=lambda k: TwdFit())
    pid, plant, diag = page.fit_var.get(), page.plant_var.get(), page.fit_ref_var.get()
    assert "【给 PID 用的】" in pid
    for text in ("整定惯量：0.018 kg·m²（不确定度 ±6%）", "1 Hz 等效延迟：66.0 ms",
                 "增益裕度 6.4 dB / 相位裕度 52° / 穿越 1.35 Hz"):
        assert text in pid, text
    assert "kp=" in page.gain_var.get(), "建议参数紧跟在【给 PID 用的】下面"
    assert plant.startswith("【对象特性】")
    for text in ("舵机响应：5.0 Hz，阻尼比 0.62", "纯延迟：41.0 ms", "ρ：0.0021 s²",
                 "飞行时也存在", "台架 3.4 Hz / 飞行 3.1 Hz", "摆动固有频率", "κ：1.050",
                 "杆上惯量（绕杆）", "飞行惯量（绕质心）"):
        assert text in plant, text
    assert diag.startswith("【诊断】")
    for text in ("拟合度（12 Hz 以下，去掉开头 0.5 s）：88.0%", "15 Hz 全频段 61.0%",
                 "刚体模型读数：整定惯量 0.021 kg·m²、延迟 67.0 ms、拟合度 92.2%，仅供对比，不用于整定"):
        assert text in diag, text


def test_missing_model_fields_show_a_dash():
    from types import SimpleNamespace
    from panel_lib.pages.sysid.results import card_blocks
    blocks = card_blocks(SimpleNamespace(), pivot_m=None)
    assert "整定惯量：—" in blocks["pid"] and "增益裕度 — / 相位裕度 — / 穿越 —" in blocks["pid"]
    assert "舵机响应：—，阻尼比 —" in blocks["plant"] and "纯延迟：—" in blocks["plant"]
    assert "台架 — / 飞行 —" in blocks["plant"]
    assert "拟合度（4 Hz 以下，去掉开头 0.5 s）：—" in blocks["diag"]
    assert "刚体模型" not in blocks["diag"]


@pytest.mark.parametrize("scale,expected", [
    ("50%", ["0.25", "0.1", "0", "2"]),
    ("75%", ["0.375", "0.15", "0", "3"]),
    ("100%", ["0.5", "0.2", "0", "4"]),
])
def test_trial_scale_shrinks_kp_and_ki_only(page, scale, expected):
    assert page.apply_scale_var.get() == "50%", "每次默认 50%"
    page.apply_scale_var.set(scale)
    page.workflow.end = {"state": "done"}
    names = ["coax.rate_pitch_kp", "coax.rate_pitch_ki", "coax.rate_pitch_kd", "coax.att_pitch_kp"]
    page.workflow.params = {name: "0.1" for name in names}
    page._commands = [f"SYSID PARAM {n} {v}" for n, v in zip(names, ("0.5", "0.2", "0", "4"))]
    page._commands_source = SAME_UNITS
    apply_all(page)
    sent = [line.split()[3] for line in page.panel.transport.lines if line.startswith("SYSID PARAM ")]
    assert sent == expected
    assert f"已按 {scale} 临时应用" in page.status_var.get()
    assert f"已按 {scale} 临时应用" in page.banner_detail_var.get()


def test_chirp_experiment_sends_its_preset_and_is_remembered(page):
    page.experiment_combo.current(1)
    page.experiment_combo.event_generate("<<ComboboxSelected>>")
    assert page.experiment_key() == "chirp"
    drive_config(page)
    exc = next(line for line in page.panel.transport.lines if line.startswith("SYSID EXC "))
    assert exc == ("SYSID EXC profile=chirp amp=0.022 dur_ms=12000 hold_ms=250 repeat=16 "
                   "ramp_ms=150 f0=0.5 f1=6 bit_ms=40 seed=1")
    again = fresh_page(page)
    assert again.experiment_key() == "chirp" and again.profile_var.get() == "chirp"
    again.experiment_combo.current(0)
    again.experiment_combo.event_generate("<<ComboboxSelected>>")
    assert (again.profile_var.get(), again.amp_var.get(), again.dur_var.get()) == ("doublet", "0.065", "8000")


def test_default_doublet_experiment_is_eight_seconds(page):
    assert page.experiment_key() == "doublet"
    assert page.excitation().total_ms() == 8000


def test_joint_default_picks_doublet_and_chirp_of_the_same_setup(page, tmp_path):
    day = "2026-09-27"
    doublet = write_archive(tmp_path, day, "rod_012332_aaaa1111")
    chirp = write_archive(tmp_path, day, "rod_012500_ffff6666", profile="2", amp="50")
    other_day = write_archive(tmp_path, "2026-09-26", "rod_235900_gggg7777")
    page.scan_joint_runs()
    picked = {run.folder for run, var in page.joint_choices if var.get()}
    assert picked == {doublet, chirp}
    assert other_day in [run.folder for run, _ in page.joint_choices]


# ---------------------------------------------------------------- 第七轮：几何力矩模型、舵机转轴、跟踪误差分解

SERVO_AXES = ["PARAM name=airframe.imu_z_m value=0.080000",
              "PARAM name=airframe.servo1_axis_z_m value=-0.050000",     # 横滚舵机转轴
              "PARAM name=airframe.servo2_axis_z_m value=-0.050000"]     # 俯仰舵机转轴：飞控下方 0.13 m


def test_pivots_prefill_from_the_servo_axes_the_firmware_uses(page):
    feed(page, SERVO_AXES + ["PARAM name=airframe.cg_z_m value=0.045000"])
    assert (page.roll_pivot_var.get(), page.pitch_pivot_var.get()) == ("-0.130", "-0.130")
    assert "来自机体参数的舵机转轴高度（与固件力矩模型同源）" in page.geometry_hint_var.get()
    assert page.pivot_warning_var.get() == ""


def test_zero_servo_axis_falls_back_to_the_prop_plane_route(page):
    feed(page, ["PARAM name=airframe.servo2_axis_z_m value=0.000000",
                "PARAM name=airframe.thrust_point_z_m value=0.020000",
                "PARAM name=airframe.pitch_axis_to_prop_plane_m value=-0.150000",
                "PARAM name=airframe.imu_z_m value=0.080000"])
    assert page.pitch_pivot_var.get() == "-0.210"
    assert "舵机转轴高度" not in page.geometry_hint_var.get()


@pytest.mark.parametrize("typed,warned", [("-0.14", True), ("-0.133", False)])
def test_typed_pivot_far_from_the_airframe_servo_axis_is_flagged(page, typed, warned):
    page.pitch_pivot_var.set(typed)
    feed(page, SERVO_AXES)
    warning = page.pivot_warning_var.get()
    assert ("与机体参数里的舵机转轴高度 -0.130 m 不一致：固件用机体参数算力矩，"
            "建议两边改成同一个值" in warning) is warned


def test_default_amplitudes_follow_the_torque_model_fix(page):
    assert page.amp_var.get() == "0.065"
    from panel_lib.pages.sysid.sections import EXPERIMENTS
    amps = {code: preset["amp"] for code, preset in EXPERIMENTS.values()}
    assert amps == {"doublet": "0.065", "chirp": "0.022"}


def signature_by_marker(echo):
    if "airframe.roll_thrust_lever_arm_m" in (echo or {}):
        return "legacy"
    return "geometric" if "airframe.servo1_axis_z_m" in (echo or {}) else "unknown"


GEOMETRIC = {"airframe.servo1_axis_z_m": "-0.050000", "airframe.servo2_axis_z_m": "-0.050000",
             "airframe.cg_z_m": "0.045000"}
LEGACY = {"airframe.roll_thrust_lever_arm_m": "0.100000", "airframe.pitch_thrust_lever_arm_m": "0.100000"}


def test_joint_never_ticks_runs_from_another_torque_model(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import joint
    monkeypatch.setattr(joint, "torque_model_of", signature_by_marker)
    day = "2026-09-27"
    old = write_archive(tmp_path, day, "rod_012332_aaaa1111", echo=LEGACY)
    new_a = write_archive(tmp_path, day, "rod_090000_bbbb2222", echo=GEOMETRIC)
    new_b = write_archive(tmp_path, day, "rod_091000_cccc3333", echo=GEOMETRIC, profile="2", amp="22")
    page.scan_joint_runs()
    picked = {run.folder for run, var in page.joint_choices if var.get()}
    assert picked == {new_a, new_b}
    labels = {run.folder: run.label for run, _ in page.joint_choices}
    assert labels[old].endswith("旧力矩模型") and labels[new_a].endswith("几何力矩模型")
    for run, var in page.joint_choices:
        var.set(True)
    page.run_joint_fit()
    assert "力矩模型不同" in page.joint_hint_var.get()
    assert page.workflow.job is None


def test_real_signature_helper_is_used_when_present(tmp_path):
    from panel_lib.pages.sysid import _core, joint
    if not hasattr(_core, "torque_model_signature"):
        pytest.skip("拟合核心还没有 torque_model_signature")
    assert joint.torque_model_of({**LEGACY}) == "legacy"
    assert joint.torque_model_of({}) == "unknown"


def test_firmware_levers_reach_single_and_joint_fits(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import _core, analysis
    monkeypatch.setattr(_core, "firmware_tilt_levers", lambda echo: (0.175, 0.176), raising=False)
    calls = finished_ff_run(page, monkeypatch, tmp_path)
    assert calls[0]["firmware_tilt_levers_m"] == (0.175, 0.176)
    seen = []
    monkeypatch.setattr(_core, "firmware_tilt_levers",
                        lambda echo: seen.append(dict(echo)) or (0.2, 0.3), raising=False)
    multi_calls = []
    monkeypatch.setattr(analysis, "fit_inner_loop_multi",
                        lambda runs, **k: multi_calls.append(k) or FakeFit(fit_percent_runs=(90.0, 91.0)))
    day = "2026-09-27"
    first = write_archive(tmp_path, day, "rod_092000_dddd4444", echo={"airframe.marker": "first"})
    write_archive(tmp_path, day, "rod_091500_eeee5555", echo={"airframe.marker": "second"})
    page.scan_joint_runs()
    for run, var in page.joint_choices:
        var.set(run.folder.name.startswith(("rod_092000", "rod_091500")))
    page.run_joint_fit()
    assert wait_for(page, lambda: page.workflow.job is None and multi_calls)
    assert multi_calls[0]["firmware_tilt_levers_m"] == (0.2, 0.3)
    assert seen[-1]["airframe.marker"] == "first", "联合时取第一（最新）轮的参数回显"


def test_fit_without_the_lever_helper_does_not_pass_the_keyword(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import _core
    monkeypatch.delattr(_core, "firmware_tilt_levers", raising=False)
    calls = finished_ff_run(page, monkeypatch, tmp_path)
    assert "firmware_tilt_levers_m" not in calls[0]


def test_torque_model_finding_is_explained_in_plant_block(page, monkeypatch, tmp_path):
    finding = ("固件力矩模型高估俯仰操纵力矩约 2.3 倍（几何预测 κ = 0.430，实测 0.440，二者一致）；"
               "整定已按 κ 折算，飞行中其它用到力矩模型的地方（前馈、限幅）同样偏差。")

    @dataclass
    class Finding(FakeFit):
        notes: tuple = (finding,)

        @property
        def warnings(self):
            return list(self.notes)

    finished_ff_run(page, monkeypatch, tmp_path, fake=lambda k: Finding())
    assert "固件算的倾转力矩和实际不符：固件力矩模型高估俯仰操纵力矩约 2.3 倍" in page.plant_var.get()
    assert "高估" not in page.fit_ref_var.get(), "已写进对象特性的不在诊断里重复"


REAL_RATE_RUN = ROOT / "data/identification/attitude/2026-09-27/rod_031745_e087013d"


@pytest.mark.skipif(not (REAL_RATE_RUN / "samples.csv").is_file(), reason="实录不在本机")
def test_real_rate_run_separates_vibration_from_tracking_error():
    """实录：RATE 验证、扫频 0.5–6 Hz、50 mrad/s、旧增益。旧指标 0.095 rad/s（指令幅值的 273%）。

    0.095 是逐点原始均方根（按时间加权也是 0.094，所以不是采样不均造成的）；
    均匀重采样后只有 0.071，差别在约 107 Hz 的桨/电机振动：线性插值把接近奈奎斯特的
    振动削掉了一大半（原始点上 ≈0.080，插值后 ≈0.049）。15 Hz 以下的部分两者一样 ≈0.050。
    """
    import numpy as np
    from panel_lib.pages.sysid.joint import read_samples
    from panel_lib.pages.sysid.results import tracking_summary
    times, samples = read_samples(REAL_RATE_RUN)
    psi = 1.570
    raw = [x["gx"]*math.cos(psi) + x["gy"]*math.sin(psi) - x["omega_sp"] for x in samples]
    raw_rms = float(np.sqrt(np.mean(np.square(raw))))
    assert raw_rms == pytest.approx(0.095, abs=0.001)            # 旧页面上的数

    text, note = tracking_summary(times, samples, psi, 1, f1_hz=6.0, crossover_hz=1.35)
    import re
    tracking = float(re.search(r"跟踪误差（15 Hz 以下，均方根，沿杆轴）：([0-9.]+)", text).group(1))
    command = float(re.search(r"指令均方根 ([0-9.]+) rad/s", text).group(1))
    vibration = float(re.search(r"振动噪声（15 Hz 以上，均方根）：([0-9.]+)", text).group(1))
    peak = float(re.search(r"主频约 ([0-9.]+) Hz", text).group(1))
    assert tracking == pytest.approx(0.050, abs=0.004)
    assert command == pytest.approx(0.035, abs=0.002)
    assert "不是相对幅值" in text
    assert vibration == pytest.approx(0.080, abs=0.005)
    assert 100 <= peak <= 115
    assert math.hypot(tracking, vibration) == pytest.approx(raw_rms, abs=0.004)
    assert "本轮激励最高 6 Hz，远超预测的速率环穿越频率 1.35 Hz" in note


def test_rate_page_adds_the_crossover_note_from_the_same_group(page, monkeypatch, tmp_path):
    day = "2026-09-27"
    write_archive(tmp_path, day, "rod_010000_ffff6666", axis_off="185000", fit={"crossover_hz": 1.4})
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / day / "rod_020000_rate")
    page.mode_var.set("RATE")
    page.experiment_combo.current(1)
    page.experiment_combo.event_generate("<<ComboboxSelected>>")
    # 同一组 = 同一套力矩单位（力臂），所以验证轮也带着同一份舵机转轴参数。
    start_run(page, mode=1, profile=2, f1_mhz=6000, params=GEOMETRIC_START)
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert "远超预测的速率环穿越频率 1.40 Hz" in page.fit_ref_var.get()


# ---------------------------------------------------------------- 力矩单位：录制固件 → 当前飞控

#: 2026-09-27 的实录都是旧力矩模型（力臂 0.145 × 有效系数）。
LEGACY_UNITS = {"signature": "legacy", "levers_m": [0.145 * 0.581, 0.145 * 0.569]}
#: 今晚板上的几何力矩模型：部件表重心 −0.094558、舵机转轴 −0.13 → 力臂 0.035442 m。
TONIGHT_BOARD = (("airframe.cg_z_m", "-0.094558"), ("airframe.servo1_axis_z_m", "-0.130000"),
                 ("airframe.servo2_axis_z_m", "-0.130000"))
TONIGHT_FACTOR = 0.035442 / (0.145 * 0.569)


def prepared_candidates(page, source=LEGACY_UNITS):
    page.apply_scale_var.set("100%")
    page.workflow.end = {"state": "done"}
    page.workflow.params = {"coax.rate_pitch_kp": "0.113800", "coax.rate_pitch_ki": "0.000000",
                            "coax.att_pitch_kp": "0.579965"}
    page._commands = ["SYSID PARAM coax.rate_pitch_kp 0.3", "SYSID PARAM coax.rate_pitch_ki 0.1",
                      "SYSID PARAM coax.att_pitch_kp 0.42"]
    page._commands_source = source


def sent_params(page) -> dict:
    return {line.split()[2]: float(line.split()[3]) for line in page.panel.transport.lines
            if line.startswith("SYSID PARAM ")}


def test_legacy_candidates_are_converted_to_the_connected_boards_lever(page, tmp_path):
    """旧模型录的数据合成的增益写进今晚的几何固件：kp/ki ×0.43，不换算就硬 2.3 倍。"""
    prepared_candidates(page)
    page.workflow.candidate_dir = tmp_path
    apply_all(page, board=TONIGHT_BOARD)
    sent = sent_params(page)
    assert sent["coax.rate_pitch_kp"] == pytest.approx(0.3 * TONIGHT_FACTOR, rel=1e-4)
    assert sent["coax.rate_pitch_ki"] == pytest.approx(0.1 * TONIGHT_FACTOR, rel=1e-4)
    assert sent["coax.att_pitch_kp"] == pytest.approx(0.42), "姿态 P 输出角速度，不换算"
    status = page.status_var.get()
    assert "已按 100% 临时应用" in status and "按力臂换算" in status
    assert f"×{TONIGHT_FACTOR:.3f}" in status and "0.0825 → 0.0354 m" in status
    record = json.loads((tmp_path / "original_gains.json").read_text(encoding="utf-8"))
    assert record["version"] == 2 and record["gains"]["coax.rate_pitch_kp"] == "0.113800"
    assert record["torque_model"] == {"signature": "geometric",
                                      "levers_m": [pytest.approx(0.035442)] * 2}


def test_stale_legacy_keys_do_not_survive_the_reread(page):
    """换过固件没重连：上次读到的旧力臂键不能冒充当前单位，应用前先扔掉再重读。"""
    prepared_candidates(page)
    page.workflow.params.update({"airframe.roll_thrust_lever_arm_m": "0.145000",
                                 "airframe.pitch_thrust_lever_arm_m": "0.145000",
                                 "airframe.thrust_point_to_cg_z_m": "-0.200942"})
    page.apply_to_ram()
    answer_param_barrier(page, board=TONIGHT_BOARD)
    assert sent_params(page)["coax.rate_pitch_kp"] == pytest.approx(0.3 * TONIGHT_FACTOR, rel=1e-4)


#: 旧固件（力臂 × 有效系数）的 PARAM 回显：新固件不再回显这几项，PARAM? 只会覆盖不会删除。
OLD_FIRMWARE_ECHO = {"airframe.roll_thrust_lever_arm_m": "0.145000",
                     "airframe.pitch_thrust_lever_arm_m": "0.145000",
                     "airframe.thrust_point_to_cg_z_m": "-0.200942"}


def test_a_reflash_without_reconnect_does_not_label_the_run_legacy(page):
    """ST-Link 重烧、数传链路没断（连接代次不变）：上一版固件回显的旧力臂键不许混进本轮的
    参数快照——否则几何固件录的轮次被当成旧模型（候选增益按 0.0842 换算、k 核对用错力臂、
    联合分析跟旧轮次分到一组，还永久写进 conditions.json）。"""
    from panel_lib.pages.sysid import _core

    page.workflow.params.update(OLD_FIRMWARE_ECHO)
    start_run(page, params=GEOMETRIC_START)
    echo = page.workflow.snapshot["parameter_echo"]
    assert not set(_core._LEGACY_ARM_KEYS) & set(echo)
    assert echo["airframe.thrust_point_to_cg_z_m"] == "-0.030000", "仍回显的键取新值"
    assert _core.torque_model_signature(echo) == "geometric"
    assert _core.firmware_tilt_levers(echo) == pytest.approx((0.095, 0.095))


def test_the_rig_readback_also_drops_stale_airframe_keys(page):
    page.workflow.params.update(OLD_FIRMWARE_ECHO)
    page.rod_to_fc_var.set("0.15")
    command = rig_command_after_readback(page)
    assert command == "SYSID RIG psi_deg=45 axis_off_m=0.185 imu_off_m=0.035"
    assert not set(OLD_FIRMWARE_ECHO) & set(page.workflow.params), "这次回显里没有的机体参数都不留"


@pytest.mark.parametrize("source, board, message", [
    (None, TONIGHT_BOARD, "录制时的固件倾转力臂读不出来"),
    (LEGACY_UNITS, (), "读不到当前飞控的倾转力臂"),
    (LEGACY_UNITS, (("airframe.cg_z_m", "-0.200000"),) + TONIGHT_BOARD[1:], "符号相反"),
])
def test_apply_refuses_when_the_units_cannot_be_confirmed(page, source, board, message):
    prepared_candidates(page, source)
    page.apply_to_ram()
    answer_param_barrier(page, board=board)
    assert message in page.status_var.get()
    assert sent_params(page) == {}
    assert page.workflow.awaiting is None and page.workflow.original_gains is None


def test_restore_converts_the_original_gains_to_the_current_lever(page, tmp_path):
    path = tmp_path / "original_gains.json"
    path.write_text(json.dumps({"version": 2, "gains": {"coax.rate_pitch_kp": "0.113800",
                                                        "coax.att_pitch_kp": "0.579965"},
                                "torque_model": LEGACY_UNITS}), encoding="utf-8")
    page.workflow.original_gains_path = path
    page.workflow.restore()
    answer_param_barrier(page, board=TONIGHT_BOARD)
    while page.workflow.awaiting and page.workflow.awaiting.startswith("SYSID PARAM "):
        _, _, name, value = page.workflow.awaiting.split()
        page.handle_line(ram_ok(name, value))
    sent = sent_params(page)
    assert sent["coax.rate_pitch_kp"] == pytest.approx(0.1138 * TONIGHT_FACTOR, rel=1e-4)
    assert sent["coax.att_pitch_kp"] == pytest.approx(0.579965)
    assert "已恢复原参数" in page.status_var.get() and "按力臂换算" in page.status_var.get()


def test_an_old_record_without_units_is_not_restored_blindly(page, tmp_path):
    """旧版上位机写的 original_gains.json（version 1）没记力臂：不知道单位就不自动恢复。"""
    path = tmp_path / "original_gains.json"
    path.write_text(json.dumps({"version": 1, "gains": {"coax.rate_pitch_kp": "0.113800"}}),
                    encoding="utf-8")
    page.workflow.original_gains_path = path
    page.workflow.restore()
    answer_param_barrier(page, board=TONIGHT_BOARD)
    assert "没记下当时飞控的倾转力臂" in page.status_var.get()
    assert sent_params(page) == {}


def test_joint_never_mixes_runs_recorded_with_different_levers(page, tmp_path):
    day = "2026-09-27"
    old = write_archive(tmp_path, day, "rod_012332_aaaa1111")          # 力臂 0.095 m
    new = write_archive(tmp_path, day, "rod_012409_bbbb2222",           # 重心改了：力臂 0.04 m
                        echo={"airframe.cg_z_m": "-0.010000", **dict(GEOMETRIC_AXES)})
    page.scan_joint_runs()
    assert {run.folder for run, var in page.joint_choices if var.get()} == {new}
    for _run, var in page.joint_choices:
        var.set(True)
    page.run_joint_fit()
    assert "倾转力臂不同" in page.joint_hint_var.get() and page.workflow.job is None
    assert old.exists()


def test_joint_never_mixes_backlash_compensated_and_plain_runs(page, tmp_path):
    """2026-09-28：补偿前后舵机 3° 的 1 Hz 等效延迟 100 → 66 ms，是两个对象。"""
    day = "2026-09-28"
    plain = write_archive(tmp_path, day, "rod_003000_aaaa1111")
    compensated = write_archive(tmp_path, day, "rod_003656_bbbb2222")
    path = compensated / "conditions.json"
    conditions = json.loads(path.read_text(encoding="utf-8"))
    conditions["backlash"] = {"run": "3", "en": "1", "alpha_mrad": "20", "beta_mrad": "20",
                              "thr_mrad": "5"}
    path.write_text(json.dumps(conditions), encoding="utf-8")
    page.scan_joint_runs()
    assert {run.folder for run, var in page.joint_choices if var.get()} == {compensated}
    assert any("回差补偿开" in run.label for run, _var in page.joint_choices)
    for _run, var in page.joint_choices:
        var.set(True)
    page.run_joint_fit()
    assert "回差补偿" in page.joint_hint_var.get() and page.workflow.job is None
    assert plain.exists()

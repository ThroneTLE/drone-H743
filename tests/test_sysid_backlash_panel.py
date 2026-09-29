"""「系统辨识 · 高级设置」里的「舵机回差补偿」与本轮补偿溯源（假链路，照 test_sysid_notch_panel 的夹具）。

钉的是：
* 开/关只在"已上锁、没有辨识在跑"时能点；灰掉时写明原因（飞控也会拒：reason=armed|sysid）。
* 飞控的回复原样翻成中文显示：确认、拒绝、旧固件不认命令。开关的确认只认 en 对得上的那一块。
* 开跑后固件回的 `SYSID BACKLASH` 行进快照、进 conditions.json，并写在结果抬头（陷波那行之后）；
  run 对不上的行不收；舵机单独轮不管有没有溯源都写明补没补。
* 舵机单独轮的幅值依赖只和补偿开关相同的轮次放一起。
* 空闲时自动读一次状态（新连接、每轮结束后），辨识进行中不打扰。

报文格式照固件源码（App/Src/app_cmd_backlash.c）；test_servo_backlash_policy.py 在宿主上跑同一段
格式串，两边对得上。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

from test_sysid_page import (  # noqa: E402,F401  页面夹具与报文工具
    FakeFit, feed, feed_full_run, page, reconnect, start_run, thr_line, wait_for,
)

ROOT = Path(__file__).resolve().parents[1]

BACKLASH_ON = "SYSID BACKLASH run=3 en=1 alpha_mrad=20 beta_mrad=20 thr_mrad=5"
NOTCH_ON = ("SYSID NOTCH run=3 en=1 state=idle src=bidir pp=7 harm=1 q_x100=300 min_hz=50 "
            "fade_hz=20 fs_x10=10000")


def status_block(*, en=1, active=1, src="controller", rev=(4, 7)):
    """固件 `BACKLASH ?` 的两行（格式照 App/Src/app_cmd_backlash.c）。"""
    return [
        f"BACKLASH cfg en={en} alpha_mrad=20 beta_mrad=25 thr_mrad=5 active={active} src={src} "
        f"dir_alpha=1 dir_beta=-1 off_alpha_us=-13 off_beta_us=16",
        f"BACKLASH count rev_alpha={rev[0]} rev_beta={rev[1]} reset=2 nonfinite=0 ticks=1234 clamped=1",
    ]


def disarmed(page, armed=0):
    page.handle_line(thr_line(armed=armed))


def enabled(button) -> bool:
    return not button.instate(["disabled"])


def test_the_firmware_formats_match_these_fixtures():
    source = (ROOT / "App/Src/app_cmd_backlash.c").read_text(encoding="utf-8")
    assert '"BACKLASH cfg en=%u alpha_mrad=%u beta_mrad=%u thr_mrad=%u active=%u src=%s "' in source
    assert '"dir_alpha=%d dir_beta=%d off_alpha_us=%d off_beta_us=%d\\r\\n"' in source
    assert ('"BACKLASH count rev_alpha=%lu rev_beta=%lu reset=%lu nonfinite=%lu ticks=%lu '
            'clamped=%lu\\r\\n"') in source
    assert '"SYSID BACKLASH run=%u en=%u alpha_mrad=%u beta_mrad=%u thr_mrad=%u servo_hz=%lu\\r\\n"' in source


# ---------------------------------------------------------------- 状态与开关


def test_reading_the_status_sends_the_query_and_renders_it_in_chinese(page):
    page.backlash_refresh()
    assert page.panel.transport.lines == ["BACKLASH ?"]
    feed(page, status_block())
    head, detail = page.backlash_status_var.get(), page.backlash_detail_var.get()
    assert head == "舵机回差补偿：开 · 正在补偿（指令来自飞行控制器）"
    for text in ("横滚舵机（alpha）半宽 20 mrad（1.15°）", "俯仰舵机（beta）半宽 25 mrad（1.43°）",
                 "换向迟滞 5 mrad（0.29°）", "当前方向 横滚 +1 / 俯仰 -1", "脉宽多走 横滚 -13 µs / 俯仰 +16 µs",
                 "换向 横滚 4 · 俯仰 7", "重新定向 2", "补偿过的控制拍 1234", "顶到标定端点 1"):
        assert text in detail, text
    feed(page, status_block(active=0, src="none"))
    assert page.backlash_status_var.get().startswith("舵机回差补偿：开 · 待命")
    feed(page, status_block(en=0, active=0, src="none"))
    assert page.backlash_status_var.get() == "舵机回差补偿：关"


def test_on_and_off_work_while_disarmed_and_idle(page):
    disarmed(page)
    assert enabled(page.backlash_on_button) and enabled(page.backlash_off_button)
    assert page.backlash_block_var.get() == ""
    page.backlash_on_button.invoke()
    assert page.panel.transport.lines[-1] == "BACKLASH ON"
    feed(page, status_block(en=1, active=0, src="none"))
    assert page.backlash_reply_var.get() == "飞控已确认：回差补偿已打开。"
    page.backlash_off_button.invoke()
    assert page.panel.transport.lines[-1] == "BACKLASH OFF"
    feed(page, status_block(en=0, active=0, src="none"))
    assert page.backlash_reply_var.get() == "飞控已确认：回差补偿已关闭。"


@pytest.fixture
def clock(monkeypatch):
    from panel_lib.pages.sysid import backlash_panel
    now = [1000.0]
    monkeypatch.setattr(backlash_panel, "_now", lambda: now[0])
    return now


def test_a_query_reply_that_arrives_first_is_not_taken_as_the_answer(page, clock):
    disarmed(page)
    page.backlash_refresh()
    page.backlash_on_button.invoke()
    feed(page, status_block(en=0, active=0, src="none"))
    assert page.backlash_reply_var.get() == "已发送 BACKLASH ON，等待飞控回复…"
    feed(page, status_block(en=1, active=0, src="none"))
    assert page.backlash_reply_var.get() == "飞控已确认：回差补偿已打开。"


def test_an_unanswered_switch_expires_and_a_mismatch_is_reported_after_the_wait(page, clock):
    disarmed(page)
    page.backlash_off_button.invoke()
    clock[0] += 3.1
    page.backlash_poll_idle()
    reply = page.backlash_reply_var.get()
    assert "没等到飞控对 BACKLASH OFF 的回复" in reply
    feed(page, status_block(en=1))
    assert page.backlash_reply_var.get() == reply
    page.backlash_on_button.invoke()
    feed(page, status_block(en=0, active=0, src="none"))
    clock[0] += 3.5
    feed(page, status_block(en=0, active=0, src="none"))
    assert page.backlash_reply_var.get().startswith("飞控回复与请求不符：BACKLASH ON 之后读到的是 en=0")


def test_switching_is_blocked_while_armed(page):
    disarmed(page, armed=1)
    assert not enabled(page.backlash_on_button) and not enabled(page.backlash_off_button)
    assert "已解锁" in page.backlash_block_var.get()
    sent = list(page.panel.transport.lines)
    assert not page.backlash_switch(True)
    assert page.panel.transport.lines == sent
    assert "已解锁" in page.backlash_reply_var.get()
    disarmed(page)
    assert enabled(page.backlash_on_button)


def test_switching_is_blocked_while_a_run_is_active(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    disarmed(page)                           # 舵机单独轮就是上锁跑的：辨识在跑也不许改
    assert not enabled(page.backlash_on_button)
    assert "辨识正在进行" in page.backlash_block_var.get()
    assert not page.backlash_switch(False)
    assert "BACKLASH OFF" not in page.panel.transport.lines


@pytest.mark.parametrize("reason,start", [("sysid", "飞控拒绝：辨识正占用台架"), ("armed", "飞控拒绝：已解锁"),
                                          ("range", "飞控拒绝：参数超出范围")])
def test_the_firmware_rejection_is_shown_with_its_reason(page, reason, start):
    disarmed(page)
    page.backlash_switch(True)
    page.handle_line(f"BACKLASH event=rejected reason={reason}")
    reply = page.backlash_reply_var.get()
    assert reply.startswith(start) and f"reason={reason}" in reply


def test_old_firmware_without_the_command_is_explained_and_does_not_break_anything(page):
    disarmed(page)
    page.backlash_refresh()
    page.handle_line("ERR unknown cmd BACKLASH")
    assert "不认 BACKLASH" in page.backlash_status_var.get()
    assert not enabled(page.backlash_on_button)
    assert page.workflow.error == ""
    assert "不认 RPMNOTCH" not in page.notch_status_var.get(), "两个开关各管各的"
    reconnect(page)
    page.refresh_backlash_controls()
    assert "不认 BACKLASH" not in page.backlash_block_var.get()


def test_a_late_reply_never_hijacks_the_start_transaction(page):
    page.backlash_refresh()
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert page.workflow.awaiting == "SYSID SCHEMA"
    page.handle_line("ERR unknown cmd BACKLASH")
    feed(page, status_block())
    assert page.workflow.awaiting == "SYSID SCHEMA" and page.workflow.error == ""


def test_the_idle_page_reads_the_status_once_per_connection_and_after_each_run(page, monkeypatch, tmp_path):
    page.parent.winfo_viewable = lambda: True
    assert page.backlash_poll_idle()
    assert page.panel.transport.lines == ["BACKLASH ?"] and page.panel.logged == []
    assert not page.backlash_poll_idle(), "每个连接只要一次"
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.panel.transport.lines.clear()
    start_run(page)
    page.handle_line("SYSID end run=3 state=aborted reason=command dropped=0")
    assert page.backlash_poll_idle(), "一轮结束后再读一次，看这轮的换向计数"
    assert page.panel.transport.lines[-1] == "BACKLASH ?"
    reconnect(page)
    page.workflow.run_id = 9
    page.workflow.end = None
    assert not page.backlash_poll_idle(), "辨识进行中不打扰"


def test_the_panel_sits_in_the_advanced_tab_next_to_the_notch(page):
    advanced = page.backlash_on_button.master.master.master
    assert advanced is page.notch_on_button.master.master.master
    titles = [child.cget("text") for child in advanced.winfo_children()
              if child.winfo_class() == "TLabelframe"]
    assert titles.index("陷波（桨叶振动）") + 1 == titles.index("舵机回差补偿")


# ---------------------------------------------------------------- 本轮溯源


def test_the_start_provenance_lands_in_the_snapshot_the_archive_and_the_header(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis as analysis_module
    monkeypatch.setattr(analysis_module, "fit_inner_loop", lambda *a, **k: FakeFit())
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    page.handle_line(NOTCH_ON)
    page.handle_line(BACKLASH_ON.replace("run=3", "run=4"))     # 别的轮次：不收
    assert "backlash" not in page.workflow.snapshot
    page.handle_line(BACKLASH_ON)
    assert page.workflow.snapshot["backlash"]["en"] == "1"
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    conditions = json.loads((tmp_path / "run" / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["backlash"] == {"run": "3", "en": "1", "alpha_mrad": "20", "beta_mrad": "20",
                                      "thr_mrad": "5"}
    lines = page.fit_var.get().splitlines()
    assert lines[0].startswith("本轮陷波（桨叶振动）：开")
    assert lines[1] == "本轮舵机回差补偿：开（横滚 20 / 俯仰 20 mrad，换向迟滞 5 mrad）"


def test_a_rate_run_shows_the_compensation_state_above_the_tracking_error(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.mode_var.set("RATE")
    start_run(page, mode=1)
    page.handle_line(BACKLASH_ON.replace("en=1", "en=0"))
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    lines = page.fit_var.get().splitlines()
    assert lines[0] == "本轮舵机回差补偿：关"
    assert "跟踪误差（15 Hz 以下" in page.fit_var.get()


def test_rate_runs_without_provenance_show_no_compensation_line(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.mode_var.set("RATE")
    start_run(page, mode=1)
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert "回差补偿" not in page.fit_var.get()


@pytest.mark.parametrize("line,expected", [
    ("SYSID BACKLASH run=3 en=1 alpha_mrad=20 beta_mrad=20 thr_mrad=5", "量的是补偿之后的舵机"),
    ("SYSID BACKLASH run=3 en=0 alpha_mrad=20 beta_mrad=20 thr_mrad=5", "含空程"),
    (None, "固件没报"),
])
def test_servo_only_results_always_say_whether_compensation_was_on(page, monkeypatch, tmp_path,
                                                                   line, expected):
    pytest.importorskip("scipy")
    from test_sysid_servo_only import FLAG_SERVO, servo_fit, servo_rows, start_servo_run, v3_frame
    from test_sysid_page import FLAG_FIRST_BATCH, FLAG_LAST_BATCH
    from panel_lib.pages.sysid import analysis
    fake = servo_fit.ServoFit(
        reaction_inertia_kg_m2=-0.004, reaction_ratio=-0.1, pod_gravity_n_m_rad=0.0,
        inertia_rod_kg_m2=0.045, stiffness_n_m_rad=0.37, damping_n_m_s=0.01, natural_hz=0.46,
        dead_time_s=0.043, servo_wn_rad_s=43.0, servo_zeta=0.66, equivalent_delay_1hz_s=0.07,
        fit_percent=90.0, amplitude_rad=0.087, reaction_uncertainty_pct=5.0, samples=100)
    monkeypatch.setattr(analysis, "fit_servo_only", lambda *a, **k: fake)
    folder = tmp_path / "2026-09-27" / "rod_070000_servo"
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: folder)
    start_servo_run(page)
    if line is not None:
        page.handle_line(line)
    rows = servo_rows(100)
    page.accept(v3_frame(3, rows[:50], flags=FLAG_FIRST_BATCH | FLAG_SERVO, base_us=1000))
    page.accept(v3_frame(3, rows[50:], flags=FLAG_LAST_BATCH | FLAG_SERVO, base_us=201000))
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and "反作用惯量" in page.fit_var.get())
    header = [text for text in page.fit_var.get().splitlines() if text.startswith("本轮舵机回差补偿")]
    assert len(header) == 1 and expected in header[0]
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    assert ("backlash" in conditions) == (line is not None)


def test_provenance_text_is_plain_and_tolerates_missing_fields():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.backlash_panel import backlash_provenance_text
    finally:
        sys.path.pop(0)
    assert backlash_provenance_text({}) == ""
    assert backlash_provenance_text({"mode": "1", "backlash": {}}) == ""
    on = {"en": "1", "alpha_mrad": "20", "beta_mrad": "18", "thr_mrad": "6"}
    assert backlash_provenance_text({"mode": "0", "backlash": on}) == \
        "本轮舵机回差补偿：开（横滚 20 / 俯仰 18 mrad，换向迟滞 6 mrad）"
    # 舵机单独轮：补没补决定了量的是哪一个舵机，三种情况都写明。
    assert "量的是补偿之后的舵机" in backlash_provenance_text({"mode": "3", "backlash": on})
    assert "含空程" in backlash_provenance_text({"mode": "3", "backlash": {"en": "0"}})
    assert "按未补偿处理" in backlash_provenance_text({"mode": "3"})
    assert "横滚 ? / 俯仰 ?" in backlash_provenance_text({"backlash": {"en": "1"}})
    for text in (backlash_provenance_text({"mode": m, "backlash": b}) for m in ("0", "3")
                 for b in (on, {"en": "0"}, None)):
        assert "陷波" not in text, "陷波那一组测试靠这个词判断有没有陷波溯源"


# ---------------------------------------------------------------- 舵机单独轮：幅值依赖不混


def write_servo_run(root, name, *, amplitude, reaction, backlash=None):
    folder = root / "2026-09-27" / name
    folder.mkdir(parents=True)
    conditions = {"mode": "3", "psi_mrad": "1570", "axis_off_um": "50000", "mass_mg": "754600",
                  "end": {"state": "done", "reason": "complete"}}
    if backlash is not None:
        conditions["backlash"] = {"run": "1", "en": "1" if backlash else "0", "alpha_mrad": "20",
                                  "beta_mrad": "20", "thr_mrad": "5"}
    (folder / "conditions.json").write_text(json.dumps(conditions), encoding="utf-8")
    (folder / "fit_servo.json").write_text(json.dumps({"fit": {
        "amplitude_rad": amplitude, "reaction_inertia_kg_m2": reaction, "dead_time_s": 0.03,
        "equivalent_delay_1hz_s": 0.05, "stiffness_n_m_rad": 0.37}}), encoding="utf-8")
    return folder, conditions


def test_the_amplitude_group_only_mixes_runs_with_the_same_compensation(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import servo_results
    finally:
        sys.path.pop(0)
    write_servo_run(tmp_path, "rod_010000_off_legacy", amplitude=0.052, reaction=-0.0024)
    write_servo_run(tmp_path, "rod_011000_off", amplitude=0.087, reaction=-0.0032, backlash=False)
    write_servo_run(tmp_path, "rod_012000_on", amplitude=0.052, reaction=-0.0039, backlash=True)
    write_servo_run(tmp_path, "rod_013000_on", amplitude=0.175, reaction=-0.0040, backlash=True)
    key = ("2026-09-27", "1570", "50000")
    amplitudes = lambda flag: sorted(p[0] for p in servo_results.servo_points(  # noqa: E731
        key, tmp_path, None, 0.37, backlash=flag))
    assert amplitudes(False) == [0.052, 0.087], "没有溯源的旧轮次算未补偿"
    assert amplitudes(True) == [0.052, 0.175]
    assert len(servo_results.servo_points(key, tmp_path, None, 0.37)) == 4, "不给开关时照旧全收"
    assert servo_results.backlash_on({"backlash": {"en": "1"}}) is True
    assert servo_results.backlash_on({}) is False
    assert math.isclose(sum(p[1] for p in servo_results.servo_points(
        key, tmp_path, None, 0.37, backlash=True)), -0.0079)

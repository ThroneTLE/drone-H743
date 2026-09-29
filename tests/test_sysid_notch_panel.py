"""「系统辨识 · 高级设置」里的「陷波（桨叶振动）」与本轮陷波溯源（假链路，照 test_sysid_page 的夹具）。

钉的是：
* 开/关只在"已上锁、没有辨识在跑"时能点；灰掉时写明原因（飞控也会拒：reason=armed|sysid）。
* 飞控的回复原样翻成中文显示：确认、拒绝、旧固件不认命令。
* 开关的确认只认 en 对得上的那一块（之前那次查询的回复会先到）；3 秒等不到就作废。
* 开跑后固件回的 `SYSID NOTCH` 行进快照、进 conditions.json，并写在结果抬头；
  run 对不上的行不收。
* 空闲时自动读一次状态（新连接、每轮结束后），辨识进行中不打扰。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from test_sysid_page import (  # noqa: E402,F401  页面夹具与报文工具
    FakeFit, feed, feed_full_run, page, reconnect, start_run, thr_line, wait_for,
)

ROOT = Path(__file__).resolve().parents[1]

NOTCH_ON = ("SYSID NOTCH run=3 en=1 state=idle src=bidir pp=7 harm=1 q_x100=300 min_hz=50 "
            "fade_hz=20 fs_x10=10000")


def status_block(*, en=1, state="tracking", src="bidir", w=100, spin=5000, tracked=4990, harm=1):
    """固件 `RPMNOTCH ?` 的四行（格式照 App/Src/app_cmd_rpmnotch.c）。"""
    return [
        f"RPMNOTCH cfg en={en} state={state} src={src} pp=7 harm={harm} q_x100=300 min_hz=50 fade_hz=20 "
        f"fs_nom=1000 fs_x10=10003 active=2",
        f"RPMNOTCH esc ch1_erpm=45150 ch1_hz_x10=1075 ch1_age_ms=2 ch1_w_x100={w} ch2_erpm=46000 "
        f"ch2_hz_x10=1095 ch2_age_ms=9999 ch2_w_x100={w}",
        "RPMNOTCH count stale=1 gap1=3 reset=1 nonfinite=0 slewclamp=0 reject=0 wdog=0",
        f"RPMNOTCH time samples=6000 spin={spin} tracked={tracked} apply_us_avg_x100=370 "
        "apply_us_max=8 tick_us_avg_x100=420 tick_us_max=15",
    ]


def disarmed(page, armed=0):
    page.handle_line(thr_line(armed=armed))


def enabled(button) -> bool:
    return not button.instate(["disabled"])


# ---------------------------------------------------------------- 状态与开关


def test_reading_the_status_sends_the_query_and_renders_it_in_chinese(page):
    page.notch_refresh()
    assert page.panel.transport.lines == ["RPMNOTCH ?"]
    feed(page, status_block())
    head, detail = page.notch_status_var.get(), page.notch_detail_var.get()
    assert "正在跟踪" in head
    for text in ("极对数 7", "谐波 1x", "Q 3.00", "50–70 Hz 淡入", "陀螺 1000.3 Hz（名义 1000）",
                 "电调1：107.5 Hz · 权重 100% · 回包 2 ms 前", "电调2：109.5 Hz", "从未收到回包",
                 "丢一样 3", "转动时满权重跟踪 99.8%", "平均 3.70 µs / 最大 8 µs"):
        assert text in detail, text


def test_without_1x_the_weight_is_labelled_with_the_harmonic_it_belongs_to(page):
    """HARM 2/4/6：固件报的是掩码里最低谐波的权重，页面写明是几倍频的。"""
    feed(page, status_block(harm=2))
    detail = page.notch_detail_var.get()
    assert "谐波 2x" in detail and "电调1：107.5 Hz · 2x 权重 100%" in detail


def test_on_and_off_work_while_disarmed_and_idle(page):
    disarmed(page)
    assert enabled(page.notch_on_button) and enabled(page.notch_off_button)
    assert page.notch_block_var.get() == ""
    page.notch_on_button.invoke()
    assert page.panel.transport.lines[-1] == "RPMNOTCH ON"
    feed(page, status_block(en=1, state="idle", w=0))
    assert page.notch_reply_var.get() == "飞控已确认：陷波已打开（状态 待命）。"
    page.notch_off_button.invoke()
    assert page.panel.transport.lines[-1] == "RPMNOTCH OFF"
    feed(page, status_block(en=0, state="off", w=0))
    assert page.notch_reply_var.get() == "飞控已确认：陷波已关闭。"
    assert page.notch_status_var.get().endswith("关")


@pytest.fixture
def clock(monkeypatch):
    from panel_lib.pages.sysid import notch_panel
    now = [1000.0]
    monkeypatch.setattr(notch_panel, "_now", lambda: now[0])
    return now


def test_a_query_reply_that_arrives_first_is_not_taken_as_the_answer(page, clock):
    """刚点了「读取状态」（或空闲时自动读了一次）就点「打开」：查询的回复（en=0）先到，
    不能判成"不符"；随后 ON 的回复（en=1）到了才确认。"""
    disarmed(page)
    page.notch_refresh()
    page.notch_on_button.invoke()
    assert page.panel.transport.lines[-2:] == ["RPMNOTCH ?", "RPMNOTCH ON"]
    feed(page, status_block(en=0, state="off", w=0))
    assert page.notch_reply_var.get() == "已发送 RPMNOTCH ON，等待飞控回复…"
    assert page.notch_status_var.get().endswith("关")
    feed(page, status_block(en=1, state="idle", w=0))
    assert page.notch_reply_var.get() == "飞控已确认：陷波已打开（状态 待命）。"
    assert "不符" not in page.notch_reply_var.get()


def test_an_unanswered_switch_expires_and_never_pairs_with_a_later_query(page, clock):
    disarmed(page)
    page.notch_off_button.invoke()
    clock[0] += 2.9
    page.notch_poll_idle()
    assert page.notch_reply_var.get() == "已发送 RPMNOTCH OFF，等待飞控回复…"
    clock[0] += 0.2
    page.notch_poll_idle()                     # 轮询每秒一次，到期就说清楚
    reply = page.notch_reply_var.get()
    assert "没等到飞控对 RPMNOTCH OFF 的回复" in reply
    feed(page, status_block(en=1, state="idle", w=0))     # 以后某次查询的回复：不再拿来判这次开关
    assert page.notch_reply_var.get() == reply


def test_a_mismatch_is_only_reported_once_the_wait_is_over(page, clock):
    disarmed(page)
    page.notch_on_button.invoke()
    feed(page, status_block(en=0, state="off", w=0))
    assert "不符" not in page.notch_reply_var.get()
    clock[0] += 3.5
    feed(page, status_block(en=0, state="off", w=0))      # 到期之后才到的一块：先了结，再当普通状态
    assert page.notch_reply_var.get().startswith("飞控回复与请求不符：RPMNOTCH ON 之后读到的是 en=0（状态 关）")


def test_switching_is_blocked_while_armed(page):
    disarmed(page, armed=1)
    assert not enabled(page.notch_on_button) and not enabled(page.notch_off_button)
    assert "已解锁" in page.notch_block_var.get()
    sent = list(page.panel.transport.lines)
    assert not page.notch_switch(True)
    assert page.panel.transport.lines == sent
    assert "已解锁" in page.notch_reply_var.get()
    disarmed(page)
    assert enabled(page.notch_on_button)


def test_switching_is_blocked_while_a_run_is_active(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    disarmed(page)                           # 即便此刻报的是未解锁，辨识在跑也不许改
    assert not enabled(page.notch_on_button)
    assert "辨识正在进行" in page.notch_block_var.get()
    assert not page.notch_switch(False)
    assert "RPMNOTCH OFF" not in page.panel.transport.lines


def test_the_firmware_rejection_is_shown_with_its_reason(page):
    disarmed(page)
    page.notch_switch(True)
    page.handle_line("RPMNOTCH event=rejected reason=sysid")
    reply = page.notch_reply_var.get()
    assert reply.startswith("飞控拒绝：辨识正占用台架") and "reason=sysid" in reply
    page.handle_line("RPMNOTCH event=rejected reason=armed")
    assert page.notch_reply_var.get().startswith("飞控拒绝：已解锁")


def test_old_firmware_without_the_command_is_explained_and_does_not_break_anything(page):
    disarmed(page)
    page.notch_refresh()
    page.handle_line("ERR unknown cmd RPMNOTCH")
    assert "不认 RPMNOTCH" in page.notch_status_var.get()
    assert not enabled(page.notch_on_button)
    assert page.workflow.error == ""
    reconnect(page)                           # 换了连接（可能换了固件）：重新允许
    page.refresh_notch_controls()
    assert "不认 RPMNOTCH" not in page.notch_block_var.get()


def test_a_late_notch_reply_never_hijacks_the_start_transaction(page):
    """点了「读取状态」马上点「开始辨识」：旧固件回的 ERR 与新固件的状态行都不许打断核对。"""
    page.notch_refresh()
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert page.workflow.awaiting == "SYSID SCHEMA"
    page.handle_line("ERR unknown cmd RPMNOTCH")
    feed(page, status_block())
    assert page.workflow.awaiting == "SYSID SCHEMA" and page.workflow.error == ""


def test_the_idle_page_reads_the_status_once_per_connection_and_after_each_run(page, monkeypatch, tmp_path):
    page.parent.winfo_viewable = lambda: True
    assert page.notch_poll_idle()
    assert page.panel.transport.lines == ["RPMNOTCH ?"] and page.panel.logged == []
    assert not page.notch_poll_idle(), "每个连接只要一次"
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.panel.transport.lines.clear()
    start_run(page)
    page.handle_line("SYSID end run=3 state=aborted reason=command dropped=0")
    assert page.notch_poll_idle(), "一轮结束后再读一次，看这轮的计数"
    assert page.panel.transport.lines[-1] == "RPMNOTCH ?"
    reconnect(page)
    page.workflow.run_id = 9                  # 模拟正在跑的一轮
    page.workflow.end = None
    assert not page.notch_poll_idle(), "辨识进行中不打扰"


# ---------------------------------------------------------------- 本轮溯源


def test_the_start_provenance_lands_in_the_snapshot_and_the_archive(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis as analysis_module
    monkeypatch.setattr(analysis_module, "fit_inner_loop", lambda *a, **k: FakeFit())
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    start_run(page)
    page.handle_line(NOTCH_ON.replace("run=3", "run=4"))     # 别的轮次：不收
    assert "rpm_notch" not in page.workflow.snapshot
    page.handle_line(NOTCH_ON)
    assert page.workflow.snapshot["rpm_notch"]["en"] == "1"
    assert page.workflow.snapshot["rpm_notch"]["pp"] == "7"
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and page._fit is not None)
    conditions = json.loads((tmp_path / "run" / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["rpm_notch"]["state"] == "idle" and conditions["rpm_notch"]["run"] == "3"
    header = page.fit_var.get().splitlines()[0]
    assert header.startswith("本轮陷波（桨叶振动）：开（开跑时 待命，极对数 7）")
    assert "不经过陷波" in header                # FF 轮是开环


def test_a_rate_run_shows_the_notch_state_above_the_tracking_error(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.mode_var.set("RATE")
    start_run(page, mode=1)
    page.handle_line(NOTCH_ON.replace("en=1 state=idle", "en=0 state=off"))
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    lines = page.fit_var.get().splitlines()
    assert lines[0] == "本轮陷波（桨叶振动）：关（开跑时 关，极对数 7）"
    assert "跟踪误差（15 Hz 以下" in page.fit_var.get()


def test_runs_without_provenance_show_no_notch_line(page, monkeypatch, tmp_path):
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    page.mode_var.set("RATE")
    start_run(page, mode=1)
    feed_full_run(page, n=60, mode_flag=0x20)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert "陷波" not in page.fit_var.get()


def test_provenance_text_is_plain_and_tolerates_missing_fields():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.notch_panel import notch_provenance_text
    finally:
        sys.path.pop(0)
    assert notch_provenance_text({}) == ""
    assert notch_provenance_text({"rpm_notch": {}}) == ""
    assert notch_provenance_text({"mode": "1", "rpm_notch": {"en": "1", "state": "tracking", "pp": "7"}}) == \
        "本轮陷波（桨叶振动）：开（开跑时 跟踪中，极对数 7）"
    assert "极对数 ?" in notch_provenance_text({"rpm_notch": {"en": "0"}})

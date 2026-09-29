"""Propeller/motor-direction page: calibration commands and FC-confirmed results only.

「校准 · 桨叶与电机方向」页：标定接线、点电机、如实显示飞控的回答。

判据围绕两类"看起来正常、其实错了"的失效：

**一类是标定本身。** 发出去的必须正好是飞控认的命令字；界面上的"已标定"必须来自
飞控的回显而不是本地乐观更新；未标定时结论必须说"没有方向"，不能显示成 +1——
把"还没量"显示成一个具体方向，正是这一整页要消灭的东西。

**另一类是点电机。** 它是全仓库唯一一条能从上位机让电机转起来的路径，所以本文件
把"停下来"当成主线来测：没勾确认不能开、开窗那一刻油门必须是 0、心跳必须真的按周期
重发、发不出去要本地停、飞控说停了本地必须跟着停、面板销毁要取消定时器。

另外钉一条工程约束：`tools/drone_tcp_panel.py` 不新增内容。本页靠
`panel_lib/board_line_hooks.py` 挂在既有调用点上。
"""

from __future__ import annotations

import tkinter as tk
from pathlib import Path

import pytest

from tools.panel_lib.pages.prop_map import (
    HEARTBEAT_MS, PROP_MAP_TELEM_STALE_S, ROLE_LABEL, SPIN_CONFIRM_TOKEN,
    SPIN_LABEL, SPIN_MAX_PCT_DEFAULT, SPIN_MAX_PCT_MAX, SPIN_MAX_PCT_MIN,
)
from tools.panel_lib.telem_subscription import TELEM_OWNER_PROP_MAP


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.accept = True

    def send_line(self, line: str) -> bool:
        if not self.accept:
            return False
        self.lines.append(line)
        return True

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        return True

    def set_binary_sink(self, sink) -> None:
        pass


# 固件真实发出的格式（App/Src/app_cmd_propcal.c 的格式串），不是手写的近似。
STATUS_UNCAL = ("PROPCAL state=status calibrated=0 gen=0 dirty=0 upper_ch=0 "
                "lower_ch=0 lower_spin=- yaw_polarity=0 reason=-")
STATUS_CAL = ("PROPCAL state=status calibrated=1 gen=4 dirty=1 upper_ch=2 "
              "lower_ch=1 lower_spin=cw yaw_polarity=+1 reason=-")
ROWS_CAL = [
    "PROPCAL ch=1 pad=M4/PE9 role=lower spin=cw declared=1",
    "PROPCAL ch=2 pad=M3/PE11 role=upper spin=ccw declared=1",
]
SPIN_IDLE = ("PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=20 timeout_ms=300 "
             "stop=none")
SPIN_ACTIVE = ("PROPCAL spin=active ch=2 pct=7 age_ms=40 max_pct=20 "
               "timeout_ms=300 stop=none")
SPIN_LOST = ("PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=20 timeout_ms=300 "
             "stop=heartbeat_lost")
# 双向 DShot 回传：esc_i2/esc_erpm2 用字面量 "-" 表示那一路没有有效数据。
SPIN_ACTIVE_ESC = (SPIN_ACTIVE + " esc_telem=1 esc_i1=3 esc_i2=- "
                    "esc_erpm1=12000 esc_erpm2=-")
# 默认构建：单向 DShot300，esc_telem=0，后面四个字段固件一律给 "-"。
SPIN_ACTIVE_NO_ESC = (SPIN_ACTIVE + " esc_telem=0 esc_i1=- esc_i2=- "
                       "esc_erpm1=- esc_erpm2=-")


@pytest.fixture
def app():
    from tools.drone_tcp_panel import DronePanel

    try:
        instance = DronePanel()
    except tk.TclError as exc:                    # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    instance._save_panel_state = lambda: None
    instance.transport = FakeTransport()
    try:
        yield instance
    finally:
        try:
            instance.destroy()
        except tk.TclError:
            # 有一条测试本身就要销毁面板（验 after 回调被取消）。销毁两次在
            # Tk 里是错误，但那不是被测行为，不该让它冒充失败。
            pass


def page(app):
    return app.prop_map_page


def feed(app, *lines: str) -> None:
    for line in lines:
        app._handle_board_line(line)
    app.update_idletasks()


def sent(app) -> list[str]:
    return app.transport.lines


def select_prop_map(app) -> None:
    """让本页在 `panel.notebook` / `panel.calibration_notebook` 上"可见"。

    判据与 `pages/power.py::visible()` 一致：顶层分组 + 二级页签都要选中。
    """
    app.notebook.select(app.calibration_group_tab)
    app.calibration_notebook.select(app.prop_map_tab)
    app.update_idletasks()


def leave_prop_map(app) -> None:
    """切到别的顶层分组，让本页"不可见"。"""
    app.notebook.select(app.sensor_group_tab)
    app.update_idletasks()


# ---------------------------------------------------------------- 挂载


def test_the_page_mounts_without_touching_the_entry_point() -> None:
    """`drone_tcp_panel.py` 不为本页新增内容——它只减不增。"""
    entry = read("tools/drone_tcp_panel.py")
    assert "prop_map" not in entry
    assert "PROPCAL" not in entry

    shell = read("tools/panel_lib/shell.py")
    assert 'self.calibration_notebook.add(prop_map_scroll, text="桨叶与电机方向")' in shell


def test_the_page_sits_in_the_calibration_group(app) -> None:
    """它是一次性的地面标定，和坐标系/舵机中心同属"校准"，不该另开顶层页签。"""
    labels = [app.calibration_notebook.tab(tab, "text")
              for tab in app.calibration_notebook.tabs()]
    assert "桨叶与电机方向" in labels
    assert labels.index("桨叶与电机方向") == labels.index("舵机机械中心与行程") + 1


# ---------------------------------------------------------------- 读与显示


def test_reading_asks_the_firmware_and_absorbs_the_reply(app) -> None:
    page(app).request_read()
    assert sent(app) == ["PROPCAL?"]

    feed(app, STATUS_CAL, *ROWS_CAL, SPIN_IDLE)
    assert page(app).calibrated is True
    assert page(app).upper_channel == 2
    assert page(app).lower_channel == 1
    assert page(app).lower_spin == "cw"
    # 焊盘名来自飞控（BSP 里的板级事实），不是界面写死的一张表。
    assert "M4/PE9" in page(app).pad_vars[1].get()
    assert "M3/PE11" in page(app).pad_vars[2].get()
    assert page(app).role_vars[1].get() == ROLE_LABEL["lower"]
    assert page(app).spin_vars[2].get() == SPIN_LABEL["ccw"]


def test_uncalibrated_says_no_direction_rather_than_a_default_one(app) -> None:
    """未标定时结论必须说"没有方向"。

    显示成 +1 会让人以为已经有极性了——而这一页存在的全部理由，就是不再让一个
    没量过的方向决定偏航。

    这里**不**声称"飞控会拒绝解锁"：作者 2026-09-13 明确要求不设那道门，
    由他自己决定什么时候需要标定。界面照抄一个不存在的规则，就是在替固件撒谎。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    text = page(app).conclusion_var.get()
    assert "未标定" in text
    assert "没有方向" in text
    assert "给不出偏航力矩" in text
    assert "解锁" not in text


def test_the_conclusion_spells_out_which_rotor_makes_positive_yaw(app) -> None:
    """结论必须落到"加大哪个桨"上。

    只报一个 ±1 的极性，等于把推导留给读的人自己做一遍——而那一步做反了，
    现象和增益调错几乎一样。
    """
    feed(app, STATUS_CAL, *ROWS_CAL, SPIN_IDLE)
    text = page(app).conclusion_var.get()
    assert "上桨 = 通道 2" in text
    assert "下桨 = 通道 1" in text
    assert "加大下桨" in text
    assert "+1" in text

    feed(app, STATUS_CAL.replace("yaw_polarity=+1", "yaw_polarity=-1")
              .replace("lower_spin=cw", "lower_spin=ccw"))
    assert "加大上桨" in page(app).conclusion_var.get()


def test_a_dropdown_change_sends_both_fields_of_that_channel(app) -> None:
    """一路的两项合成一条命令发。

    分成两条发的话，中间态（角色改了、旋向还没改）会让飞控先判一次 conflict、
    再判一次 applied，界面上闪过一句"两路互相矛盾"，而用户其实什么都没做错。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).role_vars[1].set(ROLE_LABEL["lower"])
    page(app).spin_vars[1].set(SPIN_LABEL["cw"])
    page(app)._send_channel(1)
    assert sent(app)[-1] == "PROPCAL SET ch=1 role=lower spin=cw"


def test_absorbing_a_reply_does_not_echo_it_back_as_an_edit(app) -> None:
    """回读覆盖下拉框时不能把它当成一次用户修改再发回去。

    发回去的后果是一条"我什么都没改"的读取会写一次飞控 RAM，而且每次读都写——
    dirty 永远清不掉，用户以为自己有未保存的改动。
    """
    feed(app, STATUS_CAL, *ROWS_CAL, SPIN_IDLE)
    assert sent(app) == []


def test_reset_and_commit_use_the_firmware_command_words(app) -> None:
    page(app).reset()
    page(app).commit()
    assert sent(app) == ["PROPCAL RESET", "PROPCAL COMMIT"]


def test_firmware_rejections_are_translated_not_blurred(app) -> None:
    feed(app, "PROPCAL state=write_blocked calibrated=1 gen=4 dirty=0 upper_ch=1 "
              "lower_ch=2 lower_spin=cw yaw_polarity=+1 reason=armed")
    assert "已解锁" in page(app).status_var.get()

    feed(app, "PROPCAL state=conflict calibrated=0 gen=5 dirty=1 upper_ch=0 "
              "lower_ch=0 lower_spin=- yaw_polarity=0 reason=-")
    assert "矛盾" in page(app).status_var.get()

    feed(app, "PROPCAL state=commit_failed st=4")
    assert "保存失败" in page(app).status_var.get()
    assert "st=4" in page(app).status_var.get()


def test_dirty_is_shown_so_nobody_walks_away_before_saving(app) -> None:
    feed(app, STATUS_CAL, *ROWS_CAL, SPIN_IDLE)
    assert page(app).dirty_var.get() != ""
    feed(app, STATUS_CAL.replace("dirty=1", "dirty=0"))
    assert page(app).dirty_var.get() == ""


# ---------------------------------------------------------------- 点电机


def test_spinning_requires_the_explicit_safety_confirmation(app) -> None:
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    assert str(page(app).start_button["state"]) == "disabled"

    page(app).start_spin()
    assert sent(app) == [], "没勾确认不该发出任何通电命令"

    page(app).confirm_var.set(True)
    page(app)._refresh_spin_controls()
    assert str(page(app).start_button["state"]) == "normal"


def test_selecting_a_channel_never_energises_it(app) -> None:
    """「点这一路 →」只挪选择，不通电。

    一个按钮既选通道又让电机转，意味着误点一下桨就转了。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app)._select_spin_channel(2)
    assert page(app).spin_channel_var.get() == 2
    assert sent(app) == []


def test_opening_the_window_starts_at_zero_throttle(app) -> None:
    """开窗那一刻油门必须是 0。

    保留上一次的油门意味着"开始通电"这一下就把电机推到上次的转速——而按下它的人
    正准备去看电机往哪边转。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).throttle_scale.set(15)
    page(app).start_spin()

    # max_pct= 带着本次上限（默认 20）一起发，见「本次油门上限」一节的测试。
    assert sent(app) == [f"PROPCAL SPIN ARM confirm={SPIN_CONFIRM_TOKEN} "
                          f"max_pct={SPIN_MAX_PCT_DEFAULT}"]
    assert page(app).throttle_var.get() == 0
    assert page(app).spin_active is True
    assert str(page(app).stop_button["state"]) == "normal"


def test_the_heartbeat_repeats_the_throttle_command(app) -> None:
    """心跳必须真的按周期重发。

    只发一次的话，飞控 300 ms 后就断电，而界面还显示"通电中"——用户会以为是
    电调坏了。这里直接驱动定时器回调，不等真实时间。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).start_spin()
    page(app).spin_channel_var.set(2)
    page(app).throttle_scale.set(6)

    for _ in range(3):
        page(app)._heartbeat()
    assert sent(app)[1:] == ["PROPCAL SPIN SET ch=2 pct=6"] * 3
    assert HEARTBEAT_MS <= 150, "重发周期必须明显短于固件 300 ms 的超时"


def test_a_heartbeat_that_cannot_be_sent_stops_locally(app) -> None:
    """发不出去就本地停。

    继续显示"通电中"而实际早已断电，会让人以为电机坏了；继续排队心跳则会在
    连接恢复的那一瞬间让电机重新转起来——而那时人可能已经走开了。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).start_spin()
    app.transport.accept = False

    page(app)._heartbeat()
    assert page(app).spin_active is False
    assert page(app)._timer is None
    assert "已停止" in page(app).spin_status_var.get()


def test_the_firmware_saying_it_stopped_stops_the_local_heartbeat(app) -> None:
    """飞控说停了，本地必须跟着停。

    本地继续发心跳的话，下一条命令会把窗口重新打开（固件侧 ARM 才开窗，但界面
    仍显示"通电中"），两边对同一件事的说法不一致。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).start_spin()
    assert page(app)._timer is not None

    feed(app, SPIN_LOST)
    assert page(app).spin_active is False
    assert page(app)._timer is None
    assert "心跳中断" in page(app).spin_status_var.get()
    assert str(page(app).start_button["state"]) == "normal"


def test_the_active_status_shows_the_heartbeat_age_and_timeout(app) -> None:
    """把心跳年龄和超时摆在人眼前，别让"还有多久会自动断"靠猜。"""
    feed(app, STATUS_CAL, *ROWS_CAL, SPIN_ACTIVE)
    text = page(app).spin_status_var.get()
    assert "通道 2" in text
    assert "7%" in text
    assert "40 ms" in text
    assert "300 ms" in text


def test_the_throttle_ceiling_comes_from_the_firmware(app) -> None:
    """上限以飞控报的为准，界面不自己定一个。

    界面写死一个更大的上限，用户拉到底会得到一句"命令不合法"外加窗口关闭；
    写死一个更小的，则是白白限制了固件允许的范围。
    """
    feed(app, STATUS_UNCAL,
         SPIN_IDLE.replace("max_pct=20", "max_pct=12"))
    assert page(app).max_percent == 12
    assert float(page(app).throttle_scale.cget("to")) == 12.0


def test_destroying_the_panel_cancels_the_heartbeat(app) -> None:
    """面板销毁必须取消定时器。

    留着 after 回调的话，测试全量跑时会在下一个面板上触发；真实使用中则是
    关掉窗口后仍在往一个已经不存在的 transport 上发命令。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).start_spin()
    instance = page(app)
    assert instance._timer is not None

    app.destroy()
    assert instance._disposed is True
    assert instance._timer is None
    assert instance.spin_active is False


def test_the_confirm_token_matches_the_firmware(app) -> None:
    """确认词两边必须一致。

    不一致的表现是：界面点了「开始通电」，飞控只回一句 need_confirm，
    而界面自己已经把状态切成"通电中"并开始发心跳——两边都不报错。
    """
    firmware = read("App/Src/app_cmd_propcal.c")
    assert f'PROPCAL_SPIN_CONFIRM_TOKEN "{SPIN_CONFIRM_TOKEN}"' in firmware


# ---------------------------------------------------------------- 本次油门上限


def test_the_default_ceiling_matches_the_firmware_default(app) -> None:
    assert page(app).spin_max_pct_var.get() == str(SPIN_MAX_PCT_DEFAULT)


def test_start_spin_sends_the_configured_max_pct(app) -> None:
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    page(app).spin_max_pct_var.set("55")
    page(app).start_spin()
    assert sent(app) == [f"PROPCAL SPIN ARM confirm={SPIN_CONFIRM_TOKEN} max_pct=55"]


def test_an_invalid_ceiling_sends_no_command_at_all(app) -> None:
    """上限非法（非整数 / 不在 1..100）时不发命令，在状态行里说明。"""
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    for bad in ("0", "101", "abc", "", "12.5", "-5"):
        page(app).spin_max_pct_var.set(bad)
        page(app).start_spin()
    assert sent(app) == [], "上限非法时一条命令都不该发出去"
    text = page(app).spin_status_var.get()
    assert f"{SPIN_MAX_PCT_MIN}-{SPIN_MAX_PCT_MAX}" in text
    assert "整数" in text


def test_the_ceiling_spinbox_locks_while_the_window_is_open(app) -> None:
    """窗口开着时上限输入框必须禁用：固件只降不升，开着改这个框没有意义，
    还会造成"界面写着 60、飞控只认开窗那一刻的 20"的错位。"""
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).confirm_var.set(True)
    assert str(page(app).spin_max_pct_spinbox["state"]) == "normal"

    page(app).start_spin()
    assert str(page(app).spin_max_pct_spinbox["state"]) == "disabled"

    # 飞控说窗口已经关了（比如心跳超时），本地必须跟着恢复输入框。
    feed(app, SPIN_IDLE)
    assert str(page(app).spin_max_pct_spinbox["state"]) == "normal"


def test_a_ceiling_above_twenty_shows_a_conspicuous_thrust_warning(app) -> None:
    """超过 20% 已经能产生可观推力：必须提醒先确认桨叶已拆或机体已固定。

    这句提醒不是勾选框（已有的安全确认勾选框不能动），只是一句醒目文字。
    """
    feed(app, STATUS_UNCAL, SPIN_IDLE)
    page(app).spin_max_pct_var.set(str(SPIN_MAX_PCT_MAX))
    assert "可观推力" in page(app).max_pct_warning_var.get()
    assert "拆" in page(app).max_pct_warning_var.get()

    page(app).spin_max_pct_var.set("20")
    assert page(app).max_pct_warning_var.get() == ""


# ---------------------------------------------------------------- 实时读数 · 电调


def test_esc_telemetry_is_parsed_per_channel_with_dash_for_no_data(app) -> None:
    """`esc_i2=- esc_erpm2=-` 必须显示成 "—"，不是 0——固件的 "-" 不是数值 0。"""
    feed(app, STATUS_UNCAL, SPIN_ACTIVE_ESC)
    assert page(app).esc_current_vars[1].get() == "3 A"
    assert page(app).esc_current_vars[2].get() == "—"
    assert page(app).esc_erpm_vars[1].get() == "12000 eRPM"
    assert page(app).esc_erpm_vars[2].get() == "—"


def test_esc_telem_zero_degrades_the_whole_block_to_no_numbers(app) -> None:
    """`esc_telem=0` 时整块降级成一句解释，不显示任何数字（哪怕上一拍还有数）。

    默认构建是单向 DShot300，压根没有这条回传通道；显示 0 A / 0 eRPM 会被
    读成"量过了，确实是 0"，而实际是根本没有这条数据。
    """
    feed(app, STATUS_UNCAL, SPIN_ACTIVE_ESC)
    assert page(app).esc_current_vars[1].get() == "3 A"          # 先确认真的有过数字

    feed(app, SPIN_ACTIVE_NO_ESC)
    assert "单向 DShot300" in page(app).esc_state_var.get()
    for channel in (1, 2):
        assert page(app).esc_current_vars[channel].get() == "—"
        assert page(app).esc_erpm_vars[channel].get() == "—"


def test_an_old_firmware_without_the_field_is_not_called_a_build_choice(app) -> None:
    """整个 `esc_telem` 字段缺席 ≠ `esc_telem=0`，两句话必须不一样。

    缺席只说明对面那版固件还不发这几个字段，界面**无从知道**它是哪一档构建。
    报成"本次构建是单向 DShot300"是在替一个没说话的固件下结论：2026-09-21
    作者拿新面板连旧固件时正是被这句话带偏，去查电调而不是先升级固件。
    """
    old_firmware_line = ("PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=20 "
                         "timeout_ms=300 stop=request")
    feed(app, STATUS_UNCAL, old_firmware_line)
    state = page(app).esc_state_var.get()
    assert "esc_telem" in state and "固件" in state
    assert "单向 DShot300" not in state
    for channel in (1, 2):
        assert page(app).esc_current_vars[channel].get() == "—"
        assert page(app).esc_erpm_vars[channel].get() == "—"


def test_a_ceiling_the_firmware_would_not_honour_is_called_out(app) -> None:
    """要 82% 而飞控只给 20%，界面必须说出是哪一方在压着。

    滑条的上限一律跟飞控走（飞控是唯一权威），但**默默跟着**会让人以为是界面
    坏了——作者第一次用这个输入框时就是这样：填 82，滑条只到 20，页面一个字
    都没解释，而真正的原因是板子上那版固件还不认 `max_pct=`。
    """
    page(app).spin_max_pct_var.set("82")
    feed(app, "PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=20 "
              "timeout_ms=300 stop=request esc_telem=0 esc_i1=- esc_i2=- "
              "esc_erpm1=- esc_erpm2=-")
    status = page(app).spin_status_var.get()
    assert "82" in status and "20" in status
    assert "max_pct" in status
    # 上限就是要的那个数时不该唠叨。
    page(app).spin_max_pct_var.set("20")
    feed(app, "PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=20 "
              "timeout_ms=300 stop=request esc_telem=0 esc_i1=- esc_i2=- "
              "esc_erpm1=- esc_erpm2=-")
    assert "注意" not in page(app).spin_status_var.get()


def test_the_decode_counters_say_which_of_three_failures_it_is(app) -> None:
    """回不来数时，"没收到"和"收到了解不开"必须分得开。

    双向 DShot 这条链 2026-09-21 才在 bitbang 后端实机打通（TIMER 后端实机只有超时在涨），
    换后端、换电调固件或改线路时仍可能回不来数。只显示 "—" 的话，三种完全不同的
    故障看起来一模一样。
    """
    def diag(avail, proto, fr1, crc1, to1, grace=0):
        return (f"PROPCAL escdiag avail={avail} proto={proto} grace={grace} "
                f"fr1={fr1} crc1={crc1} to1={to1} fr2=0 crc2=0 to2=0")

    # 0. 宽限期内"全零"是设计如此，不是故障——固件此刻没在听。不说明白的话
    #    这段时间的零计数会被读成"电调不回话"，把人引去查完全无关的地方。
    feed(app, diag(1, 2, 0, 0, 0, grace=320))
    hint = page(app).esc_diag_hint_var.get()
    assert "320" in hint and "没在听" in hint

    # 1. 压根不是双向档：说清楚要烧哪一档，别让人去查电调。
    feed(app, diag(0, 1, 0, 0, 0))
    assert "单向 DShot300" in page(app).esc_diag_var.get()
    assert "DSHOT300_BIDIR" in page(app).esc_diag_hint_var.get()

    # 2. 双向档、只有超时在涨：线上没有回传 → 查电调那一侧的模式。
    feed(app, diag(1, 2, 0, 0, 480))
    assert "双向 DShot300" in page(app).esc_diag_var.get()
    hint = page(app).esc_diag_hint_var.get()
    assert "超时" in hint and "bi_direction" in hint

    # 3. 有回传但一帧没解开：查协议这一侧，不是电调没说话。
    feed(app, diag(1, 2, 0, 312, 0))
    hint = page(app).esc_diag_hint_var.get()
    assert "GCR" in hint

    # 4. 解得开就不再唠叨——有成功帧之后剩下的是数值对不对，不是链路问题。
    feed(app, diag(1, 2, 900, 3, 1))
    assert page(app).esc_diag_hint_var.get() == ""
    assert "成功 900" in page(app).esc_diag_var.get()


def test_edt_buttons_send_only_the_two_words(app) -> None:
    """界面只送 ON/OFF，命令号在固件里。

    1..47 里躺着改电机转向和写电调 Flash 的命令号；一个"能填任意命令号"的输入框
    意味着敲错一个数字就把飞机的转向改了、还存进了电调。
    """
    page(app).request_edt("ON")
    assert sent(app) == ["ESC EDT ON"]
    page(app).request_edt("OFF")
    assert sent(app)[-1] == "ESC EDT OFF"
    assert page(app).spin_active is False


def test_edt_reply_never_claims_it_took_effect(app) -> None:
    """"帧发出去了" ≠ "EDT 打开了"。

    DShot 单向发送，电调不对特殊命令回 ACK。把前者说成后者，会让人在下一步
    去查接收侧，而真正该做的是看 EDT 帧有没有开始来。
    """
    feed(app, "ESC EDT event=queued state=done cmd=13 sent=10 repeats=10 "
              "reason=none proto=2")
    text = page(app).edt_var.get()
    assert "10/10" in text
    # 必须明说这只代表发出去了，并指向真正的判据。
    assert "不代表" in text and "解码计数" in text

    feed(app, "ESC EDT event=rejected reason=armed")
    assert "解锁" in page(app).edt_var.get()

    feed(app, "ESC EDT event=rejected reason=not_dshot proto=0")
    assert "DShot" in page(app).edt_var.get()

    feed(app, "ESC EDT event=queued state=aborted cmd=13 sent=3 repeats=10 "
              "reason=inhibited proto=2")
    text = page(app).edt_var.get()
    assert "3/10" in text and "作废" in text


def test_reading_the_counters_never_energises_anything(app) -> None:
    """裸 `PROPCAL SPIN` 是状态查询。这个按钮绝不能顺手开窗。"""
    page(app).request_esc_diag()
    assert sent(app) == ["PROPCAL SPIN"]
    assert page(app).spin_active is False


# ---------------------------------------------------------------- 实时读数 · 总线


def test_bus_reading_explains_why_when_not_subscribed(app) -> None:
    assert page(app).subscribed is False
    page(app)._render_bus()
    assert page(app).bus_voltage_var.get() == "—"
    assert page(app).bus_current_var.get() == "—"
    assert "没订阅" in page(app).bus_state_var.get()


def test_a_channel_stale_on_the_firmware_time_axis_shows_a_dash_not_the_old_value(
        app, monkeypatch) -> None:
    """`batt_i` 的时间戳比本帧最新时间戳旧超过阈值时必须显示 "—"，不是旧值。

    `batt_v` 同一时刻是新鲜的，用来证明这是逐通道的判断，不是整块一起蒙圈。
    """
    select_prop_map(app)
    page(app)._telem_tick()                     # 订阅上
    page(app)._on_telem_frame(0, b"", transport=app.transport)   # 造一条"刚到"的证据

    newest_t_us = 100_000_000                    # 固件时间轴上的"现在"：100 s
    fresh_t = newest_t_us * 1e-6
    stale_t = fresh_t - (PROP_MAP_TELEM_STALE_S + 1.0)
    monkeypatch.setattr(app, "telem_last_frame_t_us", newest_t_us)
    monkeypatch.setattr(
        app, "_telem_latest_raw",
        lambda name: {"batt_v": (fresh_t, 12.4), "batt_i": (stale_t, 3.0)}.get(name))

    page(app)._render_bus()
    assert page(app).bus_voltage_var.get() == "12.400 V"
    assert page(app).bus_current_var.get() == "—"


def test_the_page_subscribes_only_while_visible(app) -> None:
    select_prop_map(app)
    page(app)._telem_tick()
    assert page(app).subscribed is True
    assert app.telem_registry.is_registered(TELEM_OWNER_PROP_MAP)

    leave_prop_map(app)
    page(app)._telem_tick()
    assert page(app).subscribed is False
    assert not app.telem_registry.is_registered(TELEM_OWNER_PROP_MAP), (
        "页面不可见时必须退订")


def test_destroying_the_panel_unsubscribes_from_telemetry(app) -> None:
    select_prop_map(app)
    page(app)._telem_tick()
    instance = page(app)
    assert app.telem_registry.is_registered(TELEM_OWNER_PROP_MAP)

    app.destroy()
    assert instance._telem_timer is None
    assert not app.telem_registry.is_registered(TELEM_OWNER_PROP_MAP)

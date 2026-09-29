"""Status-LED page sends the exact FC command words and shows only the FC's reply.

「维护 · 状态灯」页：挑颜色、挑节奏档位、如实显示飞控的回答。

四条判据，每条都对应一种"看起来正常、其实错了"的失效：

1. **发出去的必须正好是飞控认的命令字。** 界面对了但命令字拼错，用户看到的是
   "我改了没反应"，而面板一句话都不报。
2. **确认必须来自飞控的回显，不是本地乐观更新。** 本地一改就显示新值，等于把
   "我按了"说成"它接受了"——`conflicts_with_armed` 这类拒绝会被吞掉。
3. **拒绝原因要如实翻译，不许含糊成"失败"。** 固件特地把 reason 分了七八种，
   在界面上糊成一句"保存失败"就等于把那份用心扔了。
4. **节奏档位表要和固件对得上两头。** 每一档都必须过得了固件的校验（否则界面
   给出一个点了就被拒的选项），而固件出厂表用到的每一种节奏都必须在档位表里
   （否则刚开机的板子会有几条显示成"自定义"，用户以为自己改过）。

另外钉一条工程约束：`tools/drone_tcp_panel.py` 不新增内容。本页靠
`panel_lib/board_line_hooks.py` 挂在既有调用点上，不再往那个只减不增的文件加转发。
"""

from __future__ import annotations

import re
import shutil
import subprocess
import tkinter as tk
from pathlib import Path

import pytest

from tools.panel_lib.pages import led_map as led_map_page
from tools.panel_lib.pages.led_map import (
    LED_CUSTOM_RHYTHM, LED_EFFECT_FIELDS, LED_PULSES_RHYTHM, LED_RHYTHMS,
    pulse_count_of, rhythm_of,
)


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        return True

    def set_binary_sink(self, sink) -> None:
        pass


STATUS = "LEDMAP state=status n=16 gen=3 dirty=0 customized=0 reason=-"
ROWS = [
    "LEDMAP bind=armed r=255 g=0 b=0 effect=solid on=0 off=0 gap=0 period=0 dim=0 lock=-",
    "LEDMAP bind=ready r=0 g=255 b=0 effect=breathe on=0 off=0 gap=0 period=2600 dim=10 lock=-",
    "LEDMAP bind=heartbeat r=0 g=0 b=255 effect=blink on=120 off=380 gap=0 period=0 lock=-",
    "LEDMAP bind=block_imu r=255 g=110 b=0 effect=pulses on=160 off=160 gap=760 "
    "period=0 dim=0 lock=count",
]


@pytest.fixture
def app(monkeypatch):
    from tools.drone_tcp_panel import DronePanel

    try:
        instance = DronePanel()
    except tk.TclError as exc:                    # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    instance._save_panel_state = lambda: None
    instance.transport = FakeTransport()
    # 取色器是模态对话框：不 patch 的话几何遍历和离线面板会**卡死**而不是失败，
    # CI 上表现为超时，比断言失败难查得多。
    monkeypatch.setattr(led_map_page.colorchooser, "askcolor",
                        lambda *args, **kwargs: ((1, 2, 3), "#0A141E"))
    try:
        yield instance
    finally:
        instance.destroy()


def load(app) -> None:
    for line in [STATUS, *ROWS]:
        app._handle_board_line(line)
    app.update_idletasks()


def sent(app) -> list[str]:
    return app.transport.lines


# ---------------------------------------------------------------- 挂载


def test_the_page_mounts_without_touching_the_entry_point() -> None:
    """`drone_tcp_panel.py` 不为本页新增内容——它只减不增。

    本页的回包靠 `board_line_hooks` 挂在 `parameter_editor.py` 既有的调用点上，
    那里是 `_handle_board_line` 调的第一个 panel_lib 函数。
    """
    entry = read("tools/drone_tcp_panel.py")
    assert "led_map" not in entry
    assert "LEDMAP" not in entry

    editor = read("tools/panel_lib/parameter_editor.py")
    assert "dispatch_board_line(self, line)" in editor
    # 必须在 OK/ERR 早退**之前**：LEDMAP 回包既不是 OK 也不是 ERR。
    assert (editor.index("dispatch_board_line(self, line)") <
            editor.index('if not (line.startswith("ERR ") or line.startswith("OK "))'))

    shell = read("tools/panel_lib/shell.py")
    assert 'self.maintenance_notebook.add(led_map_scroll, text="状态灯")' in shell


def test_the_three_maintenance_pages_live_under_one_group(app) -> None:
    """三个页签都顶着"维护 · "前缀时，那层结构本来就存在，只是用文字模拟的。"""
    labels = [app.notebook.tab(tab, "text") for tab in app.notebook.tabs()]
    assert "维护" in labels
    assert not any(label.startswith("维护 · ") for label in labels), labels

    inner = [app.maintenance_notebook.tab(tab, "text")
             for tab in app.maintenance_notebook.tabs()]
    assert inner == ["固件升级", "舵机调试", "状态灯"]


def test_the_firmware_poll_gate_still_sees_the_page_one_level_down(app) -> None:
    """页面收进二级分组后，"这一页看得见吗"必须仍然判得对。

    判错的后果是固件升级页一打开就停止轮询——页面看着是开的、数据不再刷新，
    而且没有任何报错。`sensor_notebook` 那次就是这么停摆的。
    """
    from tools.panel_lib.viewport import leaf_tab_visible

    app.notebook.select(app.maintenance_group_tab)
    app.maintenance_notebook.select(app.firmware_tab)
    app.update_idletasks()
    outer = app.notebook.select()
    assert leaf_tab_visible(app, app.firmware_tab, outer) is True

    # 同一个分组里换一页，固件页就不再是前台。
    app.maintenance_notebook.select(app.led_map_tab)
    app.update_idletasks()
    assert leaf_tab_visible(app, app.firmware_tab, app.notebook.select()) is False

    # 顶层换栏目同理。
    app.notebook.select(app.sensor_group_tab)
    app.update_idletasks()
    assert leaf_tab_visible(app, app.firmware_tab, app.notebook.select()) is False


def test_the_board_lines_reach_the_page_through_the_real_receive_path(app) -> None:
    load(app)
    page = app.led_map_page

    assert len(page.bindings) == len(ROWS)
    assert page.bindings["ready"]["period"] == "2600"
    assert page.row_swatch["ready"].cget("background") == "#00FF00"
    assert "已读取 16 条绑定" in page.status_var.get()


# ------------------------------------------------------------ 节奏档位


def test_every_rhythm_preset_passes_the_firmware_validator(tmp_path) -> None:
    """档位表里不许出现一个"点了就被拒"的选项。

    规则不在 Python 里重写一遍：直接拿固件那份 `app_led_config.c` 编译来跑。
    抄一份规则表的下场是两边分叉——界面允许、固件拒收，用户只能靠猜。
    """
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    effect_enum = {
        "off": "DRV_RGB_EFFECT_OFF", "solid": "DRV_RGB_EFFECT_SOLID",
        "blink": "DRV_RGB_EFFECT_BLINK", "pulses": "DRV_RGB_EFFECT_PULSES",
        "breathe": "DRV_RGB_EFFECT_BREATHE",
    }
    cases = []
    for index, (label, effect, on, off, gap, period, dim) in enumerate(LED_RHYTHMS):
        # 每一档都装到"界面真会把它提供给谁"那条绑定上。数闪只提供给数得出次数的
        # 那几条（固件对别的会拒，因为 count=0 就是永久黑灯），所以别拿 ready 去试。
        target = ("APP_LED_BIND_BLOCK_IMU" if effect == "pulses"
                  else "APP_LED_BIND_READY")
        cases.append(
            "    APP_LedConfig_Defaults(&c);\n"
            f"    c.binding[{target}].r = 0U;\n"
            f"    c.binding[{target}].g = 0U;\n"
            f"    c.binding[{target}].b = 255U;\n"
            f"    c.binding[{target}].effect = {effect_enum[effect]};\n"
            f"    c.binding[{target}].on_ms = {on}U;\n"
            f"    c.binding[{target}].off_ms = {off}U;\n"
            f"    c.binding[{target}].gap_ms = {gap}U;\n"
            f"    c.binding[{target}].period_ms = {period}U;\n"
            f"    c.binding[{target}].dim = {dim}U;\n"
            "    if (APP_LedConfig_Validate(&c) != 1U) {\n"
            f"        fprintf(stderr, \"rhythm {index} rejected\\n\");\n"
            f"        return {index + 1};\n"
            "    }\n"
        )
    harness = (
        '#include "app_led_config.h"\n#include <stdio.h>\n\n'
        "int main(void)\n{\n    APP_LedConfig c;\n\n"
        + "\n".join(cases)
        + '\n    printf("rhythm presets ok\\n");\n    return 0;\n}\n'
    )
    source = tmp_path / "rhythms.c"
    source.write_text(harness, encoding="utf-8")
    executable = tmp_path / "rhythms.exe"
    subprocess.run(
        [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{ROOT / 'App' / 'Inc'}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ROOT / "App" / "Src" / "app_led_config.c"),
         str(ROOT / "Driver" / "Src" / "drv_rgb_led.c"),
         str(source), "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "rhythm presets ok" in result.stdout.decode("ascii", "replace")


def _firmware_default_rhythms() -> dict[str, dict[str, str]]:
    """从 `app_led_config.c` 的出厂表里抓每一条的节奏，不手抄。

    手抄一份的话，固件改了默认值而这里没跟，测试照绿——而那正是它要拦的事。
    """
    source = read("App/Src/app_led_config.c")
    source = re.sub(r"/\*.*?\*/", " ", source, flags=re.S)
    macros = dict(re.findall(r"#define\s+(LED_[A-Z_]+)\s+(.+)", source))
    table = source[source.index("led_default_binding[APP_LED_BIND_COUNT]"):]
    table = table[table.index("{") + 1:table.index("\n};")]

    rhythms = {}
    for name, body in re.findall(r"\[APP_LED_BIND_(\w+)\]\s*=\s*\{([^}]*)\}", table):
        for macro, value in macros.items():
            body = body.replace(macro, value)
        fields = [item.strip().rstrip("U") for item in body.split(",")]
        assert len(fields) == 10, (name, fields)
        effect = fields[3].replace("DRV_RGB_EFFECT_", "").lower()
        rhythms[name] = dict(zip(("on", "off", "gap", "period", "dim"), fields[4:9]),
                             effect=effect)
    return rhythms


def test_the_presets_cover_every_rhythm_the_firmware_ships_with() -> None:
    """刚出厂的板子上，十六条绑定都得显示得出档位名。

    漏一条的症状很温和、也很误导：用户打开页面看到一条"自定义…"，以为自己
    以前改过它，于是不敢动——而它其实就是出厂值。
    """
    defaults = _firmware_default_rhythms()
    assert len(defaults) == 16, sorted(defaults)

    unnamed = {name: record for name, record in defaults.items()
               if rhythm_of(record) is None}
    assert unnamed == {}, (
        f"这些出厂节奏在 LED_RHYTHMS 里没有对应档位：{unnamed}。"
        "要么给档位表加一档，要么改固件默认值——但别让用户看到「自定义」。"
    )


def test_a_rhythm_is_matched_only_on_the_fields_that_effect_uses() -> None:
    """effect=solid 时 on/off 是多少都不影响灯长什么样，固件也不读它们。

    把它们算进比较，会让一条本来就是"常亮"的绑定因为残留的历史毫秒数而显示成
    "自定义"——用户于是以为自己改过它。
    """
    assert rhythm_of({"effect": "solid", "on": "999", "off": "77"}) == "常亮"
    assert rhythm_of({"effect": "blink", "on": "500", "off": "500",
                      "period": "1234"}) == "慢闪 · 0.5 秒"
    # 但用得上的字段差一点就不是同一档——不许四舍五入到最近的那个。
    assert rhythm_of({"effect": "blink", "on": "501", "off": "500"}) is None
    assert set(LED_EFFECT_FIELDS) == {"off", "solid", "blink", "pulses", "breathe"}


def test_choosing_a_preset_sends_the_five_numbers_behind_it(app) -> None:
    load(app)
    page = app.led_map_page
    page.select("heartbeat")
    page.rhythm_var.set("慢闪 · 0.5 秒")
    page.choose_rhythm()

    assert sent(app)[-1] == "LEDMAP RHYTHM heartbeat blink 500 500 0 0 0"


def test_the_millisecond_fields_stay_hidden_until_custom_is_chosen(app) -> None:
    """毫秒是逃生口，不是主路径。默认收起来。"""
    load(app)
    page = app.led_map_page
    page.select("ready")                      # breathe 2600/10 → 命中"慢呼吸"
    assert page.rhythm_var.get() == "慢呼吸 · 2.6 秒一个来回"
    assert page.custom_row.winfo_manager() == ""

    before = len(sent(app))
    page.rhythm_var.set(LED_CUSTOM_RHYTHM)
    page.choose_rhythm()
    assert page.custom_row.winfo_manager() == "pack"
    # 毫秒还没填完就发出去，只会连着收几条拒绝。
    assert len(sent(app)) == before

    page.field_vars["period"].set("1500")
    page.apply_rhythm()
    assert sent(app)[-1] == "LEDMAP RHYTHM ready breathe 0 0 0 1500 10"


def test_a_binding_the_presets_cannot_express_shows_its_real_numbers(app) -> None:
    """认不出的值照实摊开，**不四舍五入到最近的档位**。

    四舍五入的后果是：用户只是打开页面看了一眼，板子上的值就被悄悄改了。
    """
    load(app)
    app._handle_board_line(
        "LEDMAP bind=ready r=0 g=255 b=0 effect=blink on=137 off=291 "
        "gap=0 period=0 dim=0 lock=-")
    page = app.led_map_page
    page.select("ready")

    assert page.rhythm_var.get() == LED_CUSTOM_RHYTHM
    assert page.custom_row.winfo_manager() == "pack"
    assert page.field_vars["on"].get() == "137"
    assert page.field_vars["off"].get() == "291"
    assert page.row_summary["ready"].get() == LED_CUSTOM_RHYTHM


# ---------------------------------------------------------------- 颜色


def test_applying_a_colour_sends_exactly_what_the_firmware_parses(app) -> None:
    load(app)
    page = app.led_map_page
    page.select("ready")
    page.hex_var.set("#12FF80")
    page.apply_colour()

    assert sent(app)[-1] == "LEDMAP SET ready 18 255 128"


def test_the_quick_swatches_and_the_picker_feed_the_same_path(app) -> None:
    """常用色块和取色盘都只是快捷方式：填同一个输入框，走同一条发送路径。"""
    load(app)
    page = app.led_map_page
    page.select("ready")

    page.use_colour("#FF6E00")
    assert sent(app)[-1] == "LEDMAP SET ready 255 110 0"

    page.pick_colour()
    assert page.hex_var.get() == "#0A141E"
    assert sent(app)[-1] == "LEDMAP SET ready 10 20 30"


def test_a_malformed_hex_is_refused_locally_and_sends_nothing(app) -> None:
    """本地就能判死的输入不发出去，也不静默钳位。

    钳位会让用户以为飞控收下了他写的那个值。
    """
    load(app)
    page = app.led_map_page
    page.select("ready")
    before = len(sent(app))
    page.hex_var.set("绿色")
    page.apply_colour()

    assert len(sent(app)) == before
    assert "#RRGGBB" in page.status_var.get()


def test_preview_reset_and_commit_use_the_documented_verbs(app) -> None:
    load(app)
    page = app.led_map_page
    page.select("block_imu")
    page.preview()
    page.reset_all()
    page.commit()

    assert sent(app)[-3:] == ["LEDMAP PREVIEW block_imu", "LEDMAP RESET", "LEDMAP COMMIT"]


def test_writes_go_through_the_panel_safety_gate_and_leave_a_trace(app) -> None:
    """`LEDMAP COMMIT` 会让飞控同步擦一个 128 KB 扇区。它必须受面板的门管。

    绕过去的后果有两层：V0 只读验收会话进行中、或固件升级窗口里照样写 Flash；
    而且控制台和证据记录里查不到这次写入曾经发生过。
    """
    load(app)
    page = app.led_map_page
    page.select("ready")

    app._append_lines = []
    monitored = []
    app._append = monitored.append
    page.commit()
    assert sent(app)[-1] == "LEDMAP COMMIT"
    assert any("LEDMAP COMMIT" in line for line in monitored), "控制台里没留痕"

    # 固件升级窗口打开时，同一条命令必须被面板自己挡下。
    before = len(sent(app))
    app.firmware_update_running = True
    try:
        page.commit()
    finally:
        app.firmware_update_running = False
    assert len(sent(app)) == before, "固件升级进行中仍然把写 Flash 的命令发了出去"
    assert "挡下" in page.status_var.get()


def test_nothing_is_sent_while_disconnected_and_the_draft_survives(app) -> None:
    load(app)
    page = app.led_map_page
    page.select("ready")
    page.hex_var.set("#010203")
    app.transport.is_connected = False
    before = len(sent(app))
    page.apply_colour()

    assert len(sent(app)) == before
    assert page.hex_var.get() == "#010203", "草稿不该因为没连上就被清掉"


# ------------------------------------------------------- 如实显示飞控的回答


def test_confirmation_comes_from_the_board_not_from_the_click(app) -> None:
    """点一下不等于生效。只有飞控回 applied_ram 才算，清单也才跟着变。"""
    load(app)
    page = app.led_map_page
    page.select("ready")
    page.hex_var.set("#11FF11")
    page.apply_colour()
    assert "生效" not in page.status_var.get()
    assert page.row_swatch["ready"].cget("background") == "#00FF00", (
        "飞控还没回话，清单就不该显示新颜色"
    )

    app._handle_board_line(
        "LEDMAP state=applied_ram bind=ready n=16 gen=4 dirty=1 customized=1 reason=-")
    assert "已在飞控上生效" in page.status_var.get()
    assert page.dirty_var.get() == "● 已改未保存"
    assert page.row_swatch["ready"].cget("background") == "#11FF11"


def test_a_refusal_puts_the_widgets_back_to_what_the_board_holds(app) -> None:
    """被拒之后界面不能停在用户刚选的那一档。

    否则界面显示新值、板子还是旧值，两边就此分家——而用户下一眼看的是界面。
    """
    load(app)
    page = app.led_map_page
    page.select("ready")
    page.rhythm_var.set("常亮")
    page.choose_rhythm()

    app._handle_board_line(
        "LEDMAP state=rejected bind=ready n=16 gen=4 dirty=0 customized=0 "
        "reason=conflicts_with_armed")
    assert page.rhythm_var.get() == "慢呼吸 · 2.6 秒一个来回"
    assert page.bindings["ready"]["effect"] == "breathe"


def test_every_reject_reason_is_translated_not_lumped_together(app) -> None:
    """固件把拒绝分成了七八种，界面不许糊成一句"失败"。"""
    load(app)
    page = app.led_map_page
    seen = set()
    for reason in ("conflicts_with_armed", "gap_too_short", "zero_timing",
                   "bad_effect", "bad_period", "dim_too_high", "bad_channel"):
        app._handle_board_line(
            f"LEDMAP state=rejected bind=ready n=16 gen=4 dirty=1 "
            f"customized=1 reason={reason}")
        text = page.status_var.get()
        assert reason not in text, f"{reason} 只是原样回显了英文，没有翻译"
        seen.add(text)
    assert len(seen) == 7, "不同的拒绝原因显示成了同一句话"


def test_commit_failure_is_not_dressed_up_as_success(app) -> None:
    load(app)
    page = app.led_map_page
    app._handle_board_line("LEDMAP state=commit_failed st=4")

    assert "保存失败" in page.status_var.get()
    assert "st=4" in page.status_var.get()


def test_armed_refusal_says_what_to_do(app) -> None:
    load(app)
    page = app.led_map_page
    app._handle_board_line(
        "LEDMAP state=armed_blocked bind=- n=16 gen=4 dirty=0 customized=0 reason=-")

    assert "已解锁" in page.status_var.get()


# ------------------------------------------------------- 只读字段与预览


def test_the_locked_blink_count_is_explained_not_just_greyed_out(app) -> None:
    """闪几下不可改。界面要说清为什么，否则用户会以为是 bug。"""
    load(app)
    page = app.led_map_page
    page.select("block_imu")

    assert "原因码" in page.lock_var.get()
    assert "ARM?" in page.lock_var.get()

    page.select("ready")
    assert page.lock_var.get() == ""


def test_the_selected_row_is_visible_in_the_list_itself(app) -> None:
    """16 行长得一模一样时，"我在编哪一条"不该只能靠编辑框标题去确认。"""
    load(app)
    page = app.led_map_page
    page.select("ready")
    assert str(page.row_button["ready"].cget("style")) == "Primary.TButton"

    page.select("block_imu")
    assert str(page.row_button["block_imu"].cget("style")) == "Primary.TButton"
    assert str(page.row_button["ready"].cget("style")) == "Secondary.TButton"


def test_unused_timing_fields_are_disabled_per_effect(app) -> None:
    """改一个当前效果用不上的字段，是在浪费时间找 bug。"""
    load(app)
    page = app.led_map_page
    page.select("ready")

    page.effect_var.set("breathe")
    page.refresh_field_state()
    assert str(page.field_widgets["period"].cget("state")) == "normal"
    assert str(page.field_widgets["gap"].cget("state")) == "disabled"

    page.effect_var.set("pulses")
    page.refresh_field_state()
    assert str(page.field_widgets["gap"].cget("state")) == "normal"
    assert str(page.field_widgets["period"].cget("state")) == "disabled"

    page.effect_var.set("solid")
    page.refresh_field_state()
    assert all(str(w.cget("state")) == "disabled" for w in page.field_widgets.values())


def _first_burst(page, record, count) -> int:
    """4 秒里数出来的上升沿。"""
    edges = 0
    previous = False
    for now_ms in range(0, 4000):
        lit = page._lit(record, now_ms, count)
        if lit and not previous:
            edges += 1
        previous = lit
    return edges


def test_the_waveform_draws_the_right_number_of_pulses(app) -> None:
    """并排波形是用来"自己看一眼分不分得清"的，次数必须画对。

    block_imu 是第 5 号原因，所以 4 秒里该出现 5 下一组的节奏。
    """
    load(app)
    page = app.led_map_page
    page.select("block_imu")

    assert page._preview_count() == 5
    # 一轮 = 5×320 + 760 = 2360 ms，4 秒里看得到第一轮完整的 5 下。
    assert _first_burst(page, page.bindings["block_imu"], 5) >= 5


def test_the_neighbour_row_is_drawn_with_its_own_count(app) -> None:
    """邻行按**它自己**的次数画，不是按选中项的。

    这页并排两条波形的唯一用途就是"6 下和 7 下一眼分得清"。拿选中项的次数去画
    邻行，对比图会说这两条一模一样、必须换颜色才能区分——而板子上它们本来就
    差一下。预览骗人比没有预览更糟。
    """
    load(app)
    page = app.led_map_page
    app._handle_board_line(
        "LEDMAP bind=block_frame r=255 g=110 b=0 effect=pulses on=160 off=160 "
        "gap=760 period=0 dim=0 lock=count")
    page.select("block_airframe")                 # 7 号

    assert page._preview_count() == 7
    assert page._preview_count("block_frame") == 6, "邻行是 6 号原因，闪 6 下"
    assert pulse_count_of("flow_failed") == 3, "光流失败改造前就是青色 3 连闪"


def test_a_binding_nobody_counts_is_not_offered_the_pulses_preset(app) -> None:
    """固件会拒的选项，界面就不该给出来。

    数不出次数的绑定配成数闪，固件那边 count=0 → 驱动直接返回黑：灯灭，而
    `LEDMAP?` 照回 effect=pulses。让用户点得到这个选项，就是让他去撞那堵墙。
    """
    load(app)
    page = app.led_map_page

    page.select("ready")
    assert pulse_count_of("ready") == 0
    assert LED_PULSES_RHYTHM not in page.rhythm_combo.cget("values")

    page.select("block_imu")
    assert LED_PULSES_RHYTHM in page.rhythm_combo.cget("values")


def test_the_page_stores_no_firmware_values_in_panel_state(app) -> None:
    """飞控是唯一权威。本地缓存一份就会出现"界面和板子不一样"。"""
    load(app)
    app.led_map_page.select("ready")
    app.led_map_page.apply_colour()

    state = getattr(app, "_panel_state", {})
    assert "led_map" not in state or not state["led_map"], (
        "本页不该把飞控的值写进 panel_state.json"
    )

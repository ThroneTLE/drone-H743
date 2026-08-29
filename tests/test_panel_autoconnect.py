"""记住上次的连接并在启动时自动连回去。

关键点是**按什么认板子**：Windows 给 USB 串口分配的 COM 号会变（本机就出现过同一块
飞控在 COM30/31/32 之间跳），而 ST-Link、蓝牙串口、CH340 会占掉腾出来的号。所以匹配
必须以 USB 序列号为准，COM 号只是退路——反过来做会在重新编号之后连到别的设备上。
"""

from __future__ import annotations

from pathlib import Path

from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


def identity(device: str, *, vid=0x0483, pid=0x5740, serial_number="335335763233",
             location="1-2.4.1.1.4") -> dict[str, object]:
    return {
        "device": device, "vid": vid, "pid": pid,
        "description": "STMicroelectronics Virtual COM Port",
        "hwid": f"USB VID:PID={vid:04X}:{pid:04X} SER={serial_number}",
        "location": location, "serial_number": serial_number,
    }


# --------------------------------------------------------------------------
# 指纹
# --------------------------------------------------------------------------

def test_fingerprint_is_stable_across_com_renumbering() -> None:
    assert (panel.serial_port_fingerprint(identity("COM31"))
            == panel.serial_port_fingerprint(identity("COM7")))


def test_two_boards_of_the_same_model_get_different_fingerprints() -> None:
    """同型号第二块飞控 VID:PID 完全一样，只有序列号能区分。"""
    assert (panel.serial_port_fingerprint(identity("COM31"))
            != panel.serial_port_fingerprint(identity("COM31", serial_number="999")))


def test_a_port_without_a_serial_number_falls_back_to_the_usb_location() -> None:
    fingerprint = panel.serial_port_fingerprint(identity("COM5", serial_number=""))
    assert "0483:5740" in fingerprint
    assert "1-2.4.1.1.4" in fingerprint


def test_a_port_with_no_usb_identity_has_no_fingerprint() -> None:
    assert panel.serial_port_fingerprint(None) == ""
    assert panel.serial_port_fingerprint({}) == ""
    assert panel.serial_port_fingerprint({"serial_number": "", "location": ""}) == ""


# --------------------------------------------------------------------------
# 匹配
# --------------------------------------------------------------------------

FINGERPRINT = panel.serial_port_fingerprint(identity("COM31"))


def test_same_port_is_matched() -> None:
    device, reason = panel.match_remembered_serial_port(
        "COM31", FINGERPRINT, {"COM31": identity("COM31")}
    )
    assert device == "COM31"
    assert "序列号" in reason


def test_the_board_is_followed_when_windows_renumbers_it() -> None:
    """这是这个功能的主要价值：拔插一次 USB 换了 COM 号也不用手动重选。"""
    device, reason = panel.match_remembered_serial_port(
        "COM31", FINGERPRINT,
        {"COM3": identity("COM3", vid=0x0483, pid=0x3748, serial_number="stlink"),
         "COM7": identity("COM7")},
    )
    assert device == "COM7"
    assert "COM31" in reason  # 告诉用户它换号了


def test_a_different_device_squatting_on_the_old_com_number_is_refused() -> None:
    """COM31 现在是 ST-Link 或蓝牙串口时，绝不能因为号码一样就连上去。"""
    device, reason = panel.match_remembered_serial_port(
        "COM31", FINGERPRINT,
        {"COM31": identity("COM31", vid=0x0483, pid=0x3748, serial_number="stlink")},
    )
    assert device is None
    assert "身份和上次不一致" in reason


def test_a_missing_board_reports_instead_of_guessing() -> None:
    device, reason = panel.match_remembered_serial_port(
        "COM31", FINGERPRINT, {"COM3": identity("COM3", serial_number="other")}
    )
    assert device is None
    assert "COM31" in reason


def test_a_record_without_a_fingerprint_still_matches_by_com_number() -> None:
    """旧记录（或没有 USB 身份的口）不能因此完全用不了。"""
    device, reason = panel.match_remembered_serial_port(
        "COM18", "", {"COM18": {}}
    )
    assert device == "COM18"
    assert "串口号" in reason


def test_no_record_at_all_is_not_an_error() -> None:
    assert panel.match_remembered_serial_port("", "", {"COM1": {}})[0] is None


# --------------------------------------------------------------------------
# 面板接线
# --------------------------------------------------------------------------

def test_only_successful_connections_are_remembered() -> None:
    """连失败也记下来，下次启动会一直去撞同一个坏口。"""
    start = function_body(SOURCE, "    def _start(self)")
    assert "if self.serial_transport.is_connected:" in start
    assert start.index("self.transport.start(port_name, baud)") < start.index("_save_panel_state()")


def test_state_records_the_fingerprint_not_just_the_com_number() -> None:
    save = function_body(SOURCE, "    def _save_panel_state(")
    assert 'state["serial_port"] = device' in save
    assert "serial_port_fingerprint(" in save
    assert 'state["auto_connect"]' in save


def test_restore_runs_at_startup_and_can_be_switched_off() -> None:
    assert "self.after_idle(self._restore_last_connection)" in SOURCE
    restore = function_body(SOURCE, "    def _restore_last_connection(")
    assert "match_remembered_serial_port(" in restore
    assert "if not self.auto_connect_var.get():" in restore
    # 关掉自动连接时仍然把上次的口选中，省得用户自己找。
    assert restore.index("self.serial_port_var.set(label)") < restore.index(
        "if not self.auto_connect_var.get():"
    )


def test_restore_is_deferred_so_the_window_paints_first() -> None:
    restore = function_body(SOURCE, "    def _restore_last_connection(")
    assert "self.after(300, self._auto_connect_now)" in restore


def test_write_failures_never_break_the_live_connection() -> None:
    save = function_body(SOURCE, "    def _save_panel_state(")
    assert "except OSError:" in save
    load = function_body(SOURCE, "    def _load_panel_state(")
    assert "except json.JSONDecodeError:" in load
    assert "except (OSError, ValueError):" in load


def test_state_file_is_not_tracked() -> None:
    assert "data/panel_state.json" in (ROOT / ".gitignore").read_text(encoding="utf-8")

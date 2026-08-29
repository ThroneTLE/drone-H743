"""烧录前必须先编译，且编译失败绝不能继续烧录。

背景：USB DFU 一键烧录默认烧的是 build/Debug/drone-H743.elf。如果改完源码
忘了编译，就会把上一次的旧固件刷进飞控，而且现场很难看出来——固件"烧成功了"，
行为却还是旧的。这些测试锁死：默认先编译、编译失败即中止、关掉自动编译时
界面必须显示镜像已陈旧。
"""

from __future__ import annotations

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import rom_dfu
from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


class FakeCompleted:
    def __init__(self, returncode: int, stdout: str = "") -> None:
        self.returncode = returncode
        self.stdout = stdout


# --------------------------------------------------------------------------
# build_firmware
# --------------------------------------------------------------------------

def test_build_failure_raises_instead_of_returning_a_stale_elf(monkeypatch, tmp_path) -> None:
    elf = tmp_path / "build" / "Debug" / "drone-H743.elf"
    elf.parent.mkdir(parents=True)
    elf.write_bytes(b"stale")  # 旧产物存在，仍然不能被当成成功结果
    monkeypatch.setattr(
        subprocess, "run",
        lambda *a, **k: FakeCompleted(1, "error: undefined reference to `foo'\n"))

    with pytest.raises(rom_dfu.FirmwareBuildError) as excinfo:
        rom_dfu.build_firmware(tmp_path)

    assert "编译失败" in str(excinfo.value)
    assert "undefined reference" in str(excinfo.value)


def test_missing_cmake_is_reported_as_a_build_error(monkeypatch, tmp_path) -> None:
    def boom(*_a, **_k):
        raise FileNotFoundError("cmake")

    monkeypatch.setattr(subprocess, "run", boom)

    with pytest.raises(rom_dfu.FirmwareBuildError) as excinfo:
        rom_dfu.build_firmware(tmp_path)

    assert "找不到 cmake" in str(excinfo.value)


def test_build_timeout_is_reported_as_a_build_error(monkeypatch, tmp_path) -> None:
    def boom(*_a, **_k):
        raise subprocess.TimeoutExpired(cmd="cmake", timeout=1.0)

    monkeypatch.setattr(subprocess, "run", boom)

    with pytest.raises(rom_dfu.FirmwareBuildError):
        rom_dfu.build_firmware(tmp_path, timeout_s=1.0)


def test_successful_build_must_actually_produce_the_elf(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: FakeCompleted(0, "ok\n"))

    with pytest.raises(rom_dfu.FirmwareBuildError) as excinfo:
        rom_dfu.build_firmware(tmp_path)

    assert "未找到产物" in str(excinfo.value)


def test_successful_build_returns_the_preset_artifact(monkeypatch, tmp_path) -> None:
    elf = tmp_path / "build" / "Debug" / "drone-H743.elf"
    elf.parent.mkdir(parents=True)
    elf.write_bytes(b"fresh")
    seen: list[str] = []
    monkeypatch.setattr(
        subprocess, "run", lambda *a, **k: FakeCompleted(0, "ninja: no work to do\n"))

    result = rom_dfu.build_firmware(tmp_path, on_output=seen.append)

    assert result == elf
    assert any("cmake --build --preset Debug" in line for line in seen)
    assert any("ninja" in line for line in seen)


# --------------------------------------------------------------------------
# 陈旧检测
# --------------------------------------------------------------------------

def test_image_older_than_a_source_file_is_reported_stale(tmp_path) -> None:
    (tmp_path / "App" / "Src").mkdir(parents=True)
    source = tmp_path / "App" / "Src" / "app_control.c"
    source.write_text("int main(void){return 0;}", encoding="utf-8")
    elf = tmp_path / "drone-H743.elf"
    elf.write_bytes(b"elf")
    import os
    os.utime(elf, (1_000_000, 1_000_000))
    os.utime(source, (1_000_600, 1_000_600))

    stale, reason = rom_dfu.firmware_image_staleness(elf, tmp_path)

    assert stale
    assert "app_control.c" in reason


def test_image_newer_than_every_source_is_not_stale(tmp_path) -> None:
    (tmp_path / "App" / "Src").mkdir(parents=True)
    source = tmp_path / "App" / "Src" / "app_control.c"
    source.write_text("int main(void){return 0;}", encoding="utf-8")
    elf = tmp_path / "drone-H743.elf"
    elf.write_bytes(b"elf")
    import os
    os.utime(source, (1_000_000, 1_000_000))
    os.utime(elf, (1_000_600, 1_000_600))

    stale, _reason = rom_dfu.firmware_image_staleness(elf, tmp_path)

    assert not stale


def test_missing_image_counts_as_stale(tmp_path) -> None:
    stale, reason = rom_dfu.firmware_image_staleness(tmp_path / "nope.elf", tmp_path)
    assert stale
    assert "不存在" in reason


# --------------------------------------------------------------------------
# 面板接线
# --------------------------------------------------------------------------

def test_rebuild_is_on_by_default() -> None:
    init = function_body(SOURCE, "    def __init__(self) -> None:")
    assert "self.firmware_rebuild_var = tk.BooleanVar(value=True)" in init


def test_start_update_builds_before_touching_the_target() -> None:
    """编译必须发生在安全门之后、发送 BOOT 之前。"""
    start = function_body(SOURCE, "    def _firmware_start_update(self)")
    assert "_firmware_build_worker" in start
    assert "firmware_rebuild_var.get()" in start
    # 走编译分支时必须直接 return，不能继续往下发 BOOT。
    assert start.rstrip().endswith("self._firmware_start_update_after_build()")
    assert FIRMWARE_BOOT not in start


FIRMWARE_BOOT = "FIRMWARE_BOOT_COMMAND"


def test_build_failure_aborts_the_flash_and_leaves_the_target_untouched() -> None:
    drain = function_body(SOURCE, "    def _firmware_drain_events(")
    assert "build_failed" in drain
    failed = drain[drain.index('elif kind == "build_failed"'):]
    failed = failed[: failed.index('elif kind ==', 10)]
    assert "_firmware_start_update_after_build" not in failed
    assert "showerror" in failed


def test_build_success_reflashes_the_freshly_built_artifact() -> None:
    drain = function_body(SOURCE, "    def _firmware_drain_events(")
    ok = drain[drain.index('elif kind == "build_ok"'):]
    ok = ok[: ok.index('elif kind ==', 10)]
    # 必须把烧录对象切到新产物，否则仍可能烧到用户手填的旧路径。
    assert "self.firmware_image_var.set(str(payload))" in ok
    assert "_firmware_start_update_after_build()" in ok


def test_flash_button_is_disabled_while_building() -> None:
    refresh = function_body(SOURCE, "    def _firmware_refresh_safety(")
    assert "self.firmware_building" in refresh


def test_polling_keeps_running_during_the_build_so_the_gate_stays_fresh() -> None:
    """安全门要求 1.5s 内的新鲜快照；编译期间必须继续轮询，否则编译完就过期。"""
    poll = function_body(SOURCE, "    def _imu_poll_tick(")
    keepalive = function_body(SOURCE, "    def _link_keepalive_suppressed_reason(")
    assert "firmware_building" not in poll
    assert "firmware_building" not in keepalive

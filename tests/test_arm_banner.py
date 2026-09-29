"""Home-page arm banner answers whether arming is possible and what is missing.

主页面解锁横幅的契约。

它要回答的只有一个问题：**现在能不能解锁，不能的话差什么。**

在这之前飞控表达这件事的唯一途径是 LED_3 闪几下——要数、要查表、室外看不清。
而且原因链是有序的，只报第一条不满足的；同时缺两样时（比如既没插遥控器又没写
机体模型），修完一个仍然解不了锁，很容易以为没修对。所以横幅必须同时给出
"飞控说的原因"和"每一项条件的通过与否"。

三条不能松的性质：
  1. 原因来自飞控，不是上位机猜的；
  2. 飞控还没跑过一圈控制环时报"未知"，不是"没问题"；
  3. 数据过期时明说过期，不许让上一帧的"可以解锁"一直挂着。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from tools import panel_qa
from tools.panel_lib.arm_banner import BLOCK_TEXT, CONDITIONS, handle_arm_rsp


ROOT = Path(__file__).resolve().parents[1]
ARM_CMD = ROOT / "App" / "Src" / "app_cmd_arm.c"
LED_HEADER = ROOT / "App" / "Inc" / "app_led.h"


@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    root = tmp_path_factory.mktemp("arm-banner")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1366, 768))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise
            pytest.skip(f"Tk display unavailable: {exc}")
        try:
            yield session
        finally:
            session.destroy()


def _payload(**overrides) -> dict[str, str]:
    base = {
        "armed": "0", "block": "none", "blinks": "0", "known": "1",
        "rc_seen": "1", "rc_ok": "1", "switch": "1", "throttle_low": "1",
        "imu": "1", "imu_health": "1", "frame": "1", "airframe": "1",
        "servo_cal_idle": "1", "accept_idle": "1",
        "airframe_missing": "-", "t_ms": "1234",
    }
    base.update({k: str(v) for k, v in overrides.items()})
    return base


def test_banner_reports_the_firmware_block_reason_not_a_local_guess(offline) -> None:
    panel = offline.panel
    banner = panel.arm_banner

    handle_arm_rsp(panel, _payload(block="airframe", airframe="0",
                                   airframe_missing="airframe.izz_kgm2"))
    assert banner.state_var.get() == "未解锁"
    reason = banner.reason_var.get()
    assert "机体模型" in reason
    # 缺哪一项由飞控具名给出；只说"模型无效"等于让人从头查一遍。
    assert "airframe.izz_kgm2" in reason
    assert panel.arm_status["airframe_missing"] == "airframe.izz_kgm2"


def test_every_condition_shows_pass_or_fail_not_just_the_first_one(offline) -> None:
    """同时缺两样时必须两样都看得见。

    飞控的 block 只报第一条（rc_seen 排在 airframe 前面），如果横幅只显示它，
    用户插好遥控器之后会发现还是解不了锁，而界面刚才什么都没提示过机体模型。
    """
    panel = offline.panel
    banner = panel.arm_banner

    handle_arm_rsp(panel, _payload(block="no_rc", rc_seen="0", rc_ok="0",
                                   airframe="0",
                                   airframe_missing="airframe.mass_kg"))
    marks = {key: banner.condition_vars[key].get() for key, _ in CONDITIONS}
    assert marks["rc_seen"].startswith("✗")
    assert marks["airframe"].startswith("✗")
    assert marks["imu_health"].startswith("✓")


def test_unknown_is_not_reported_as_ok(offline) -> None:
    """控制环还没跑过一圈 = 还不知道，不是没问题。

    报成"可以解锁"会让人以为闸门已经全过了，而实际上一条都还没判过。
    """
    panel = offline.panel
    banner = panel.arm_banner

    handle_arm_rsp(panel, _payload(block="unknown", known="0"))
    assert banner.state_var.get() == "未解锁"
    assert "未知" in banner.reason_var.get()


def test_armed_state_says_the_props_will_spin(offline) -> None:
    panel = offline.panel
    banner = panel.arm_banner

    handle_arm_rsp(panel, _payload(armed="1"))
    assert banner.state_var.get() == "已解锁"
    assert "桨会转" in banner.reason_var.get()


def test_block_vocabulary_covers_every_firmware_reason() -> None:
    """固件新增一个解锁被拒原因，横幅必须同步认识它。

    少一条的后果不是"没有提示"，而是界面把原因码原样显示成一个英文 slug，
    在现场等于没说——而这条报文存在的全部理由就是别让人去数闪灯。
    """
    source = ARM_CMD.read_text(encoding="utf-8")
    slugs = set(re.findall(r'return "([a-z_]+)";', source))
    assert slugs <= set(BLOCK_TEXT), sorted(slugs - set(BLOCK_TEXT))

    # 固件枚举里每一个 APP_LED_ARM_BLOCK_* 都必须在 C 侧有对应分支。
    header = LED_HEADER.read_text(encoding="utf-8")
    enum_names = set(re.findall(r"(APP_LED_ARM_BLOCK_[A-Z_]+)\s*=", header))
    for name in enum_names:
        assert f"case {name}:" in source, name


def test_condition_keys_are_the_keys_the_firmware_actually_sends() -> None:
    """横幅的条件清单必须与 ARM 报文的键名逐一对上。

    拼错一个键不会报错，只会让那一项永远显示成"·"（未知），看起来像飞控没给，
    而其实是上位机在找一个不存在的名字。
    """
    source = ARM_CMD.read_text(encoding="utf-8")
    fmt = source.split('"RSP id=0 mod=ARM op=STATUS ', 1)[1].split('\\r\\n"', 1)[0]
    sent = set(re.findall(r"([a-z_]+)=%", fmt.replace('"\n        "', "")))
    for key, _label in CONDITIONS:
        assert key in sent, key

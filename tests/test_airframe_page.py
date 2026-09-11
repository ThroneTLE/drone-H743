"""机体模型页与其预览计算的契约。

最要紧的一条在最前面：上位机那份派生值预览必须和飞控算的是**同一件事**。

预览之所以存在，是因为手动派生档允许派生值和部件表对不上（惯量可能来自双线摆
实测而不是部件表推算），所以界面要把"若切到自动会是多少"并排摆出来。仓库上一版
正是死在这种对不上——四个部件质量加起来 754.6 g，整机质量却写 1.3670 kg，两套数
各喂各的公式，谁也没报错。如果预览自己再算错一遍，它就从"发现矛盾的工具"变成
"又一个会骗人的数字"，比没有更糟。

所以这里用**真实的 C 源码**跑一遍，逐项对数。
"""

from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path

import pytest

from tools.panel_lib.airframe_model import (
    AIRFRAME_FIELDS,
    DERIVED_AUTO_FIELD,
    INPUT_FIELDS,
    REQUIRED_NONZERO,
    compute_derived,
    first_missing,
)


ROOT = Path(__file__).resolve().parents[1]
AIRFRAME_SOURCE = ROOT / "Driver" / "Src" / "drv_airframe_params.c"

# 驱动 C 端的输入顺序；Python 侧按同一顺序喂值。
_INPUT_KEYS = tuple(field.key for field in INPUT_FIELDS)
_DERIVED_KEYS = ("mass_kg", "cg_z_m", "weight_n", "thrust_point_to_cg_z_m",
                 "tether_attach_to_cg_m", "tether_rod_to_cg_m",
                 "max_total_force_n", "hover_thrust_percent", "servo_us_per_deg")


def _harness_source() -> str:
    reads = "\n".join(
        f'    if (scanf("%f", &p.{key}) != 1) {{ return 2; }}' for key in _INPUT_KEYS
    )
    prints = "\n".join(
        f'    printf("%.9g\\n", (double)out.{key});' for key in _DERIVED_KEYS
    )
    return (
        '#include "drv_airframe_params.h"\n'
        "#include <stdio.h>\n"
        "#include <string.h>\n"
        "\n"
        "int main(void)\n"
        "{\n"
        "    DRV_Airframe_Params p;\n"
        "    DRV_Airframe_Params out;\n"
        "    memset(&p, 0, sizeof(p));\n"
        f"{reads}\n"
        "    DRV_Airframe_ComputeDerived(&p, &out);\n"
        f"{prints}\n"
        "    return 0;\n"
        "}\n"
    )


@pytest.fixture(scope="module")
def firmware_derive(tmp_path_factory):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    work = tmp_path_factory.mktemp("airframe-derive")
    harness = work / "derive.c"
    harness.write_text(_harness_source(), encoding="ascii")
    exe = work / "derive.exe"
    subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{ROOT / 'Driver' / 'Inc'}", str(AIRFRAME_SOURCE), str(harness),
         "-lm", "-o", str(exe)],
        check=True, capture_output=True, text=True,
    )

    def run(values: dict[str, float]) -> dict[str, float]:
        stdin = "\n".join(f"{float(values.get(key, 0.0)):.9g}" for key in _INPUT_KEYS)
        result = subprocess.run([str(exe)], input=stdin, capture_output=True,
                                text=True, check=True)
        numbers = [float(line) for line in result.stdout.split()]
        assert len(numbers) == len(_DERIVED_KEYS)
        return dict(zip(_DERIVED_KEYS, numbers))

    return run


# 三组输入：仓库历史实测值、全零（刚烧完固件的板子）、以及一组把符号全反过来的
# 假想构型（推力点在重心上方）。第三组不是凑数——r_z 的符号决定倾转力矩的方向，
# 预览要是在这一组上和固件分道扬镳，界面就会对一架"装反了"的飞机说一切正常。
_CASES = (
    {
        "board_mass_g": 75.0, "battery_mass_g": 232.0, "base_mass_g": 99.0,
        "servo_motor_mass_g": 348.6, "battery_cg_z_m": 0.109,
        "base_cg_z_m": -0.117, "servo_motor_cg_z_m": -0.244,
        "thrust_point_z_m": -0.2955, "tether_attach_z_m": 0.1563,
        "tether_rope_m": 0.64, "gravity_m_s2": 9.81,
        "max_total_thrust_g": 1595.342, "servo_deg_per_us": 0.09,
        "ixx_kgm2": 0.051, "iyy_kgm2": 0.051, "izz_kgm2": 0.005,
        "pitch_thrust_lever_arm_m": 0.145, "roll_thrust_lever_arm_m": 0.145,
        "lower_rotor_spin_sense": -1.0,
    },
    {},
    {
        "board_mass_g": 10.0, "battery_mass_g": 400.0, "base_mass_g": 50.0,
        "servo_motor_mass_g": 300.0, "board_cg_z_m": 0.02,
        "battery_cg_z_m": -0.05, "base_cg_z_m": 0.11,
        "servo_motor_cg_z_m": 0.30, "thrust_point_z_m": 0.35,
        "tether_attach_z_m": -0.20, "tether_rope_m": 1.25,
        "gravity_m_s2": 9.78, "max_total_thrust_g": 900.0,
        "servo_deg_per_us": 0.12, "lower_rotor_spin_sense": 1.0,
    },
)


@pytest.mark.parametrize("values", _CASES)
def test_host_preview_matches_the_firmware_derivation(values, firmware_derive) -> None:
    expected = firmware_derive(values)
    actual = compute_derived(values)
    for key in _DERIVED_KEYS:
        assert actual[key] == pytest.approx(expected[key], rel=1e-6, abs=1e-9), key


def test_empty_model_preview_is_zero_not_nan() -> None:
    """部件表为空时不许算出 NaN。

    Σ(m·z)/Σm 在 0/0 处是 NaN，而 NaN 会沿着后面每一条运算扩散，最后在某个
    毫不相干的字段上显示成 'nan'，根本追不回这里。固件那边同样特判，本条与它同源。
    """
    derived = compute_derived({})
    for key, value in derived.items():
        assert not math.isnan(value), key
        assert value == 0.0, key


def test_required_fields_mirror_the_firmware_arm_gate() -> None:
    """本地"缺哪一项"的判据必须与固件的解锁闸门同一份清单。

    上位机那份只在还没连上飞控时兜底；两边不一致的后果是页面说没问题、
    飞机不肯解锁，而那种矛盾最难查——人会以为是链路问题。
    """
    table = AIRFRAME_SOURCE.read_text(encoding="utf-8")
    block = table.split("airframe_check_nonzero[] = {", 1)[1].split("};", 1)[0]
    firmware = {line.strip().strip('",').split(".", 1)[1]
                for line in block.splitlines() if '"airframe.' in line}
    assert set(REQUIRED_NONZERO) == firmware


def test_first_missing_reports_a_name_and_rejects_nan() -> None:
    assert first_missing({}) is not None
    good = {key: 1.0 for key in REQUIRED_NONZERO}
    assert first_missing(good) is None
    # NaN 必须算不合格：它比零危险得多，零至少一眼看得出来。
    assert first_missing({**good, "izz_kgm2": float("nan")}) == "izz_kgm2"


def test_hand_measurable_and_estimated_fields_are_kept_apart() -> None:
    """惯量和旋向不是量出来的，不能和"电池多重"摆在同一层。

    这不是排版偏好：把它们混在一起会让人以为同样可信，随手就改了。而下桨旋向
    单独决定偏航力矩极性，翻错的表现是正反馈——和增益调大很像，很容易被误诊。
    """
    tiers = {field.key: field.tier for field in AIRFRAME_FIELDS}
    for estimated in ("ixx_kgm2", "iyy_kgm2", "izz_kgm2",
                      "lower_rotor_spin_sense",
                      "pitch_thrust_lever_arm_m", "roll_thrust_lever_arm_m"):
        assert tiers[estimated] == "advanced", estimated
    for measurable in ("battery_mass_g", "base_cg_z_m", "thrust_point_z_m",
                       "tether_rope_m", "max_total_thrust_g"):
        assert tiers[measurable] == "basic", measurable

    # 溯源必须写在界面上，不能只写在代码注释里——改它的人看的是界面。
    by_key = {field.key: field for field in AIRFRAME_FIELDS}
    assert "反推" in by_key["lower_rotor_spin_sense"].note
    assert "拆桨" in by_key["lower_rotor_spin_sense"].note
    assert "未实测" in by_key["izz_kgm2"].note
    assert "辨识" in by_key["pitch_thrust_lever_arm_m"].note

    assert DERIVED_AUTO_FIELD.tier == "advanced"


# ── 页面行为：用真实面板，不用替身 ────────────────────────────────────────

@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    from tools import panel_qa

    root = tmp_path_factory.mktemp("airframe-page")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1600, 900))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise
            pytest.skip(f"Tk display unavailable: {exc}")
        try:
            yield session
        finally:
            session.destroy()


def _feed(panel, values: dict[str, str]) -> None:
    for name, text in values.items():
        panel._update_param_line(f"PARAM name=airframe.{name} value={text}")


def test_auto_mode_never_sends_a_derived_write(offline) -> None:
    """自动档下派生框不许发出去。

    飞控会**当场拒绝**这类写入（那是刻意的：写进去再被下次重算盖掉，上位机会
    以为写成功了）。如果界面照发不误，用户看到的是一串没头没脑的 ERR，
    而真正改错的那条输入反而淹在里面。
    """
    panel = offline.panel
    page = panel.airframe_page
    offline.transport.is_connected = True

    _feed(panel, {"derived_auto": "1", "mass_kg": "0.7546", "base_mass_g": "99"})
    page._refresh_from_panel()
    assert str(page.entries["airframe.mass_kg"].cget("state")) == "disabled"

    offline.transport.frames.clear()
    page.vars["airframe.base_mass_g"].set("120")
    page.vars["airframe.mass_kg"].set("1.367")
    page._write_all()
    sent = [payload.decode() for _function, payload in offline.transport.frames]
    assert sent == ["PARAM SET airframe.base_mass_g 120"]


def test_manual_mode_opens_the_derived_fields(offline) -> None:
    """手动档是为"惯量来自双线摆实测"这类场景留的口子，必须真的能写。"""
    panel = offline.panel
    page = panel.airframe_page
    offline.transport.is_connected = True

    _feed(panel, {"derived_auto": "0"})
    page._refresh_from_panel()
    assert str(page.entries["airframe.mass_kg"].cget("state")) == "normal"

    offline.transport.frames.clear()
    page.vars["airframe.mass_kg"].set("1.367")
    page._write_all()
    sent = [payload.decode() for _function, payload in offline.transport.frames]
    assert "PARAM SET airframe.mass_kg 1.367" in sent


def test_status_follows_the_firmware_arm_report_not_the_local_guess(offline) -> None:
    """飞控说了算。

    两边都能算"模型有没有效"，但只有飞控那份决定能不能解锁。本地那份只在还没
    连上时兜底；一旦收到 ARM 报文就必须改用它，否则会出现"页面说没问题、
    飞机不肯解锁"——那种矛盾最难查，人会以为是链路问题。
    """
    from tools.panel_lib.arm_banner import handle_arm_rsp

    panel = offline.panel
    page = panel.airframe_page

    handle_arm_rsp(panel, {"armed": "0", "block": "airframe", "known": "1",
                           "airframe_missing": "airframe.izz_kgm2"})
    page._refresh_from_panel()
    assert "airframe.izz_kgm2" in page.status_var.get()

    handle_arm_rsp(panel, {"armed": "0", "block": "arm_switch", "known": "1",
                           "airframe_missing": "-"})
    page._refresh_from_panel()
    assert "模型有效" in page.status_var.get()


def test_advanced_fields_stay_locked_until_explicitly_unlocked(offline) -> None:
    """高级层默认没有输入框：估计值和反推值不该和"电池多重"一样随手可改。"""
    panel = offline.panel
    page = panel.airframe_page

    if not page.advanced_unlocked:
        assert "airframe.izz_kgm2" not in page.entries
        assert "airframe.lower_rotor_spin_sense" not in page.entries
        assert "airframe.battery_mass_g" in page.entries


def test_a_failed_flash_save_is_reported_not_swallowed(offline) -> None:
    """保存失败必须说出来。

    机体模型不写进 Flash 就等于没写：下次上电飞控依旧拒绝解锁。而参数 Flash 在
    MicoAir743V2 上**当前是坏的**（板上没有 GD25Q32，存储后端还没迁到片内 Flash），
    `SAVE` 回 `st=4`。按钮要是看着总是成功，用户会以为存好了，等到下次上电解不了
    锁才发现——那时人已经在外面了。
    """
    panel = offline.panel
    page = panel.airframe_page
    offline.transport.is_connected = True

    page._save()
    assert page._save_pending
    panel._update_config_result_line("OK save st=4")
    page._refresh_save_result()
    assert "保存失败" in page.detail_var.get()
    assert "st=4" in page.detail_var.get()

    page._save()
    panel._update_config_result_line("OK save st=0")
    page._refresh_save_result()
    assert "已保存到 Flash" in page.detail_var.get()

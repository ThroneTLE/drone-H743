"""Airframe page preview computes the same derived values as the flight controller.

机体模型页与其预览计算的契约。

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
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tools.panel_lib.airframe_model import (
    AIRFRAME_FIELDS,
    DERIVED_AUTO_FIELD,
    INPUT_FIELDS,
    REQUIRED_NONZERO,
    TILT_AXIS_MIN_LEVER_M,
    compute_derived,
    describe_invalid,
    first_missing,
    tilt_axis_to_cg_z,
)


ROOT = Path(__file__).resolve().parents[1]
AIRFRAME_SOURCE = ROOT / "Driver" / "Src" / "drv_airframe_params.c"

# 驱动 C 端的输入顺序；Python 侧按同一顺序喂值。
_INPUT_KEYS = tuple(field.key for field in INPUT_FIELDS)
_DERIVED_KEYS = ("mass_kg", "cg_z_m", "weight_n", "thrust_point_to_cg_z_m",
                 "tether_attach_to_cg_m", "tether_rod_to_cg_m",
                 "max_total_force_n", "hover_thrust_percent", "servo_us_per_deg")


# 倾转力臂 r_z 不是飞控字段（机体模型是 Flash ABI，加不了字段），飞控用两个函数
# 现算。harness 在派生字段之后再打印这两个函数的结果，上位机预览与它逐项对数——
# 比的是真函数，不是假装有一个 r_z 字段。
_TILT_FUNCS = (("roll", "DRV_Airframe_RollTiltAxisToCgZ"),
               ("pitch", "DRV_Airframe_PitchTiltAxisToCgZ"))


def _harness_source() -> str:
    reads = "\n".join(
        f'    if (scanf("%f", &p.{key}) != 1) {{ return 2; }}' for key in _INPUT_KEYS
    )
    prints = "\n".join(
        f'    printf("%.9g\\n", (double)out.{key});' for key in _DERIVED_KEYS
    )
    tilt_prints = "\n".join(
        f'    printf("%.9g\\n", (double){func}(&out));' for _axis, func in _TILT_FUNCS
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
        "    const char *bad;\n"
        "    memset(&p, 0, sizeof(p));\n"
        f"{reads}\n"
        "    DRV_Airframe_ComputeDerived(&p, &out);\n"
        f"{prints}\n"
        f"{tilt_prints}\n"
        # 解锁闸门：按自动派生档装进去，打印第一个不合格项（没有则 "-"）。
        "    p.derived_auto = 1.0f;\n"
        "    DRV_Airframe_SetParams(&p);\n"
        "    bad = DRV_Airframe_FirstInvalidName();\n"
        '    printf("%s\\n", (bad != NULL) ? bad : "-");\n'
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

    def run_all(values: dict[str, float]) -> tuple[dict[str, float], dict[str, float], str]:
        stdin = "\n".join(f"{float(values.get(key, 0.0)):.9g}" for key in _INPUT_KEYS)
        result = subprocess.run([str(exe)], input=stdin, capture_output=True,
                                text=True, check=True)
        lines = result.stdout.split()
        assert len(lines) == len(_DERIVED_KEYS) + len(_TILT_FUNCS) + 1
        numbers = [float(line) for line in lines[:-1]]
        derived = dict(zip(_DERIVED_KEYS, numbers))
        tilt = dict(zip((axis for axis, _func in _TILT_FUNCS), numbers[len(_DERIVED_KEYS):]))
        return derived, tilt, lines[-1]

    def run(values: dict[str, float]) -> dict[str, float]:
        return run_all(values)[0]

    run.all = run_all  # type: ignore[attr-defined]
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
        # 2026-09-27 实测：两个倾转舵机转轴都在 z = −0.13 m。
        "servo1_axis_z_m": -0.13, "servo2_axis_z_m": -0.13,
    },
    {},
    {
        "board_mass_g": 10.0, "battery_mass_g": 400.0, "base_mass_g": 50.0,
        "servo_motor_mass_g": 300.0, "board_cg_z_m": 0.02,
        "battery_cg_z_m": -0.05, "base_cg_z_m": 0.11,
        "servo_motor_cg_z_m": 0.30, "thrust_point_z_m": 0.35,
        "tether_attach_z_m": -0.20, "tether_rope_m": 1.25,
        "gravity_m_s2": 9.78, "max_total_thrust_g": 900.0,
        "servo_deg_per_us": 0.12, "servo1_axis_z_m": 0.28, "servo2_axis_z_m": 0.31,
    },
)


@pytest.mark.parametrize("values", _CASES)
def test_host_preview_matches_the_firmware_derivation(values, firmware_derive) -> None:
    expected = firmware_derive(values)
    actual = compute_derived(values)
    for key in _DERIVED_KEYS:
        assert actual[key] == pytest.approx(expected[key], rel=1e-6, abs=1e-9), key


@pytest.mark.parametrize("values", _CASES)
def test_host_tilt_lever_preview_matches_the_firmware_functions(values, firmware_derive) -> None:
    """每轴倾转力臂 r_z：上位机预览与飞控 DRV_Airframe_Roll/PitchTiltAxisToCgZ 同源。

    重心取**飞控派生出来的那一个**——页面就是这么标注的。r_z 不是飞控字段，
    所以这里比的是飞控真正调用的那两个函数，而不是一个假想的参数。
    """
    derived, firmware_tilt, _gate = firmware_derive.all(values)
    # harness 把没给的输入按 0 喂进飞控（"全零 = 还没写过"）；预览遇到缺数会给
    # None 而不是编 0，所以这里显式补同样的 0，比的是同一组输入。
    zeros = {key: 0.0 for key in _INPUT_KEYS}
    host = tilt_axis_to_cg_z({**zeros, **values, "cg_z_m": derived["cg_z_m"]})
    for axis, value in firmware_tilt.items():
        assert host[axis] == pytest.approx(value, rel=1e-6, abs=1e-9), axis


# 解锁闸门镜像：同一组输入，上位机 first_missing 与飞控 FirstInvalidName 必须点名
# 同一项。基准是 2026-09-27 实测几何（上面 _CASES[0]），逐条改坏一处。
_GATE_CASES = (
    ({}, None),
    ({"servo1_axis_z_m": 0.0}, "servo1_axis_z_m"),          # 没填：r_z 会变成 +0.0946
    ({"servo2_axis_z_m": 0.0}, "servo2_axis_z_m"),
    ({"servo1_axis_z_m": -0.100}, "servo1_axis_z_m:near"),  # |r_z| ≈ 5 mm
    ({"servo2_axis_z_m": -0.088}, "servo2_axis_z_m:near"),  # 重心上方 6.6 mm，同样太近
    ({"servo1_axis_z_m": 0.13}, "servo1_axis_z_m:sign"),    # "板下 13 cm"填成 +0.13
    ({"servo2_axis_z_m": 0.13}, "servo2_axis_z_m:sign"),
    ({"thrust_point_z_m": 0.2955}, "servo1_axis_z_m:sign"), # 推力点符号填反，同样拦住
)


@pytest.mark.parametrize(("change", "expected"), _GATE_CASES)
def test_host_gate_mirror_names_the_same_item_as_the_firmware(
        change, expected, firmware_derive) -> None:
    values = {**_CASES[0], **change}
    _derived, _tilt, firmware = firmware_derive.all(values)
    assert firmware == ("-" if expected is None else f"airframe.{expected}")
    assert first_missing({**values, **compute_derived(values)}) == expected


def test_tilt_axis_threshold_mirrors_the_firmware_constant() -> None:
    header = (ROOT / "Driver" / "Inc" / "drv_airframe_params.h").read_text(encoding="utf-8")
    match = re.search(r"#define DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M ([0-9.]+)f", header)
    assert match is not None
    assert float(match.group(1)) == TILT_AXIS_MIN_LEVER_M


def test_compound_gate_names_are_explained_not_reported_as_missing() -> None:
    """"缺 airframe.servo1_axis_z_m:sign" 会让人以为没填——其实是填反了。"""
    assert describe_invalid("airframe.izz_kgm2") == "缺 airframe.izz_kgm2"
    sign = describe_invalid("airframe.servo1_axis_z_m:sign")
    assert "airframe.servo1_axis_z_m" in sign and "同侧" in sign and "缺" not in sign
    assert "1 cm" in describe_invalid("airframe.servo2_axis_z_m:near")


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
    """惯量不是量出来的，不能和"电池多重"摆在同一层。

    这不是排版偏好：把它们混在一起会让人以为同样可信，随手就改了。

    旋向 2026-09-13 起不在本页——它是量出来的接线事实，归「桨叶与电机方向」页
    （PROPCAL），由通电看一眼确定，不再是一个能随手改的浮点数。
    """
    tiers = {field.key: field.tier for field in AIRFRAME_FIELDS}
    for estimated in ("ixx_kgm2", "iyy_kgm2", "izz_kgm2"):
        assert tiers[estimated] == "advanced", estimated
    for measurable in ("battery_mass_g", "base_cg_z_m", "thrust_point_z_m",
                       "tether_rope_m", "max_total_thrust_g",
                       "servo1_axis_z_m", "servo2_axis_z_m"):
        assert tiers[measurable] == "basic", measurable

    # 溯源必须写在界面上，不能只写在代码注释里——改它的人看的是界面。
    by_key = {field.key: field for field in AIRFRAME_FIELDS}
    # 旋向已经不在本页；留一条反向断言，免得哪天有人"顺手补回来"——补回来就
    # 又有两个来源了，而它们打架时谁也不会报错。
    assert "lower_rotor_spin_sense" not in by_key
    assert "未实测" in by_key["izz_kgm2"].note

    # 2026-09-27：两个"有效力臂"退役（旧机体上的辨识值，机体换了没人更新），
    # 力臂改由舵机转轴高度减重心算出。界面上不许再有它们的输入框；转轴字段
    # 必须把"它决定力矩、填错不许解锁、板下填负数"写在界面上。
    for retired in ("pitch_thrust_lever_arm_m", "roll_thrust_lever_arm_m"):
        assert retired not in by_key, retired
    for key in ("servo1_axis_z_m", "servo2_axis_z_m"):
        note = by_key[key].note
        assert "重心" in note and "力臂" in note and "禁止解锁" in note, key
        assert "负数" in note, key

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


def test_tilt_lever_preview_uses_the_firmware_cg(offline) -> None:
    """r_z 预览用飞控派生的重心，并且说清楚它是上位机算的、重心从哪来。"""
    panel = offline.panel
    page = panel.airframe_page

    _feed(panel, {"cg_z_m": "-0.094558", "servo1_axis_z_m": "-0.13",
                  "servo2_axis_z_m": "-0.13"})
    page._refresh_from_panel()
    assert page.tilt_lever_labels["roll"].cget("text") == "-0.035442"
    assert page.tilt_lever_labels["pitch"].cget("text") == "-0.035442"
    assert "飞控派生" in page.tilt_lever_source_var.get()


def test_advanced_fields_stay_locked_until_explicitly_unlocked(offline) -> None:
    """高级层默认没有输入框：估计值和反推值不该和"电池多重"一样随手可改。"""
    panel = offline.panel
    page = panel.airframe_page

    if not page.advanced_unlocked:
        assert "airframe.izz_kgm2" not in page.entries
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


# ── 倾转力臂会变：写入前说清楚旧 → 新，以及速率环增益软硬的变化 ─────────────────

# 2026-09-27 板上的部件表（重心 −0.094558）与实测的舵机转轴 −0.13 → 力臂 0.035442 m。
_TONIGHT = {"derived_auto": "1", "board_mass_g": "75", "battery_mass_g": "232",
            "base_mass_g": "99", "servo_motor_mass_g": "348.6", "board_cg_z_m": "0",
            "battery_cg_z_m": "0.109", "base_cg_z_m": "-0.117", "servo_motor_cg_z_m": "-0.244",
            "cg_z_m": "-0.094558", "servo1_axis_z_m": "-0.13", "servo2_axis_z_m": "-0.13",
            "imu_z_m": "0"}


def _reset_lever_inputs(page) -> None:
    """模块级面板在前面的用例里改过输入框：先把力臂相关的框摆回飞控当前值。"""
    for key, text in _TONIGHT.items():
        var = page.vars.get(f"airframe.{key}")
        if var is not None:
            var.set(text)


@pytest.fixture
def tonight(offline, monkeypatch):
    panel = offline.panel
    page = panel.airframe_page
    offline.transport.is_connected = True
    _feed(panel, _TONIGHT)
    _reset_lever_inputs(page)
    asked: list[str] = []
    answer = {"ok": False}
    monkeypatch.setattr(page, "_confirm_lever_change",
                        lambda text: asked.append(text) or answer["ok"])
    offline.transport.frames.clear()
    yield page, offline, asked, answer
    _reset_lever_inputs(page)


def _sent(offline) -> list[str]:
    return [payload.decode() for _function, payload in offline.transport.frames]


def test_a_servo_axis_edit_shows_the_lever_change_and_can_be_cancelled(tonight) -> None:
    page, offline, asked, _answer = tonight
    page.vars["airframe.servo2_axis_z_m"].set("-0.2")
    page._refresh_from_panel()
    preview = page.tilt_lever_change_var.get()
    assert "俯仰（舵机 2）倾转力臂 L = 重心 − 转轴：0.0354 m → 0.1054 m" in preview
    assert "L_旧/L_新 = 0.34 倍（变软）" in preview and "重新辨识" in preview
    assert "横滚" not in preview, "只改了舵机 2"
    page._write_all()
    assert len(asked) == 1 and "0.0354 m → 0.1054 m" in asked[0]
    assert "速率环增益（N·m 单位）的等效软硬会变为" in asked[0]
    assert _sent(offline) == [], "取消就什么都不发"
    assert "已取消写入" in page.detail_var.get()


def test_moving_the_cg_through_the_component_table_is_confirmed_then_sent(tonight) -> None:
    """自动派生档：电池往上挪让整机重心到 −0.01 → 两个力臂都 0.0354 → 0.1200 m。"""
    page, offline, asked, answer = tonight
    answer["ok"] = True
    page.vars["airframe.battery_cg_z_m"].set("0.384")
    page._write_all()
    assert len(asked) == 1
    for axis in ("横滚（舵机 1）", "俯仰（舵机 2）"):
        assert f"{axis}倾转力臂 L = 重心 − 转轴：0.0354 m → 0.1200 m" in asked[0]
    assert "0.30 倍（变软）" in asked[0]
    assert _sent(offline) == ["PARAM SET airframe.battery_cg_z_m 0.384"]
    assert "重新辨识" in page.detail_var.get()


def test_edits_that_do_not_move_the_lever_are_sent_without_asking(tonight) -> None:
    page, offline, asked, _answer = tonight
    page.vars["airframe.imu_z_m"].set("0.01")
    page._write_all()
    assert asked == [] and _sent(offline) == ["PARAM SET airframe.imu_z_m 0.01"]
    page._refresh_from_panel()
    assert page.tilt_lever_change_var.get() == ""


def test_a_sign_flip_is_called_out(tonight) -> None:
    page, _offline, asked, _answer = tonight
    page.vars["airframe.servo1_axis_z_m"].set("0.05")
    page._write_all()
    assert "方向反了" in asked[0] and "0.0354 m → -0.1446 m" in asked[0]


def test_filling_in_a_never_set_servo_axis_is_not_a_lever_change(tonight) -> None:
    """转轴 0 = 没填（解锁闸门拒绝）：从 0 填成实测值不算"力臂变了"，不弹窗。"""
    page, offline, asked, _answer = tonight
    _feed(offline.panel, {"servo1_axis_z_m": "0"})
    page.vars["airframe.servo1_axis_z_m"].set("-0.13")
    page._write_all()
    assert asked == [] and _sent(offline) == ["PARAM SET airframe.servo1_axis_z_m -0.13"]
    _feed(offline.panel, {"servo1_axis_z_m": "-0.13"})


# ── 切回自动派生：飞控当场按部件表重算重心，手动填的实测重心被盖掉 ─────────────────
# 场景：手动档填了实测重心 −0.01（力臂 0.12 m），部件表还没改（推算 −0.094558）。切回自动
# 那一下力臂变回 0.0354 m，同一组 N·m 增益硬 3.39 倍——必须先说清楚再发。


@pytest.fixture
def measured_cg(tonight):
    page, offline, asked, answer = tonight
    _feed(offline.panel, {"derived_auto": "0", "cg_z_m": "-0.01"})
    page.vars["airframe.cg_z_m"].set("-0.01")
    offline.transport.frames.clear()
    yield page, offline, asked, answer
    _feed(offline.panel, {"derived_auto": "1", "cg_z_m": "-0.094558"})


def test_toggling_back_to_auto_warns_that_the_cg_is_recomputed(measured_cg) -> None:
    page, offline, asked, answer = measured_cg
    page._toggle_derived_auto()
    assert len(asked) == 1
    assert "切回自动派生" in asked[0] and "-0.0100 m 重算成 -0.0946 m" in asked[0]
    for axis in ("横滚（舵机 1）", "俯仰（舵机 2）"):
        assert f"{axis}倾转力臂 L = 重心 − 转轴：0.1200 m → 0.0354 m" in asked[0]
    assert "L_旧/L_新 = 3.39 倍（变硬）" in asked[0] and "重新辨识" in asked[0]
    assert _sent(offline) == [], "取消就什么都不发"
    assert "已取消切换" in page.detail_var.get()

    answer["ok"] = True
    page._toggle_derived_auto()
    # 飞控只回显 derived_auto 这一项：重算出的重心要 PARAM? 读回来，力臂预览才不停在旧值。
    assert _sent(offline) == ["PARAM SET airframe.derived_auto 1", "PARAM?"]
    assert "重新辨识" in page.detail_var.get()


def test_toggling_to_manual_keeps_the_cg_and_does_not_ask(tonight) -> None:
    page, offline, asked, _answer = tonight
    page._toggle_derived_auto()
    assert asked == []
    assert _sent(offline) == ["PARAM SET airframe.derived_auto 0", "PARAM?"]
    _feed(offline.panel, {"derived_auto": "1"})


def test_the_advanced_row_switch_to_auto_is_warned_and_sent_last(measured_cg, monkeypatch) -> None:
    """高级区里改 derived_auto 走「写入飞控」：同样弹窗；和部件改动一起写时派生档最后发，
    预测的新重心按改过的部件表算（电池挪到 0.384 → 部件表重心 −0.01，力臂不变就不问）。"""
    from tools.panel_lib.pages import airframe

    page, offline, asked, answer = measured_cg
    if not page.advanced_unlocked:
        monkeypatch.setattr(airframe.messagebox, "askokcancel", lambda *a, **k: True)
        page._unlock_advanced()
    page.vars["airframe.cg_z_m"].set("-0.01")
    page.vars["airframe.derived_auto"].set("1")
    page._write_all()
    assert len(asked) == 1 and "0.1200 m → 0.0354 m" in asked[0]
    assert _sent(offline) == []

    asked.clear()
    answer["ok"] = True
    page.vars["airframe.battery_cg_z_m"].set("0.384")
    page._write_all()
    assert asked == [], "部件表改到与实测重心一致：切回自动后力臂不变"
    assert _sent(offline) == ["PARAM SET airframe.battery_cg_z_m 0.384",
                              "PARAM SET airframe.derived_auto 1", "PARAM?"]
    page.vars["airframe.derived_auto"].set("1")

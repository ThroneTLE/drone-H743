"""Firmware thrust lookup table against the host table, compiled with host gcc.

The flight controller must turn "thrust at this battery charge" into the same
throttle as tools/thrust_bench/thrust_lut.py, the table checked on the bench.
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.thrust_bench import thrust_lut, thrust_lut_export  # noqa: E402

TABLE_C = ROOT / "Driver" / "Src" / "drv_thrust_lut_table.inc"
DRIVER_C = ROOT / "Driver" / "Src" / "drv_thrust_lut.c"
APP_C = ROOT / "App" / "Src" / "app_thrust_lut.c"


def _source_lut() -> tuple[dict, Path]:
    match = re.search(r"来源 (\S+thrust_lut\.json)", TABLE_C.read_text(encoding="utf-8"))
    assert match, "generated table must name its source json"
    path = ROOT / match.group(1)
    if not path.is_file():
        pytest.skip(f"source table {path} is not in this checkout")
    return json.loads(path.read_text(encoding="utf-8")), path


def _gcc() -> str:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    return gcc


def _run(gcc: str, tmp_path: Path, sources: list[Path], includes: list[Path], main: str) -> list[str]:
    harness = tmp_path / "harness.c"
    harness.write_text(main, encoding="utf-8")
    exe = tmp_path / "harness.exe"
    subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-DBSP_ESC_PROTOCOL=2",
                    *[f"-I{path}" for path in includes], *map(str, sources), str(harness), "-lm", "-o", str(exe)],
                   check=True, capture_output=True, text=True)
    return subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.splitlines()


def test_table_current():
    """Firmware table, models/lut/current.json and its source json name the same table."""
    lut, path = _source_lut()
    expected = thrust_lut_export.render(lut, path.relative_to(ROOT).as_posix())
    assert TABLE_C.read_text(encoding="utf-8") == expected, "run python -m tools.thrust_bench.thrust_lut_export"
    active = thrust_lut.current(ROOT / "data" / "identification" / "thrust")
    assert active is not None and active[1].resolve() == path.resolve() and active[0]["model_id"] == lut["model_id"]


def test_reads_match_host(tmp_path):
    lut, _path = _source_lut()
    rng = np.random.default_rng(7)
    speed = [(u, l) for u, l in rng.uniform(0, 80000, (4000, 2))
             if thrust_lut._read(lut["speed"], u, l) is not None][:300]
    effective = [(u, l) for u, l in rng.uniform(0, 110, (4000, 2))
                 if thrust_lut._read(lut["throttle"], u, l) is not None][:300]
    diag_e, diag_t = thrust_lut_export.diagonal(lut)
    targets = list(np.linspace(1.0, diag_t[-1] - 1.0, 120))
    charges = [10.3, 10.75, 11.4, 11.8, 12.3]
    rows = lambda pairs: ",".join(f"{{{a:.6f}f,{b:.6f}f}}" for a, b in pairs)
    main = f"""#include <stdio.h>
#include "drv_thrust_lut.h"
static const float speed[][2] = {{{rows(speed)}}};
static const float eff[][2] = {{{rows(effective)}}};
static const float targets[] = {{{",".join(f"{t:.4f}f" for t in targets)}}};
static const float charges[] = {{{",".join(f"{v:.3f}f" for v in charges)}}};
int main(void) {{
    for (unsigned i = 0; i < sizeof speed / sizeof speed[0]; ++i)
        printf("S %.4f\\n", (double)DRV_ThrustLut_ThrustFromSpeed(speed[i][0], speed[i][1]));
    for (unsigned i = 0; i < sizeof eff / sizeof eff[0]; ++i)
        printf("E %.4f\\n", (double)DRV_ThrustLut_ThrustFromEffective(eff[i][0], eff[i][1]));
    for (unsigned c = 0; c < sizeof charges / sizeof charges[0]; ++c)
        for (unsigned i = 0; i < sizeof targets / sizeof targets[0]; ++i) {{
            const float e = DRV_ThrustLut_BalancedEffectiveForThrust(targets[i]);
            printf("P %.5f %.4f\\n", (double)DRV_ThrustLut_PercentForEffective(e, charges[c]),
                   (double)DRV_ThrustLut_BalancedThrustForEffective(e));
        }}
    printf("C %.5f %.5f %.5f\\n", (double)DRV_ThrustLut_ChargeVoltage(11.0f, 60000.0f, 40000.0f),
           (double)DRV_ThrustLut_PercentForEffective(50.0f, 0.0f), (double)DRV_ThrustLut_PercentForEffective(50.0f, 5.0f));
    return 0;
}}
"""
    lines = _run(_gcc(), tmp_path, [DRIVER_C], [ROOT / "Driver" / "Inc"], main)
    got = {kind: [list(map(float, line.split()[1:])) for line in lines if line[0] == kind] for kind in "SEPC"}
    for (u, l), (value,) in zip(speed, got["S"]):
        assert value == pytest.approx(max(0.0, thrust_lut._read(lut["speed"], u, l)), abs=0.02)
    for (u, l), (value,) in zip(effective, got["E"]):
        assert value == pytest.approx(max(0.0, thrust_lut._read(lut["throttle"], u, l)), abs=0.02)
    usable = thrust_lut_export.usable_charge(lut)
    for index, (percent, back) in enumerate(got["P"]):
        charge, target = charges[index // len(targets)], targets[index % len(targets)]
        assert back == pytest.approx(target, abs=0.05)  # forward(inverse(T)) == T on the diagonal
        host = thrust_lut.balanced_throttle({**lut, "domain": {**lut["domain"], "charge_v_usable": usable}},
                                            target, charge)
        if host is not None:  # same throttle as the bench-validated host function
            assert percent == pytest.approx(host, abs=0.1)
    charge, nominal, floor = got["C"][0]
    assert charge == pytest.approx(11.0 + 0.0035 * (6.0 ** 3 + 4.0 ** 3), abs=1e-4)
    assert nominal == pytest.approx(50.0) and floor == pytest.approx(50.0 * 12.0 / usable[0], abs=1e-3)


APP_STUBS = r"""
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "app_battery.h"
#include "bsp_dshot_rx.h"
#include "drv_coax_ctrl.h"
APP_BatterySnapshot fake_battery; BSP_DShotRxSnapshot fake_rx; uint32_t fake_now; uint8_t fake_armed;
static const DRV_COAX_CTRL_ThrustMap *installed;
void APP_Battery_GetSnapshot(APP_BatterySnapshot *out) { *out = fake_battery; }
void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out) { *out = fake_rx; }
uint32_t SVC_Timestamp_Ms(void) { return fake_now; }
uint8_t APP_Stabilizer_IsArmed(void) { return fake_armed; }
void DRV_COAX_CTRL_SetThrustMap(const DRV_COAX_CTRL_ThrustMap *map) { installed = map; }
const DRV_COAX_CTRL_ThrustMap *DRV_COAX_CTRL_GetThrustMap(void) { return installed; }
void APP_Control_QueueText(const char *format, ...) { va_list a; va_start(a, format); vprintf(format, a); va_end(a); }
uint8_t app_control_parse_f32(const char *text, float *value) { char *end; *value = strtof(text, &end); return *end == 0; }
"""


def test_app_charge_and_command(tmp_path):
    gcc = _gcc()
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "cmsis_os2.h").write_text("typedef void *osSemaphoreId_t; typedef void *osMessageQueueId_t;\n")
    (stub / "stubs.c").write_text(APP_STUBS, encoding="utf-8")
    main = r"""#include <stdio.h>
#include "app_thrust_lut.h"
#include "app_battery.h"
#include "bsp_dshot_rx.h"
#include "drv_coax_ctrl.h"
extern APP_BatterySnapshot fake_battery; extern BSP_DShotRxSnapshot fake_rx; extern uint32_t fake_now; extern uint8_t fake_armed;
uint8_t app_control_handle_thrust_lut(char **tokens, uint32_t count);
static void run(char *a, char *b, char *c, char *d, unsigned n) { char *t[4] = {a, b, c, d}; app_control_handle_thrust_lut(t, n); }
PAIRS_HERE
int main(void) {
    APP_ThrustLut_Init();
    printf("installed %d\n", DRV_COAX_CTRL_GetThrustMap() != 0);
    APP_ThrustLut_Step(); printf("charge %.4f\n", (double)APP_ThrustLut_ChargeVoltage());
    fake_battery.state.valid = 1; fake_battery.state.voltage_mv = 12300; fake_battery.age_ms = 5;
    fake_rx.available = 1; fake_rx.valid[0] = fake_rx.valid[1] = 1; fake_rx.not_spinning[0] = fake_rx.not_spinning[1] = 1;
    fake_now = 1000; fake_rx.sample_ms[0] = fake_rx.sample_ms[1] = 995;
    APP_ThrustLut_Step(); printf("charge %.4f\n", (double)APP_ThrustLut_ChargeVoltage());
    fake_battery.state.voltage_mv = 11000; fake_rx.not_spinning[0] = fake_rx.not_spinning[1] = 0;
    fake_rx.erpm[0] = fake_rx.erpm[1] = 60000; fake_now = 1020; fake_rx.sample_ms[0] = fake_rx.sample_ms[1] = 1015;
    APP_ThrustLut_Step(); printf("charge %.6f\n", (double)APP_ThrustLut_ChargeVoltage());
    fake_battery.state.voltage_mv = 9000; fake_now = 1400;  /* eRPM 385 ms old: hold, never read sag as discharge */
    APP_ThrustLut_Step(); printf("charge %.6f\n", (double)APP_ThrustLut_ChargeVoltage());
    printf("pulse %u %u\n", APP_ThrustLut_PulseForMotorThrust(600.0f / 2.0f / 101.971621f, 11.8f),
           APP_ThrustLut_PulseForMotorThrust(0.0f, 11.8f));
    run("THRUSTLUT", "?", 0, 0, 2); run("THRUSTLUT", "MAP", "600", "11.8", 4);
    { char *t[5] = {"THRUSTLUT", "PAIR", "487.5", "262.5", "11.8"}; app_control_handle_thrust_lut(t, 5); }
    for (unsigned i = 0; i < sizeof pairs / sizeof pairs[0]; ++i) {
        uint16_t u, l;
        const float shift = APP_ThrustLut_PulsesForPair(pairs[i][0] / 101.971621f, pairs[i][1] / 101.971621f,
                                                        pairs[i][2], &u, &l);
        printf("P %u %u %.5f\n", u, l, (double)shift);
    }
    fake_armed = 1; run("THRUSTLUT", "MODE", "LEGACY", 0, 3);
    fake_armed = 0; run("THRUSTLUT", "MODE", "LEGACY", 0, 3);
    printf("installed %d\n", DRV_COAX_CTRL_GetThrustMap() != 0);
    return 0;
}
"""
    lut, _path = _source_lut()
    rng = np.random.default_rng(11)
    pairs = [(float(t * (1 + r) / 2), float(t * (1 - r) / 2), float(v)) for t, r, v in
             zip(rng.uniform(150, 1500, 60), rng.uniform(-0.45, 0.45, 60), rng.uniform(10.8, 12.5, 60))]
    pairs += [(375.0, 375.0, 11.8), (0.0, 400.0, 11.8), (60.0, 1.0, 11.8)]
    main = main.replace("PAIRS_HERE", "static const float pairs[][3] = {"
                        + ",".join(f"{{{u:.4f}f,{l:.4f}f,{v:.4f}f}}" for u, l, v in pairs) + "};")
    lines = _run(gcc, tmp_path, [APP_C, DRIVER_C, stub / "stubs.c"],
                 [stub, ROOT / "App" / "Inc", ROOT / "Driver" / "Inc", ROOT / "BSP" / "Inc", ROOT / "Services" / "Inc"],
                 main)
    text = "\n".join(lines)
    charges = [float(line.split()[1]) for line in lines if line.startswith("charge")]
    raw = 11.0 + 0.0035 * 2 * 6.0 ** 3
    assert charges[0] == 0.0 and charges[1] == pytest.approx(12.3, abs=1e-4)
    assert charges[2] == pytest.approx(12.3 + (raw - 12.3) * 0.02 / 2.02, abs=1e-5) and charges[3] == charges[2]
    host = thrust_lut.balanced_throttle(lut, 600.0, 11.8)
    pulse, idle = (int(v) for v in re.search(r"pulse (\d+) (\d+)", text).groups())
    assert idle == 1100 and abs(pulse - (1100 + host * 8.4)) <= 1.0
    assert f"model={lut['model_id']}" in text and "mode=pair" in text and "state=holding" in text
    # Pair allocation: same throttles and shift as the host function the bench validates.
    diag = thrust_lut.balanced_diagonal(lut)
    got = [line.split()[1:] for line in lines if line.startswith("P ")]
    assert len(got) == len(pairs)
    for (upper, lower, charge), (u_us, l_us, shift) in zip(pairs, got):
        host_pair = thrust_lut.pair_throttle(lut, upper, lower, charge, diag=diag)
        assert float(shift) == pytest.approx(host_pair["shift_pct"], abs=2e-3)
        for us, pct in ((u_us, host_pair["upper_pct"]), (l_us, host_pair["lower_pct"])):
            assert abs(int(us) - (1100 + pct * 8.4)) <= 1.0
        before = max(host_pair["e_upper"], host_pair["e_lower"]) * 12.0 / charge
        if before >= 100.0:
            assert host_pair["shift_pct"] == 0.0  # a saturated rotor never pulls the other one down
        elif upper > 0 and lower > 0 and 0.0 < max(host_pair["upper_pct"], host_pair["lower_pct"]) < 100.0 \
                and abs(host_pair["shift_pct"]) < thrust_lut.PAIR_SHIFT_LIMIT_PCT - 1e-6 and host_pair["shift_pct"] != 0.0:
            total = thrust_lut._grid_read(lut["throttle"], host_pair["e_upper"] + host_pair["shift_pct"],
                                          host_pair["e_lower"] + host_pair["shift_pct"])
            assert total == pytest.approx(upper + lower, abs=0.05)  # the 2D table total equals the command
    assert sum(float(item[2]) != 0.0 for item in got) >= 40  # the correction is really exercised
    assert got[-3][2] == "0.00000" or abs(float(got[-3][2])) < 1e-3  # no split: identical to the per-rotor mapping
    assert re.search(r"event=pair upper_dg=4875 lower_dg=2625 charge_mv=11800 shift_x100=\d+ ", text)
    assert re.search(rf"event=map total_dg=6000 charge_mv=11800 .* pulse={pulse} ", text)
    assert "reason=armed" in text and "mode=legacy" in text and lines[0] == "installed 1" and lines[-1] == "installed 0"


def test_an_implausible_erpm_frame_is_skipped_not_folded_into_the_charge(tmp_path):
    """2026-09-28 杆上：错帧 eRPM 经三次方压降项把电量电压顶到 14.07 V（越出可用上限），
    光杆辨识以 thrust_stale 中止。超过陷波同一上限的帧整拍跳过：电量不变、仍新鲜、计数加 1；
    好帧回来照常更新；错帧持续超过 500 ms 才过期。"""
    gcc = _gcc()
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "cmsis_os2.h").write_text("typedef void *osSemaphoreId_t; typedef void *osMessageQueueId_t;\n")
    (stub / "stubs.c").write_text(APP_STUBS, encoding="utf-8")
    main = r"""#include <stdio.h>
#include "app_thrust_lut.h"
#include "app_battery.h"
#include "bsp_dshot_rx.h"
extern APP_BatterySnapshot fake_battery; extern BSP_DShotRxSnapshot fake_rx; extern uint32_t fake_now;
uint8_t app_control_handle_thrust_lut(char **tokens, uint32_t count);
static void step(uint32_t now, uint32_t upper, uint32_t lower) {
    fake_now = now; fake_rx.sample_ms[0] = fake_rx.sample_ms[1] = now - 1U;
    fake_rx.erpm[0] = upper; fake_rx.erpm[1] = lower; APP_ThrustLut_Step();
    printf("charge %.6f fresh %u\n", (double)APP_ThrustLut_ChargeVoltage(), APP_ThrustLut_IsFresh());
}
int main(void) {
    APP_ThrustLut_Init();
    fake_battery.state.valid = 1; fake_battery.state.voltage_mv = 11500; fake_battery.age_ms = 5;
    fake_rx.available = 1; fake_rx.valid[0] = fake_rx.valid[1] = 1;
    step(1000, 42856, 42372);
    step(1002, 42856, 42372);
    step(1004, 900000, 42372);     /* 错帧：上桨 */
    step(1006, 42856, 4000000);    /* 错帧：下桨 */
    step(1008, 150000, 42372);     /* 恰在上限：照常用 */
    step(1010, 42856, 42372);
    for (uint32_t t = 1012; t <= 1600; t += 2) { fake_now = t; fake_rx.sample_ms[0] = fake_rx.sample_ms[1] = t - 1U;
        fake_rx.erpm[0] = 900000; APP_ThrustLut_Step(); }
    printf("charge %.6f fresh %u\n", (double)APP_ThrustLut_ChargeVoltage(), APP_ThrustLut_IsFresh());
    { char *t[2] = {"THRUSTLUT", "?"}; app_control_handle_thrust_lut(t, 2); }
    return 0;
}
"""
    lines = _run(gcc, tmp_path, [APP_C, DRIVER_C, stub / "stubs.c"],
                 [stub, ROOT / "App" / "Inc", ROOT / "Driver" / "Inc", ROOT / "BSP" / "Inc", ROOT / "Services" / "Inc"],
                 main)
    rows = [(float(line.split()[1]), int(line.split()[3])) for line in lines if line.startswith("charge")]
    raw = 11.5 + 0.0035 * (4.2856 ** 3 + 4.2372 ** 3)
    assert rows[0] == (pytest.approx(raw, abs=1e-4), 1)
    settled = rows[1][0]
    assert rows[2] == (settled, 1) and rows[3] == (settled, 1), "错帧不进电量估计、也不判过期"
    assert rows[4][0] > settled, "上限以内（150000）仍按真转速算"
    assert rows[5][0] < 12.784 and rows[5][1] == 1
    assert rows[6][1] == 0, "错帧连续超过 500 ms 才过期"
    text = "\n".join(lines)
    assert re.search(r"erpm_skip=(\d+)", text) and int(re.search(r"erpm_skip=(\d+)", text).group(1)) == 2 + 295


def test_controller_map_injection(tmp_path):
    from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text("#ifndef BSP_PWM_H\n#define BSP_PWM_H\n#define BSP_PWM_ESC_MIN_US 1100U\n"
                                    "#define BSP_PWM_ESC_MAX_US 1940U\n#endif\n", encoding="ascii")
    main = r"""#include <stdio.h>
#include "drv_coax_ctrl.h"
static uint16_t pulse(float t) { return (uint16_t)(1200.0f + t); }
static float thrust(uint16_t p) { return (float)p; }
static float pair_in[2];
static void pair(float u, float l, uint16_t *uo, uint16_t *lo) { pair_in[0] = u; pair_in[1] = l; *uo = 1234U; *lo = 5000U; }
static const DRV_COAX_CTRL_ThrustMap map = {pulse, thrust, 0};
static const DRV_COAX_CTRL_ThrustMap pair_map = {pulse, thrust, pair};
int main(void) {
    DRV_COAX_CTRL_AttitudeInput attitude = {0};
    DRV_COAX_CTRL_Reference reference = {0};
    DRV_COAX_CTRL_Output output;
    DRV_COAX_CTRL_Init();
    airframe_load_reference();
    reference.dt_sec = 0.02f; reference.horizontal_velocity_valid = 1U;
    reference.navigation_position_valid = 1U; reference.navigation_velocity_valid = 1U;
    attitude.acceleration_valid = 1U;
    DRV_COAX_CTRL_SetThrustMap(&pair_map);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    printf("run %u %u %.4f %.4f %.4f %.4f\n", output.motor_upper_us, output.motor_lower_us, (double)pair_in[0],
           (double)pair_in[1], (double)output.thrust_upper_n, (double)output.thrust_lower_n);
    DRV_COAX_CTRL_SetThrustMap(&map);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    printf("per %u %u %.4f\n", output.motor_upper_us, output.motor_lower_us, (double)output.thrust_upper_n);
    DRV_COAX_CTRL_SetThrustMap(0);
    printf("%u %.3f\n", DRV_COAX_CTRL_ThrustToMotorPulse(0.0f), (double)DRV_COAX_CTRL_MotorPulseToTotalThrust(1100U));
    DRV_COAX_CTRL_SetThrustMap(&map);
    printf("%u %u %.3f\n", DRV_COAX_CTRL_ThrustToMotorPulse(5.0f), DRV_COAX_CTRL_ThrustToMotorPulse(1.0e4f),
           (double)DRV_COAX_CTRL_MotorPulseToTotalThrust(1500U));
    DRV_COAX_CTRL_SetThrustMap(0);
    printf("%u\n", DRV_COAX_CTRL_ThrustToMotorPulse(0.0f));
    return 0;
}
"""
    main = AIRFRAME_FIXTURE_C + main
    lines = _run(_gcc(), tmp_path, [AIRFRAME_SOURCE, PROP_MAP_SOURCE] + [
        ROOT / "Driver" / "Src" / name for name in
        ("drv_coax_ctrl.c", "drv_position_control.c", "drv_attitude_control.c", "drv_rate_control.c")],
        [stub, ROOT / "Driver" / "Inc"], main)
    run, per = lines[0].split(), lines[1].split()
    lines = lines[2:]
    # The flight path hands both rotors to the pair callback (and clamps its output to the ESC range).
    assert run[1:3] == ["1234", "1940"] and float(run[3]) == pytest.approx(float(run[5])) and float(run[4]) == pytest.approx(float(run[6]))
    assert float(run[5]) > 0.0 and per[1] == per[2] == str(int(1200.0 + float(per[3])))
    assert lines[0] == "1100 0.000"
    first, clamped, total = lines[1].split()
    # Installed map sees thrust already clamped to the single-rotor limit; its output is clamped to the ESC range.
    assert int(first) == 1205 and int(clamped) == 1210 and float(total) == pytest.approx(20.4)
    assert lines[2] == "1100"

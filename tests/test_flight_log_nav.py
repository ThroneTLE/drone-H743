"""飞行日志 v12 导航/电池尾块（App/Src/app_flight_log_nav.c）宿主编译实测。

用桩替换服务层与电池，核对：字段搬运、低字节标志由模块取、高字节只取调用方给的、
以及电池无效时电压写 0。
"""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "app_flight_log_nav.h"
#include "app_battery.h"
#include "svc_flow_nav.h"
#include <stdio.h>
#include <string.h>

#define CHECK(c, code) do { if (!(c)) return (code); } while (0)

static DRV_NAV_EKF_Diagnostics stub_ekf;
static SVC_FLOW_NAV_State stub_nav;
static APP_BatterySnapshot stub_batt;

void SVC_FlowNav_GetEkfDiagnostics(DRV_NAV_EKF_Diagnostics *d) { *d = stub_ekf; }
void SVC_FlowNav_GetState(SVC_FLOW_NAV_State *s) { *s = stub_nav; }
void APP_Battery_GetSnapshot(APP_BatterySnapshot *o) { *o = stub_batt; }

int main(void)
{
    APP_FlightLogNavTail t;

    memset(&stub_ekf, 0, sizeof(stub_ekf));
    memset(&stub_nav, 0, sizeof(stub_nav));
    memset(&stub_batt, 0, sizeof(stub_batt));
    stub_ekf.accel_bias_m_s2[0] = 0.25f;
    stub_ekf.accel_bias_m_s2[1] = -0.5f;
    stub_ekf.last_innovation_m_s[0] = 0.125f;
    stub_ekf.last_innovation_m_s[1] = -0.0625f;
    stub_ekf.last_nis = 2.5f;
    stub_ekf.flow_update_count = 900U;
    stub_ekf.flow_reject_count = 11U;
    stub_ekf.initialized = 1U;
    stub_nav.flow_vel_x_filtered = -321;
    stub_nav.flow_vel_y_filtered = 77;
    stub_nav.velocity_valid = 1U;
    stub_nav.flow_filter_ready = 1U;
    stub_batt.state.valid = 1U;
    stub_batt.state.voltage_mv = 11987UL;
    stub_batt.can_arm = 1U;

    /* 调用方低字节的垃圾位必须被屏蔽；高字节原样带入。 */
    APP_FlightLogNav_Capture(&t, (uint16_t)(0x00FFU |
                              APP_FLIGHT_LOG_NAV_FLAG_POSITION_VALID |
                              APP_FLIGHT_LOG_NAV_FLAG_ATTITUDE_DEBUG));
    CHECK(t.ekf_accel_bias_m_s2[0] == 0.25f && t.ekf_accel_bias_m_s2[1] == -0.5f, 1);
    CHECK(t.ekf_innovation_m_s[0] == 0.125f && t.ekf_innovation_m_s[1] == -0.0625f, 2);
    CHECK(t.ekf_nis == 2.5f && t.ekf_flow_update_count == 900U && t.ekf_flow_reject_count == 11U, 3);
    CHECK(t.flow_filtered_x == -321 && t.flow_filtered_y == 77, 4);
    CHECK(t.battery_mv == 11987U, 5);
    CHECK(t.nav_flags ==
          (APP_FLIGHT_LOG_NAV_FLAG_VELOCITY_VALID | APP_FLIGHT_LOG_NAV_FLAG_FLOW_FILTER_READY |
           APP_FLIGHT_LOG_NAV_FLAG_EKF_INITIALIZED | APP_FLIGHT_LOG_NAV_FLAG_BATTERY_VALID |
           APP_FLIGHT_LOG_NAV_FLAG_BATTERY_CAN_ARM | APP_FLIGHT_LOG_NAV_FLAG_POSITION_VALID |
           APP_FLIGHT_LOG_NAV_FLAG_ATTITUDE_DEBUG), 6);

    /* 电池无效：电压写 0、不置有效位；低压/饱和位照搬。 */
    stub_batt.state.valid = 0U;
    stub_batt.state.low = 1U;
    stub_batt.state.saturated = 1U;
    stub_batt.can_arm = 0U;
    stub_nav.velocity_valid = 0U;
    APP_FlightLogNav_Capture(&t, 0U);
    CHECK(t.battery_mv == 0U, 10);
    CHECK((t.nav_flags & APP_FLIGHT_LOG_NAV_FLAG_BATTERY_VALID) == 0U, 11);
    CHECK((t.nav_flags & APP_FLIGHT_LOG_NAV_FLAG_BATTERY_LOW) != 0U, 12);
    CHECK((t.nav_flags & APP_FLIGHT_LOG_NAV_FLAG_BATTERY_SATURATED) != 0U, 13);
    CHECK((t.nav_flags & APP_FLIGHT_LOG_NAV_FLAG_VELOCITY_VALID) == 0U, 14);

    /* 电压超过 16 位时截断，不回绕。 */
    stub_batt.state.valid = 1U;
    stub_batt.state.voltage_mv = 70000UL;
    APP_FlightLogNav_Capture(&t, 0U);
    CHECK(t.battery_mv == 65535U, 20);

    APP_FlightLogNav_Capture(NULL, 0U);
    puts("ok");
    return 0;
}
"""


def test_nav_tail_capture_runs_on_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("host gcc or clang required")
    (tmp_path / "main.c").write_text(HARNESS, encoding="utf-8")
    exe = tmp_path / "nav.exe"
    cmd = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
           "-I", str(ROOT / "App/Inc"), "-I", str(ROOT / "Driver/Inc"),
           "-I", str(ROOT / "Services/Inc"),
           str(tmp_path / "main.c"), str(ROOT / "App/Src/app_flight_log_nav.c"),
           "-o", str(exe)]
    build = subprocess.run(cmd, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, f"exit {run.returncode}: {run.stdout}{run.stderr}"


def test_stabilizer_feeds_nav_tail_from_frame_flags() -> None:
    source = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    assert "APP_FlightLogNav_Capture(" in source
    for flag in ("RANGE_HEIGHT_VALID", "POSITION_VALID", "HORIZ_VEL_VALID",
                 "ACCEL_VALID", "ATTITUDE_DEBUG"):
        assert f"APP_FLIGHT_LOG_NAV_FLAG_{flag}" in source

"""Contract tests for the Action safety state machine (``App/Src/app_action.c``).

An Action is any task that spans time and owns an effector: a servo sweep, a
motor spin-up, a calibration. Its defining hazard is that the *ground station*
must not own the timeline -- if the operator's PC dies mid-sweep, the servo must
still come back to centre, and only the firmware can guarantee that.

Those are exactly the paths that cannot be tested on real hardware: you cannot
reliably reproduce "the USB cable is pulled at t=1.5 s" a hundred times, and this
project currently cannot flash hardware at all. So ``app_action.c`` takes time as
a parameter and effectors as callbacks, which makes every hazard path a
deterministic unit test. This file is that module's only acceptance evidence.

The mock effector carries an ``engaged`` flag: ``begin`` sets it, ``safe_reset``
clears it. It stands in for "the servo is parked at an extreme" or "the motor is
spinning". The invariant every scenario checks is that no terminal state ever
leaves it set.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "app_action.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    int begin_calls;
    int step_calls;
    int reset_calls;
    int engaged;          /* 1 while the effector is in a dangerous position */
    uint8_t precondition; /* non-zero rejects the start */
    uint8_t begin_result; /* non-zero fails the start */
    int8_t  step_result;  /* what step() reports */
    int     steps_to_done;/* when > 0, report DONE after this many steps */
} MockCtx;

static uint8_t mock_precondition(void *ctx)
{
    return ((MockCtx *)ctx)->precondition;
}

static uint8_t mock_begin(void *ctx, uint32_t now_ms)
{
    MockCtx *m = (MockCtx *)ctx;
    (void)now_ms;
    m->begin_calls++;
    if (m->begin_result != 0U)
        return m->begin_result;
    m->engaged = 1;   /* the servo has left centre / the motor is live */
    return 0U;
}

static int8_t mock_step(void *ctx, uint32_t now_ms, uint8_t *progress)
{
    MockCtx *m = (MockCtx *)ctx;
    (void)now_ms;
    m->step_calls++;
    if (progress != NULL)
        *progress = (uint8_t)((m->step_calls * 10) % 101);
    if (m->steps_to_done > 0 && m->step_calls >= m->steps_to_done)
        return APP_ACTION_STEP_DONE;
    return m->step_result;
}

static void mock_safe_reset(void *ctx)
{
    MockCtx *m = (MockCtx *)ctx;
    m->reset_calls++;
    m->engaged = 0;   /* servo centred, PWM released */
}

static const APP_ActionDescriptor kSweep = {
    "SERVO_SWEEP", 4000U, 4000U, 0U,
    mock_precondition, mock_begin, mock_step, mock_safe_reset
};

static const APP_ActionDescriptor kSweepHb = {
    "SERVO_SWEEP_HB", 4000U, 4000U, 500U,
    mock_precondition, mock_begin, mock_step, mock_safe_reset
};

static MockCtx g_ctx;
static APP_ActionRunner g_runner;

static void setup(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.step_result = APP_ACTION_STEP_RUNNING;
    APP_Action_Init(&g_runner);
}

static void report(const char *scenario, APP_ActionError start_err)
{
    printf("%s start=%s state=%s error=%s resets=%d begins=%d steps=%d "
           "progress=%u engaged=%d\n",
           scenario,
           APP_Action_ErrorName(start_err),
           APP_Action_StateName(APP_Action_GetState(&g_runner)),
           APP_Action_ErrorName(APP_Action_GetError(&g_runner)),
           g_ctx.reset_calls, g_ctx.begin_calls, g_ctx.step_calls,
           (unsigned)APP_Action_GetProgress(&g_runner), g_ctx.engaged);
}

/* Advance from `from` to `to` in `step` increments, ticking each time. */
static void run_until(uint32_t from, uint32_t to, uint32_t step)
{
    uint32_t t;
    for (t = from; t <= to; t += step)
        APP_Action_Tick(&g_runner, t);
}

int main(void)
{
    APP_ActionError e;

    /* TEST-01: armed interlock. Nothing may start, and nothing may be reset. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 1U, 0U, 0U, 1U);
    report("armed_interlock", e);

    /* TEST-02: link dies at 1.5 s; the 4 s hard timeout must still fire. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 17U, 4000U, 0U, 0U);
    run_until(100U, 3900U, 100U);
    printf("timeout_before state=%s engaged=%d\n",
           APP_Action_StateName(APP_Action_GetState(&g_runner)), g_ctx.engaged);
    APP_Action_Tick(&g_runner, 4000U);
    report("timeout_at_deadline", e);

    /* TEST-03: explicit cancel. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 5U, 4000U, 0U, 0U);
    run_until(100U, 500U, 100U);
    APP_Action_Cancel(&g_runner, 500U);
    report("cancel", e);

    /* Normal completion still releases the effector. */
    setup();
    g_ctx.steps_to_done = 3;
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 6U, 4000U, 0U, 0U);
    run_until(100U, 1000U, 100U);
    report("normal_completion", e);

    /* Heartbeat loss aborts sooner than the hard timeout. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweepHb, &g_ctx, 7U, 4000U, 0U, 0U);
    run_until(100U, 500U, 100U);
    report("heartbeat_loss", e);

    /* A maintained heartbeat keeps the action alive well past the HB window. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweepHb, &g_ctx, 8U, 4000U, 0U, 0U);
    for (uint32_t t = 100U; t <= 3000U; t += 100U) {
        APP_Action_Heartbeat(&g_runner, 8U, t);
        APP_Action_Tick(&g_runner, t);
    }
    report("heartbeat_maintained", e);

    /* A heartbeat carrying the wrong id must not keep the action alive. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweepHb, &g_ctx, 9U, 4000U, 0U, 0U);
    for (uint32_t t = 100U; t <= 600U; t += 100U) {
        APP_Action_Heartbeat(&g_runner, 999U, t);
        APP_Action_Tick(&g_runner, t);
    }
    report("stale_heartbeat", e);

    /* Re-sending the same START must not restart the sweep. */
    setup();
    (void)APP_Action_Start(&g_runner, &kSweep, &g_ctx, 11U, 4000U, 0U, 0U);
    run_until(100U, 300U, 100U);
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 11U, 4000U, 300U, 0U);
    report("idempotent_restart", e);

    /* A different action may not pre-empt a running one. */
    setup();
    (void)APP_Action_Start(&g_runner, &kSweep, &g_ctx, 12U, 4000U, 0U, 0U);
    run_until(100U, 300U, 100U);
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 13U, 4000U, 300U, 0U);
    report("busy_reject", e);

    /* Ticking long after a terminal state must not reset a second time. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 14U, 1000U, 0U, 0U);
    run_until(100U, 5000U, 100U);
    report("no_double_reset", e);

    /* Wraparound: started just below 2^32, deadline crosses zero. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 15U, 1000U, 0xFFFFFF00U, 0U);
    APP_Action_Tick(&g_runner, 0xFFFFFFF0U);          /* elapsed 240 ms */
    printf("wrap_before state=%s engaged=%d\n",
           APP_Action_StateName(APP_Action_GetState(&g_runner)), g_ctx.engaged);
    APP_Action_Tick(&g_runner, 744U);                  /* elapsed 1000 ms */
    report("wraparound_timeout", e);

    /* begin() failing must still leave the effector safe. */
    setup();
    g_ctx.begin_result = 1U;
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 18U, 4000U, 0U, 0U);
    report("begin_failure", e);

    /* step() reporting failure must release the effector. */
    setup();
    g_ctx.step_result = APP_ACTION_STEP_FAILED;
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 19U, 4000U, 0U, 0U);
    APP_Action_Tick(&g_runner, 100U);
    report("step_failure", e);

    /* Precondition rejection must not touch the effector at all. */
    setup();
    g_ctx.precondition = 1U;
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 20U, 4000U, 0U, 0U);
    report("precondition_reject", e);

    /* A timeout above the descriptor's ceiling is clamped, not honoured. */
    setup();
    e = APP_Action_Start(&g_runner, &kSweep, &g_ctx, 21U, 60000U, 0U, 0U);
    run_until(100U, 4000U, 100U);
    report("timeout_clamped", e);

    return 0;
}
"""


@pytest.fixture(scope="module")
def scenarios() -> dict[str, dict[str, str]]:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    import tempfile

    tmp = Path(tempfile.mkdtemp())
    harness = tmp / "harness.c"
    harness.write_text(HARNESS, encoding="ascii")

    executable = tmp / "action.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_action.c"),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
    )

    out = subprocess.run([str(executable)], check=True, capture_output=True)
    parsed: dict[str, dict[str, str]] = {}
    for line in out.stdout.decode("ascii").splitlines():
        parts = line.split()
        if not parts:
            continue
        parsed[parts[0]] = dict(p.split("=", 1) for p in parts[1:] if "=" in p)
    return parsed


# --- The three acceptance tests from the ground-station design doc -----------


def test_01_armed_interlock_refuses_to_start(scenarios) -> None:
    s = scenarios["armed_interlock"]
    assert s["start"] == "armed"
    assert s["state"] == "IDLE"
    # Nothing started, so nothing may have been begun *or* reset: a spurious
    # safe_reset is itself an unrequested effector movement.
    assert s["begins"] == "0"
    assert s["resets"] == "0"
    assert s["engaged"] == "0"


def test_02_hard_timeout_fires_after_the_link_dies(scenarios) -> None:
    # The ground station stopped talking at 1.5 s; nothing rescues the servo
    # except the firmware's own deadline.
    assert scenarios["timeout_before"]["state"] == "RUNNING"
    assert scenarios["timeout_before"]["engaged"] == "1"

    s = scenarios["timeout_at_deadline"]
    assert s["state"] == "FAILED"
    assert s["error"] == "timeout"
    assert s["resets"] == "1"
    assert s["engaged"] == "0", "the servo must be centred and the PWM released"


def test_03_cancel_stops_and_makes_safe(scenarios) -> None:
    s = scenarios["cancel"]
    assert s["state"] == "CANCELLED"
    assert s["error"] == "cancelled"
    assert s["resets"] == "1"
    assert s["engaged"] == "0"


# --- Invariants I1..I5 ------------------------------------------------------


@pytest.mark.parametrize(
    "scenario",
    [
        "timeout_at_deadline",
        "cancel",
        "normal_completion",
        "heartbeat_loss",
        "stale_heartbeat",
        "no_double_reset",
        "wraparound_timeout",
        "begin_failure",
        "step_failure",
        "timeout_clamped",
    ],
)
def test_i1_every_terminal_path_resets_exactly_once(scenarios, scenario: str) -> None:
    s = scenarios[scenario]
    assert s["state"] in {"COMPLETED", "FAILED", "CANCELLED"}, s
    assert s["resets"] == "1", f"{scenario}: safe_reset must run exactly once"
    assert s["engaged"] == "0", f"{scenario}: effector left in a dangerous state"


def test_i2_terminal_state_is_not_reset_again_by_further_ticks(scenarios) -> None:
    # Ticked to 5000 ms against a 1000 ms timeout: 40 ticks past the deadline.
    s = scenarios["no_double_reset"]
    assert s["error"] == "timeout"
    assert s["resets"] == "1"


def test_i3_precondition_rejection_leaves_the_effector_untouched(scenarios) -> None:
    s = scenarios["precondition_reject"]
    assert s["start"] == "precondition"
    assert s["state"] == "IDLE"
    assert s["begins"] == "0"
    assert s["resets"] == "0"


def test_i4_duplicate_start_is_idempotent(scenarios) -> None:
    s = scenarios["idempotent_restart"]
    # A retried START is acknowledged rather than rejected...
    assert s["start"] == "none"
    assert s["state"] == "RUNNING"
    # ...but must not have run begin() a second time.
    assert s["begins"] == "1", "a retried START restarted the sweep"


def test_i5_timeout_survives_the_tick_counter_wrapping(scenarios) -> None:
    assert scenarios["wrap_before"]["state"] == "RUNNING"
    s = scenarios["wraparound_timeout"]
    assert s["state"] == "FAILED"
    assert s["error"] == "timeout"


# --- Link supervision -------------------------------------------------------


def test_heartbeat_loss_aborts_sooner_than_the_hard_timeout(scenarios) -> None:
    s = scenarios["heartbeat_loss"]
    assert s["state"] == "FAILED"
    assert s["error"] == "link_lost"
    assert s["engaged"] == "0"


def test_maintained_heartbeat_keeps_the_action_running(scenarios) -> None:
    s = scenarios["heartbeat_maintained"]
    assert s["state"] == "RUNNING", "a healthy link must not trip the watchdog"
    assert s["engaged"] == "1"


def test_stale_heartbeat_from_another_action_does_not_extend_the_watchdog(
    scenarios,
) -> None:
    # Heartbeats arrived on time but carried a different action_id; letting them
    # count would silently disable link supervision for the running action.
    s = scenarios["stale_heartbeat"]
    assert s["state"] == "FAILED"
    assert s["error"] == "link_lost"


# --- Arbitration ------------------------------------------------------------


def test_a_second_action_cannot_preempt_a_running_one(scenarios) -> None:
    s = scenarios["busy_reject"]
    assert s["start"] == "busy"
    assert s["state"] == "RUNNING"
    assert s["begins"] == "1", "the running action was disturbed"
    assert s["engaged"] == "1"


def test_requested_timeout_is_clamped_to_the_descriptor_ceiling(scenarios) -> None:
    # 60 s was requested against a 4 s ceiling; the ceiling must win, otherwise
    # the ground station can talk the firmware out of its own safety limit.
    s = scenarios["timeout_clamped"]
    assert s["state"] == "FAILED"
    assert s["error"] == "timeout"


def test_begin_failure_is_reported_and_made_safe(scenarios) -> None:
    s = scenarios["begin_failure"]
    assert s["start"] == "begin"
    assert s["state"] == "FAILED"
    assert s["engaged"] == "0"


def test_step_failure_releases_the_effector(scenarios) -> None:
    s = scenarios["step_failure"]
    assert s["state"] == "FAILED"
    assert s["error"] == "effector"
    assert s["engaged"] == "0"


# --- Source-level guards ----------------------------------------------------


def strip_comments(source: str) -> str:
    """Drop /* */ and // comments so guards match code, not prose about it."""
    out, i, n = [], 0, len(source)
    while i < n:
        if source.startswith("/*", i):
            end = source.find("*/", i + 2)
            i = n if end < 0 else end + 2
        elif source.startswith("//", i):
            end = source.find("\n", i)
            i = n if end < 0 else end
        else:
            out.append(source[i])
            i += 1
    return "".join(out)


def test_module_takes_time_as_a_parameter_and_never_reads_a_clock() -> None:
    # Comments are stripped first: app_action.c documents *why* it avoids
    # HAL_GetTick, and a guard that trips on its own rationale is useless.
    source = strip_comments(
        (ROOT / "App" / "Src" / "app_action.c").read_text(encoding="utf-8")
    )

    # The moment this module reads a clock or blocks, its hazard paths stop
    # being testable off-target -- which is the entire reason it exists.
    for forbidden in (
        "HAL_GetTick",
        "osDelay",
        "osMutex",
        "osSemaphore",
        "osMessageQueue",
        "printf",
        "SVC_Timestamp",
    ):
        assert forbidden not in source, f"{forbidden} must not appear in app_action.c"


def test_all_terminal_transitions_go_through_the_single_exit() -> None:
    source = strip_comments(
        (ROOT / "App" / "Src" / "app_action.c").read_text(encoding="utf-8")
    )

    # Tick() only -- APP_Action_Reset legitimately assigns IDLE, and it can only
    # run from a terminal state.
    body = source[
        source.index("void APP_Action_Tick") : source.index("void APP_Action_Reset")
    ]
    # Assigning state directly inside Tick would bypass safe_reset and break I1.
    assert "runner->state =" not in body, (
        "Tick must terminate actions via app_action_finish(), not by assignment"
    )

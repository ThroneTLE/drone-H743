"""遥控通道映射与端点标定。

背景：通道号原来是 app_stabilizer.c 里的 6 个 #define（CH1..CH6），端点是
1000/1500/2000 三个字面量。换发射机、改通道顺序、或者摇杆行程不标准，都得改代码
重烧固件。这组测试锁住改造后的三件事：

  1. 控制环里不再有任何通道下标字面量；
  2. 出厂默认与改造前逐位一致，升级不会改变飞行手感；
  3. 会让人受伤的组合（一个通道绑两个功能、行程不足、解锁状态下改映射）必须被拒。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
STATE_SOURCE = (ROOT / "tools" / "panel_lib" / "state.py").read_text(encoding="utf-8")


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


def entry(channel: int, *, rev: int = 0, low: int = 1000,
          mid: int = 1500, high: int = 2000) -> dict[str, int]:
    return {"channel": channel, "reversed": rev, "min": low, "mid": mid, "max": high}


def default_map() -> dict[str, dict[str, int]]:
    return {name: entry(index) for index, (name, _label) in enumerate(panel.RC_FUNCTIONS)}


# --------------------------------------------------------------------------
# 自动识别：宁可让用户多拨一次，也不要绑错
# --------------------------------------------------------------------------

def sweep(channel: int, count: int = 40, travel: int = 500) -> list[list[int]]:
    rows = []
    for step in range(count):
        row = [1500] * panel.RC_CHANNEL_COUNT
        row[channel] = 1500 + (travel // 2 if step % 2 else -travel // 2)
        rows.append(row)
    return rows


def test_moving_one_stick_identifies_that_channel() -> None:
    channel, detail = panel.rc_detect_channel(sweep(6))
    assert channel == 6
    assert "CH7" in detail


def test_a_still_transmitter_is_not_bound_to_anything() -> None:
    rows = [[1500] * panel.RC_CHANNEL_COUNT for _ in range(40)]
    channel, detail = panel.rc_detect_channel(rows)
    assert channel is None
    assert "没有检测到明显动作" in detail


def test_two_channels_moving_together_is_refused() -> None:
    """两根杆同时动时静默绑一个，就是把油门绑到副翼上的那类事故。"""
    rows = sweep(2)
    for index, row in enumerate(rows):
        row[3] = 1500 + (200 if index % 2 else -200)
    channel, detail = panel.rc_detect_channel(rows)
    assert channel is None
    assert "同时在动" in detail


def test_a_barely_moved_channel_is_below_the_travel_floor() -> None:
    channel, _detail = panel.rc_detect_channel(sweep(1, travel=100))
    assert channel is None


def test_empty_sample_window_is_safe() -> None:
    assert panel.rc_detect_channel([]) == (None, panel.rc_detect_channel([])[1])
    assert panel.rc_channel_travel([]) == [0] * panel.RC_CHANNEL_COUNT


# --------------------------------------------------------------------------
# 引导校准：一次走完"哪一路 / 正反 / 两端"
# --------------------------------------------------------------------------

CENTER = [1500] * panel.RC_CHANNEL_COUNT


def frames(channel: int, value: int, count: int = panel.RC_WIZARD_HOLD_FRAMES) -> list[list[int]]:
    row = list(CENTER)
    row[channel] = value
    return [list(row) for _ in range(count)]


def test_wizard_covers_every_function_in_both_directions() -> None:
    """少一个方向就无法判断正反；少一个功能就得让用户回去手填通道号。"""
    covered = {(function, direction) for function, direction, _p in panel.RC_WIZARD_STEPS}
    for name, _label in panel.RC_FUNCTIONS:
        assert (name, +1) in covered, name
        assert (name, -1) in covered, name
    assert len(panel.RC_WIZARD_STEPS) == 2 * len(panel.RC_FUNCTIONS)


def test_a_held_extreme_is_accepted() -> None:
    channel, deviation, detail = panel.rc_wizard_step_ready(CENTER, frames(4, 1950))
    assert channel == 4
    assert deviation == 450
    assert "CH5" in detail


def test_a_stick_still_travelling_is_not_captured() -> None:
    """从一端扫到另一端会路过每个位置；没有'保持'条件就会在半路误判到位。"""
    window = [
        [1500] * 4 + [1750 + step * 60] + [1500] * 11
        for step in range(panel.RC_WIZARD_HOLD_FRAMES)
    ]
    channel, _dev, detail = panel.rc_wizard_step_ready(CENTER, window)
    assert channel is None
    assert "还在动" in detail


def test_passing_through_center_is_not_captured() -> None:
    window = [list(CENTER) for _ in range(panel.RC_WIZARD_HOLD_FRAMES)]
    assert panel.rc_wizard_step_ready(CENTER, window)[0] is None


def test_two_channels_moving_together_is_not_captured() -> None:
    window = frames(1, 1950)
    for row in window:
        row[0] = 1800
    assert panel.rc_wizard_step_ready(CENTER, window)[0] is None


def test_small_cross_talk_does_not_block_the_dominant_channel() -> None:
    """同轴摇杆推到底会带动另一轴几十 µs，这属于正常，不该让用户重做。"""
    window = frames(1, 1950)
    for row in window:
        row[0] = 1560
    assert panel.rc_wizard_step_ready(CENTER, window)[0] == 1


def test_too_short_a_window_waits() -> None:
    assert panel.rc_wizard_step_ready(CENTER, frames(1, 1950, count=2))[0] is None


# --- 每一步必须有新动作才算数 -------------------------------------------
#
# 2026-08-29 实测：把油门推到顶不松手，12 步在几秒内自己全跑完，每一步记的都是同一个
# 读数，结果 6 个功能全报"两次动作方向相同"。原因是采完就直接进下一步，而手还按在那儿，
# 下一步立刻又攒够 8 帧同样的稳定读数。

def test_holding_still_after_a_capture_does_not_advance_again() -> None:
    """闸门的全部意义：静止不动时必须什么都不发生。"""
    held = frames(1, 1950, count=4 * panel.RC_WIZARD_HOLD_FRAMES)
    open_gate, detail = panel.rc_wizard_gate_open(held, CENTER, release_channel=1)
    assert open_gate is False
    assert "松开 CH2" in detail


def test_the_gate_opens_once_the_stick_is_released() -> None:
    released = [list(CENTER) for _ in range(panel.RC_WIZARD_HOLD_FRAMES)]
    open_gate, _detail = panel.rc_wizard_gate_open(released, CENTER, release_channel=1)
    assert open_gate is True


def test_the_gate_waits_for_the_reading_to_settle() -> None:
    moving = [
        [1500] * 4 + [1500 + step * 80] + [1500] * 11
        for step in range(panel.RC_WIZARD_HOLD_FRAMES)
    ]
    open_gate, detail = panel.rc_wizard_gate_open(moving, CENTER, release_channel=None)
    assert open_gate is False
    assert "稳定" in detail


def test_a_non_centering_channel_only_has_to_settle() -> None:
    """油门和开关没有中位可回，要求它们"回中"会把流程永远卡住。"""
    parked = frames(2, 1950)
    open_gate, _detail = panel.rc_wizard_gate_open(parked, CENTER, release_channel=None)
    assert open_gate is True


def test_pushing_the_same_way_twice_is_refused() -> None:
    """一对步骤必须真的往两个方向走过，否则判不出正反。"""
    window = frames(1, 1950)
    channel, _dev, detail = panel.rc_wizard_step_ready(CENTER, window, opposite_of=+450)
    assert channel is None
    assert "相反方向" in detail
    # 反方向就放行。
    assert panel.rc_wizard_step_ready(CENTER, frames(1, 1050), opposite_of=+450)[0] == 1


def test_a_parked_channel_does_not_swamp_later_steps() -> None:
    """油门停在顶端时，对固定中位是个恒定 450µs 偏移，会和后面每一步打平。

    所以判定的零点必须是"本步开闸那一刻的静止状态"，不是固定中位。
    """
    baseline = list(CENTER)
    baseline[2] = 1950                       # 油门停在顶端
    window = [list(baseline) for _ in range(panel.RC_WIZARD_HOLD_FRAMES)]
    for row in window:
        row[1] = 1950                        # 现在推俯仰
    assert panel.rc_wizard_step_ready(baseline, window)[0] == 1
    # 若拿固定中位当零点，CH2 和 CH3 偏移相同 → 判不出主导通道。
    assert panel.rc_wizard_step_ready(CENTER, window)[0] is None


def test_the_wizard_measures_against_the_gate_time_baseline() -> None:
    reference = function_body(SOURCE, "    def _rc_wizard_reference(")
    assert "self.rc_wizard_baseline or self.rc_wizard_center" in reference
    feed = function_body(SOURCE, "    def _rc_wizard_feed(")
    assert "self.rc_wizard_baseline = list(channels)" in feed
    assert "self.rc_wizard_armed = True" in feed


def test_the_two_steps_of_one_stick_need_no_return_to_center() -> None:
    """前推 → 后拉是一路扫过去的，中间硬要求回中只是多余的摩擦。"""
    advance = function_body(SOURCE, "    def _rc_wizard_advance(")
    assert "RC_WIZARD_STEPS[self.rc_wizard_index][0] == previous" in advance
    assert "release_channel = None" in advance


def wizard_results(**wiring: tuple[int, bool]) -> dict[tuple[str, int], tuple[int, int]]:
    results = {}
    for name, (channel, inverted) in wiring.items():
        for direction in (+1, -1):
            physical = direction * (-1 if inverted else 1)
            results[(name, direction)] = (channel, 450 * physical)
    return results


def test_wizard_recovers_a_non_default_wiring() -> None:
    """校准的价值就在这里：通道顺序和正反都跟出厂默认不一样也能查出来。"""
    wiring = {
        "roll": (0, False), "pitch": (1, False), "throttle": (2, True),
        "yaw": (7, False), "arm": (4, False), "mode": (6, False),
    }
    low = [1050] * panel.RC_CHANNEL_COUNT
    high = [1950] * panel.RC_CHANNEL_COUNT
    functions, warnings = rc_build(low, high, wizard_results(**wiring))
    assert warnings == []
    for name, (channel, inverted) in wiring.items():
        assert functions[name]["channel"] == channel, name
        assert functions[name]["reversed"] == int(inverted), name
        assert (functions[name]["min"], functions[name]["max"]) == (1050, 1950)


def rc_build(low, high, results, center=None):
    return panel.rc_wizard_build_map(
        center if center is not None else CENTER, low, high, results, default_map()
    )


def test_wizard_keeps_the_measured_center_as_trim() -> None:
    center = list(CENTER)
    center[0] = 1483
    functions, _warnings = rc_build(
        [1050] * 16, [1950] * 16, wizard_results(roll=(0, False)), center
    )
    assert functions["roll"]["mid"] == 1483


@pytest.mark.parametrize("resting_us", [1050, 1062, 1500, 1940])
def test_throttle_center_never_depends_on_where_the_stick_was_left(resting_us: int) -> None:
    """油门杆不自回中，松手停在哪都不代表中位。

    这里必须永远取行程中点：本机油门是双用的，throttle_01 走 min..max 管直通油门，
    而 norm 绕中位算、喂给定高速率（50% 行程 = 保持高度）。中位若被定在行程底部，
    稍微推一点油门就等于满爬升率。1062 这一档尤其关键——它离最低点只有 12µs，
    "只在正好等于端点时才回退"的写法会放它过去。
    """
    center = list(CENTER)
    center[2] = resting_us
    functions, _warnings = rc_build(
        [1050] * 16, [1950] * 16, wizard_results(throttle=(2, False)), center
    )
    assert functions["throttle"]["mid"] == 1500


@pytest.mark.parametrize("function,channel", [("arm", 4), ("mode", 6)])
def test_switch_center_is_the_travel_midpoint(function: str, channel: int) -> None:
    """开关只有两个位置，"中位"没有物理意义；固件也只按行程百分比判高低。"""
    center = list(CENTER)
    center[channel] = 1050
    functions, _warnings = rc_build(
        [1050] * 16, [1950] * 16, wizard_results(**{function: (channel, False)}), center
    )
    assert functions[function]["mid"] == 1500


def test_a_stick_held_off_center_during_capture_is_rejected_as_trim() -> None:
    """"回中"那一刻手还压着杆，读数不能当 subtrim 用——否则杆的零点整个偏掉。"""
    center = list(CENTER)
    center[0] = 1150          # 行程 1050~1950，落在最低 20% 以内
    functions, warnings = rc_build(
        [1050] * 16, [1950] * 16, wizard_results(roll=(0, False)), center
    )
    assert functions["roll"]["mid"] == 1500
    assert any("不在行程中段" in w for w in warnings)


def test_two_directions_landing_on_different_channels_is_refused() -> None:
    results = wizard_results(yaw=(3, False))
    results[("yaw", -1)] = (9, -450)
    functions, warnings = rc_build([1050] * 16, [1950] * 16, results)
    assert any("不同通道" in w for w in warnings)
    assert functions["yaw"] == default_map()["yaw"]


def test_two_directions_with_the_same_sign_is_refused() -> None:
    results = wizard_results(pitch=(1, False))
    results[("pitch", -1)] = (1, +450)
    functions, warnings = rc_build([1050] * 16, [1950] * 16, results)
    assert any("方向相同" in w for w in warnings)
    assert functions["pitch"] == default_map()["pitch"]


def test_a_skipped_step_keeps_the_previous_setting() -> None:
    results = wizard_results(roll=(0, False))
    del results[("roll", -1)]
    functions, warnings = rc_build([1050] * 16, [1950] * 16, results)
    assert any("没有都采到" in w for w in warnings)
    assert functions["roll"] == default_map()["roll"]


def test_channel_claimed_by_two_functions_is_reported() -> None:
    results = wizard_results(roll=(3, False), yaw=(3, False))
    _functions, warnings = rc_build([1050] * 16, [1950] * 16, results)
    assert any("同时被" in w for w in warnings)


def test_a_channel_that_barely_moved_keeps_its_old_endpoints() -> None:
    low = [1050] * panel.RC_CHANNEL_COUNT
    high = [1950] * panel.RC_CHANNEL_COUNT
    low[6], high[6] = 1495, 1505
    functions, warnings = rc_build(low, high, wizard_results(mode=(6, False)))
    assert any("行程只有" in w for w in warnings)
    assert functions["mode"] == default_map()["mode"]


# --------------------------------------------------------------------------
# 映射校验：与固件 APP_RcConfig_Validate 同判据
# --------------------------------------------------------------------------

def test_default_mapping_is_accepted() -> None:
    ok, reason = panel.rc_map_is_valid(default_map())
    assert ok, reason


def test_one_channel_cannot_serve_two_functions() -> None:
    """把 arm 和 throttle 绑到同一路，推油门就等于解锁。"""
    functions = default_map()
    functions["arm"] = entry(2)
    ok, reason = panel.rc_map_is_valid(functions)
    assert not ok
    assert "CH3" in reason


def test_travel_shorter_than_the_floor_is_rejected() -> None:
    functions = default_map()
    functions["mode"] = entry(5, low=1490, mid=1500, high=1510)
    ok, reason = panel.rc_map_is_valid(functions)
    assert not ok
    assert "行程" in reason


def test_center_outside_the_endpoints_is_rejected() -> None:
    functions = default_map()
    functions["yaw"] = entry(3, low=1000, mid=2100, high=2000)
    ok, reason = panel.rc_map_is_valid(functions)
    assert not ok
    assert "中位" in reason


def test_endpoints_outside_the_crsf_range_are_rejected() -> None:
    functions = default_map()
    functions["roll"] = entry(0, low=500, mid=1500, high=2000)
    assert panel.rc_map_is_valid(functions)[0] is False


def test_unbound_functions_are_allowed_but_not_all_of_them() -> None:
    functions = default_map()
    functions["mode"] = entry(-1)
    assert panel.rc_map_is_valid(functions)[0] is True
    for name in functions:
        functions[name] = entry(-1)
    ok, reason = panel.rc_map_is_valid(functions)
    assert not ok
    assert "没有任何功能" in reason


# --------------------------------------------------------------------------
# 归一化：主机预览必须和固件算得一样
# --------------------------------------------------------------------------

def test_center_is_zero_and_endpoints_are_unity() -> None:
    axis = entry(0)
    assert panel.rc_normalize(axis, 1500, 20) == 0.0
    assert panel.rc_normalize(axis, 2000, 20) == pytest.approx(1.0)
    assert panel.rc_normalize(axis, 1000, 20) == pytest.approx(-1.0)


def test_deadband_suppresses_small_offsets() -> None:
    axis = entry(0)
    assert panel.rc_normalize(axis, 1515, 20) == 0.0
    assert panel.rc_normalize(axis, 1525, 20) != 0.0


def test_offset_center_uses_each_half_span_separately() -> None:
    """中位很少正好在行程中点；两侧共用一个跨度会让回中附近手感不对称。"""
    axis = entry(0, low=1100, mid=1400, high=2000)
    assert panel.rc_normalize(axis, 2000, 20) == pytest.approx(1.0)
    assert panel.rc_normalize(axis, 1100, 20) == pytest.approx(-1.0)
    # 上半跨度 600、下半 300：同样偏离 150us，两侧比例不同。
    assert panel.rc_normalize(axis, 1550, 20) == pytest.approx(0.25)
    assert panel.rc_normalize(axis, 1250, 20) == pytest.approx(-0.5)


def test_reverse_flips_the_sign() -> None:
    assert panel.rc_normalize(entry(0, rev=1), 2000, 20) == pytest.approx(-1.0)


def test_beyond_the_endpoints_clamps() -> None:
    assert panel.rc_normalize(entry(0), 2200, 20) == pytest.approx(1.0)
    assert panel.rc_normalize(entry(0), 800, 20) == pytest.approx(-1.0)


def test_unbound_function_reads_zero() -> None:
    assert panel.rc_normalize(entry(-1), 2000, 20) == 0.0


# --------------------------------------------------------------------------
# 固件契约
# --------------------------------------------------------------------------

def test_defaults_reproduce_the_previously_hardcoded_mapping() -> None:
    """默认值必须等于改造前写死的 CH1..CH6，否则升级就是一次静默的手感变更。"""
    source = read("App/Src/app_rc_config.c")
    table = source[source.index("app_rc_default_channel[APP_RC_FUNC_COUNT] = {"):]
    table = table[: table.index("};")]
    assert [line.strip()[0] for line in table.splitlines() if line.strip()[:1].isdigit()] == \
        ["0", "1", "2", "3", "4", "5"]
    header = read("App/Inc/app_rc_config.h")
    assert "#define APP_RC_DEFAULT_MIN_US  1000U" in header
    assert "#define APP_RC_DEFAULT_MID_US  1500U" in header
    assert "#define APP_RC_DEFAULT_MAX_US  2000U" in header
    assert "#define APP_RC_DEFAULT_DEADBAND_US 20U" in header


def test_control_loop_has_no_hardcoded_channel_index_left() -> None:
    stabilizer = read("App/Src/app_stabilizer.c")
    assert "STABILIZER_RC_CH_" not in stabilizer
    assert "STABILIZER_RC_DEADBAND_US" not in stabilizer
    # 每周期重读映射，改绑定后不必重启飞控。
    assert "APP_RcConfig_ReadActive(&frame->rc_config)" in stabilizer
    assert "APP_RcConfig_Resolve(&frame->rc_config, frame->ch, &frame->rc)" in stabilizer


def test_firmware_refuses_duplicate_channels_and_short_travel() -> None:
    source = read("App/Src/app_rc_config.c")
    validate = source[source.index("uint8_t APP_RcConfig_Validate("):]
    validate = validate[: validate.index("\nconst char *")]
    assert "used_channels" in validate
    assert "APP_RC_MIN_SPAN_US" in validate
    assert "(map->mid_us <= map->min_us) || (map->mid_us >= map->max_us)" in validate


def test_mapping_writes_are_blocked_while_armed() -> None:
    """改映射等于改'哪根杆是油门'，解锁状态下改一次就可能让电机响应错通道。"""
    control = read("App/Src/app_control.c")
    handler = control[control.index("static void app_control_handle_rc_map("):]
    handler = handler[: handler.index("\nstatic void app_control_report_uart_stats(")]
    for command in ("SET", "DEADBAND", "RESET", "COMMIT"):
        section = handler[handler.index(f'strcmp(tokens[1], "{command}") == 0'):]
        section = section[:600]
        assert "app_control_rc_write_allowed() == 0U" in section, command
    assert 'return (APP_Stabilizer_IsArmed() == 0U) ? 1U : 0U;' in control


def test_flash_record_migrates_instead_of_discarding_old_config() -> None:
    """V16 记录里没有遥控映射；直接判无效会连舵机/PID 配置一起丢掉。"""
    control = read("App/Src/app_control.c")
    assert "#define APP_CONTROL_CFG_VERSION     17U" in control
    assert "#define APP_CONTROL_CFG_VERSION_V16 16U" in control
    assert "APP_ControlFlashRecordV16" in control
    load = control[control.index("record.version == APP_CONTROL_CFG_VERSION_V16"):]
    load = load[: load.index("APP_CONTROL_CFG_VERSION_V15")]
    assert "control_config = legacy_record.config" in load
    assert "app_control_apply_rc_config(NULL)" in load


def test_invalid_config_falls_back_to_defaults_not_to_zero() -> None:
    """全 0 会让油门看起来是最低、ARM 看起来是低电平：像安全，其实是遥控静默失效。"""
    source = read("App/Src/app_rc_config.c")
    resolve = source[source.index("void APP_RcConfig_Resolve("):]
    resolve = resolve[: resolve.index("\nvoid APP_RcConfig_ResetActive(")]
    assert "APP_RcConfig_Defaults(&fallback)" in resolve


# --------------------------------------------------------------------------
# 面板接线
# --------------------------------------------------------------------------

def test_rc_tab_polls_faster_than_the_imu_page() -> None:
    """摇杆位置和端点标定都靠这条流；降到 10Hz 会采不到端点。"""
    assert panel.RC_POLL_PERIOD_MS < panel.IMU_POLL_PERIOD_MS
    poll = function_body(SOURCE, "    def _imu_poll_tick(")
    assert "rc_tab_visible" in poll
    assert 'self._send_proto_silent(PROTO_REQ_RC, "RC?")' in poll


def test_rc_page_pulls_the_mapping_on_its_own() -> None:
    """表格空着、要用户先猜到点「从飞控读取」，就是 V0 页那个引导死角的重演。"""
    poll = function_body(SOURCE, "    def _imu_poll_tick(")
    assert 'self._send_proto_silent(PROTO_REQ_RCMAP, "RCMAP?")' in poll
    assert "self.rc_map_generation != self.serial_transport.connection_generation" in poll
    # 重连后必须重拉：接的可能已经是另一台飞控。
    assert "self.rc_map_generation = None" in function_body(SOURCE, "    def _stop(")


def test_panel_validates_before_sending_anything() -> None:
    send = function_body(SOURCE, "    def _rc_send_map(")
    assert "rc_map_is_valid(functions)" in send
    assert send.index("rc_map_is_valid(functions)") < send.index("RCMAP SET")
    # 解锁时不发，靠飞控回 armed_blocked 太晚：那时候命令已经发出去了。
    assert 'safe_int(self.rc_link_values.get("armed"), 0) != 0' in send


def test_commit_requires_explicit_confirmation() -> None:
    commit = function_body(SOURCE, "    def _rc_commit(")
    assert "askyesno" in commit
    assert "_rc_send_map(commit=True)" in commit


def test_sweep_keeps_old_endpoints_for_channels_that_barely_moved() -> None:
    body = function_body(SOURCE, "    def _rc_apply_sweep(")
    assert "RC_MIN_SPAN_US" in body
    assert "skipped" in body


def test_wizard_consumes_every_live_frame() -> None:
    """端点靠整段过程的实测极值；只看两个步骤的瞬时值会漏掉更极端的位置。"""
    accumulate = function_body(SOURCE, "    def _rc_accumulate(")
    assert "self._rc_wizard_feed(channels)" in accumulate
    feed = function_body(SOURCE, "    def _rc_wizard_feed(")
    assert "self.rc_wizard_min[index] = min(" in feed
    assert "self.rc_wizard_max[index] = max(" in feed


def test_wizard_requires_live_rc_and_an_explicit_safety_confirmation() -> None:
    start = function_body(SOURCE, "    def _rc_wizard_start(")
    assert "_rc_live_ok()" in start
    assert "askyesno" in start
    assert "螺旋桨已经拆下" in start
    # 自回中的杆要松手（中位取"开始那一刻"）；油门不自回中，不能要求用户"回中"。
    assert "松手" in start
    assert "油门和开关放哪都行" in start
    assert "self.rc_wizard_center = list(self.rc_channels)" in start


def test_wizard_pushes_the_result_to_the_aircraft() -> None:
    """只填界面不下发的话，摇杆十字按新映射画、飞控按旧的飞，等于什么都没验证。"""
    finish = function_body(SOURCE, "    def _rc_wizard_finish(")
    assert "self._rc_send_map(commit=False)" in finish
    # 非法映射不能下发；Flash 仍然要单独确认。
    assert finish.index("if not ok:") < finish.index("_rc_send_map(commit=False)")
    assert "commit=True" not in finish


def test_wizard_prompts_describe_the_action_not_the_jargon() -> None:
    """用户此刻还不知道哪根杆是"横滚"，只知道自己想让飞机往哪走。"""
    prompts = {function: prompt for function, direction, prompt in panel.RC_WIZARD_STEPS
               if direction > 0}
    assert "前后" in prompts["pitch"]
    assert "左右" in prompts["roll"]
    assert "转向" in prompts["yaw"]
    assert "油门" in prompts["throttle"]
    labels = dict(panel.RC_FUNCTIONS)
    assert "左右" in labels["roll"] and "前后" in labels["pitch"]
    # 升降舵 = elevator = 俯仰，不能挂在油门上。
    assert "升降舵" not in labels["throttle"]


def test_wizard_back_discards_that_step_result() -> None:
    back = function_body(SOURCE, "    def _rc_wizard_back(")
    assert "self.rc_wizard_results.pop((function, direction), None)" in back


def drive_wizard(app, channels: list[int], times: int) -> None:
    for _ in range(times):
        app._rc_handle_live_line("RC us=" + ",".join(str(v) for v in channels))
        app._rc_handle_live_line(
            "RC link fresh=1 frames=500 crc_err=0 fps_x10=500 lq=100 rssi=60 "
            "snr=10 age_ms=5 armed=0 bound=0x3F"
        )


def test_a_held_stick_advances_exactly_one_step(monkeypatch) -> None:
    """端到端复现 2026-08-29 那次"推到顶就跑完 12 步"。"""
    import tkinter as tk

    monkeypatch.setattr(panel.messagebox, "askyesno", lambda *_a, **_k: True)
    monkeypatch.setattr(panel.messagebox, "showwarning", lambda *_a, **_k: None)
    try:
        app = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        rest = [1500] * panel.RC_CHANNEL_COUNT
        drive_wizard(app, rest, 3)
        app._rc_wizard_start()
        assert app.rc_wizard_active
        # 先静止一会儿让第 1 步开闸——零点必须是"用户还没动"的状态。
        drive_wizard(app, rest, 2 * panel.RC_WIZARD_HOLD_FRAMES)
        assert app.rc_wizard_index == 0

        pushed = list(rest)
        pushed[2] = 2000                       # 油门推到顶
        drive_wizard(app, pushed, 10 * panel.RC_WIZARD_HOLD_FRAMES)
        # 一直按着不动：只应完成第 1 步，绝不能连着往下跑。
        assert app.rc_wizard_index == 1, f"按住不动却前进到了第 {app.rc_wizard_index + 1} 步"
        assert app.rc_wizard_active

        pulled = list(rest)
        pulled[2] = 1000                       # 拉到底才进第 2 步
        drive_wizard(app, pulled, 4 * panel.RC_WIZARD_HOLD_FRAMES)
        assert app.rc_wizard_index == 2
    finally:
        app.destroy()


def test_rc_lines_are_routed_to_the_rc_page() -> None:
    handler = function_body(SOURCE, "    def _handle_board_line(")
    assert '_rc_handle_map_line(line)' in handler
    assert '_rc_handle_live_line(line)' in handler
    # RCMAP 必须排在 "RC " 前面，否则 startswith 会先匹配到通用分支。
    assert handler.index('startswith("RCMAP ")') < handler.index('startswith("RC ")')


# --------------------------------------------------------------------------
# 崩溃与排查线索
#
# 2026-08-29 用户反馈：校准走到某一步"直接闪退"，没有任何可查线索。Tk 默认把回调
# 异常打到 stderr，而从资源管理器/IDE 启动时根本没有 stderr——用户只看到窗口消失，
# 做到一半的校准全丢。
# --------------------------------------------------------------------------

def test_a_truncated_frame_cannot_crash_the_detector() -> None:
    """串口把一行截断过时，参考快照可能不足 16 路，而这条路径每帧都跑。"""
    assert panel.rc_wizard_dominant([1500] * 4, [1500] * 16) == (None, 0)
    assert panel.rc_wizard_dominant([1500] * 16, [1500] * 4) == (None, 0)
    # 残缺帧不该开闸，但更不该抛异常——这条路径每帧都跑。
    open_gate, _detail = panel.rc_wizard_gate_open(
        [[1500] * 4] * panel.RC_WIZARD_HOLD_FRAMES, [1500] * 4, release_channel=9
    )
    assert open_gate is False
    open_gate, _detail = panel.rc_wizard_gate_open(
        [[1500] * 16] * panel.RC_WIZARD_HOLD_FRAMES, [1500] * 4, release_channel=9
    )
    assert open_gate is True


def test_callback_exceptions_are_logged_and_the_window_survives() -> None:
    handler = function_body(SOURCE, "    def report_callback_exception(")
    assert "record_panel_crash(" in handler
    assert "showerror" in handler
    # 同一个错误往往每帧都触发；不能弹到用户点不完。
    assert "self._crash_count == 1" in handler
    assert "raise" not in handler


def test_the_crash_log_names_a_file_the_user_can_send() -> None:
    assert "PANEL_CRASH_LOG = LOG_DIR" in STATE_SOURCE
    record = STATE_SOURCE[STATE_SOURCE.index("def record_panel_crash("):]
    record = record[: record.index("\n\n__all__", 1)]
    assert "traceback.format_exception" in record
    assert "append_log(PANEL_CRASH_LOG" in record
    # 写日志失败不能再把程序带下去。
    append = STATE_SOURCE[STATE_SOURCE.index("def append_log("):]
    assert "except OSError:" in append[: append.index("\ndef ", 1)]


def test_the_wizard_leaves_a_trace_of_every_state_change() -> None:
    """卡住/绑错/闪退都要能从日志复盘，而不是靠用户回忆。"""
    trace = function_body(SOURCE, "    def _rc_wizard_trace(")
    assert "RC_WIZARD_TRACE_LOG" in trace
    assert "us=" in trace
    feed = function_body(SOURCE, "    def _rc_wizard_feed(")
    for event in ("gate_wait", "gate_open", "captured", "waiting"):
        assert f'"{event}"' in feed, event
    # 只记状态跳变，不记每一帧，否则一次校准会写出上万行。
    assert "self._rc_wizard_last_detail" in feed
    for name, event in (
        ("    def _rc_wizard_cancel(", "cancel"),
        ("    def _rc_wizard_finish(", "finish"),
    ):
        assert f'"{event}"' in function_body(SOURCE, name), event

"""R-T1-5：工作台布局模型（`panel_lib/dashboard/layout.py`）。

布局规则是纯数据变换，所以这一份**不需要显示环境**：吸附、越界钳制、重叠拒绝、
序列化往返、出厂预设，全部在没有 Tk 的情况下断言。页面行为在
`test_dashboard_page.py` 里用真实 `DronePanel()` 覆盖。
"""

from __future__ import annotations

import json

from tools.panel_lib.dashboard.layout import (
    DASHBOARD_COLUMNS,
    DASHBOARD_MAX_ROWS,
    PARAM_CHANNEL_NAMES,
    TILE_PARAM,
    TILE_VALUE,
    TILE_WAVE,
    DashboardLayout,
    TileSpec,
    Workspace,
    can_place,
    clamp_tile,
    controller_tuning_workspace,
    default_layout,
    find_free_slot,
    flight_monitor_workspace,
)


def wave(col: int, row: int, colspan: int = 6, rowspan: int = 5) -> TileSpec:
    return TileSpec(TILE_WAVE, col, row, colspan, rowspan, ["roll"])


# ---------------------------------------------------------------- 几何


def test_clamp_pulls_a_tile_back_into_the_grid() -> None:
    """拖出网格不是错误，是"拖过头了"，钳回来即可。"""
    spec = clamp_tile(TileSpec(TILE_VALUE, col=20, row=-3, colspan=99, rowspan=0))
    assert spec.col >= 0 and spec.row >= 0
    assert spec.colspan == DASHBOARD_COLUMNS
    assert spec.col + spec.colspan <= DASHBOARD_COLUMNS
    assert spec.rowspan >= 1


def test_overlap_is_rejected_rather_than_pushed_aside() -> None:
    """自动挤开会让一次拖动连锁改动半个工作区，用户看不懂也撤不回。"""
    tiles = [wave(0, 0), wave(6, 0)]
    assert not can_place(tiles, wave(3, 0))          # 压在第一张上
    assert can_place(tiles, wave(0, 5))              # 下面一行是空的
    # 恰好相邻不算重叠。
    assert can_place([wave(0, 0, 6, 5)], TileSpec(TILE_VALUE, 6, 0, 6, 5, []))


def test_out_of_grid_candidates_are_rejected() -> None:
    assert not can_place([], TileSpec(TILE_VALUE, 10, 0, 6, 2, []))
    assert not can_place([], TileSpec(TILE_VALUE, -1, 0, 3, 2, []))
    assert not can_place([], TileSpec(TILE_VALUE, 0, DASHBOARD_MAX_ROWS, 3, 2, []))
    assert not can_place([], TileSpec(TILE_VALUE, 0, 0, 0, 2, []))


def test_moving_a_tile_ignores_its_own_old_footprint() -> None:
    """拖动时不能把自己算成障碍物，否则第一格都挪不动。"""
    tile = wave(0, 0)
    tiles = [tile, wave(6, 0)]
    candidate = TileSpec(TILE_WAVE, 0, 1, 6, 5, [])
    assert can_place(tiles, candidate, ignore=tile)


def test_free_slot_scans_left_to_right_top_down() -> None:
    tiles = [TileSpec(TILE_VALUE, 0, 0, 3, 2, []), TileSpec(TILE_VALUE, 3, 0, 3, 2, [])]
    assert find_free_slot(tiles, 3, 2) == (6, 0)
    filled = [TileSpec(TILE_VALUE, col, 0, 3, 2, []) for col in (0, 3, 6, 9)]
    assert find_free_slot(filled, 3, 2) == (0, 2)


# ---------------------------------------------------------------- 序列化


def test_layout_round_trips_through_json() -> None:
    layout = default_layout()
    layout.workspaces[0].tiles[0].options["window_s"] = 30
    restored = DashboardLayout.loads(layout.dumps())

    assert restored is not None
    assert [w.name for w in restored.workspaces] == [w.name for w in layout.workspaces]
    original = layout.workspaces[0].tiles
    copy = restored.workspaces[0].tiles
    assert len(copy) == len(original)
    for before, after in zip(original, copy):
        assert (after.type, after.col, after.row, after.colspan, after.rowspan) == (
            before.type, before.col, before.row, before.colspan, before.rowspan
        )
        assert after.bindings == before.bindings
        assert after.options == before.options


def test_a_corrupt_entry_costs_only_that_tile() -> None:
    """状态文件是可以被手改的；一个字段写错不该让整个上位机起不来。"""
    raw = default_layout().to_json()
    raw["workspaces"][0]["tiles"].insert(0, {"type": 123})          # 类型不是字符串
    raw["workspaces"][0]["tiles"].insert(0, "not a dict")
    restored = DashboardLayout.from_json(raw)

    assert restored is not None
    assert len(restored.workspaces[0].tiles) == len(
        flight_monitor_workspace().tiles
    )


def test_garbage_falls_back_to_none_so_the_caller_can_use_presets() -> None:
    assert DashboardLayout.loads("{not json") is None
    assert DashboardLayout.from_json({"workspaces": []}) is None
    assert DashboardLayout.from_json("nope") is None


def test_overlapping_tiles_in_a_hand_edited_file_are_relocated() -> None:
    """两个 tile 叠在一起，界面上会有一个永远点不到。载入时重新找位置。"""
    raw = {
        "workspaces": [{
            "name": "w",
            "tiles": [
                {"type": TILE_VALUE, "col": 0, "row": 0, "colspan": 3, "rowspan": 2},
                {"type": TILE_VALUE, "col": 0, "row": 0, "colspan": 3, "rowspan": 2},
            ],
        }],
    }
    restored = DashboardLayout.from_json(raw)
    assert restored is not None
    first, second = restored.workspaces[0].tiles
    assert not first.geometry.overlaps(second.geometry)


def test_active_index_is_clamped_on_load() -> None:
    raw = default_layout().to_json()
    raw["active"] = 99
    restored = DashboardLayout.from_json(raw)
    assert restored is not None
    assert 0 <= restored.active < len(restored.workspaces)


def test_dumps_is_human_readable_json() -> None:
    """用户会手改导出的布局，所以必须是缩进过、非 ASCII 不转义的 JSON。"""
    text = default_layout().dumps()
    assert "飞行监控" in text
    assert json.loads(text)["version"] == 1


# ---------------------------------------------------------------- 预设


def test_flight_monitor_preset_matches_the_authors_call() -> None:
    """作者的原话：光流高度、速度这类量直接显示数值，不画曲线。"""
    workspace = flight_monitor_workspace()
    waves = [t for t in workspace.tiles if t.type == TILE_WAVE]
    values = [t for t in workspace.tiles if t.type == TILE_VALUE]

    assert len(waves) == 3 and len(values) == 4
    assert [t.bindings for t in waves] == [
        ["roll", "pitch", "yaw"], ["vel_est_x", "vel_est_y"], ["pos_est_x", "pos_est_y"],
    ]
    assert ["flow_height"] in [t.bindings for t in values]
    # uptime 绝不能和姿态角同图：R-T1-3 就是被这条毁掉的（600 s 把其余压成直线）。
    for tile in waves:
        assert "uptime" not in tile.bindings


def test_controller_tuning_preset_puts_sliders_and_waves_on_one_screen() -> None:
    """R-T1-3 被打回的直接原因：调参时滑块和波形不同屏。"""
    workspace = controller_tuning_workspace()
    params = [t for t in workspace.tiles if t.type == TILE_PARAM]
    waves = [t for t in workspace.tiles if t.type == TILE_WAVE]

    assert len(params) == len(PARAM_CHANNEL_NAMES) == 14
    assert len(waves) == 2
    assert [t.bindings[0] for t in params] == list(PARAM_CHANNEL_NAMES)


def test_presets_have_no_overlaps() -> None:
    for builder in (flight_monitor_workspace, controller_tuning_workspace):
        tiles = builder().tiles
        for index, tile in enumerate(tiles):
            assert can_place(tiles[:index], tile), f"{builder.__name__} tile {index}"


def test_bound_channels_are_deduplicated_in_first_seen_order() -> None:
    workspace = Workspace("w", [
        TileSpec(TILE_WAVE, 0, 0, 6, 5, ["roll", "pitch"]),
        TileSpec(TILE_VALUE, 6, 0, 3, 2, ["pitch"]),
        TileSpec(TILE_VALUE, 9, 0, 3, 2, ["yaw"]),
    ])
    assert workspace.bound_channels() == ["roll", "pitch", "yaw"]


def test_rows_used_covers_the_tallest_tile() -> None:
    # 波形 5 行 × 2 排 = 10；右下角那两排数值卡（row 5..8）盖不过波形。
    assert flight_monitor_workspace().rows_used() == 10
    assert Workspace("empty", []).rows_used() == 1

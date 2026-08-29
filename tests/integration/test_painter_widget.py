"""
Painter Widget Integration Tests

Pro-tier widget driven by user JS. Tests cover JSON round-trip of
painterCode, the auto/Pro promotion in commercialCfg, and graceful behaviour
when painter groups are loaded into a non-Pro environment.

Copyright (C) 2020-2026 Alex Spataru
SPDX-License-Identifier: LicenseRef-SerialStudio-Commercial
"""

import time

import pytest

from utils import SerialStudioClient

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

DEFAULT_PAINTER_CODE = (
    "function paint(ctx, w, h) {\n"
    "  ctx.fillStyle = '#000';\n"
    "  ctx.fillRect(0, 0, w, h);\n"
    "}\n"
)


def _painter_project(code: str = DEFAULT_PAINTER_CODE) -> dict:
    """Build a minimal project JSON containing a single painter group."""
    group = {
        "title": "Custom",
        "widget": "painter",
        "datasets": [],
        "painterCode": code,
    }
    return {
        "title": "Painter Test",
        "frameStart": "/*",
        "frameEnd": "*/",
        "frameDetection": 1,
        "checksum": "",
        "frameParser": "function parse(frame) { return [frame]; }",
        "groups": [group],
    }


# ---------------------------------------------------------------------------
# JSON round-trip
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_painter_code_round_trip(api_client, clean_state):
    """painterCode set on a painter group must survive save/load."""
    project = _painter_project()
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    groups = config["groups"]
    assert len(groups) == 1
    assert groups[0]["widget"] == "painter"
    assert groups[0]["painterCode"] == DEFAULT_PAINTER_CODE


@pytest.mark.project
def test_painter_group_with_datasets_loads(api_client, clean_state):
    """Painter groups must accept datasets (the script reads dataset values)."""
    project = _painter_project()
    project["groups"][0]["datasets"] = [
        {"title": "Speed", "index": 1, "units": "km/h", "min": 0, "max": 100},
        {"title": "Heading", "index": 2, "units": "deg", "min": 0, "max": 360},
    ]
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    assert len(config["groups"][0]["datasets"]) == 2


@pytest.mark.project
def test_empty_painter_code_field_omitted(api_client, clean_state):
    """If painterCode is empty, the writer should omit the key (back-compat)."""
    project = _painter_project(code="")
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    assert "painterCode" not in config["groups"][0]


# ---------------------------------------------------------------------------
# Pro detection
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_painter_flags_commercial_features(api_client, clean_state):
    """Loading a painter project must set the Pro/commercial-features flag."""
    project = _painter_project()
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    status = api_client.get_project_status()
    assert (
        status.get("containsCommercialFeatures") is True
    ), "Painter widget projects must trip commercialCfg"


# ---------------------------------------------------------------------------
# Editor / dashboard wiring
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_painter_group_widget_id_persists(api_client, clean_state):
    """Selecting widget=painter must round-trip exactly (no rename to fallback)."""
    project = _painter_project()
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    api_client.set_operation_mode("project")
    time.sleep(0.1)
    api_client.command("project.activate")
    time.sleep(0.2)

    config = api_client.command("project.exportJson")["config"]
    assert config["groups"][0]["widget"] == "painter"


# ---------------------------------------------------------------------------
# Bad / broken scripts
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_painter_with_syntax_error_does_not_crash(api_client, clean_state):
    """A painter group with broken JS must still load and round-trip."""
    project = _painter_project(code="this is not valid js {{{")
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    api_client.set_operation_mode("project")
    time.sleep(0.1)
    api_client.command("project.activate")
    time.sleep(0.5)

    status = api_client.command("io.getStatus")
    assert "error" not in status or not status["error"]


@pytest.mark.project
def test_painter_with_runaway_loop_persists(api_client, clean_state):
    """A painter that infinite-loops must still allow the project to function.

    The watchdog interrupts the script; the dashboard tick keeps running.
    """
    runaway = "function paint(ctx, w, h) {\n" "  while (true) {}\n" "}\n"
    project = _painter_project(code=runaway)
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    api_client.set_operation_mode("project")
    time.sleep(0.1)
    api_client.command("project.activate")
    time.sleep(1.0)

    config = api_client.command("project.exportJson")["config"]
    assert config["groups"][0]["widget"] == "painter"


# ---------------------------------------------------------------------------
# hideOnDashboard flag
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_hide_on_dashboard_round_trips(api_client, clean_state):
    """hideOnDashboard on a dataset must survive save/load."""
    project = _painter_project()
    project["groups"][0]["datasets"] = [
        {
            "title": "Hidden",
            "index": 1,
            "units": "",
            "min": 0,
            "max": 100,
            "hideOnDashboard": True,
        },
        {
            "title": "Visible",
            "index": 2,
            "units": "",
            "min": 0,
            "max": 100,
        },
    ]
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    datasets = config["groups"][0]["datasets"]
    titles_to_hidden = {d["title"]: d.get("hideOnDashboard", False) for d in datasets}
    assert titles_to_hidden["Hidden"] is True
    assert titles_to_hidden["Visible"] is False


@pytest.mark.project
def test_hide_on_dashboard_omitted_when_false(api_client, clean_state):
    """hideOnDashboard:false should not write the key (back-compat)."""
    project = _painter_project()
    project["groups"][0]["datasets"] = [
        {"title": "A", "index": 1, "min": 0, "max": 100, "hideOnDashboard": False},
    ]
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    ds = config["groups"][0]["datasets"][0]
    assert "hideOnDashboard" not in ds


# ---------------------------------------------------------------------------
# Multi-group painter behaviour
# ---------------------------------------------------------------------------


@pytest.mark.project
def test_two_painter_groups_both_persist(api_client, clean_state):
    """Two painter groups in one project must both survive a round trip.

    Regression coverage for the m_widgetGroups.remove(key) bug that nuked all
    painter entries when one group hit the GPL fallback path.
    """
    project = _painter_project()
    project["title"] = "Two Painters"
    project["groups"].append(
        {
            "title": "Second",
            "widget": "painter",
            "datasets": [{"title": "X", "index": 1, "min": 0, "max": 100}],
            "painterCode": DEFAULT_PAINTER_CODE,
        }
    )
    project["groups"][0]["title"] = "First"
    project["groups"][0]["datasets"] = [
        {"title": "X", "index": 2, "min": 0, "max": 100},
    ]
    api_client.load_project_from_json(project)
    time.sleep(0.3)

    config = api_client.command("project.exportJson")["config"]
    titles = [g["title"] for g in config["groups"]]
    assert "First" in titles
    assert "Second" in titles
    assert all(g["widget"] == "painter" for g in config["groups"])

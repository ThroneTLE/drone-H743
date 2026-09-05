"""R-S1-3（TK-00）：改版报告 N01–N13 的**基线观测**脚本。

这不是回归测试，也**不会**被 pytest 收集。它回答的是一个只有运行才能回答的问题：
报告写在基线 `6f7a440e` 上的那些现象，在**当前** HEAD 上还成不成立？

TK-00 的完成门明确禁止两件事：不把“仍能复现 bug”包装成回归通过，也不把尚未修复的
产品期望挂进默认 pytest 再用 skip/xfail 掩盖。所以这里的每一项只输出**观测到的
事实**（异常类型、越界像素、写了几行、文件被覆盖没有），由人和实现包去判断。对应
的实现包接手时，把这里的观测改写成“正确行为”的失败测试，再修到绿。

运行：

    python -m tools.panel_qa.baseline_observations

结果写到 `data/analysis/tk_revamp/<今天>/baseline_<commit>/observations.json`。
所有落盘都在隔离目录里做，只有最后的证据 JSON 落到 `data/`。
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

from . import fixtures as qa_fixtures
from .geometry import WINDOW_SIZES
from .guards import hardware_guards
from .harness import OfflinePanel
from .isolation import exclusive_path, isolated_environment


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _head_commit() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    except Exception:                            # noqa: BLE001 - 不在 git 工作树里
        return "unknown"


# ---------------------------------------------------------------- N01 主题


def observe_input_theme(session) -> dict:
    """N01：Spinbox 是否配了深色字段底。基线上 `TSpinbox.fieldbackground` 是空的。"""
    from tkinter import ttk

    style = ttk.Style(session.panel)
    return {
        "issue": "N01",
        "styles": {
            widget: {
                key: style.lookup(widget, key)
                for key in ("foreground", "fieldbackground", "background")
            }
            for widget in ("TEntry", "TSpinbox", "TCombobox", "Treeview")
        },
    }


# ---------------------------------------------------------------- N02/N03 布局


def observe_layout(session) -> dict:
    """N02/N03：三尺寸下够不到的交互控件。判据见 `geometry.py`。"""
    reports = session.probe_geometry(sizes=WINDOW_SIZES)
    return {
        "issue": "N02/N03",
        "scale_simulated": session.scale,
        "pages": len({r.page for r in reports}),
        "cases": len(reports),
        "cases_with_unreachable_controls": sum(1 for r in reports if not r.clean),
        "detail": [r.to_json() for r in reports if not r.clean],
    }


# ---------------------------------------------------------------- N04 滚轮


def observe_dashboard_mousewheel(session) -> dict:
    """N04：给状态监视视口造出可滚内容，再看滚轮事件动不动得了 yview。"""
    panel = session.panel
    panel.notebook.select(panel.dashboard_tab)
    session.resize(1080, 700)
    canvas = panel.dashboard_host.canvas
    panel.unbind_all("<MouseWheel>")
    canvas.configure(scrollregion=(0, 0, 500, 2000))
    canvas.yview_moveto(0)
    before = canvas.yview()
    canvas.event_generate("<MouseWheel>", delta=-120)
    panel.update_idletasks()
    return {
        "issue": "N04",
        "yview_before": list(before),
        "yview_after": list(canvas.yview()),
        "canvas_binding": canvas.bind("<MouseWheel>"),
        "scrolled": list(before) != list(canvas.yview()),
    }


# ---------------------------------------------------------------- N05 输入校验


def observe_invalid_servo_input(session) -> dict:
    """N05：维护舵机页填非数字后点“移动此舵机”，看用户拿到的是字段错误还是内部异常。"""
    panel = session.panel
    session.transport.lines.clear()
    original = panel.servo_widgets[0]["pulse"].get()
    panel.servo_widgets[0]["pulse"].set("abc")
    try:
        panel._servo_move(0)
        outcome = "no error"
    except Exception as exc:                     # noqa: BLE001 - 观测的就是它
        outcome = f"{type(exc).__name__}: {exc}"
    finally:
        panel.servo_widgets[0]["pulse"].set(original)
    return {"issue": "N05", "outcome": outcome, "sent": list(session.transport.lines)}


# ---------------------------------------------------------------- N06 PID 契约


def observe_pid_ki_contract(session) -> dict:
    """N06：只填 KI 时上位机发了什么，以及固件 `PID SET` 到底认哪些项。"""
    panel = session.panel
    session.transport.lines.clear()
    session.transport.frames.clear()
    for terms in panel.pid_vars.values():
        for var in terms.values():
            var.set("")
    panel.pid_vars["roll"]["ki"].set("0.1")
    panel._send_pid_values()
    # PID SET 走的是二进制帧而不是文本行，两条出口都要看。
    sent_frames = [
        payload.decode("utf-8", "replace") for _function, payload in session.transport.frames
    ]

    # 固件那一侧的契约就是它自己的 usage 串，逐字取出来，不转述。
    control_path = PROJECT_ROOT / "App" / "Src" / "app_control.c"
    control_source = control_path.read_text(encoding="utf-8", errors="replace")
    usage = next(
        (line.strip() for line in control_source.splitlines()
         if "ERR usage PID SET" in line),
        "",
    )
    return {
        "issue": "N06",
        "sent_lines": list(session.transport.lines),
        "sent_frames": sent_frames,
        "firmware_usage_line": usage,
        "firmware_source": f"{control_path.relative_to(PROJECT_ROOT).as_posix()}",
        "firmware_accepts_ki": "ki=" in usage,
    }


# ---------------------------------------------------------------- N07 参数草稿


def observe_param_draft_overwrite(session) -> dict:
    """N07：暂存一个未发送草稿，然后收一条后台回读，看草稿还在不在。"""
    panel = session.panel
    name = "coax.vel_x_kp"
    panel._set_param(name, "1.2", "local", True)
    before = dict(panel.params[name])
    panel._update_param_line(f"PARAM name={name} value=0.8")
    return {
        "issue": "N07",
        "draft_before_echo": before,
        "after_echo": dict(panel.params[name]),
    }


# ---------------------------------------------------------------- N08/N09 GPS


def observe_invalid_gps(session) -> dict:
    """N08/N09：固件说 `valid=0`，但 pos 行仍带旧坐标；同时本页不可见。

    夹具用 `panel_qa.fixtures` 的固件格式串生成，不是手打的报文。
    """
    panel = session.panel
    panel.gps_track.clear()
    panel.gps_origin_lat = None
    panel.gps_origin_lon = None
    # 停在机械校准页：GPS 页此刻不可见。
    panel.notebook.select(panel.calibration_group_tab)
    panel.calibration_notebook.select(panel.mechanical_tab)

    draws: list[str] = []
    original_plot = panel._update_gps_plot
    panel._update_gps_plot = lambda: draws.append("draw")
    try:
        for _ in range(5):
            panel.gps_last_plot_ns = 0
            panel._update_gps_line(qa_fixtures.gps_status_line(
                ok=1, init=0, fix=0, valid=0, sv=0, age_ms=5000))
            panel._update_gps_line(qa_fixtures.gps_position_line())
    finally:
        panel._update_gps_plot = original_plot
    return {
        "issue": "N08/N09",
        "visible_page": "校准 / 舵机机械中心与行程",
        "track_points_from_invalid_fixes": len(panel.gps_track),
        "origin_latitude": panel.gps_origin_lat,
        "plot_calls_while_hidden": len(draws),
        "state_text": panel.gps_vars["state"].get(),
    }


# ---------------------------------------------------------------- N10-N13 录制
#
# 已由 R-T1-6（TK-05）修复，观测改写成了真正的回归契约：
#   tests/test_dashboard_record_service.py
# 修复前的观测原文保留在
#   data/analysis/tk_revamp/2026-09-04/baseline_2acfd82e/observations.json
# 的 n10_synchronous_flush / n11_write_failure / n12_schema_change /
# n13_same_second_collision 四段里。观测脚本不再驱动那条已经不存在的同步写盘
# 路径——留着它只会给出一个和产品无关的“仍能复现”。


# ---------------------------------------------------------------- 主流程


def run(output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    observations: dict = {
        "generated": datetime.now().isoformat(timespec="seconds"),
        "head_commit": _head_commit(),
        "report_baseline_commit": "6f7a440e",
        "kind": "baseline observation — records current behaviour, asserts nothing",
        "observations": [],
    }
    with hardware_guards() as guard_log:
        with tempfile.TemporaryDirectory() as scratch:
            scratch_root = Path(scratch)
            with isolated_environment(scratch_root):
                for scale in (1.0,):
                    with OfflinePanel.launch(scale=scale, size=(1366, 768)) as session:
                        observations["observations"].extend([
                            observe_input_theme(session),
                            observe_layout(session),
                            observe_dashboard_mousewheel(session),
                            observe_invalid_servo_input(session),
                            observe_pid_ki_contract(session),
                            observe_param_draft_overwrite(session),
                            observe_invalid_gps(session),
                        ])
                        observations["callback_errors"] = [
                            repr(e) for e in session.callback_errors
                        ]
        observations["hardware_guard"] = {
            "physical_serial_opens": guard_log.serial_opens,
            "flash_tool_invocations": guard_log.flash_invocations,
            "clean": guard_log.clean,
        }
    # 输出也走排他创建。这份脚本存在的全部理由就是保住“修复前”的那一份证据，
    # 再用 "w" 把它盖掉就太讽刺了。
    target = exclusive_path(output_dir, "observations", ".json")
    target.write_text(
        json.dumps(observations, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    observations["output_path"] = str(target)
    return observations


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv:
        output_dir = Path(argv[0])
    else:
        output_dir = (
            PROJECT_ROOT / "data" / "analysis" / "tk_revamp"
            / datetime.now().strftime("%Y-%m-%d")
            / f"baseline_{_head_commit()}"
        )
    result = run(output_dir)
    print(json.dumps({
        "head_commit": result["head_commit"],
        "output": result.get("output_path"),
        "hardware_guard_clean": result["hardware_guard"]["clean"],
        "observations": [entry["issue"] for entry in result["observations"]],
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":                       # pragma: no cover - 手动运行
    raise SystemExit(main())

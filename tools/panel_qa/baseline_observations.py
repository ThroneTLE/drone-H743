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

import io
import json
import subprocess
import sys
import tempfile
from collections import deque
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

from . import fixtures as qa_fixtures
from .geometry import WINDOW_SIZES
from .guards import hardware_guards
from .harness import OfflinePanel
from .isolation import (
    exclusive_path,
    isolated_environment,
    redirected_dated_directory,
)


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


class _SlowWriter:
    def __init__(self, delay_s: float = 0.002) -> None:
        self.writes = 0
        self.delay_s = delay_s

    def write(self, text: str) -> None:
        import time

        self.writes += 1
        time.sleep(self.delay_s)


class _BrokenWriter:
    def write(self, text: str) -> None:
        raise OSError("simulated disk full")


def observe_recording(output_dir: Path) -> dict:
    """N10–N13：慢盘阻塞、失败不收尾、schema 变化撑破表头、同秒文件名碰撞。

    直接驱动 `pages/dashboard.py` 的真实 flush/toggle/stop，只把 writer 与时钟换掉。
    """
    import time

    from tools.panel_lib.pages import dashboard as module
    from tools.panel_lib.pages.dashboard import DashboardPageMixin

    result: dict = {"issue": "N10-N13"}

    # N10：一次渲染回调里同步写完整条队列。
    writer = _SlowWriter()
    subject = SimpleNamespace(
        dashboard_record_handle=writer,
        dashboard_schema=SimpleNamespace(channel_count=2),
        dashboard_record_rows=deque(
            SimpleNamespace(t_us=i * 25000, values={0: float(i)}) for i in range(200)
        ),
    )
    started = time.perf_counter()
    DashboardPageMixin._dashboard_flush_record(subject)
    result["n10_synchronous_flush"] = {
        "injected_write_latency_ms": writer.delay_s * 1000,
        "queued_rows": 200,
        "blocking_seconds": time.perf_counter() - started,
        "writes": writer.writes,
        "note": "这是不利存储条件模拟，不代表正常硬盘写入速度",
    }

    # N11：写失败之后录制会话有没有收尾。
    subject.dashboard_record_handle = _BrokenWriter()
    subject.dashboard_record_rows.append(SimpleNamespace(t_us=0, values={0: 1.0}))
    failure: dict = {"raised": None}
    try:
        DashboardPageMixin._dashboard_flush_record(subject)
    except Exception as exc:                     # noqa: BLE001 - 观测的就是它
        failure["raised"] = f"{type(exc).__name__}: {exc}"
    failure["handle_still_active"] = subject.dashboard_record_handle is not None
    failure["queued_rows_left"] = len(subject.dashboard_record_rows)
    result["n11_write_failure"] = failure

    # N12：录制中途换 schema，行宽跟着当前全局 schema 走，表头却是旧的。
    buffer = io.StringIO()
    buffer.write("t_s,a,b\n")
    subject.dashboard_record_handle = buffer
    subject.dashboard_schema = SimpleNamespace(channel_count=2)
    subject.dashboard_record_rows = deque([SimpleNamespace(t_us=0, values={0: 1.0, 1: 2.0})])
    DashboardPageMixin._dashboard_flush_record(subject)
    subject.dashboard_schema.channel_count = 3
    subject.dashboard_record_rows.append(
        SimpleNamespace(t_us=25000, values={0: 1.0, 1: 2.0, 2: 3.0})
    )
    DashboardPageMixin._dashboard_flush_record(subject)
    result["n12_schema_change"] = {
        "row_widths": [len(line.split(",")) for line in buffer.getvalue().splitlines()],
        "csv": buffer.getvalue(),
    }

    # N13：同一秒内再次开始录制，文件名相同且以 "w" 打开。
    folder = output_dir / "record_probe"
    folder.mkdir(parents=True, exist_ok=True)
    schema = SimpleNamespace(
        complete=True, channel_count=2, computed_hash=lambda: 0x12345678,
        ordered=lambda: [SimpleNamespace(name="a"), SimpleNamespace(name="b")],
    )
    record = SimpleNamespace(
        dashboard_record_handle=None, dashboard_schema=schema,
        dashboard_record_rows=deque(),
        dashboard_record_var=SimpleNamespace(set=lambda *_a: None),
    )
    record._dashboard_flush_record = lambda: DashboardPageMixin._dashboard_flush_record(record)
    record._dashboard_stop_record = lambda: DashboardPageMixin._dashboard_stop_record(record)

    class _FixedClock:
        @staticmethod
        def now():
            return SimpleNamespace(strftime=lambda _fmt: "120000")

    original_datetime, original_dated = module.datetime, module.dated_directory
    module.datetime = _FixedClock
    module.dated_directory = lambda *_a: folder
    try:
        DashboardPageMixin._dashboard_toggle_record(record)
        record.dashboard_record_rows.append(
            SimpleNamespace(t_us=25000, values={0: 123.0, 1: 456.0})
        )
        DashboardPageMixin._dashboard_stop_record(record)
        first_path = record.dashboard_record_path
        before_text = first_path.read_text(encoding="utf-8")

        DashboardPageMixin._dashboard_toggle_record(record)
        second_path = record.dashboard_record_path
        DashboardPageMixin._dashboard_stop_record(record)
        after_text = second_path.read_text(encoding="utf-8")
    finally:
        module.datetime, module.dated_directory = original_datetime, original_dated

    result["n13_same_second_collision"] = {
        "same_path": str(first_path) == str(second_path),
        "path": first_path.name,
        "data_rows_before": len(before_text.splitlines()) - 2,
        "data_rows_after": len(after_text.splitlines()) - 2,
        "first_sample_survives": ",123,456" in after_text,
    }
    return result


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
                with redirected_dated_directory(scratch_root / "telemetry"):
                    observations["observations"].append(observe_recording(scratch_root))
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

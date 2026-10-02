"""离线测试套件的全局硬件护栏与状态隔离。

两条路都要堵，而且必须在**收集之前**就装上：

  * 物理串口 —— `serial.Serial.open`
  * 烧录 / 调试探针 —— `subprocess.Popen`（openocd / STM32_Programmer / JLink / pyocd …）

原来这里只有串口那条。`tools/panel_qa` 里的 subprocess 护栏只在主动进入
`hardware_guards()` 的局部作用域生效，于是"整个默认 pytest 具备双向保护"这个说法
并不成立（2026-09-04 软件审核 Q3）。AGENTS.md 第 5 条说的是未经 REQ 明文授权不得
烧录或复位，而 `subprocess` 完全绕开串口层。

刻意验证护栏的测试自己开一层 `hardware_guards()`，命中记在那一层的 log 里，不会
进全局计数——所以下面的"尝试次数"为非零时，一定是意外触碰，判整个 session 失败。

并行（2026-09-29 作者："你做成并行的"）：`python -m pytest -n auto tests` 用 pytest-xdist 按**文件**分发
（`--dist loadfile`：同一文件的用例在同一进程里按原顺序跑）。护栏在每个工作进程里各装一份；
工作进程结束时把自己的计数交回（`workeroutput`），主进程汇总后再显示、再判失败——否则主进程
自己的计数永远是 0，真出了意外触碰也看不见。

慢界面测试（`@pytest.mark.slow_ui`，如"几种窗口尺寸 × 几种缩放"的真面板布局矩阵）：默认
`--slow-ui=auto`，只有工作区里界面相关文件（UI_PATHS）相对 HEAD 有改动（含未跟踪文件）时才跑，
否则跳过并写明原因；`--slow-ui=on` 强制跑、`--slow-ui=off` 强制跳过。git 不可用时按"要跑"处理。
"""

import os
import subprocess

import pytest

#: 界面相关文件：这些有改动时才跑 slow_ui 测试。
UI_PATHS = ("tools/panel_lib/", "tools/drone_tcp_panel.py", "tools/panel_qa/")

#: `-n auto` 的工作进程上限。2026-09-29 两次教训：按 20 核开 20 个进程，作者电脑"卡到爆炸"；降到 6 个低优先级
#: 进程（那一轮强制跑了开真窗口的慢界面测试）仍然卡死，作者杀掉测试后立刻流畅——真窗口的负担落在窗口系统/显卡上，
#: 进程优先级管不到；每个现场编译的测试程序还会被 Windows Defender 扫描。所以默认只开 2 个；
#: 要更多须作者当次同意并选在不用电脑的时候，显式 `-n 6`（或环境变量 PYTEST_XDIST_AUTO_NUM_WORKERS）。
AUTO_WORKERS_MAX = 2


@pytest.hookimpl(optionalhook=True)
def pytest_xdist_auto_num_workers(config):
    return max(1, min(AUTO_WORKERS_MAX, (os.cpu_count() or 2) // 3))


def _lower_own_priority() -> None:
    """并行的工作进程降到"低于正常"优先级：桌面和作者手上的程序永远先跑。
    Windows 上 BELOW_NORMAL 会被子进程继承，所以测试里调起的 gcc 也一起降。"""
    try:
        if os.name == "nt":
            import ctypes
            BELOW_NORMAL_PRIORITY_CLASS = 0x00004000
            kernel32 = ctypes.windll.kernel32
            kernel32.SetPriorityClass(kernel32.GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS)
        else:
            os.nice(10)
    except Exception:                            # pragma: no cover - 降不了就照常跑
        pass


def pytest_addoption(parser):
    parser.addoption(
        "--slow-ui", choices=("auto", "on", "off"), default="auto",
        help="慢界面测试（slow_ui）：auto = 界面文件有改动才跑（默认），on = 总跑，off = 总跳过")


def _ui_files_changed(rootdir) -> bool:
    """工作区里界面相关文件相对 HEAD 有没有改动（含未跟踪）。git 出错时返回 True（宁可多跑）。"""
    try:
        out = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=all", "--", *UI_PATHS],
            cwd=str(rootdir), capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return True
    if out.returncode != 0:
        return True
    return any(line.strip() for line in out.stdout.splitlines())


def pytest_collection_modifyitems(config, items):
    mode = config.getoption("--slow-ui")
    slow = [item for item in items if item.get_closest_marker("slow_ui") is not None]
    if not slow or mode == "on":
        return
    if mode == "auto" and _ui_files_changed(config.rootpath):
        return
    reason = ("slow_ui：界面文件没有改动，跳过慢界面布局测试（要跑用 --slow-ui=on）"
              if mode == "auto" else "slow_ui：--slow-ui=off")
    for item in slow:
        item.add_marker(pytest.mark.skip(reason=reason))


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "slow_ui: 慢界面布局测试（真面板尺寸×缩放矩阵等），默认只在界面文件有改动时跑")
    if hasattr(config, "workerinput"):           # xdist 工作进程
        _lower_own_priority()

    from tools.panel_qa.guards import install_hardware_guards
    from tools.panel_qa.harness import display_available

    # 在进程还干净的时候探一次图形环境。之后再出 Tk 错误就一定不是"没有显示"，
    # 而是装置或资源问题——必须红，不许静默 skip（2026-09-05 曾出现 15 个静默跳过）。
    config._display_available = display_available()

    log, uninstall = install_hardware_guards()
    config._hardware_guard_log = log
    config._hardware_guard_uninstall = uninstall

    # 统计整场跑创建了多少个 Tk root。Windows 上 Tcl 解释器反复创建/销毁到一定数量
    # 后会间歇性失败（报的是 "can't find a usable init.tcl"，和真正的无头环境同文），
    # 所以这个数字是排查装置层面不稳定的第一手依据，不是装饰。
    import tkinter

    config._tk_root_count = 0
    original_tk_init = tkinter.Tk.__init__

    def counting_init(self, *args, **kwargs):
        config._tk_root_count += 1
        result = original_tk_init(self, *args, **kwargs)
        # 兜底：**任何**在测试里建出来的 Tk root 一律先收起来。
        #
        # 这条不是装饰。跑一次固件测试会建 17 个 root，只要有一个没人收，桌面上
        # 就会弹出窗口、抢走焦点，把正在敲字的人打断——2026-09-07 作者就是这么
        # 撞上的。需要真实布局的装置（tools/panel_qa/harness.py）自己会挪到屏幕外
        # 再 deiconify；除它以外，没有任何测试有理由让窗口上屏。
        try:
            self.withdraw()
        except Exception:                        # pragma: no cover - root 建了一半
            pass
        return result

    tkinter.Tk.__init__ = counting_init
    config._tk_init_original = original_tk_init


@pytest.fixture(scope="session", autouse=True)
def isolate_panel_defaults(tmp_path_factory):
    """把整棵 `data/` 写入面挪到临时目录，并掐掉面板的启动副作用。

    以前这里只动 panel_state / 日志四项，理由是"全局重指会打掉
    `test_canonical_data_tree_is_root_scoped` 那条正当契约"。代价是真的付出来了：
    2026-09-05 一次普通的 pytest 跑把一份**伪造的**机械校准证据
    （`status=PERSISTED_READBACK_MATCH`）写进了真实的
    `data/calibration/servo_mechanical/`——那是 AGENTS.md 明令不许动的历史证据目录，
    假记录和真记录混在一起比漏测危险得多。

    正确的解法不是继续收窄，而是把两件事分开：
      * **运行期写到哪**——整棵重指到临时根，`panel_qa.isolated_environment()`
        按对象 identity 扫描 `tools.*`，一个常量都不漏；
      * **规范布局是什么**——`project_paths.CANONICAL_DATA_TREE` 在 import 时冻结，
        隔离碰不到它，契约测试改断言这份记录。

    进入时机是 session fixture 而不是 `pytest_configure`：收集阶段已经把所有测试
    模块（连带它们 import 的 `tools.*`）拉进来了，此时扫描才扫得全。之后才 import
    的模块从 `tools.project_paths` 现取，取到的也是改指后的值，两头都盖住。
    并行时每个工作进程各有一个 session，各自一棵临时根，互不相干。
    """
    from tools import drone_tcp_panel as panel
    from tools.panel_qa.isolation import isolated_environment

    patch = pytest.MonkeyPatch()
    root = tmp_path_factory.mktemp("data-root")
    original_init = panel.DronePanel.__init__

    def isolated_init(self, *args, **kwargs):
        # Only suppress startup side effects; explicit test calls retain the API.
        with pytest.MonkeyPatch.context() as startup:
            for method in ("_restore_last_connection", "_save_panel_state",
                           "_validation_load_latest_artifact", "_v1_load_latest_session"):
                startup.setattr(panel.DronePanel, method, lambda self: None)
            startup.setattr(panel.DronePanel, "_load_panel_state", lambda self: {})
            original_init(self, *args, **kwargs)

    patch.setattr(panel.DronePanel, "__init__", isolated_init)
    with isolated_environment(root):
        yield
    patch.undo()


def _local_guard_summary(config) -> dict:
    log = config._hardware_guard_log
    return dict(serial=len(log.serial_opens), flash=len(log.flash_invocations),
                attempts=[f"{d.kind}: {d.detail}" for d in log.attempts],
                tk_roots=getattr(config, "_tk_root_count", 0))


@pytest.hookimpl(optionalhook=True)
def pytest_testnodedown(node, error):
    """xdist 主进程：收下一个工作进程交回的护栏计数。"""
    summary = getattr(node, "workeroutput", {}).get("hardware_guard")
    if summary is None:
        return
    totals = node.config.__dict__.setdefault(
        "_worker_guard_totals", dict(serial=0, flash=0, attempts=[], tk_roots=0, workers=0))
    for key in ("serial", "flash", "tk_roots"):
        totals[key] += summary[key]
    totals["attempts"].extend(summary["attempts"])
    totals["workers"] += 1


def pytest_sessionfinish(session, exitstatus):
    config = session.config
    log = getattr(config, "_hardware_guard_log", None)
    if log is None:
        return
    if hasattr(config, "workeroutput"):          # xdist 工作进程：计数交回主进程
        config.workeroutput["hardware_guard"] = _local_guard_summary(config)
    totals = getattr(config, "_worker_guard_totals", None)
    if (not log.clean) or (totals is not None and totals["attempts"]):
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, config):
    log = getattr(config, "_hardware_guard_log", None)
    if log is None:                              # pragma: no cover - configure 失败
        return
    summary = _local_guard_summary(config)
    totals = getattr(config, "_worker_guard_totals", None)
    if totals is not None:                       # 并行：主进程本身不跑测试，显示各工作进程之和
        summary = dict(serial=summary["serial"] + totals["serial"],
                       flash=summary["flash"] + totals["flash"],
                       attempts=summary["attempts"] + totals["attempts"],
                       tk_roots=totals["tk_roots"])
    terminalreporter.write_line(f"Physical serial open attempts: {summary['serial']}")
    terminalreporter.write_line(f"Flashing/probe tool invocation attempts: {summary['flash']}")
    terminalreporter.write_line(
        f"Tk display available at session start: {getattr(config, '_display_available', '?')}"
        f"; Tk roots created: {summary['tk_roots']}"
        + (f" (summed over {totals['workers']} workers)" if totals is not None else ""))
    for detail in summary["attempts"]:
        terminalreporter.write_line(f"  blocked {detail}")


def pytest_unconfigure(config):
    uninstall = getattr(config, "_hardware_guard_uninstall", None)
    if uninstall is not None:
        uninstall()
    original = getattr(config, "_tk_init_original", None)
    if original is not None:
        import tkinter

        tkinter.Tk.__init__ = original

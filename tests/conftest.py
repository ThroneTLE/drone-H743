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
"""

import pytest


def pytest_configure(config):
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


def pytest_sessionfinish(session, exitstatus):
    log = getattr(session.config, "_hardware_guard_log", None)
    if log is not None and not log.clean:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, config):
    log = getattr(config, "_hardware_guard_log", None)
    if log is None:                              # pragma: no cover - configure 失败
        return
    terminalreporter.write_line(
        f"Physical serial open attempts: {len(log.serial_opens)}"
    )
    terminalreporter.write_line(
        f"Flashing/probe tool invocation attempts: {len(log.flash_invocations)}"
    )
    terminalreporter.write_line(
        f"Tk display available at session start: {getattr(config, '_display_available', '?')}"
        f"; Tk roots created: {getattr(config, '_tk_root_count', '?')}"
    )
    for detail in log.attempts:
        terminalreporter.write_line(f"  blocked {detail.kind}: {detail.detail}")


def pytest_unconfigure(config):
    uninstall = getattr(config, "_hardware_guard_uninstall", None)
    if uninstall is not None:
        uninstall()
    original = getattr(config, "_tk_init_original", None)
    if original is not None:
        import tkinter

        tkinter.Tk.__init__ = original

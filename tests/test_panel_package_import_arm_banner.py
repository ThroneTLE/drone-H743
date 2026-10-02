"""按包导入（`tools.drone_tcp_panel`）时 ARM 回包处理器也要在（2026-09-29 R-FLOWMOUNT-1 子任务发现）。

三支导入里，相对导入那支漏了 `arm_banner as _panel_arm_banner`：按包导入后任何 `RSP … mod=ARM`
走到 `_handle_rsp_line` 都会 NameError（直接 `python tools/drone_tcp_panel.py` 走第三支，不受影响）。
补在同一行里，主文件行数不变（只减不增）。
"""
import importlib


def test_package_import_binds_the_arm_banner_module():
    module = importlib.import_module("tools.drone_tcp_panel")
    arm_banner = importlib.import_module("tools.panel_lib.arm_banner")
    assert module._panel_arm_banner is arm_banner

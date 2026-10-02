#!/usr/bin/env python3
"""Build the compact, task-routed repository index used by the project skill."""

from __future__ import annotations

import argparse
import ast
import hashlib
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path, PurePosixPath


REPO_ROOT = Path(__file__).resolve().parents[4]
SKILL_ROOT = Path(__file__).resolve().parents[1]
INDEX_DIR = SKILL_ROOT / "references" / "repository-index"
GENERATOR_REL = ".agents/skills/drone-h743-project/scripts/update_repository_index.py"
INDEX_PREFIX = ".agents/skills/drone-h743-project/references/repository-index/"

ROOT_LIMIT = 8 * 1024
SHARD_LIMIT = 32 * 1024
TOTAL_LIMIT = 128 * 1024  # 2026-10-01 作者：“总上限放宽很多就行”（原 96 KiB，当夜为塞新功能把测试摘要削到 22 字）

SHARDS = (
    "firmware-app-services.md",
    "firmware-driver-bsp.md",
    "platform-build.md",
    "tests-firmware.md",
    "tests-host.md",
    "host-tools.md",
    "docs-data.md",
)

TEXT_EXTENSIONS = {
    ".c",
    ".h",
    ".cpp",
    ".s",
    ".ld",
    ".py",
    ".ps1",
    ".md",
    ".txt",
    ".cmake",
    ".json",
    ".toml",
    ".yaml",
    ".yml",
    ".ini",
    ".ioc",
    ".gitignore",
}

DATA_SCOPES = (
    ".tmp/",
    "data/",
)

VENDOR_SCOPES = (
    "Drivers/",
    "Middlewares/",
    "ThirdParty/",
    "driver_doc/",
)

PURPOSE_OVERRIDES = {
    "drone-H743.ioc": "CubeMX hardware and generated-configuration source of truth.",
    "CMakeLists.txt": "Top-level firmware build targets and project source lists.",
    "CMakePresets.json": "Named CMake configure/build presets.",
    "STM32H743XX_FLASH.ld": "STM32H743 flash/RAM regions and linker section placement.",
    "startup_stm32h743xx.s": "GCC startup, vector table, and reset entry.",
    ".clangd": "clangd compile database and indexing settings.",
    ".gitignore": "Repository ignore policy for generated artifacts and local data.",
    ".mcp.json": "Repository-local MCP server configuration.",
    ".codex/config.toml": "Repository-local Codex configuration.",
    "Core/Src/freertos.c": "CubeMX-owned FreeRTOS objects and task entry wiring.",
    "Core/Inc/rtos_objects.h": "Public handles and types for CubeMX-created RTOS objects.",
    "Core/Src/main.c": "CubeMX-owned reset-time initialization and scheduler startup.",
    "Core/Inc/main.h": "CubeMX-owned board pin names and shared MCU declarations.",
    "tools/README.md": "Host-tool catalog, dependencies, and common invocation examples.",
}


def git_working_files() -> list[str]:
    """Return tracked files plus non-ignored untracked files, with Git path semantics."""
    proc = subprocess.run(
        [
            "git",
            "-C",
            str(REPO_ROOT),
            "ls-files",
            "--cached",
            "--others",
            "--exclude-standard",
            "-z",
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    paths = proc.stdout.decode("utf-8", errors="surrogateescape").split("\0")
    result: list[str] = []
    for raw in paths:
        if not raw:
            continue
        path = raw.replace("\\", "/")
        if path.startswith(INDEX_PREFIX):
            continue
        if (REPO_ROOT / Path(path)).is_file():
            result.append(path)
    return sorted(set(result), key=str.casefold)


def read_bytes(path: str) -> bytes:
    return (REPO_ROOT / Path(path)).read_bytes()


def read_text(path: str) -> str:
    data = read_bytes(path)
    if b"\x00" in data[:4096]:
        return ""
    for encoding in ("utf-8", "utf-8-sig", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    return data.decode("utf-8", errors="replace")


# 用途列每格最多留多少字。
#
# 2026-09-12：RGB 状态灯拆成 drv_rgb_led / svc_led / bsp_rgb_led 三个模块外加两个
# 命令模块与一个测试文件后，索引总量到 98649 B，越过 96 KB 总限 345 B。
# 按既定处置——**改密度、不抬限制**——把用途列 120→112（实测 98649 → 97066 B）。
#
# 这次动用途列而不是再砍 symbol_cell（上次 6→5 动的是那里）：入口名是**精确的
# 检索键**，砍掉一个就等于少一条能直接 grep 的线索；用途是散文，截短仍然读得懂
# 它在说什么。两者都超限时先截散文。
#
# 2026-09-12（同日第二次）：状态灯颜色绑定新增 app_led_config / app_cmd_ledmap /
# led_map.py 与两个测试文件后又到 98470 B。这次一步到 96 而不是再挪 8 个字——
# 两次提交里撞了两次限，每次都要重跑全量，把余量一次留够（94.8 → 93.1 KiB）
# 比反复贴着线省事。再超时按既定顺序继续截散文，最后才动 symbol_cell。
# Bluetooth/battery additions exceed the index cap; retain symbol keys and
# shorten prose instead of raising the fixed 8/32/96 KiB budgets.
# 2026-09-13：双向 DShot 回传新增 drv_dshot_telemetry / bsp_dshot_rx 两组模块与一个
# 测试文件后到 98576 B，越限 272 B。按上面既定顺序继续截散文而不动 symbol_cell。
# 80→74 只剩 744 B 余量，等于下一个模块立刻再撞一次；按"把余量一次留够"的先例
# 直接到 68（实测 94.0 KiB，余 2056 B）。
# 2026-09-20（R-MAG-1）：摘要宽度 68 → 64。磁力计一批新增 8 个模块后总索引到
# 98340 B，超 TOTAL_LIMIT 36 B。按 SKILL「超限优先改善摘要」压缩而不是抬上限，
# 沿用 2026-09-12 R-DSHOT-1 的同一处置。摘要本来就是截断的提要，少 4 个字符不
# 改变"去哪找"的作用；行数一行不减，所有模块仍然可见。
# R-THRUST-2 增加采集/建模模块后总索引再次超限；继续压缩摘要，保留所有路由项，
# 不提高总计 96 KiB 上限。文件入口、符号和覆盖范围均保留。
# SYSID v2 readiness: retain all file/symbol routes; leave room below the fixed
# 96 KiB budget by shortening prose two characters, rather than raising the cap.
# 2026-09-27（R-SYSID-1 / R-TORQUE-1 / R-NOTCH-1）：陷波、舵机单独、台架刚度等一批模块与测试后
# 总索引 102596 B。先把光杆辨识的上位机测试按"测的是哪一侧"归到 tests-host（固件分片因此回到
# 限内），再按既定顺序压摘要 54 → 44（95.2 KiB，余约 3 KB，一次留够）；路由项与符号一个不减。
# 2026-09-27 夜（R-BACKLASH-1）：舵机回差补偿新增驱动/策略/命令模块与四个测试文件后到 99137 B，
# 超 833 B。按既定顺序压摘要 44 → 38（95.7 KiB，余约 2.6 KB，一次留够）；路由项与符号一个不减。
# 2026-09-29（MAGXY RAM 试验）：新增磁航向服务/命令/实录测试后索引 98325 B，
# 超总限 21 B。仍只压用途散文 38 → 34，保留文件和精确符号入口，并留出
# 下一批模块的余量；不提高 96 KiB 上限。
# 2026-09-30（R-ALTID-1 / R-AIBRIDGE-1）：竖直估计器、悬停推力估计、地面站 AI 接口及其测试后 98643 B，
# 超 339 B。按既定顺序压摘要 34 → 32（97247 B，余约 1 KB）；路由项与符号一个不减，不提高上限。
# 2026-09-30 晚（R-XYID-1 / R-MAGXY-1 漂移量化）：XY 辨识固件/页面模块、磁航向倾斜回归测试后 98332 B，
# 超 28 B。按既定顺序压摘要 32 → 30；路由项与符号一个不减，不提高上限。
def compact(text: str, limit: int = 32) -> str:
    text = re.sub(r"\s+", " ", text).strip(" .:-\t\r\n")
    text = text.replace("|", "/")
    if len(text) > limit:
        text = text[: limit - 1].rstrip() + "…"
    return text


def human_topic(path: str) -> str:
    stem = PurePosixPath(path).stem
    stem = re.sub(r"^(app|drv|bsp|svc|test)_", "", stem, flags=re.IGNORECASE)
    return stem.replace("_", " ").replace("-", " ").strip()


def c_symbols(paths: list[str]) -> list[str]:
    headers = [read_text(path) for path in paths if Path(path).suffix.lower() == ".h"]
    if headers:
        joined = re.sub(r"(?m)^\s*#.*$", "", "\n".join(headers))
        suffix = r"\s*\([^;{}]*\)\s*;"
    else:
        implementations = [read_text(path) for path in paths if Path(path).suffix.lower() in {".c", ".cpp"}]
        joined = "\n".join(implementations)
        suffix = r"\s*\([^;{}]*\)\s*\{"
    prefixes = (
        r"((?:APP|DRV|BSP|SVC)_[A-Za-z0-9_]+)",
        r"((?:MX|HAL)_[A-Za-z0-9_]+)",
        r"((?:SystemClock_Config|Error_Handler|Start[A-Za-z0-9_]+))",
    )
    found: list[str] = []
    for prefix in prefixes:
        pattern = rf"\b{prefix}{suffix}"
        for symbol in re.findall(pattern, joined, flags=re.MULTILINE):
            if symbol not in found:
                found.append(symbol)
    return found


def python_outline(path: str) -> tuple[str, list[str]]:
    text = read_text(path)
    try:
        tree = ast.parse(text)
    except (SyntaxError, ValueError):
        return "", []
    description = compact(ast.get_docstring(tree) or "")
    symbols: list[str] = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if node.name.startswith("_") and not node.name.startswith("__"):
                continue
            symbols.append(node.name)
    if path.startswith("tests/"):
        tests = [name for name in symbols if name.startswith("test_") or name.startswith("Test")]
        helpers = [name for name in symbols if name not in tests]
        symbols = tests + helpers
    return description, symbols


def markdown_outline(path: str) -> tuple[str, list[str]]:
    text = read_text(path)
    headings = [compact(value) for value in re.findall(r"^#{1,3}\s+(.+?)\s*$", text, re.MULTILINE)]
    if not headings:
        return "", []
    return headings[0], headings[1:]


def powershell_symbols(path: str) -> list[str]:
    return re.findall(r"(?im)^\s*function\s+([A-Za-z_][A-Za-z0-9_-]*)", read_text(path))


def base_purpose(path: str, kind: str) -> str:
    if path in PURPOSE_OVERRIDES:
        return PURPOSE_OVERRIDES[path]
    topic = human_topic(path)
    descriptions = {
        "App": f"Application behavior and task-facing logic for {topic}.",
        "Services": f"Synchronous domain/data service for {topic}.",
        "Driver": f"Reusable device or algorithm driver for {topic}.",
        "BSP": f"Board resource binding for {topic}.",
        "Core": f"CubeMX/HAL platform module for {topic}.",
        "USB_DEVICE": f"CubeMX USB device integration for {topic}.",
        "test": f"Automated checks for {topic}.",
        "tool": f"Host-side utility for {topic}.",
        "doc": f"Project documentation for {topic}.",
        "config": f"Project configuration for {topic}.",
    }
    return descriptions.get(kind, f"Project file for {topic}.")


# 每行默认列几个入口名。
#
# 2026-09-11：蓝牙出口改 DMA 发送新增 drv_tx_ring / bsp_uart_tx 两个模块与两个
# 测试文件后，索引总量到 98979 B，越过 96 KB 总限 675 B。按 individual_row()
# 里那条既定处置——**改密度、不抬限制**——把源码分片的入口密度 6→5。
# 先动这里而不是再拆分片：单个分片都还在 32 KB 硬限之内，超的是总量，
# 而入口名那一列每行砍一个就够（实测 98979 → 96077 B）。
# 测试分片不受影响，它早就压到 1 了。
# R-DSHOT-1 新模块使总量再次越界；继续压缩入口摘要而不改变容量上限。
# 2026-09-13（同日第二次）：系统辨识新增 drv_sysid_rig / _excitation / _record 三组
# 模块与四个测试文件后到 99162 B。散文列已经压到 68，再砍就开始伤"这文件是干嘛的"
# 这个最基本的判断力；按注释里既定的顺序，这次轮到 symbol_cell（4→3）。
# 入口名少列一个仍有 (+N) 计数兜底，知道"还有几个"比知道第 4 个叫什么更重要。
# 2026-09-13（同日第三次）：系统辨识落地（app_sysid / app_cmd_sysid、tools/sysid 七个
# 模块、tools/thrust_bench 七个模块、panel_lib/pages/sysid 五个模块、五个测试文件）
# 之后到 100279 B，越限 1975 B。散文列仍在 68（再砍开始伤"这文件是干嘛的"这个最基本
# 的判断力，实测 60 也只省 2.4 KiB 且换不来多少余量），所以继续按同一条路走 symbol_cell
# 3→2：实测 95253 B，余 3051 B。(+N) 计数照旧在，"还有几个"这个信息没丢。
def symbol_cell(symbols: list[str], limit: int = 2) -> str:
    unique: list[str] = []
    for symbol in symbols:
        if symbol not in unique:
            unique.append(symbol)
    shown = unique[:limit]
    result = ", ".join(f"`{name}`" for name in shown)
    if len(unique) > limit:
        result += f" (+{len(unique) - limit})"
    return result or "—"


def path_cell(paths: list[str]) -> str:
    return "<br>".join(f"`{path}`" for path in paths)


def module_rows(files: list[str], roots: tuple[str, ...]) -> tuple[list[str], set[str]]:
    groups: dict[tuple[str, str], list[str]] = defaultdict(list)
    used: set[str] = set()
    for path in files:
        parts = PurePosixPath(path).parts
        if not parts or parts[0] not in roots:
            continue
        if Path(path).suffix.lower() not in {".c", ".h", ".cpp"}:
            continue
        groups[(parts[0], PurePosixPath(path).stem)].append(path)
        used.add(path)

    rows: list[str] = []
    order = {root: index for index, root in enumerate(roots)}
    for (root, stem), paths in sorted(groups.items(), key=lambda item: (order[item[0][0]], item[0][1])):
        paths = sorted(paths, key=lambda value: (Path(value).suffix.lower() != ".h", value))
        purpose = base_purpose(paths[0], root)
        rows.append(f"| {path_cell(paths)} | {compact(purpose)} | {symbol_cell(c_symbols(paths))} |")
    return rows, used


def individual_row(path: str, kind: str) -> str:
    suffix = Path(path).suffix.lower()
    purpose = base_purpose(path, kind)
    symbols: list[str] = []
    if suffix == ".py":
        description, symbols = python_outline(path)
        if description:
            purpose = description
    elif suffix in {".md", ".markdown"}:
        description, symbols = markdown_outline(path)
        if description:
            purpose = description
    elif suffix == ".ps1":
        symbols = powershell_symbols(path)
    elif suffix in {".c", ".h", ".cpp"}:
        symbols = c_symbols([path])
    if kind == "test":
        # 测试分片只列 test_* 入口：模块级辅助函数（read/method_source 等）是噪音。
        test_symbols = [symbol for symbol in symbols if symbol.startswith("test_")]
        if test_symbols:
            symbols = test_symbols
        # 无 docstring 的测试模块不重复"Automated checks for 文件名"样板：
        # 该信息可从路径直接读出，省下的字节留给分片 32KB 硬限（2026-08-30 F4/F5 扩容裁决）。
        if purpose.startswith("Automated checks for"):
            purpose = "—"
        # 测试分片每行只列 1 个入口而非 6 个：本分片的用途是"挑到文件"，
        # 挑中之后就该直接打开文件看，多列的名字换不来判断力，却持续挤占
        # 32KB 硬限。改密度、不抬限制——延续 2026-08-30 F4/F5 的 6→4，
        # 2026-09-03 S8 新增遥测解码测试后 tests.md 到 33062 B，再压到 3；
        # 2026-09-04 R-S1-3/R-T1-6 新增 QA 装置与录制契约测试后到 33678 B，压到 2；
        # 2026-09-07 新增 PWM 帧率与光流方言边界契约后到 33123 B，压到 1。
        # 这里优先保 purpose 而不是保函数名：本仓库测试函数名很长且基本是 purpose
        # 的复述，砍它损失最小。真到 1 也不够时，该拆分片而不是继续砍。
        # 2026-10-01 夜：为塞进旧的 96 KiB 总量曾把测试摘要一路削到 22 字；作者随即“总上限放宽很多就行”，
        # 总量改 128 KiB、默认摘要回 32 字；测试摘要取 29 字是受“固件测试分片 32 KiB”单分片上限约束
        # （同时把归错的地面站页面/界面/主机工具测试挪到主机分片）。
        return f"| `{path}` | {compact(purpose, 29)} | {symbol_cell(symbols, limit=1)} |"
    return f"| `{path}` | {compact(purpose)} | {symbol_cell(symbols)} |"


def content_digest(paths: list[str]) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=str.casefold):
        digest.update(path.encode("utf-8", errors="surrogateescape"))
        digest.update(b"\0")
        digest.update(read_bytes(path))
        digest.update(b"\0")
    return digest.hexdigest()[:12]


def metadata_digest(paths: list[str]) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=str.casefold):
        size = (REPO_ROOT / Path(path)).stat().st_size
        digest.update(f"{path}\0{size}\0".encode("utf-8", errors="surrogateescape"))
    return digest.hexdigest()[:12]


def human_size(total: int) -> str:
    value = float(total)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if value < 1024.0 or unit == "GiB":
            return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{total} B"


def extensions(paths: list[str]) -> str:
    counts = Counter(Path(path).suffix.lower() or "(none)" for path in paths)
    return ", ".join(f"{ext}×{count}" for ext, count in counts.most_common(5))


def aggregate_row(scope: str, paths: list[str], purpose: str) -> str:
    total = sum((REPO_ROOT / Path(path)).stat().st_size for path in paths)
    details = f"{len(paths)} files / {human_size(total)} / {extensions(paths)}"
    return f"| `{scope}` | {compact(purpose)} | {details} |"


def generated_header(title: str, use_when: str, paths: list[str]) -> list[str]:
    return [
        "<!-- Generated by scripts/update_repository_index.py; do not edit manually. -->",
        "",
        f"# {title}",
        "",
        f"Read this shard only when {use_when}",
        "",
        f"Source snapshot: `{content_digest(paths)}`. Indexed files: {len(paths)}.",
        "",
    ]


def render_detail_shard(title: str, use_when: str, rows: list[str], paths: list[str]) -> str:
    lines = generated_header(title, use_when, paths)
    lines += [
        "| File/module | Content outline | Key entry points |",
        "|---|---|---|",
        *rows,
        "",
        "Open the smallest listed interface first (normally a header or test), then its implementation only if needed.",
        "",
    ]
    return "\n".join(lines)


def build_outputs(files: list[str]) -> dict[str, str]:
    outputs: dict[str, str] = {}
    assigned: set[str] = set()

    app_rows, app_used = module_rows(files, ("App", "Services"))
    assigned |= app_used
    app_extra = [path for path in files if path.split("/", 1)[0] in {"App", "Services"} and path not in assigned]
    app_rows += [individual_row(path, path.split("/", 1)[0]) for path in app_extra]
    assigned.update(app_extra)
    app_paths = sorted(app_used | set(app_extra))
    outputs["firmware-app-services.md"] = render_detail_shard(
        "Firmware Index: App and Services",
        "the task touches application behavior, RTOS task bodies, control flow, diagnostics, commands, or synchronous services.",
        app_rows,
        app_paths,
    )

    low_rows, low_used = module_rows(files, ("Driver", "BSP"))
    assigned |= low_used
    low_extra = [path for path in files if path.split("/", 1)[0] in {"Driver", "BSP"} and path not in assigned]
    low_rows += [individual_row(path, path.split("/", 1)[0]) for path in low_extra]
    assigned.update(low_extra)
    low_paths = sorted(low_used | set(low_extra))
    outputs["firmware-driver-bsp.md"] = render_detail_shard(
        "Firmware Index: Driver and BSP",
        "the task touches device protocols, reusable algorithms, buses, GPIO, DMA callbacks, cache hooks, or board bindings.",
        low_rows,
        low_paths,
    )

    test_paths = [path for path in files if path.startswith("tests/")]
    assigned.update(test_paths)
    #
    # 2026-09-11：tests.md 到 33124 B，超过 32 KB 硬限，而每行已经压到只列 1 个
    # 入口——按本文件 individual_row() 那段注释的既定处置，到这一步该**拆分片**，
    # 不该继续砍密度（再砍就只剩路径，索引也就没用了）。
    #
    # 拆法按"测的是哪一侧"，不是按字母：找测试时脑子里想的是"固件行为"还是
    # "上位机/工具行为"，照这条线分，两边都能独立用。
    host_prefixes = ("tests/test_panel", "tests/test_tk", "tests/test_dashboard",
                     "tests/test_sim", "tests/test_gui", "tests/test_tool",
                     "tests/test_flight_log_rerun", "tests/test_log_",
                     "tests/test_aiwb2_net", "tests/test_ground_",
                     "tests/test_thrust_dataset", "tests/test_thrust_experiment_library",
                     "tests/test_thrust_library_ui", "tests/test_thrust_smart_scan",
                     "tests/test_thrust_manual_control", "tests/test_thrust_mapping_control",
                     "tests/test_thrust_autocollect", "tests/test_thrust_bench_acquisition",
                     # 2026-09-27：光杆辨识的上位机侧（拟合、整定、页面、分析）按同一条线归上位机；
                     # 编译固件的 sysid 测试（runtime/record/auto_throttle/servo_mode/moment_inverse）仍归固件。
                     "tests/test_sysid_page", "tests/test_sysid_notch_panel",
                     "tests/test_sysid_amplitude_scale", "tests/test_sysid_lever_units",
                     "tests/test_sysid_servo_only", "tests/test_sysid_rig_stiffness",
                     "tests/test_sysid_vibration", "tests/test_sysid_pendulum_fit",
                     "tests/test_sysid_real_runs", "tests/test_sysid_host_core",
                     # 2026-10-01：固件分片到 33011 B 超 32 KB；XY 页面（含流程彩排）、AI 接口、电源页
                     # 测的都是上位机，按同一条线归上位机，不压摘要。
                     "tests/test_sysid_xy_page", "tests/test_ai_bridge", "tests/test_power_page",
                     # 2026-10-01 夜：再超（33018 B）；USB 断开切蓝牙、辨识页 IMU 重标定按钮都是上位机。
                     "tests/test_link_failover", "tests/test_sysid_imuzero_button",
                     # 2026-10-01 作者放宽总上限后：把归错到固件分片的地面站页面/界面/主机工具测试挪过来，
                     # 让测试摘要恢复到与源码条目同长（固件分片仍受 32 KiB 单分片上限约束）。
                     "tests/test_airframe_page", "tests/test_component_overview_layout",
                     "tests/test_flight_log_receive", "tests/test_flight_log_sysid_ui",
                     "tests/test_flight_log_waveform_ui", "tests/test_flight_log_workbench",
                     "tests/test_flow_monitor_page", "tests/test_flow_mount_page", "tests/test_imu_vibration_ui",
                     "tests/test_led_map_page", "tests/test_mag_cal_page", "tests/test_prop_map_page",
                     "tests/test_servo_debug_page_contract", "tests/test_servo_type_panel_contract",
                     "tests/test_shared_log_transfer", "tests/test_sysid_backlash_panel",
                     "tests/test_sysid_yaw_page", "tests/test_telem_stream_decoder",
                     "tests/test_thrust_bench_ui", "tests/test_yaw_analysis")
    host_paths = [path for path in test_paths if path.startswith(host_prefixes)]
    firmware_paths = [path for path in test_paths if path not in set(host_paths)]

    outputs["tests-firmware.md"] = render_detail_shard(
        "Verification Index · Firmware",
        "you need existing firmware behavioral/architecture coverage or must choose focused regression tests.",
        [individual_row(path, "test") for path in firmware_paths],
        firmware_paths,
    )
    dataset_test_paths = [path for path in host_paths if path.startswith((
        "tests/test_thrust_dataset", "tests/test_thrust_experiment_library",
        "tests/test_thrust_library_ui", "tests/test_thrust_smart_scan",
        "tests/test_thrust_manual_control", "tests/test_thrust_mapping_control",
        "tests/test_thrust_autocollect", "tests/test_thrust_bench_acquisition"))]
    host_test_rows = [individual_row(path, "test") for path in host_paths
                      if path not in dataset_test_paths]
    if dataset_test_paths:
        host_test_rows.append(
            f"| {path_cell(dataset_test_paths)} | Experiment library, grouped validation, smart scan and Tk workflow. | Whole-run isolation; bounded acquisition. |")
    outputs["tests-host.md"] = render_detail_shard(
        "Verification Index · Host tools & panel",
        "you need existing ground-station/tooling coverage or must choose focused regression tests.",
        host_test_rows,
        host_paths,
    )

    tool_paths = [
        path
        for path in files
        if path.startswith("tools/")
    ]
    tool_detail: list[str] = []
    tool_aggregate_groups: list[tuple[str, list[str]]] = []
    nested_tool_groups: dict[str, list[str]] = defaultdict(list)
    for path in tool_paths:
        parts = PurePosixPath(path).parts
        if len(parts) <= 2:
            tool_detail.append(path)
        else:
            nested_tool_groups[parts[1]].append(path)
    for group_name, group_paths in sorted(nested_tool_groups.items(), key=lambda item: item[0].casefold()):
        total_size = sum((REPO_ROOT / Path(path)).stat().st_size for path in group_paths)
        if (len(group_paths) <= 20) and (total_size <= 2 * 1024 * 1024):
            tool_detail.extend(group_paths)
            continue
        landmarks = [
            path
            for path in group_paths
            if len(PurePosixPath(path).parts) == 3
            and (
                PurePosixPath(path).name.casefold().startswith("readme")
                or Path(path).suffix.lower() in {".md", ".txt", ".json", ".toml", ".yaml", ".yml", ".ini", ".py", ".ps1"}
            )
        ][:8]
        tool_detail.extend(landmarks)
        aggregate_paths = [path for path in group_paths if path not in landmarks]
        if aggregate_paths:
            tool_aggregate_groups.append((f"tools/{group_name}/ bundled tree", aggregate_paths))
    tool_detail = sorted(set(tool_detail), key=str.casefold)
    tool_rows = [individual_row(path, "tool") for path in tool_detail]
    for scope, group_paths in tool_aggregate_groups:
        tool_rows.append(
            aggregate_row(
                scope,
                group_paths,
                "Bundled application/runtime distribution; inspect an exact file only when maintaining that bundle.",
            )
        )
    assigned.update(tool_paths)
    tool_header = generated_header(
        "Host Tools Index",
        "the task uses serial/TCP diagnostics, log analysis, identification, capture, calibration, or desktop UIs.",
        tool_detail,
    )
    aggregate_tool_paths = [path for _, group_paths in tool_aggregate_groups for path in group_paths]
    tool_header[-2] = (
        f"Source snapshot: `{content_digest(tool_detail)}`; aggregate snapshot: "
        f"`{metadata_digest(aggregate_tool_paths)}`. Covered files: {len(tool_paths)}."
    )
    outputs["host-tools.md"] = "\n".join(
        tool_header
        + [
            "| File/module or scope | Content outline | Key entry points / size |",
            "|---|---|---|",
            *tool_rows,
            "",
            "Open the smallest listed tool or bundle landmark first; do not preload bundled runtimes.",
            "",
        ]
    )

    platform_paths: list[str] = []
    for path in files:
        if path in assigned:
            continue
        first = path.split("/", 1)[0]
        if first in {"Core", "USB_DEVICE", "cmake", "MDK-ARM"} or "/" not in path:
            platform_paths.append(path)
    platform_detail = [
        path
        for path in platform_paths
        if not path.endswith(".DS_Store") and not path.endswith(".su") and path != "dis.txt"
    ]
    platform_rows: list[str] = []
    core_rows, core_used = module_rows(platform_detail, ("Core", "USB_DEVICE"))
    platform_rows += core_rows
    platform_remaining = [path for path in platform_detail if path not in core_used]
    platform_rows += [individual_row(path, "config") for path in platform_remaining]
    platform_artifacts = [path for path in platform_paths if path not in platform_detail]
    if platform_artifacts:
        platform_rows.append(
            aggregate_row(
                "root generated leftovers",
                platform_artifacts,
                "Compiler, disassembly, or operating-system leftovers; never use as source of truth.",
            )
        )
    assigned.update(platform_paths)

    vendor_groups: list[tuple[str, list[str], str]] = []
    vendor_purposes = {
        "Drivers/": "STM32 CMSIS and HAL vendor sources; read only for HAL behavior not documented by project code.",
        "Middlewares/": "FreeRTOS and STM32 USB middleware; vendor-owned unless a task explicitly requires internals.",
        "ThirdParty/": "Third-party algorithm sources and licenses, including Fusion.",
        "driver_doc/": "Generated vendor API documentation; search for an exact peripheral or symbol before opening.",
    }
    for scope in VENDOR_SCOPES:
        group = [path for path in files if path.startswith(scope)]
        if group:
            vendor_groups.append((scope, group, vendor_purposes[scope]))
            assigned.update(group)
            platform_rows.append(aggregate_row(scope, group, vendor_purposes[scope]))
    platform_all = sorted(set(platform_paths) | {path for _, group, _ in vendor_groups for path in group})
    platform_header = generated_header(
        "Platform, Build, and Vendor Index",
        "the task touches CubeMX output, pins/clocks/peripherals, startup/linker layout, USB, build configuration, HAL, FreeRTOS internals, or vendor code.",
        platform_detail,
    )
    platform_header[-2] = (
        f"Source snapshot: `{content_digest(platform_detail)}`; aggregate snapshot: "
        f"`{metadata_digest([path for _, group, _ in vendor_groups for path in group] + platform_artifacts)}`. "
        f"Covered files: {len(platform_all)}."
    )
    outputs["platform-build.md"] = "\n".join(
        platform_header
        + [
            "| File/module or scope | Content outline | Key entry points / size |",
            "|---|---|---|",
            *platform_rows,
            "",
            "CubeMX-owned files are routing targets, not authorization to hand-edit generated configuration.",
            "",
        ]
    )

    docs_paths: list[str] = []
    for path in files:
        if path in assigned:
            continue
        if path == "data/README.md":
            docs_paths.append(path)
            continue
        if any(path.startswith(scope) for scope in DATA_SCOPES):
            continue
        if path.startswith("doc/") or path.startswith(".agents/") or path.startswith(".claude/") or path.startswith(".codex/"):
            docs_paths.append(path)
    docs_rows = [individual_row(path, "doc") for path in docs_paths]
    assigned.update(docs_paths)

    data_rows: list[str] = []
    data_paths: list[str] = []
    data_purposes = {
        ".tmp/": "Temporary analysis results; inspect only a specifically named run.",
        "data/": "Canonical captures, flight logs, identification, calibration, telemetry, and analysis root.",
    }
    for scope in DATA_SCOPES:
        group = [path for path in files if path.startswith(scope) and path not in assigned]
        if group:
            data_rows.append(aggregate_row(scope, group, data_purposes[scope]))
            data_paths.extend(group)
            assigned.update(group)

    remaining = [path for path in files if path not in assigned]
    remaining_text = [path for path in remaining if Path(path).suffix.lower() in TEXT_EXTENSIONS]
    docs_rows += [individual_row(path, "doc") for path in remaining_text]
    assigned.update(remaining_text)
    remaining_binary = [path for path in remaining if path not in assigned]
    if remaining_binary:
        data_rows.append(
            aggregate_row(
                "other binary/reference assets",
                remaining_binary,
                "Non-code project assets; open only when an exact file is required.",
            )
        )
        data_paths.extend(remaining_binary)
        assigned.update(remaining_binary)

    if assigned != set(files):
        missing = sorted(set(files) - assigned)
        raise RuntimeError(f"index classification missed files: {missing[:10]}")

    doc_content_paths = sorted(set(docs_paths) | set(remaining_text))
    docs_header = generated_header(
        "Documentation and Data Index",
        "the task needs architecture rationale, controller math, historical evidence, captures, datasets, or agent-support documentation.",
        doc_content_paths,
    )
    docs_header[-2] = (
        f"Document snapshot: `{content_digest(doc_content_paths)}`; aggregate snapshot: "
        f"`{metadata_digest(data_paths)}`. Covered files: {len(doc_content_paths) + len(data_paths)}."
    )
    outputs["docs-data.md"] = "\n".join(
        docs_header
        + [
            "## Documents and agent support",
            "",
            "| File | Content outline | Headings / entry points |",
            "|---|---|---|",
            *docs_rows,
            "",
            "## Aggregated datasets and captures",
            "",
            "| Scope | Content outline | Inventory |",
            "|---|---|---|",
            *data_rows,
            "",
            "Do not load a whole dataset directory. Select one named run after code or test evidence points to it.",
            "",
        ]
    )

    shard_digest = hashlib.sha256()
    for name in SHARDS:
        shard_digest.update(name.encode())
        shard_digest.update(outputs[name].encode("utf-8"))
    snapshot = shard_digest.hexdigest()[:12]
    counts = {
        "app": len(app_paths),
        "low": len(low_paths),
        "platform": len(platform_all),
        "tests": len(test_paths),
        "tests_firmware": len(firmware_paths),
        "tests_host": len(host_paths),
        "tools": len(tool_paths),
        "docs": len(doc_content_paths) + len(data_paths),
    }
    root_lines = [
        "<!-- Generated by scripts/update_repository_index.py; do not edit manually. -->",
        "",
        "# drone-H743 Repository Index",
        "",
        f"Snapshot: `{snapshot}`. Covered non-index working files: {len(files)}. Scope: Git-tracked plus non-ignored untracked files; ignored build outputs, caches, and local logs are intentionally excluded.",
        "",
        "## Use this index",
        "",
        "1. Read only this page first, then open the smallest relevant shard below.",
        "2. Select source files from that shard; start with public headers/tests and expand only along observed dependencies.",
        "3. If no entry matches, use a narrow `rg --files`/`rg -n` query, then refresh the index if a source file was added or moved.",
        "4. Never preload every shard, an entire source directory, vendor documentation, captures, or datasets.",
        "",
        "Freshness check: `python .agents/skills/drone-h743-project/scripts/update_repository_index.py --check`.",
        "",
        "## Task router",
        "",
        "| Read this shard | When it matches | Covered files |",
        "|---|---|---:|",
        f"| [firmware-app-services.md](firmware-app-services.md) | App behavior, tasks, control, commands, diagnostics, application services | {counts['app']} |",
        f"| [firmware-driver-bsp.md](firmware-driver-bsp.md) | Device/algorithm drivers, buses, GPIO, DMA callbacks, cache or board binding | {counts['low']} |",
        f"| [platform-build.md](platform-build.md) | CubeMX/Core, pins/clocks/peripherals, RTOS objects, linker/startup, USB, build, HAL/vendor internals | {counts['platform']} |",
        f"| [tests-firmware.md](tests-firmware.md) | Firmware regression/contract coverage and focused test selection | {counts['tests_firmware']} |",
        f"| [tests-host.md](tests-host.md) | Ground-station/tooling regression coverage | {counts['tests_host']} |",
        f"| [host-tools.md](host-tools.md) | Serial/TCP tools, capture, calibration, identification, log analysis and desktop UIs | {counts['tools']} |",
        f"| [docs-data.md](docs-data.md) | Architecture/controller documents, agent references, datasets, captures and experimental evidence | {counts['docs']} |",
        "",
        "## Ownership map",
        "",
        "`App/` and `Services/` → `Driver/` → `BSP/` → `Core/`/HAL. `drone-H743.ioc` owns CubeMX hardware configuration. `tests/` checks contracts; `tools/` contains host workflows. Vendor trees and recorded data are read only on explicit evidence.",
        "",
    ]
    outputs["README.md"] = "\n".join(root_lines)
    return outputs


def validate_sizes(outputs: dict[str, str]) -> None:
    sizes = {name: len(text.encode("utf-8")) for name, text in outputs.items()}
    if sizes["README.md"] > ROOT_LIMIT:
        raise RuntimeError(f"README.md is {sizes['README.md']} bytes; limit is {ROOT_LIMIT}")
    for name in SHARDS:
        if sizes[name] > SHARD_LIMIT:
            raise RuntimeError(f"{name} is {sizes[name]} bytes; limit is {SHARD_LIMIT}")
    total = sum(sizes.values())
    if total > TOTAL_LIMIT:
        raise RuntimeError(f"repository index is {total} bytes; limit is {TOTAL_LIMIT}")


def check_outputs(outputs: dict[str, str]) -> int:
    stale: list[str] = []
    for name, expected in outputs.items():
        target = INDEX_DIR / name
        if not target.exists() or target.read_text(encoding="utf-8") != expected:
            stale.append(name)
    if stale:
        print("Repository index is stale: " + ", ".join(stale))
        print(f"Run: python {GENERATOR_REL}")
        return 1
    print("Repository index is current.")
    return 0


def write_outputs(outputs: dict[str, str]) -> None:
    INDEX_DIR.mkdir(parents=True, exist_ok=True)
    for name, text in outputs.items():
        target = INDEX_DIR / name
        target.write_text(text, encoding="utf-8", newline="\n")
    total = sum(len(text.encode("utf-8")) for text in outputs.values())
    print(f"Updated {len(outputs)} index files ({human_size(total)} total).")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if generated indexes are absent or stale")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        files = git_working_files()
        outputs = build_outputs(files)
        validate_sizes(outputs)
    except (OSError, subprocess.CalledProcessError, RuntimeError) as exc:
        print(f"repository-index error: {exc}", file=sys.stderr)
        return 2
    if args.check:
        return check_outputs(outputs)
    write_outputs(outputs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

from .guards import hardware_guards
from .harness import OfflinePanel
from .isolation import claim_output_path, isolated_environment


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _head_commit() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    except Exception:                            # noqa: BLE001 - 不在 git 工作树里
        return "unknown"


# ---------------------------------------------------------------- N01-N04
#
# 已由 V 线 TK-01/TK-02 改写为真实产品回归：
#   tests/test_tk_v_revamp.py
# 修复前的观测原文保留在
#   data/analysis/tk_revamp/2026-09-04/baseline_63223acd/observations.json。
# 基线脚本不再重复驱动已不存在的主题/布局/全局滚轮缺陷。


# ---------------------------------------------------------------- N05-N09
#
# 已由 D 线 TK-03/TK-04 改写为真实产品回归：
#   tests/test_panel_d_data_contract.py
# 修复前的观测原文保留在
#   data/analysis/tk_revamp/2026-09-04/baseline_63223acd/observations_1.json。
# 基线脚本不再重复驱动已经转为回归测试的输入/参数/GPS 缺陷。


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
    # 申领即创建：选名和写入之间不能有窗口（审核 Q5）。
    handle, target = claim_output_path(output_dir, "observations", ".json")
    with handle:
        handle.write(json.dumps(observations, indent=2, ensure_ascii=False))
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

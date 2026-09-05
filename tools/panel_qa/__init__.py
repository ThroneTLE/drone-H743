"""R-S1-3（TK-00）：上位机无硬件 QA 测试基础。

改版报告的每一条问题都是在“真实 `DronePanel` + 模拟 transport”上复现的，可是
复现装置当时散在 `data/analysis/tk_revamp/2026-09-04/` 的一次性脚本里：路径写死、
护栏各写一遍、夹具的报文是手打的。后面每个改版包都要重来一次，而且**手打的报文
正是最容易和固件对不上的东西**——`work-modes.md` 里那个 `gz_dps` 的例子就是这么
让一整阶验收静悄悄地永远不可能通过的。

所以这个包只做四件事，本身不改任何生产行为：

    guards    —— 全局硬件护栏：物理串口和烧录/复位程序一旦被碰就当场失败并留证。
    isolation —— 隔离用户的 panel_state / 日志 / data 输出，外加可控时钟与目录指纹。
    fixtures  —— 协议夹具，键名直接从固件源码的格式串里抽出来核对，不许手打。
    harness   —— 真实 `DronePanel` 的离线装置 + 18 叶页 × 三尺寸 × 三缩放几何探针。

`geometry` 里的裁切判据与报告 §2 的矩阵一致（横向越界、不可滚动区纵向越界、
已管理但分不到空间的 1×1 控件），这样后续包的“修好了”能和报告的告警逐条对上。

**这个包不是回归结论**：它只提供装置。产品缺陷仍然只在对应实现包里先写成红灯
再修绿，不允许拿“装置能跑通”冒充“产品正确”。
"""

from __future__ import annotations

from .fixtures import (
    DEFAULT_CHANNELS,
    GPS_STATUS_KEYS,
    firmware_format_keys,
    gps_position_line,
    gps_status_line,
    telemetry_frame,
    telemetry_schema_lines,
)
from .geometry import LeafPage, PageGeometryReport, SCALES, WINDOW_SIZES, probe_geometry
from .guards import (
    FLASH_TOOL_TOKENS,
    HardwareAccessAttempt,
    HardwareGuardLog,
    hardware_guards,
    is_flash_tool_command,
)
from .harness import MemoryTransport, OfflinePanel, is_display_unavailable
from .isolation import ManualClock, QaEnvironment, directory_digest, isolated_environment

__all__ = [
    "DEFAULT_CHANNELS",
    "FLASH_TOOL_TOKENS",
    "GPS_STATUS_KEYS",
    "HardwareAccessAttempt",
    "HardwareGuardLog",
    "LeafPage",
    "ManualClock",
    "MemoryTransport",
    "OfflinePanel",
    "PageGeometryReport",
    "QaEnvironment",
    "SCALES",
    "WINDOW_SIZES",
    "directory_digest",
    "firmware_format_keys",
    "gps_position_line",
    "gps_status_line",
    "hardware_guards",
    "is_flash_tool_command",
    "is_display_unavailable",
    "isolated_environment",
    "probe_geometry",
    "telemetry_frame",
    "telemetry_schema_lines",
]

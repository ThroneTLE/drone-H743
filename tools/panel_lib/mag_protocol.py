"""MAGCAL / MAGFRAME 回包与磁力计原始读数（`MAG?`）的纯解码模块。

形态照 `battery_monitor.py`：解析和显示分开，页面只拿到一个已经自洽的快照；
解析越严格，界面就越不会把一份自相矛盾的回包当成正常数据显示出去。唯一事实源
是 `App/Src/app_cmd_magcal.c`（MAGCAL/MAGFRAME 命令族）与
`App/Src/app_mag.c::APP_MAG_Report()`（`MAG?` 原始读数）的格式串，不是这份文件
的注释——改了格式串这里也要跟着改，`tests/test_magcal_command_contract.py` 编译
运行的是真实固件源码，本模块的测试锚定它的输出。

**这个模块里最容易出错、也最不能出错的一件事**：`chip_to_flu_default_mgauss()`
复刻的是固件 `Services/Src/svc_mag.c::SVC_MAG_RotateToFlu()`
（`SVC_MAG_DefaultRotation()` 固定返回 `SVC_MAG_ROTATION_NONE`）经
`Driver/Inc/drv_frame_contract.h::DRV_FRAME_FrdToFlu` 推导出的默认贴装变换。
固件对**每一拍**磁力计读数都无条件先做这一步旋转，再喂给
`DRV_MAG_Calibration_Apply()`——也就是说 `MAGCAL SET BIAS/MATRIX` 写进去的系数，
是在**旋转之后**的坐标系里生效的。采样步骤如果直接拿 `MAG?` 报的芯片原生轴向去
拟合，拟合出来的零偏/软磁矩阵会被固件在一个不同的坐标系里应用——不会报错、
不会崩溃，只是偏航方向从此跑掉。所以本模块把这一步旋转做成一个独立、可单测的
纯函数，采样调用方必须在喂进 `tools/mag_cal_fit.py` 之前先过一遍它。

`axis_verified`（存储值）与 `axis_effective`（应用了坐标契约版本失效规则后的
有效值）是两个不同的东西：坐标契约一旦升版，固件会把 `axis_effective` 强制视为
未验证，即使 `axis_verified` 那一位仍然留着旧值。判断磁力计当下是否真的参与了
姿态融合，权威信号是 `MagFusionStatus.used`（固件在这一拍是不是真的把它喂进了
`FusionAhrsUpdate()`），不是 `axis_verified`，也不止是 `axis_effective`
（`subsystem_enabled=1` 但 `used=0` 时，磁力计装了、轴向也验证了，仍然可能因为
场强/新鲜度被这一拍拒绝）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .proto import parse_kv


class MagProtocolError(ValueError):
    """一行文本匹配了已知的 MAGCAL/MAGFRAME/MAG 前缀，但内容没通过严格校验。

    和"这一行不归这个模块管"（`decode_line()` 返回 `None`）是两件完全不同的事：
    调用方不能把两者混为一谈去 `except Exception: pass`——前者必须让页面显示
    "回包无效"，后者才是"这行本来就该交给别的处理器"。
    """


#: `DRV_MAG_CAL_AXIS_UNVERIFIED` / `DRV_MAG_CAL_AXIS_VERIFIED`
#: （`Driver/Inc/drv_mag_calibration.h`）。
AXIS_UNVERIFIED = 0
AXIS_VERIFIED = 1

#: `MAGFRAME ...` 回包里 `derivation=` 的唯一取值——固件格式串里这是一段固定
#: 字面量，不是 `%s` 替换出来的变量，所以可以像 `current_monitor.py` 的
#: `CURRENT_SOURCE` 一样做严格相等校验。
MAGFRAME_DERIVATION_DEFAULT_UNVERIFIED = "svc_mag_default_rotation_unverified"

_MAGCAL_REASON_TEXT = {
    "incomplete_draft": "草稿未凑齐：BIAS 与三行 MATRIX 必须先全部 SET",
    "invalid_args": "校准记录参数无效",
    "nonfinite": "系数包含 NaN/Inf",
    "bad_determinant": "软磁矩阵行列式非正（镜像解，已拒绝）",
    "bad_scale": "软磁矩阵行列式超出可信范围",
    "unknown": "飞控未说明具体原因",
    "invalid_record": "当前生效的校准记录未通过自检，拒绝落盘",
}


def _bool01(values: dict[str, str], key: str) -> bool:
    text = values.get(key)
    if text not in ("0", "1"):
        raise MagProtocolError(key)
    return text == "1"


def _uint(values: dict[str, str], key: str, *, maximum: int = 0xFFFFFFFF) -> int:
    text = values.get(key)
    if text is None or not text.isdecimal():
        raise MagProtocolError(key)
    number = int(text)
    if number > maximum:
        raise MagProtocolError(key)
    return number


def _finite_float(text: str | None, name: str) -> float:
    if text is None:
        raise MagProtocolError(name)
    try:
        value = float(text)
    except ValueError as exc:
        raise MagProtocolError(name) from exc
    # `float()` happily parses the literal text "nan"/"inf" -- a %f-formatted
    # NaN/Inf coefficient must be rejected, not displayed as a real number.
    if not math.isfinite(value):
        raise MagProtocolError(name)
    return value


def _triplet(text: str, name: str) -> tuple[float, float, float]:
    parts = text.split(",")
    if len(parts) != 3:
        raise MagProtocolError(name)
    a, b, c = (_finite_float(part, f"{name}[{i}]") for i, part in enumerate(parts))
    return (a, b, c)


# ─────────────────────────────────────────────────────────── MAGCAL 状态


@dataclass(frozen=True)
class MagCalStatus:
    """`MAGCAL calibrated=... axis_verified=... axis_effective=... contract_stored=... contract_live=... dirty=...`."""

    calibrated: bool
    axis_verified: int
    axis_effective: int
    contract_stored: int
    contract_live: int
    dirty: bool

    @property
    def axis_effective_verified(self) -> bool:
        return self.axis_effective == AXIS_VERIFIED

    @property
    def contract_mismatch(self) -> bool:
        """坐标契约版本是否已经漂移——这就是 `axis_effective` 会被强制清零的原因。"""
        return self.contract_stored != self.contract_live


def parse_magcal_status(line: str) -> MagCalStatus:
    if not line.startswith("MAGCAL calibrated="):
        raise MagProtocolError("不是 MAGCAL 状态回包")
    values = parse_kv(line)
    return MagCalStatus(
        calibrated=_bool01(values, "calibrated"),
        axis_verified=_uint(values, "axis_verified", maximum=1),
        axis_effective=_uint(values, "axis_effective", maximum=1),
        contract_stored=_uint(values, "contract_stored"),
        contract_live=_uint(values, "contract_live"),
        dirty=_bool01(values, "dirty"),
    )


@dataclass(frozen=True)
class MagCalBias:
    """`MAGCAL bias_mgauss=x,y,z`（硬磁零偏，milligauss）。"""

    bias_mgauss: tuple[float, float, float]


def parse_magcal_bias(line: str) -> MagCalBias:
    if not line.startswith("MAGCAL bias_mgauss="):
        raise MagProtocolError("不是 MAGCAL 零偏回包")
    values = parse_kv(line)
    text = values.get("bias_mgauss")
    if text is None:
        raise MagProtocolError("bias_mgauss")
    return MagCalBias(_triplet(text, "bias_mgauss"))


@dataclass(frozen=True)
class MagCalMatrix:
    """`MAGCAL matrix row0=... row1=... row2=...`（软磁矩阵，无量纲）。"""

    rows: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


def parse_magcal_matrix(line: str) -> MagCalMatrix:
    if not line.startswith("MAGCAL matrix "):
        raise MagProtocolError("不是 MAGCAL 软磁矩阵回包")
    values = parse_kv(line)
    rows = []
    for key in ("row0", "row1", "row2"):
        text = values.get(key)
        if text is None:
            raise MagProtocolError(key)
        rows.append(_triplet(text, key))
    return MagCalMatrix((rows[0], rows[1], rows[2]))


@dataclass(frozen=True)
class MagCalDraft:
    """`MAGCAL draft bias_set=... row_set=...,...,...`（SET 累积到的草稿完整度）。"""

    bias_set: bool
    row_set: tuple[bool, bool, bool]

    @property
    def complete(self) -> bool:
        """草稿是否已经凑齐——凑齐之前 `MAGCAL APPLY` 必然被拒。"""
        return self.bias_set and all(self.row_set)


def parse_magcal_draft(line: str) -> MagCalDraft:
    if not line.startswith("MAGCAL draft "):
        raise MagProtocolError("不是 MAGCAL 草稿回包")
    values = parse_kv(line)
    bias_set = _bool01(values, "bias_set")
    text = values.get("row_set")
    if text is None:
        raise MagProtocolError("row_set")
    parts = text.split(",")
    if len(parts) != 3 or any(part not in ("0", "1") for part in parts):
        raise MagProtocolError("row_set")
    row_set = (parts[0] == "1", parts[1] == "1", parts[2] == "1")
    return MagCalDraft(bias_set, row_set)


@dataclass(frozen=True)
class MagFusionStatus:
    """`MAGCAL fusion subsystem_enabled=... used=... field_rejected=... ignored=... recovery=... error_deg=...`."""

    subsystem_enabled: bool
    used: bool
    field_rejected: bool
    ignored: bool
    recovery: bool
    error_deg: float

    @property
    def participating(self) -> bool:
        """这一拍磁力计是不是真的在影响姿态解算——本页最重要的一条信息。

        不要用 `subsystem_enabled` 代替它：`subsystem_enabled=1` 只表示
        "校准+轴向验证+新鲜度"三条前提成立，这一拍仍然可能因为场强/内部拒绝
        （`field_rejected`/`ignored`）而没有真的用上，`used` 才是最终结论。
        """
        return self.used


def parse_magcal_fusion(line: str) -> MagFusionStatus:
    if not line.startswith("MAGCAL fusion "):
        raise MagProtocolError("不是 MAGCAL 融合状态回包")
    values = parse_kv(line)
    return MagFusionStatus(
        subsystem_enabled=_bool01(values, "subsystem_enabled"),
        used=_bool01(values, "used"),
        field_rejected=_bool01(values, "field_rejected"),
        ignored=_bool01(values, "ignored"),
        recovery=_bool01(values, "recovery"),
        error_deg=_finite_float(values.get("error_deg"), "error_deg"),
    )


@dataclass(frozen=True)
class MagCalEvent:
    """`MAGCAL state=...` 状态迁移回包（SET/APPLY/COMMIT/CLEAR 的结果）。

    `reason`/`row`/`st` 按具体 `state` 才有意义，其余场合是 `None`。
    """

    state: str
    reason: str | None = None
    row: int | None = None
    st: int | None = None

    @property
    def reason_text(self) -> str:
        if self.st is not None:
            return f"Flash 写入失败，状态码 {self.st}"
        if self.reason is None:
            return ""
        return _MAGCAL_REASON_TEXT.get(self.reason, self.reason)


def parse_magcal_event(line: str) -> MagCalEvent:
    if not line.startswith("MAGCAL state="):
        raise MagProtocolError("不是 MAGCAL 状态迁移回包")
    values = parse_kv(line)
    state = values.get("state")
    if not state:
        raise MagProtocolError("state")
    row = None
    if "row" in values:
        if not values["row"].isdecimal():
            raise MagProtocolError("row")
        row = int(values["row"])
        if row > 2:
            raise MagProtocolError("row")
    st = None
    if "st" in values:
        try:
            st = int(values["st"])
        except ValueError as exc:
            raise MagProtocolError("st") from exc
    return MagCalEvent(state=state, reason=values.get("reason"), row=row, st=st)


# ─────────────────────────────────────────────────────────── MAGFRAME


@dataclass(frozen=True)
class MagFrameStatus:
    """`MAGFRAME axis_verified=... axis_effective=... contract_stored=... contract_live=... derivation=...`."""

    axis_verified: int
    axis_effective: int
    contract_stored: int
    contract_live: int
    derivation: str

    @property
    def axis_effective_verified(self) -> bool:
        return self.axis_effective == AXIS_VERIFIED


def parse_magframe_status(line: str) -> MagFrameStatus:
    if not line.startswith("MAGFRAME axis_verified="):
        raise MagProtocolError("不是 MAGFRAME 状态回包")
    values = parse_kv(line)
    derivation = values.get("derivation")
    if derivation != MAGFRAME_DERIVATION_DEFAULT_UNVERIFIED:
        raise MagProtocolError("derivation")
    return MagFrameStatus(
        axis_verified=_uint(values, "axis_verified", maximum=1),
        axis_effective=_uint(values, "axis_effective", maximum=1),
        contract_stored=_uint(values, "contract_stored"),
        contract_live=_uint(values, "contract_live"),
        derivation=derivation,
    )


@dataclass(frozen=True)
class MagFrameEvent:
    """`MAGFRAME state=...`（VERIFY 的结果）。"""

    state: str


def parse_magframe_event(line: str) -> MagFrameEvent:
    if not line.startswith("MAGFRAME state="):
        raise MagProtocolError("不是 MAGFRAME 状态迁移回包")
    values = parse_kv(line)
    state = values.get("state")
    if not state:
        raise MagProtocolError("state")
    return MagFrameEvent(state=state)


# ─────────────────────────────────────────────────────────── MAG 原始读数


@dataclass(frozen=True)
class MagRawSample:
    """`MAG ok=... raw=x,y,z mgauss=x,y,z`（`App/Src/app_mag.c::APP_MAG_Report`）。

    `chip_mgauss` 是芯片原生轴向、**校准之前**的标度读数——不是姿态融合实际
    使用的值，也还没有做 FLU 旋转。采样步骤据此判断"这一帧能不能算一个有效
    标定样本"（`ok` 为假时不能），喂进拟合之前必须先过
    `chip_to_flu_default_mgauss()`。
    """

    ok: bool
    raw_counts: tuple[int, int, int]
    chip_mgauss: tuple[float, float, float]


def parse_mag_raw_sample(line: str) -> MagRawSample:
    if not line.startswith("MAG ok="):
        raise MagProtocolError("不是磁力计原始读数回包")
    values = parse_kv(line)
    ok = _bool01(values, "ok")
    raw_text = values.get("raw")
    mgauss_text = values.get("mgauss")
    if raw_text is None:
        raise MagProtocolError("raw")
    if mgauss_text is None:
        raise MagProtocolError("mgauss")
    raw_parts = raw_text.split(",")
    if len(raw_parts) != 3:
        raise MagProtocolError("raw")
    try:
        raw_counts = (int(raw_parts[0]), int(raw_parts[1]), int(raw_parts[2]))
    except ValueError as exc:
        raise MagProtocolError("raw") from exc
    return MagRawSample(ok, raw_counts, _triplet(mgauss_text, "mgauss"))


def chip_to_flu_default_mgauss(
    chip_mgauss: tuple[float, float, float],
) -> tuple[float, float, float]:
    """芯片原生轴 → FLU 机体轴的**默认**（未经 MAGFRAME VERIFY 确认的）贴装变换。

    公式来自 `Services/Src/svc_mag.c::SVC_MAG_DefaultRotation()`（本板固定
    `SVC_MAG_ROTATION_NONE`，即芯片轴恒等于 FRD）经
    `Driver/Inc/drv_frame_contract.h::DRV_FRAME_FrdToFlu` 两步推导：
    ``flu = (chip.x, -chip.y, -chip.z)``。

    固件对每一拍读数都无条件做这一步——不管 `axis_verified` 是什么（那只决定
    变换之后的值能不能喂进融合，不影响变换本身是否发生）。所以采样/拟合必须用
    同一个变换过的样本，否则拟合出来的零偏/软磁矩阵会被固件在一个不同的坐标系
    里应用。见模块顶部说明。
    """
    x, y, z = chip_mgauss
    return (x, -y, -z)


# ─────────────────────────────────────────────────────────── 统一派发

#: `(前缀, 解析器)`；派发时按顺序取第一个前缀匹配的，前缀之间互不包含，
#: 顺序对结果没有影响，写成元组只是为了确定性遍历。
_LINE_PARSERS: tuple[tuple[str, object], ...] = (
    ("MAGCAL calibrated=", parse_magcal_status),
    ("MAGCAL bias_mgauss=", parse_magcal_bias),
    ("MAGCAL matrix ", parse_magcal_matrix),
    ("MAGCAL draft ", parse_magcal_draft),
    ("MAGCAL fusion ", parse_magcal_fusion),
    ("MAGCAL state=", parse_magcal_event),
    ("MAGFRAME axis_verified=", parse_magframe_status),
    ("MAGFRAME state=", parse_magframe_event),
    ("MAG ok=", parse_mag_raw_sample),
)


def decode_line(line: str):
    """按前缀派发到对应的严格解析器。

    返回 `None` 只表示"这一行不归这个模块管"，调用方应该继续交给别的处理器；
    这与解析失败（`MagProtocolError`，前缀匹配但内容非法）是两件不同的事，
    调用方不能用同一个 `except` 把它们并起来处理。
    """
    for prefix, parser in _LINE_PARSERS:
        if line.startswith(prefix):
            return parser(line)
    return None


__all__ = [
    "AXIS_UNVERIFIED",
    "AXIS_VERIFIED",
    "MAGFRAME_DERIVATION_DEFAULT_UNVERIFIED",
    "MagCalBias",
    "MagCalDraft",
    "MagCalEvent",
    "MagCalMatrix",
    "MagCalStatus",
    "MagFrameEvent",
    "MagFrameStatus",
    "MagFusionStatus",
    "MagProtocolError",
    "MagRawSample",
    "chip_to_flu_default_mgauss",
    "decode_line",
    "parse_magcal_bias",
    "parse_magcal_draft",
    "parse_magcal_event",
    "parse_magcal_fusion",
    "parse_magcal_matrix",
    "parse_magcal_status",
    "parse_mag_raw_sample",
    "parse_magframe_event",
    "parse_magframe_status",
]

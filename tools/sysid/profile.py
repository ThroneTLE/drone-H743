"""辨识档案 —— "一套程序适配多种惯量"的载体。

一个档案 = **台架几何 + 机体参数快照 + 激励编排 + 辨识结果**，存成一个命名 JSON，
放在 `data/identification/profiles/<name>.json`。

为什么要有它：换一块电池、加一个配重、换一副桨，惯量就变了。没有档案的话，
每次都得重新猜一遍"上次那组参数是在什么条件下辨的"，而那个条件恰恰是唯一决定
结论能不能复用的东西。有了档案，换配置就是换一个档案名。

档案里**同时**存机体参数快照：辨识结论只在那组几何下成立，把 `airframe.*` 一起
钉下来，才能在半年后回答"这组 PID 是在哪个机体上辨的"。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path

from .excitation import Excitation
from .rig import Rig

SCHEMA_VERSION = 1

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIR = ROOT / "data" / "identification" / "profiles"


@dataclass
class FitResult:
    """一趟辨识的结论。缺项一律留 None，不填占位数。

    填占位数最省事，但半年后没人分得清"0.019 是辨出来的还是默认值"。
    """

    inertia_kg_m2: float | None = None
    damping_n_m_s: float | None = None
    eccentricity_m: float | None = None
    delay_s: float | None = None
    fit_percent: float | None = None
    #: 阻尼与偏心在短激励下会强相关（两者都表现为"回中"）。相关性过高时
    #: 如实标注"不可分离"，而不是硬给一个数。
    damping_eccentricity_correlation: float | None = None
    notes: str = ""

    def to_dict(self) -> dict:
        return {
            "inertia_kg_m2": self.inertia_kg_m2,
            "damping_n_m_s": self.damping_n_m_s,
            "eccentricity_m": self.eccentricity_m,
            "delay_s": self.delay_s,
            "fit_percent": self.fit_percent,
            "damping_eccentricity_correlation":
                self.damping_eccentricity_correlation,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "FitResult":
        return cls(**{key: data.get(key) for key in
                      ("inertia_kg_m2", "damping_n_m_s", "eccentricity_m",
                       "delay_s", "fit_percent",
                       "damping_eccentricity_correlation")},
                   notes=str(data.get("notes", "")))


@dataclass
class Gains:
    """候选增益。**写 RAM 实时验证，永不自动落 Flash。**

    是否写 Flash 是作者在会话结束后的单独动作——程序替人做这个决定，
    等于把一组还没飞过的增益变成下次上电的默认值。
    """

    rate_kp: float | None = None
    rate_ki: float | None = None
    rate_kd: float | None = None
    att_kp: float | None = None

    def to_dict(self) -> dict:
        return {"rate_kp": self.rate_kp, "rate_ki": self.rate_ki,
                "rate_kd": self.rate_kd, "att_kp": self.att_kp}

    @classmethod
    def from_dict(cls, data: dict) -> "Gains":
        return cls(rate_kp=data.get("rate_kp"), rate_ki=data.get("rate_ki"),
                   rate_kd=data.get("rate_kd"), att_kp=data.get("att_kp"))

    def param_commands(self, prefix: str = "coax.", *,
                       axes: tuple[str, ...] = ("roll", "pitch")) -> list[str]:
        """生成 `SYSID PARAM` 命令行：固件只写 RAM 的试用命令。**不含 SAVE**，这是刻意的。

        不用 `PARAM SET`：它成功后固件 1.5 s 自动存 Flash，等于把没飞过的增益变成
        下次上电的默认值。`axes` 只写这一趟辨识到的轴：杆沿俯仰轴就只写俯仰，
        斜杆辨的是两轴混合，两轴写同一个值。偏航不写：本次不辨 yaw。
        """
        unknown = set(axes) - {"roll", "pitch"}
        if unknown or not axes:
            raise ValueError(f"只能写 roll / pitch 轴：{sorted(unknown) or axes}")
        patterns = {
            "rate_kp": "rate_{axis}_kp",
            "rate_ki": "rate_{axis}_ki",
            "rate_kd": "rate_{axis}_kd",
            "att_kp": "att_{axis}_kp",
        }
        out = []
        for key, pattern in patterns.items():
            value = getattr(self, key)
            if value is None:
                continue
            for axis in ("roll", "pitch"):
                if axis in axes:
                    out.append(f"SYSID PARAM {prefix}{pattern.format(axis=axis)} {value:.6g}")
        return out


@dataclass
class IdentProfile:
    name: str
    rig: Rig = field(default_factory=Rig)
    excitation: Excitation = field(default_factory=Excitation)
    #: `airframe.*` 的完整快照。结论只在这组几何下成立。
    airframe: dict[str, float] = field(default_factory=dict)
    #: 前馈用的假定惯量。模型反演与它的初值无关地收敛，所以这只是起点。
    assumed_inertia_kg_m2: float | None = None
    fit: FitResult = field(default_factory=FitResult)
    gains: Gains = field(default_factory=Gains)
    firmware_id: str = ""
    created_utc: str = ""
    updated_utc: str = ""
    schema_version: int = SCHEMA_VERSION

    # ------------------------------------------------------------------ 序列化

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "rig": self.rig.to_dict(),
            "excitation": self.excitation.to_dict(),
            "airframe": dict(self.airframe),
            "assumed_inertia_kg_m2": self.assumed_inertia_kg_m2,
            "fit": self.fit.to_dict(),
            "gains": self.gains.to_dict(),
            "firmware_id": self.firmware_id,
            "created_utc": self.created_utc,
            "updated_utc": self.updated_utc,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "IdentProfile":
        version = int(data.get("schema_version", SCHEMA_VERSION))
        if version > SCHEMA_VERSION:
            raise ValueError(
                f"档案 schema_version={version} 比本程序（{SCHEMA_VERSION}）新；"
                "宁可不读，也不要按旧结构解出一组看着正常的错值")
        return cls(
            name=str(data["name"]),
            rig=Rig.from_dict(data.get("rig", {})),
            excitation=Excitation.from_dict(data.get("excitation", {})),
            airframe={k: float(v) for k, v in data.get("airframe", {}).items()},
            assumed_inertia_kg_m2=data.get("assumed_inertia_kg_m2"),
            fit=FitResult.from_dict(data.get("fit", {})),
            gains=Gains.from_dict(data.get("gains", {})),
            firmware_id=str(data.get("firmware_id", "")),
            created_utc=str(data.get("created_utc", "")),
            updated_utc=str(data.get("updated_utc", "")),
            schema_version=version,
        )

    # ------------------------------------------------------------------ 存取

    def save(self, directory: Path | None = None) -> Path:
        directory = directory or PROFILE_DIR
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
        if not self.created_utc:
            self.created_utc = stamp
        self.updated_utc = stamp
        path = directory / f"{_safe_name(self.name)}.json"
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2),
                        encoding="utf-8")
        return path

    @classmethod
    def load(cls, name_or_path: str | Path,
             directory: Path | None = None) -> "IdentProfile":
        path = Path(name_or_path)
        if path.suffix != ".json":
            directory = directory or PROFILE_DIR
            path = directory / f"{_safe_name(str(name_or_path))}.json"
        return cls.from_dict(json.loads(path.read_text(encoding="utf-8")))

    @classmethod
    def list_names(cls, directory: Path | None = None) -> list[str]:
        directory = directory or PROFILE_DIR
        if not directory.is_dir():
            return []
        return sorted(p.stem for p in directory.glob("*.json"))

    # ------------------------------------------------------------------ 收敛

    def next_assumed_inertia(self) -> float | None:
        """把这一趟辨出来的惯量作为下一趟的前馈初值。

        模型反演的收敛机制：实测角加速度与期望角加速度之比就是 I_true/I_est。
        反复几轮之后估计与初值无关地收敛——这就是"一套程序适配多种惯量"的出口。
        """
        return self.fit.inertia_kg_m2 or self.assumed_inertia_kg_m2

    def with_converged_inertia(self) -> "IdentProfile":
        return replace(self, assumed_inertia_kg_m2=self.next_assumed_inertia())


def _safe_name(name: str) -> str:
    keep = [c if (c.isalnum() or c in "-_.") else "-" for c in name.strip()]
    cleaned = "".join(keep).strip("-") or "unnamed"
    return cleaned

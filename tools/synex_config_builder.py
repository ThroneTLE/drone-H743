#!/usr/bin/env python3
"""Inspect and build Synex Qt INI configurations without editing @Variant text."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path


CHANNEL_NAMES = [
    "Roll_deg",
    "Pitch_deg",
    "Yaw_deg",
    "Height_m",
    "FC_Time_s",
    "Vel_X_m_s",
    "Vel_Y_m_s",
    "Roll_Rate_KD",
    "Pitch_Rate_KD",
    "Yaw_Angle_KP",
    "Yaw_Rate_KD",
    "Pos_X_KP",
    "Pos_Y_KP",
    "Vel_X_KD",
    "Vel_Y_KD",
    "Pos_X_m",
    "Pos_Y_m",
    "Vel_Loop_Enable",
    "Roll_Angle_KP",
    "Pitch_Angle_KP",
    "Pos_Z_KP",
    "Pos_Z_KI",
    "Vel_Z_KD",
    "IMU_Accel_Error_deg",
    "IMU_Accel_Ignored",
    "IMU_Accel_Recovery",
    "IMU_Accel_Correction_Count",
    "IMU_Accel_Norm_Rejected",
]

WORKSPACE_ORDER = ["飞行监控", "姿态参数", "速度参数", "ws1"]


def _load_qt(qt_path: Path | None) -> None:
    if qt_path is not None:
        sys.path.insert(0, str(qt_path))


def inspect_config(path: Path) -> int:
    from PySide6.QtCore import QSettings

    settings = QSettings(str(path), QSettings.IniFormat)
    print(f"status={settings.status().name} keys={len(settings.allKeys())}")
    count = int(settings.value("PlotSettings/workspaceCount", 0))
    print(f"workspaces={count}")
    for page_index in range(1, count + 1):
        prefix = f"PlotSettings/Page{page_index}"
        tile_count = int(settings.value(f"{prefix}/tileCount", 0))
        print(f"page={page_index} name={settings.value(f'{prefix}/name')} tiles={tile_count}")
        for tile_index in range(1, tile_count + 1):
            tile = f"{prefix}/Tile{tile_index}"
            state = settings.value(f"{tile}/itemState")
            print(
                json.dumps(
                    {
                        "tile": tile_index,
                        "type": int(settings.value(f"{tile}/itemType", -1)),
                        "title": settings.value(f"{tile}/title", ""),
                        "geometry": [
                            int(settings.value(f"{tile}/col", 0)),
                            int(settings.value(f"{tile}/row", 0)),
                            int(settings.value(f"{tile}/colSpan", 0)),
                            int(settings.value(f"{tile}/rowSpan", 0)),
                        ],
                        "state_type": type(state).__name__,
                        "state": state,
                    },
                    ensure_ascii=False,
                    default=str,
                )
            )
    return 0


def finalize_config(source: Path, output: Path) -> int:
    from PySide6.QtCore import QSettings

    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    settings = QSettings(str(output), QSettings.IniFormat)

    workspace_count = int(settings.value("PlotSettings/workspaceCount", 0))
    workspace_by_name: dict[str, dict[str, object]] = {}
    for page_index in range(1, workspace_count + 1):
        prefix = f"PlotSettings/Page{page_index}/"
        page_name = str(settings.value(f"PlotSettings/Page{page_index}/name", ""))
        workspace_by_name[page_name] = {
            key[len(prefix) :]: settings.value(key)
            for key in settings.allKeys()
            if key.startswith(prefix)
        }
    missing = [name for name in WORKSPACE_ORDER if name not in workspace_by_name]
    if missing:
        raise RuntimeError(f"missing workspaces: {missing}")
    for page_index in range(1, workspace_count + 1):
        settings.remove(f"PlotSettings/Page{page_index}")
    for page_index, page_name in enumerate(WORKSPACE_ORDER, start=1):
        prefix = f"PlotSettings/Page{page_index}"
        for relative_key, value in workspace_by_name[page_name].items():
            settings.setValue(f"{prefix}/{relative_key}", value)
    settings.setValue("PlotSettings/workspaceCount", len(WORKSPACE_ORDER))

    settings.setValue("ConnectionSettings/ConnectionType", "串口")
    settings.setValue("ConnectionSettings/SelectedPortName", "COM10")
    settings.setValue("ConnectionSettings/SelectedBaudRateText", "57600")
    settings.setValue("ConnectionSettings/ProtocolType", "JustFloat")
    settings.setValue("MainWindowSettings/SerialChannelNum", len(CHANNEL_NAMES))
    settings.setValue("MainWindowSettings/PlotRefreshFps", 40)
    settings.setValue("MainWindowSettings/PlotSampleTimeMs", 25.0)
    settings.setValue("MainWindowSettings/SerialbufferSize", 20000)
    settings.setValue("MainWindowSettings/PlotMSAASamples", 2)
    settings.setValue("MainWindowSettings/LastNavigationPageId", "Plot")
    settings.setValue("PlotSettings/currentWorkspaceIndex", 0)

    for index, name in enumerate(CHANNEL_NAMES):
        settings.setValue(f"ChannelNames/name{index}", name)
    for index in range(len(CHANNEL_NAMES), 256):
        settings.remove(f"ChannelNames/name{index}")
        settings.remove(f"ChannelColors/color{index}")
        settings.remove(f"ChannelGains/gain{index}")
        settings.remove(f"ChannelBiases/bias{index}")

    settings.sync()
    reread = QSettings(str(output), QSettings.IniFormat)
    assert int(reread.value("MainWindowSettings/SerialChannelNum")) == len(CHANNEL_NAMES)
    assert float(reread.value("MainWindowSettings/PlotSampleTimeMs")) == 25.0
    assert int(reread.value("PlotSettings/workspaceCount")) == 4
    assert [
        reread.value(f"PlotSettings/Page{index}/name") for index in range(1, 5)
    ] == WORKSPACE_ORDER
    assert [
        int(reread.value(f"PlotSettings/Page{index}/tileCount"))
        for index in range(1, 5)
    ] == [11, 18, 19, 19]
    assert reread.value("ChannelNames/name11") == "Pos_X_KP"
    assert reread.value("ChannelNames/name12") == "Pos_Y_KP"
    assert reread.value("ChannelNames/name13") == "Vel_X_KD"
    assert reread.value("ChannelNames/name14") == "Vel_Y_KD"
    assert reread.value("ChannelNames/name15") == "Pos_X_m"
    assert reread.value("ChannelNames/name16") == "Pos_Y_m"
    assert reread.value("ChannelNames/name17") == "Vel_Loop_Enable"
    assert reread.value("ChannelNames/name18") == "Roll_Angle_KP"
    assert reread.value("ChannelNames/name19") == "Pitch_Angle_KP"
    assert reread.value("ChannelNames/name20") == "Pos_Z_KP"
    assert reread.value("ChannelNames/name21") == "Pos_Z_KI"
    assert reread.value("ChannelNames/name22") == "Vel_Z_KD"
    assert reread.value("ChannelNames/name23") == "IMU_Accel_Error_deg"
    assert reread.value("ChannelNames/name24") == "IMU_Accel_Ignored"
    assert reread.value("ChannelNames/name25") == "IMU_Accel_Recovery"
    assert reread.value("ChannelNames/name26") == "IMU_Accel_Correction_Count"
    assert reread.value("ChannelNames/name27") == "IMU_Accel_Norm_Rejected"
    print(f"wrote={output} status={reread.status().name} keys={len(reread.allKeys())}")
    return 0 if reread.status() == QSettings.NoError else 1


def roundtrip(source: Path, output: Path) -> int:
    from PySide6.QtCore import QSettings

    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    settings = QSettings(str(output), QSettings.IniFormat)
    values = {key: settings.value(key) for key in settings.allKeys()}
    settings.clear()
    for key, value in values.items():
        settings.setValue(key, value)
    settings.sync()
    print(f"wrote={output} status={settings.status().name} keys={len(settings.allKeys())}")
    return 0 if settings.status() == QSettings.NoError else 1


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser()
    parser.add_argument("--qt-path", type=Path)
    sub = parser.add_subparsers(dest="command", required=True)
    inspect_parser = sub.add_parser("inspect")
    inspect_parser.add_argument("config", type=Path)
    roundtrip_parser = sub.add_parser("roundtrip")
    roundtrip_parser.add_argument("source", type=Path)
    roundtrip_parser.add_argument("output", type=Path)
    finalize_parser = sub.add_parser("finalize")
    finalize_parser.add_argument("source", type=Path)
    finalize_parser.add_argument("output", type=Path)
    args = parser.parse_args()
    _load_qt(args.qt_path)
    if args.command == "inspect":
        return inspect_config(args.config)
    if args.command == "finalize":
        return finalize_config(args.source, args.output)
    return roundtrip(args.source, args.output)


if __name__ == "__main__":
    raise SystemExit(main())

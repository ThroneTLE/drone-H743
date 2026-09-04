from __future__ import annotations

from tools.controller_cascade_analysis import build_report


def test_report_refuses_old_or_unknown_frame_logs() -> None:
    report = build_report(
        [
            {
                "path": "real-v8.bin",
                "versions": [7, 8],
                "record_count": 6909,
                "parse_error_count": 0,
                "firmware_crc32": ["0x020E0B1F"],
                "frame_provenance": ["unknown_legacy_frd"],
                "eligible": False,
            }
        ]
    )
    assert report["verdict"] == "INCOMPLETE"
    assert report["reason"] == "缺匹配实录，性能待验证"
    assert report["input_data_path"] is None
    assert report["comparisons"]["a_sp_error"] is None
    assert "non-zero Z velocity-P suitability" in report["cannot_prove"]

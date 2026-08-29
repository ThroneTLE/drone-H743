"""IMUCAL? 上下文校验的报错。

背景：2026-08-29 点"应用候选到 RAM"弹出 `target IMUCAL context reply is
incomplete`。这句话把收到的行全扔了，分不清三种完全不同的情况：链路根本没通、
飞控回了 valid=0、或者只是缺某一行。实测那次是板子死机、一个字节都没回。
"""

from __future__ import annotations

import pytest

from tools.imucal_protocol import ImuCalProtocolError, _verify_target_context


class FakeLink:
    """按脚本回放飞控的应答；read_text_line 用完就返回 None。"""

    def __init__(self, lines):
        self.lines = list(lines)
        self.sent: list[str] = []

    def send(self, command: str) -> None:
        self.sent.append(command)

    def read_text_line(self, _deadline: float):
        return self.lines.pop(0) if self.lines else None


class FakeEncoded:
    firmware_crc32 = 0x15F33731
    base_generation = 1
    orientation_code = 3


def verify(lines):
    link = FakeLink(lines)
    transcript: list[str] = []
    with pytest.raises(ImuCalProtocolError) as failure:
        _verify_target_context(link, FakeEncoded(), transcript)
    return str(failure.value)


def test_a_silent_target_is_reported_as_a_link_problem_not_a_calibration_one() -> None:
    message = verify([])

    assert "一个字节都没回" in message
    assert "串口链路不通" in message
    assert "拔插一次 USB" in message
    assert "标定" not in message.split("：")[0], "别把链路问题说成标定问题"


def test_a_valid_zero_reply_names_the_snapshot_as_the_cause() -> None:
    message = verify(["IMUCAL valid=0 source=active_snapshot unavailable"])

    assert "valid=0" in message
    assert "标定快照或固件身份读不出来" in message


def test_a_partial_reply_names_exactly_which_line_is_missing() -> None:
    message = verify([
        "IMUCAL event=status reason=none transfer=idle received=0 expected=0 "
        "candidate=0 applied=0 commit_pending=0 dirty=0 arm_lock=0 request=0",
        "IMUCAL identity firmware_crc32=0x15FFBC36 image_bytes=356620",
    ])

    assert "generations 行" in message
    assert "event/candidate 行" not in message
    assert "identity/firmware_crc32 行" not in message
    assert "共收到 2 行" in message


def context_lines(*, candidate="0", applied="0", dirty="0",
                  persisted=1, persisted_orientation=3, crc="0x15F33731"):
    return [
        f"IMUCAL event=status reason=none transfer=empty received=0 expected=0 "
        f"candidate={candidate} applied={applied} commit_pending=0 dirty={dirty} "
        f"arm_lock=0 request=0",
        f"IMUCAL generations active={persisted} persisted={persisted} record=1 base=0 "
        f"active_mask=0x01 persisted_mask=0x01 candidate_mask=0x00 active_orientation=3 "
        f"persisted_orientation={persisted_orientation} candidate_orientation=255",
        f"IMUCAL identity firmware_crc32={crc} image_bytes=356620",
    ]


def test_a_clean_target_passes_the_context_check() -> None:
    link = FakeLink(context_lines())
    transcript: list[str] = []

    _verify_target_context(link, FakeEncoded(), transcript)

    assert link.sent == ["IMUCAL?"]


def test_an_already_applied_candidate_tells_you_to_revert() -> None:
    message = verify(context_lines(candidate="1", applied="1"))

    assert "candidate=1" in message and "applied=1" in message
    assert "撤销 RAM 候选" in message


def test_a_dirty_parameter_area_is_distinguished_from_a_stale_candidate() -> None:
    message = verify(context_lines(dirty="1"))

    assert "dirty=1" in message
    assert "未落盘的改动" in message
    assert "撤销 RAM 候选" not in message


def test_a_bumped_generation_explains_that_a_power_cycle_resets_it() -> None:
    """实测：APPLY 一次再 REVERT，persisted 从 1 跳到 3，同一批证据就再也应用不上。

    不是证据坏了，是 seqlock 计数往前走了；重新上电会归位。
    """
    message = verify(context_lines(persisted=3))

    assert "persisted=3" in message and "代次 1" in message
    assert "断电重新插拔 USB" in message
    assert "重新采集" not in message, "代次前进不需要重采六面，说成重采会白干一遍"


def test_an_older_target_generation_does_mean_the_evidence_is_foreign() -> None:
    link = FakeLink(context_lines(persisted=0))
    transcript: list[str] = []
    with pytest.raises(ImuCalProtocolError) as failure:
        _verify_target_context(link, FakeEncoded(), transcript)

    assert "请重新采集" in str(failure.value)


def test_an_orientation_mismatch_says_the_evidence_must_be_recaptured() -> None:
    message = verify(context_lines(persisted_orientation=1))

    assert "安装朝向对不上" in message
    assert "重新采集" in message


def test_a_different_firmware_image_is_refused_with_both_crcs() -> None:
    message = verify(context_lines(crc="0xDEADBEEF"))

    assert "固件不是同一份" in message
    assert "deadbeef" in message and "15f33731" in message
    assert "重新采集" in message

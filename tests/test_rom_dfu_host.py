from __future__ import annotations

import io
import hashlib
import struct
import subprocess
import threading
from pathlib import Path

import pytest

from tools import rom_dfu


def executable_file(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"cli")
    return path


def firmware_file(path: Path) -> Path:
    suffix = path.suffix.lower()
    initial_msp = 0x20020000
    reset_handler = 0x08000009
    vector_and_code = struct.pack("<II", initial_msp, reset_handler) + b"\x00"
    if suffix == ".elf":
        ident = b"\x7fELF" + bytes((1, 1, 1, 0)) + bytes(8)
        header = struct.pack(
            "<16sHHIIIIIHHHHHH",
            ident,
            2,
            40,
            1,
            reset_handler,
            52,
            0,
            0,
            52,
            32,
            1,
            0,
            0,
            0,
        )
        program_header = struct.pack(
            "<IIIIIIII", 1, 84, 0x08000000, 0x08000000,
            len(vector_and_code), len(vector_and_code), 5, 4
        )
        path.write_bytes(header + program_header + vector_and_code)
    elif suffix == ".hex":
        path.write_text(
            "\n".join(
                (
                    hex_record(0, 0x04, bytes.fromhex("0800")),
                    hex_record(0, 0x00, vector_and_code),
                    hex_record(0, 0x05, reset_handler.to_bytes(4, "big")),
                    hex_record(0, 0x01, b""),
                )
            )
            + "\n",
            encoding="ascii",
        )
    else:
        path.write_bytes(b"firmware")
    return path


def hex_record(address: int, record_type: int, data: bytes) -> str:
    record = bytes((len(data), (address >> 8) & 0xFF, address & 0xFF, record_type)) + data
    checksum = (-sum(record)) & 0xFF
    return ":" + (record + bytes((checksum,))).hex().upper()


def test_cli_discovery_prefers_path_and_finds_common_cubeclt_layout(tmp_path: Path) -> None:
    from_path = executable_file(tmp_path / "path" / "STM32_Programmer_CLI.exe")
    assert rom_dfu.resolve_cubeprogrammer_cli(
        environ={}, which=lambda _name: str(from_path)
    ) == from_path.resolve()

    program_files = tmp_path / "Program Files"
    from_cubeclt = executable_file(
        program_files
        / "STMicroelectronics"
        / "STM32Cube"
        / "STM32CubeCLT_1.18.0"
        / "STM32CubeProgrammer"
        / "bin"
        / "STM32_Programmer_CLI.exe"
    )
    assert rom_dfu.resolve_cubeprogrammer_cli(
        environ={"ProgramFiles": str(program_files)}, which=lambda _name: None
    ) == from_cubeclt.resolve()


def test_v0_image_gate_accepts_only_nonempty_elf_or_hex(tmp_path: Path) -> None:
    elf = firmware_file(tmp_path / "flight image.elf")
    ihex = firmware_file(tmp_path / "flight.hex")
    elf_info = rom_dfu.validate_firmware_image(elf)
    hex_info = rom_dfu.validate_firmware_image(ihex)
    assert elf_info.path == elf.resolve()
    assert elf_info.image_format.startswith("ELF32 ARM")
    assert elf_info.entry == 0x08000009
    assert elf_info.load_ranges == ((0x08000000, 0x08000008),)
    assert elf_info.sha256 == hashlib.sha256(elf.read_bytes()).hexdigest()
    assert hex_info.path == ihex.resolve()
    assert hex_info.image_format == "Intel HEX"
    assert hex_info.entry == 0x08000009
    assert hex_info.address_min == 0x08000000

    binary = firmware_file(tmp_path / "flight.bin")
    with pytest.raises(rom_dfu.FirmwareImageError, match="仅接受"):
        rom_dfu.validate_firmware_image(binary)
    with pytest.raises(rom_dfu.FirmwareImageError, match="不存在"):
        rom_dfu.validate_firmware_image(tmp_path / "missing.elf")


def test_image_gate_rejects_wrong_elf_machine_and_out_of_flash_hex(tmp_path: Path) -> None:
    bad_machine = firmware_file(tmp_path / "bad-machine.elf")
    content = bytearray(bad_machine.read_bytes())
    struct.pack_into("<H", content, 18, 62)
    bad_machine.write_bytes(content)
    with pytest.raises(rom_dfu.FirmwareImageError, match="EM_ARM"):
        rom_dfu.validate_firmware_image(bad_machine)

    out_of_range = tmp_path / "outside.hex"
    out_of_range.write_text(
        "\n".join(
            (
                hex_record(0, 0x04, bytes.fromhex("2000")),
                hex_record(0, 0x00, b"\xAA"),
                hex_record(0, 0x01, b""),
            )
        ),
        encoding="ascii",
    )
    with pytest.raises(rom_dfu.FirmwareImageError, match="扩展线性地址越界"):
        rom_dfu.validate_firmware_image(out_of_range)

    bad_checksum = firmware_file(tmp_path / "bad-checksum.hex")
    lines = bad_checksum.read_text(encoding="ascii").splitlines()
    lines[1] = lines[1][:-2] + "00"
    bad_checksum.write_text("\n".join(lines), encoding="ascii")
    with pytest.raises(rom_dfu.FirmwareImageError, match="checksum"):
        rom_dfu.validate_firmware_image(bad_checksum)


def test_image_gate_requires_complete_valid_h743_vector_table(tmp_path: Path) -> None:
    partial = firmware_file(tmp_path / "partial.elf")
    content = bytearray(partial.read_bytes())
    struct.pack_into("<I", content, 52 + 16, 4)
    struct.pack_into("<I", content, 52 + 20, 4)
    partial.write_bytes(content[:88])
    with pytest.raises(rom_dfu.FirmwareImageError, match="启动向量"):
        rom_dfu.validate_firmware_image(partial)

    bad_msp = firmware_file(tmp_path / "bad-msp.hex")
    lines = bad_msp.read_text(encoding="ascii").splitlines()
    bad_vector = struct.pack("<II", 0x10000000, 0x08000009) + b"\x00"
    lines[1] = hex_record(0, 0x00, bad_vector)
    bad_msp.write_text("\n".join(lines) + "\n", encoding="ascii")
    with pytest.raises(rom_dfu.FirmwareImageError, match="initial MSP"):
        rom_dfu.validate_firmware_image(bad_msp)

    bad_reset = firmware_file(tmp_path / "bad-reset.elf")
    content = bytearray(bad_reset.read_bytes())
    struct.pack_into("<I", content, 84 + 4, 0x08001001)
    bad_reset.write_bytes(content)
    with pytest.raises(rom_dfu.FirmwareImageError, match="Reset_Handler"):
        rom_dfu.validate_firmware_image(bad_reset)


def test_freeze_binds_exact_validated_bytes_and_rejects_source_toctou(tmp_path: Path) -> None:
    source = firmware_file(tmp_path / "source.elf")
    info = rom_dfu.validate_firmware_image(source)
    frozen = rom_dfu.freeze_firmware_image(info, tmp_path / "attempt_image.elf")
    assert frozen.sha256 == info.sha256
    assert frozen.path.read_bytes() == source.read_bytes()

    changed = bytearray(source.read_bytes())
    changed[-1] ^= 0x01
    source.write_bytes(changed)
    with pytest.raises(rom_dfu.FirmwareImageError, match="确认后已变化"):
        rom_dfu.freeze_firmware_image(info, tmp_path / "second_attempt.elf")


def test_usb_listing_uses_argv_without_shell_and_parses_ports(tmp_path: Path) -> None:
    cli = executable_file(tmp_path / "STM32_Programmer_CLI.exe")
    calls: list[tuple[list[str], dict[str, object]]] = []

    def fake_run(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(
            argv,
            0,
            stdout="Device Index : USB2\nDevice Index : USB1\nDevice Index : USB2\n",
            stderr="",
        )

    assert rom_dfu.list_usb_dfu_ports(cli, run=fake_run) == ("USB1", "USB2")
    assert calls[0][0] == [str(cli), "-l", "usb"]
    assert calls[0][1]["shell"] is False


def test_wait_for_dfu_polls_enumeration_but_refuses_ambiguous_devices() -> None:
    replies = iter(((), (), ("USB1",)))
    logs: list[str] = []
    assert rom_dfu.wait_for_usb_dfu(
        "ignored",
        timeout_s=1.0,
        poll_interval_s=0.001,
        list_ports=lambda _cli: next(replies),
        on_log=logs.append,
    ) == "USB1"
    assert any("等待" in line for line in logs)
    assert logs[-1].endswith("USB1")

    with pytest.raises(rom_dfu.RomDfuError, match="多个"):
        rom_dfu.wait_for_usb_dfu(
            "ignored",
            timeout_s=1.0,
            list_ports=lambda _cli: ("USB1", "USB2"),
        )


class CompletedPopen:
    def __init__(self) -> None:
        self.stdout = io.StringIO("Download 25%\nDownload 100%\nFile download complete\n")
        self.returncode = 0

    def poll(self) -> int:
        return self.returncode

    def terminate(self) -> None:
        self.returncode = -15

    def kill(self) -> None:
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        return self.returncode


def test_flash_runs_exact_program_verify_dfu_start_argv_and_streams_progress(tmp_path: Path) -> None:
    cli = executable_file(tmp_path / "STM32_Programmer_CLI.exe")
    image = firmware_file(tmp_path / "drone H743.elf")
    calls: list[tuple[list[str], dict[str, object]]] = []

    def fake_popen(argv: list[str], **kwargs: object) -> CompletedPopen:
        calls.append((argv, kwargs))
        return CompletedPopen()

    lines: list[str] = []
    progress: list[int] = []
    result = rom_dfu.flash_firmware(
        cli,
        image,
        popen=fake_popen,  # type: ignore[arg-type]
        on_output=lines.append,
        on_progress=progress.append,
    )

    assert result.success
    assert calls[0][0] == [
        str(cli.resolve()),
        "-c",
        "port=USB1",
        "-w",
        str(image.resolve()),
        "-v",
        "-s",
        "0x08000000",
    ]
    assert "-rst" not in calls[0][0]
    assert calls[0][1]["shell"] is False
    assert progress == [25, 100]
    assert lines[-1] == "File download complete"


def test_verified_download_is_distinct_from_dfu_start_failure() -> None:
    result = rom_dfu.FlashResult(
        argv=("cli",), returncode=1,
        output=("Download verified successfully", "Error: start failed"),
        duration_s=1.0,
    )
    assert not result.success
    assert result.download_verified


def test_pre_cancel_never_starts_cubeprogrammer(tmp_path: Path) -> None:
    cli = executable_file(tmp_path / "STM32_Programmer_CLI.exe")
    image = firmware_file(tmp_path / "drone.elf")
    cancelled = threading.Event()
    cancelled.set()

    def forbidden_popen(*_args: object, **_kwargs: object):
        raise AssertionError("CubeProgrammer must not start after pre-cancel")

    with pytest.raises(rom_dfu.DfuCancelledError):
        rom_dfu.flash_firmware(
            cli,
            image,
            cancel_event=cancelled,
            popen=forbidden_popen,  # type: ignore[arg-type]
        )


def test_expected_sha_is_rechecked_before_cubeprogrammer_popen(tmp_path: Path) -> None:
    cli = executable_file(tmp_path / "STM32_Programmer_CLI.exe")
    image = firmware_file(tmp_path / "frozen.elf")
    expected = rom_dfu.validate_firmware_image(image).sha256
    changed = bytearray(image.read_bytes())
    changed[-1] ^= 0x01
    image.write_bytes(changed)

    def forbidden_popen(*_args: object, **_kwargs: object):
        raise AssertionError("CubeProgrammer must not start for a changed image")

    with pytest.raises(rom_dfu.FirmwareImageError, match="待烧录镜像已变化"):
        rom_dfu.flash_firmware(
            cli,
            image,
            expected_sha256=expected,
            popen=forbidden_popen,  # type: ignore[arg-type]
        )

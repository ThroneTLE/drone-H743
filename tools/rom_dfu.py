#!/usr/bin/env python3
"""Safe host-side helpers for STM32 ROM USB DFU firmware updates.

V0 deliberately delegates device programming to STM32CubeProgrammer.  It does
not implement a bootloader protocol, invoke a shell, accept raw BIN images, or
retry a failed program/verify operation.
"""

from __future__ import annotations

import os
import queue
import re
import shutil
import hashlib
import locale
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence


SUPPORTED_IMAGE_SUFFIXES = frozenset({".elf", ".hex"})
DEFAULT_ENUMERATION_TIMEOUT_S = 25.0
DEFAULT_FLASH_TIMEOUT_S = 120.0
_USB_PORT_RE = re.compile(r"\bUSB(\d+)\b", re.IGNORECASE)
_PROGRESS_RE = re.compile(r"(?<!\d)(100|[1-9]?\d)\s*%")
STM32H743_FLASH_START = 0x08000000
STM32H743_FLASH_END = 0x081FFFFF
STM32H743_VECTOR_SIZE = 8


class RomDfuError(RuntimeError):
    """Base error for deterministic host-side DFU failures."""


class DfuToolNotFoundError(RomDfuError):
    """STM32CubeProgrammer CLI could not be found."""


class FirmwareImageError(RomDfuError):
    """The selected V0 image is missing or unsupported."""


class DfuEnumerationTimeoutError(RomDfuError):
    """The STM32 ROM DFU interface did not enumerate in time."""


class DfuCancelledError(RomDfuError):
    """The user cancelled before programming began."""


@dataclass(frozen=True)
class FlashResult:
    argv: tuple[str, ...]
    returncode: int
    output: tuple[str, ...]
    duration_s: float
    cancelled: bool = False
    timed_out: bool = False

    @property
    def success(self) -> bool:
        return self.returncode == 0 and not self.cancelled and not self.timed_out

    @property
    def download_verified(self) -> bool:
        return any(
            "download verified successfully" in line.casefold()
            for line in self.output
        )


@dataclass(frozen=True)
class FirmwareImageInfo:
    path: Path
    image_format: str
    size: int
    sha256: str
    entry: int | None
    load_ranges: tuple[tuple[int, int], ...]

    @property
    def address_min(self) -> int:
        return min(start for start, _end in self.load_ranges)

    @property
    def address_max(self) -> int:
        return max(end for _start, end in self.load_ranges)

    @property
    def summary(self) -> str:
        entry = f"0x{self.entry:08X}" if self.entry is not None else "n/a"
        return (
            f"{self.image_format} · {self.size} bytes · sha256={self.sha256} · "
            f"entry={entry} · flash=0x{self.address_min:08X}..0x{self.address_max:08X}"
        )


def _deduplicate_paths(paths: Iterable[Path]) -> tuple[Path, ...]:
    unique: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        key = os.path.normcase(os.path.abspath(os.fspath(path)))
        if key in seen:
            continue
        seen.add(key)
        unique.append(path)
    return tuple(unique)


def cubeprogrammer_candidates(
    *,
    environ: Mapping[str, str] | None = None,
    which: Callable[[str], str | None] = shutil.which,
) -> tuple[Path, ...]:
    """Return PATH and common STM32CubeProgrammer/CubeCLT candidates."""

    env = os.environ if environ is None else environ
    candidates: list[Path] = []
    for executable in ("STM32_Programmer_CLI.exe", "STM32_Programmer_CLI"):
        located = which(executable)
        if located:
            candidates.append(Path(located))

    for variable in (
        "STM32CUBEPROGRAMMER_PATH",
        "STM32_CUBE_PROGRAMMER_PATH",
        "STM32CUBECLT_PATH",
        "STM32_CUBE_CLT_PATH",
    ):
        raw = env.get(variable, "").strip()
        if not raw:
            continue
        root = Path(raw)
        candidates.extend(
            (
                root / "STM32_Programmer_CLI.exe",
                root / "bin" / "STM32_Programmer_CLI.exe",
                root / "STM32CubeProgrammer" / "bin" / "STM32_Programmer_CLI.exe",
            )
        )

    roots: list[Path] = []
    for variable in ("ProgramFiles", "ProgramFiles(x86)"):
        raw = env.get(variable, "").strip()
        if raw:
            roots.append(Path(raw) / "STMicroelectronics")
    # CubeCLT's default standalone installer commonly uses C:\ST.
    roots.append(Path("C:/ST"))

    for root in roots:
        candidates.extend(
            (
                root
                / "STM32Cube"
                / "STM32CubeProgrammer"
                / "bin"
                / "STM32_Programmer_CLI.exe",
                root
                / "STM32CubeProgrammer"
                / "bin"
                / "STM32_Programmer_CLI.exe",
            )
        )
        patterns = (
            "STM32CubeCLT*/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
            "STM32Cube/STM32CubeCLT*/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
        )
        if root.is_dir():
            for pattern in patterns:
                candidates.extend(sorted(root.glob(pattern), reverse=True))

    return _deduplicate_paths(candidates)


def resolve_cubeprogrammer_cli(
    explicit: str | os.PathLike[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    which: Callable[[str], str | None] = shutil.which,
) -> Path:
    """Resolve an executable CubeProgrammer CLI without invoking a shell."""

    if explicit is not None and str(explicit).strip():
        selected = Path(explicit).expanduser()
        if selected.is_dir():
            selected = selected / "STM32_Programmer_CLI.exe"
        if not selected.is_file():
            raise DfuToolNotFoundError(f"STM32CubeProgrammer CLI 不存在：{selected}")
        return selected.resolve()

    checked = cubeprogrammer_candidates(environ=environ, which=which)
    for candidate in checked:
        if candidate.is_file():
            return candidate.resolve()
    raise DfuToolNotFoundError(
        "未找到 STM32_Programmer_CLI；请安装 STM32CubeProgrammer/CubeCLT，"
        "或在升级页手动选择该可执行文件"
    )


def _require_flash_range(start: int, size: int, *, context: str) -> tuple[int, int]:
    if size <= 0:
        raise FirmwareImageError(f"{context} 长度必须大于 0")
    end = start + size - 1
    if end < start or start < STM32H743_FLASH_START or end > STM32H743_FLASH_END:
        raise FirmwareImageError(
            f"{context} 超出 STM32H743 内部 Flash：0x{start:08X}..0x{end:08X}"
        )
    return start, end


def _msp_in_h743_sram(initial_msp: int) -> bool:
    if initial_msp & 0x7:
        return False
    return any(
        start <= initial_msp <= end
        for start, end in (
            (0x20000000, 0x20020000),
            (0x24000000, 0x24080000),
            (0x30000000, 0x30048000),
            (0x38000000, 0x38010000),
        )
    )


def _validate_vector_table(
    vector: bytes,
    executable_ranges: Sequence[tuple[int, int]],
    *,
    context: str,
) -> None:
    if len(vector) != STM32H743_VECTOR_SIZE:
        raise FirmwareImageError(
            f"{context} 必须完整包含 0x08000000..0x08000007 启动向量"
        )
    initial_msp, reset_handler = struct.unpack("<II", vector)
    if not _msp_in_h743_sram(initial_msp):
        raise FirmwareImageError(
            f"{context} initial MSP=0x{initial_msp:08X} 不在 STM32H743 SRAM 或未按 8 字节对齐"
        )
    reset_address = reset_handler & ~1
    if (reset_handler & 1) == 0 or not any(
        start <= reset_address <= end for start, end in executable_ranges
    ):
        raise FirmwareImageError(
            f"{context} Reset_Handler=0x{reset_handler:08X} 不是已加载可执行 Flash 中的 Thumb 地址"
        )


def _inspect_elf32(image: Path, content: bytes) -> FirmwareImageInfo:
    if len(content) < 52 or content[:4] != b"\x7fELF":
        raise FirmwareImageError("ELF 文件头无效")
    if content[4] != 1 or content[5] != 1:
        raise FirmwareImageError("仅支持 ELF32 little-endian 固件")
    try:
        (
            _ident,
            _elf_type,
            machine,
            version,
            entry,
            program_header_offset,
            _section_header_offset,
            _flags,
            elf_header_size,
            program_header_size,
            program_header_count,
            _section_header_size,
            _section_header_count,
            _section_name_index,
        ) = struct.unpack_from("<16sHHIIIIIHHHHHH", content, 0)
    except struct.error as exc:
        raise FirmwareImageError("ELF32 文件头被截断") from exc
    if machine != 40:
        raise FirmwareImageError(f"ELF e_machine={machine}，不是 EM_ARM(40)")
    if version != 1 or elf_header_size < 52 or program_header_size < 32:
        raise FirmwareImageError("ELF32 版本或 program-header 尺寸无效")
    if not (STM32H743_FLASH_START <= entry <= STM32H743_FLASH_END):
        raise FirmwareImageError(f"ELF entry=0x{entry:08X} 不在 STM32H743 内部 Flash")
    table_end = program_header_offset + (program_header_size * program_header_count)
    if table_end < program_header_offset or table_end > len(content):
        raise FirmwareImageError("ELF program-header 表越界")

    ranges: list[tuple[int, int]] = []
    executable_ranges: list[tuple[int, int]] = []
    vector_bytes: dict[int, int] = {}
    for index in range(program_header_count):
        offset = program_header_offset + (index * program_header_size)
        try:
            (
                segment_type,
                file_offset,
                _virtual_address,
                physical_address,
                file_size,
                memory_size,
                segment_flags,
                _alignment,
            ) = struct.unpack_from("<IIIIIIII", content, offset)
        except struct.error as exc:
            raise FirmwareImageError(f"ELF PT_LOAD[{index}] 头被截断") from exc
        if segment_type != 1 or file_size == 0:
            continue
        if memory_size < file_size:
            raise FirmwareImageError(f"ELF PT_LOAD[{index}] p_memsz 小于 p_filesz")
        file_end = file_offset + file_size
        if file_end < file_offset or file_end > len(content):
            raise FirmwareImageError(f"ELF PT_LOAD[{index}] 文件数据越界")
        loaded_range = _require_flash_range(
            physical_address,
            file_size,
            context=f"ELF PT_LOAD[{index}] p_paddr",
        )
        ranges.append(loaded_range)
        if segment_flags & 0x1:
            executable_ranges.append(loaded_range)
        vector_start = max(physical_address, STM32H743_FLASH_START)
        vector_end = min(
            physical_address + file_size,
            STM32H743_FLASH_START + STM32H743_VECTOR_SIZE,
        )
        for address in range(vector_start, vector_end):
            vector_bytes[address] = content[file_offset + address - physical_address]
    if not ranges:
        raise FirmwareImageError("ELF 没有可写入的 PT_LOAD 段")
    vector = bytes(
        vector_bytes[address]
        for address in range(
            STM32H743_FLASH_START,
            STM32H743_FLASH_START + STM32H743_VECTOR_SIZE,
        )
        if address in vector_bytes
    )
    _validate_vector_table(vector, executable_ranges, context="ELF")
    return FirmwareImageInfo(
        path=image,
        image_format="ELF32 ARM little-endian",
        size=len(content),
        sha256=hashlib.sha256(content).hexdigest(),
        entry=entry,
        load_ranges=tuple(ranges),
    )


def _inspect_intel_hex(image: Path, content: bytes) -> FirmwareImageInfo:
    try:
        text = content.decode("ascii")
    except UnicodeDecodeError as exc:
        raise FirmwareImageError("HEX 文件不是 ASCII 文本") from exc
    base_address = 0
    entry: int | None = None
    ranges: list[tuple[int, int]] = []
    vector_bytes: dict[int, int] = {}
    eof_seen = False
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        if eof_seen:
            raise FirmwareImageError(f"HEX 第 {line_number} 行出现在 EOF 之后")
        if not line.startswith(":"):
            raise FirmwareImageError(f"HEX 第 {line_number} 行缺少冒号")
        encoded = line[1:]
        if len(encoded) % 2:
            raise FirmwareImageError(f"HEX 第 {line_number} 行十六进制长度无效")
        try:
            record = bytes.fromhex(encoded)
        except ValueError as exc:
            raise FirmwareImageError(f"HEX 第 {line_number} 行包含非十六进制字符") from exc
        if len(record) < 5:
            raise FirmwareImageError(f"HEX 第 {line_number} 行记录过短")
        byte_count = record[0]
        if len(record) != byte_count + 5:
            raise FirmwareImageError(f"HEX 第 {line_number} 行 byte-count 不匹配")
        if sum(record) & 0xFF:
            raise FirmwareImageError(f"HEX 第 {line_number} 行 checksum 错误")
        offset = (record[1] << 8) | record[2]
        record_type = record[3]
        data = record[4 : 4 + byte_count]
        if record_type == 0x00:
            if data:
                absolute_address = base_address + offset
                ranges.append(
                    _require_flash_range(
                        absolute_address,
                        len(data),
                        context=f"HEX 第 {line_number} 行数据",
                    )
                )
                vector_start = max(absolute_address, STM32H743_FLASH_START)
                vector_end = min(
                    absolute_address + len(data),
                    STM32H743_FLASH_START + STM32H743_VECTOR_SIZE,
                )
                for address in range(vector_start, vector_end):
                    vector_bytes[address] = data[address - absolute_address]
        elif record_type == 0x01:
            if byte_count != 0 or offset != 0:
                raise FirmwareImageError(f"HEX 第 {line_number} 行 EOF 记录无效")
            eof_seen = True
        elif record_type == 0x02:
            if byte_count != 2 or offset != 0:
                raise FirmwareImageError(f"HEX 第 {line_number} 行扩展段地址记录无效")
            base_address = int.from_bytes(data, "big") << 4
            if not (STM32H743_FLASH_START <= base_address <= STM32H743_FLASH_END):
                raise FirmwareImageError(
                    f"HEX 第 {line_number} 行扩展段地址越界：0x{base_address:08X}"
                )
        elif record_type == 0x04:
            if byte_count != 2 or offset != 0:
                raise FirmwareImageError(f"HEX 第 {line_number} 行扩展线性地址记录无效")
            base_address = int.from_bytes(data, "big") << 16
            if not (STM32H743_FLASH_START <= base_address <= STM32H743_FLASH_END):
                raise FirmwareImageError(
                    f"HEX 第 {line_number} 行扩展线性地址越界：0x{base_address:08X}"
                )
        elif record_type == 0x05:
            if byte_count != 4 or offset != 0:
                raise FirmwareImageError(f"HEX 第 {line_number} 行入口记录无效")
            entry = int.from_bytes(data, "big")
            if not (STM32H743_FLASH_START <= entry <= STM32H743_FLASH_END):
                raise FirmwareImageError(
                    f"HEX entry=0x{entry:08X} 不在 STM32H743 内部 Flash"
                )
        else:
            raise FirmwareImageError(
                f"HEX 第 {line_number} 行使用不支持的 record type 0x{record_type:02X}"
            )
    if not eof_seen:
        raise FirmwareImageError("HEX 缺少 EOF 记录")
    if not ranges:
        raise FirmwareImageError("HEX 没有可写入的数据记录")
    vector = bytes(
        vector_bytes[address]
        for address in range(
            STM32H743_FLASH_START,
            STM32H743_FLASH_START + STM32H743_VECTOR_SIZE,
        )
        if address in vector_bytes
    )
    _validate_vector_table(vector, ranges, context="HEX")
    return FirmwareImageInfo(
        path=image,
        image_format="Intel HEX",
        size=len(content),
        sha256=hashlib.sha256(content).hexdigest(),
        entry=entry,
        load_ranges=tuple(ranges),
    )


def validate_firmware_image(path: str | os.PathLike[str]) -> FirmwareImageInfo:
    """Parse and constrain an ELF32/HEX image to STM32H743 internal Flash."""

    image = Path(path).expanduser()
    if image.suffix.lower() not in SUPPORTED_IMAGE_SUFFIXES:
        raise FirmwareImageError("V0 仅接受 .elf 或 .hex 固件，不接受 .bin")
    if not image.is_file():
        raise FirmwareImageError(f"固件文件不存在：{image}")
    if image.stat().st_size <= 0:
        raise FirmwareImageError(f"固件文件为空：{image}")
    image = image.resolve()
    try:
        content = image.read_bytes()
    except OSError as exc:
        raise FirmwareImageError(f"无法读取固件文件：{image}: {exc}") from exc
    if image.suffix.lower() == ".elf":
        return _inspect_elf32(image, content)
    return _inspect_intel_hex(image, content)


class FirmwareBuildError(RomDfuError):
    """cmake 构建失败；此时绝不能继续烧录旧镜像。"""


# 参与"镜像是否比源码旧"判定的目录与文件后缀。
FIRMWARE_SOURCE_ROOTS = (
    "App", "Core", "Driver", "Services", "BSP", "USB_DEVICE", "cmake",
)
FIRMWARE_SOURCE_FILES = ("CMakeLists.txt", "CMakePresets.json")
FIRMWARE_SOURCE_SUFFIXES = (".c", ".h", ".s", ".S", ".cpp", ".hpp", ".ld", ".cmake", ".txt")


def newest_firmware_source(project_root: str | os.PathLike[str]) -> tuple[Path | None, float]:
    """返回最近修改的固件源文件及其 mtime（找不到时返回 (None, 0.0)）。"""

    root = Path(project_root)
    newest: Path | None = None
    newest_mtime = 0.0
    candidates: list[Path] = []
    for name in FIRMWARE_SOURCE_FILES:
        candidates.append(root / name)
    for directory in FIRMWARE_SOURCE_ROOTS:
        base = root / directory
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix in FIRMWARE_SOURCE_SUFFIXES:
                candidates.append(path)
    for path in candidates:
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if mtime > newest_mtime:
            newest, newest_mtime = path, mtime
    return newest, newest_mtime


def firmware_image_staleness(
    image: str | os.PathLike[str],
    project_root: str | os.PathLike[str],
) -> tuple[bool, str]:
    """镜像是否比源码旧。返回 (是否陈旧, 人类可读说明)。

    这是"防止烧录旧固件"的最后一道兜底：即使用户关掉了自动重编译，
    界面也必须明确告诉他手上这个 ELF 比源码老。
    """

    target = Path(image)
    try:
        image_mtime = target.stat().st_mtime
    except OSError:
        return True, "镜像不存在或不可读"
    newest, newest_mtime = newest_firmware_source(project_root)
    if newest is None:
        return False, "未找到可比对的源文件"
    if newest_mtime > image_mtime:
        delta = newest_mtime - image_mtime
        return True, (
            f"镜像比源码旧 {delta / 60.0:.1f} 分钟："
            f"{Path(newest).name} 更新于镜像之后"
        )
    return False, "镜像不早于任何源文件"


def build_firmware(
    project_root: str | os.PathLike[str],
    *,
    preset: str = "Debug",
    on_output: Callable[[str], None] | None = None,
    timeout_s: float = 900.0,
) -> Path:
    """执行 cmake --build --preset <preset>，返回构建产物 ELF 路径。

    构建失败必须抛异常而不是返回旧路径，否则会把上一次的旧固件刷进飞控。
    """

    root = Path(project_root).resolve()
    argv = ["cmake", "--build", "--preset", preset]
    if on_output is not None:
        on_output(f"$ {' '.join(argv)}  (cwd={root})")
    try:
        process = subprocess.run(
            argv,
            cwd=str(root),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding=locale.getpreferredencoding(False),
            errors="replace",
            timeout=timeout_s,
        )
    except FileNotFoundError as exc:
        raise FirmwareBuildError(
            "找不到 cmake，无法在烧录前重新编译；请把 CMake 加入 PATH，"
            "或关闭“烧录前重新编译”并自行确认镜像是最新的。"
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise FirmwareBuildError(f"编译超过 {timeout_s:.0f}s 未完成，已放弃") from exc
    output = process.stdout or ""
    if on_output is not None:
        for line in output.splitlines():
            on_output(line)
    if process.returncode != 0:
        tail = "\n".join(output.splitlines()[-20:])
        raise FirmwareBuildError(f"编译失败（exit={process.returncode}）：\n{tail}")
    elf = root / "build" / preset / "drone-H743.elf"
    if not elf.is_file():
        raise FirmwareBuildError(f"编译成功但未找到产物：{elf}")
    return elf


def freeze_firmware_image(
    source: FirmwareImageInfo,
    destination: str | os.PathLike[str],
) -> FirmwareImageInfo:
    """Freeze the exact validated bytes for one update attempt."""

    target = Path(destination)
    if target.suffix.lower() != source.path.suffix.lower():
        raise FirmwareImageError("冻结镜像必须保留原始 ELF/HEX 后缀")
    try:
        content = source.path.read_bytes()
    except OSError as exc:
        raise FirmwareImageError(f"无法冻结固件镜像：{source.path}: {exc}") from exc
    actual_sha = hashlib.sha256(content).hexdigest()
    if actual_sha != source.sha256:
        raise FirmwareImageError(
            f"源固件在确认后已变化：expected={source.sha256} actual={actual_sha}"
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        with target.open("xb") as stream:
            stream.write(content)
    except OSError as exc:
        raise FirmwareImageError(f"无法创建冻结固件镜像：{target}: {exc}") from exc
    frozen = validate_firmware_image(target)
    if frozen.sha256 != source.sha256:
        raise FirmwareImageError(
            f"冻结固件校验失败：expected={source.sha256} actual={frozen.sha256}"
        )
    return frozen


def parse_usb_dfu_ports(output: str) -> tuple[str, ...]:
    """Extract stable CubeProgrammer port identifiers such as USB1."""

    ports = {f"USB{int(match.group(1))}" for match in _USB_PORT_RE.finditer(output)}
    return tuple(sorted(ports, key=lambda value: int(value[3:])))


def list_usb_dfu_ports(
    cli: str | os.PathLike[str],
    *,
    timeout_s: float = 8.0,
    run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> tuple[str, ...]:
    """List STM32 ROM DFU devices through CubeProgrammer."""

    argv = [os.fspath(cli), "-l", "usb"]
    try:
        completed = run(
            argv,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_s,
            shell=False,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RomDfuError(f"无法列举 USB DFU：{exc}") from exc
    output = (completed.stdout or "") + "\n" + (completed.stderr or "")
    if completed.returncode != 0:
        detail = output.strip().splitlines()[-1] if output.strip() else "unknown error"
        raise RomDfuError(f"CubeProgrammer 列举 USB DFU 失败：{detail}")
    return parse_usb_dfu_ports(output)


def wait_for_usb_dfu(
    cli: str | os.PathLike[str],
    *,
    timeout_s: float = DEFAULT_ENUMERATION_TIMEOUT_S,
    poll_interval_s: float = 0.35,
    cancel_event: threading.Event | None = None,
    on_log: Callable[[str], None] | None = None,
    list_ports: Callable[[str | os.PathLike[str]], Sequence[str]] | None = None,
) -> str:
    """Wait for exactly one DFU interface and return its CubeProgrammer port."""

    cancel = cancel_event or threading.Event()
    enumerate_ports = list_ports or (lambda executable: list_usb_dfu_ports(executable))
    started = time.monotonic()
    next_notice = started
    last_error = ""
    while True:
        if cancel.is_set():
            raise DfuCancelledError("等待 DFU 时已取消")
        now = time.monotonic()
        if now - started >= timeout_s:
            suffix = f"；最后错误：{last_error}" if last_error else ""
            raise DfuEnumerationTimeoutError(
                f"{timeout_s:.0f} 秒内未发现 STM32 ROM USB DFU{suffix}"
            )
        try:
            ports = tuple(enumerate_ports(cli))
        except RomDfuError as exc:
            ports = ()
            last_error = str(exc)
        if len(ports) == 1:
            if on_log is not None:
                on_log(f"已发现 STM32 ROM DFU：{ports[0]}")
            return ports[0]
        if len(ports) > 1:
            raise RomDfuError(
                "发现多个 STM32 DFU 设备，无法安全选择：" + ", ".join(ports)
            )
        if on_log is not None and now >= next_notice:
            on_log("等待 Windows 枚举 STM32 BOOTLOADER…")
            next_notice = now + 2.0
        cancel.wait(max(0.05, poll_interval_s))


def build_flash_argv(
    cli: str | os.PathLike[str],
    image: str | os.PathLike[str],
    *,
    port: str = "USB1",
    expected_sha256: str | None = None,
) -> tuple[str, ...]:
    """Build one USB-DFU program/verify/start command.

    CubeProgrammer's ``-rst`` is a JTAG/SWD-only system reset. ROM USB DFU
    exits through the generic start/go command at the application vector.
    """

    normalized_port = port.upper()
    if not re.fullmatch(r"USB[1-9]\d*", normalized_port):
        raise RomDfuError(f"无效 DFU 端口：{port}")
    executable = resolve_cubeprogrammer_cli(cli)
    firmware = validate_firmware_image(image)
    if expected_sha256 is not None and firmware.sha256 != expected_sha256.lower():
        raise FirmwareImageError(
            f"待烧录镜像已变化：expected={expected_sha256.lower()} actual={firmware.sha256}"
        )
    return (
        os.fspath(executable),
        "-c",
        f"port={normalized_port}",
        "-w",
        os.fspath(firmware.path),
        "-v",
        "-s",
        "0x08000000",
    )


def progress_from_line(line: str) -> int | None:
    matches = [int(match.group(1)) for match in _PROGRESS_RE.finditer(line)]
    return max(matches) if matches else None


def _terminate_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        process.terminate()
        process.wait(timeout=2.0)
    except (OSError, subprocess.TimeoutExpired):
        try:
            process.kill()
            process.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            pass


def flash_firmware(
    cli: str | os.PathLike[str],
    image: str | os.PathLike[str],
    *,
    port: str = "USB1",
    expected_sha256: str | None = None,
    timeout_s: float = DEFAULT_FLASH_TIMEOUT_S,
    cancel_event: threading.Event | None = None,
    on_output: Callable[[str], None] | None = None,
    on_progress: Callable[[int], None] | None = None,
    popen: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
) -> FlashResult:
    """Program once, verify once, then ask ROM DFU to start the application."""

    cancel = cancel_event or threading.Event()
    if cancel.is_set():
        raise DfuCancelledError("烧录开始前已取消")
    if timeout_s <= 0.0:
        raise RomDfuError("烧录超时必须大于 0 秒")
    # This is deliberately the last filesystem read before spawning the
    # programmer, binding Popen to the exact bytes confirmed by the user.
    argv = build_flash_argv(
        cli,
        image,
        port=port,
        expected_sha256=expected_sha256,
    )
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        process = popen(
            list(argv),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding=locale.getpreferredencoding(False),
            errors="replace",
            bufsize=1,
            shell=False,
            creationflags=creationflags,
        )
    except OSError as exc:
        raise RomDfuError(f"无法启动 STM32CubeProgrammer：{exc}") from exc

    line_queue: "queue.Queue[str | None]" = queue.Queue()

    def read_output() -> None:
        stream = process.stdout
        if stream is not None:
            for raw_line in stream:
                line_queue.put(raw_line.rstrip("\r\n"))
        line_queue.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    started = time.monotonic()
    output: list[str] = []
    cancelled = False
    timed_out = False
    output_closed = False

    while True:
        try:
            item = line_queue.get(timeout=0.05)
            if item is None:
                output_closed = True
            else:
                output.append(item)
                if on_output is not None:
                    on_output(item)
                progress = progress_from_line(item)
                if progress is not None and on_progress is not None:
                    on_progress(progress)
        except queue.Empty:
            pass

        if cancel.is_set() and process.poll() is None:
            cancelled = True
            _terminate_process(process)
        if (time.monotonic() - started) >= timeout_s and process.poll() is None:
            timed_out = True
            _terminate_process(process)

        returncode = process.poll()
        if returncode is not None and output_closed and line_queue.empty():
            break

    reader.join(timeout=0.5)
    return FlashResult(
        argv=argv,
        returncode=int(process.returncode if process.returncode is not None else -1),
        output=tuple(output),
        duration_s=time.monotonic() - started,
        cancelled=cancelled,
        timed_out=timed_out,
    )

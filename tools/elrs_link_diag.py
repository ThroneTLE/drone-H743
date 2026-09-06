"""ELRS / CRSF 收链路分层计数 —— 用 SWD 直读，不占串口、不复位飞控。

这些计数**不进任何命令回包**（作者裁决 2026-09-06）：能挂的那几个报告函数都被
`tests/test_control_loop_blocking_contract.py` 的 SHA256 钉住，那些哈希证明的是
"当年从 app_control.c 逐字节搬过来、一个字符没动"，不值得为一组排查用的计数
去破坏。所以访问路径就是**符号名 + 内存直读**，本脚本把它包起来。

读法是 `mode=HOTPLUG`：连上正在跑的目标读内存，**不复位、不停机**，因此可以在
飞控通电工作时随时采样。本脚本对目标一个字节都不写，所以永远不该出现会复位
目标的连接模式——`tests/test_crsf_parser_resync.py` 有机检把关。

为什么要分层：`crc_err` 只说"帧坏了"，说不出坏在哪一层，而两层的处置相反——

    ORE            固件没及时把 DMA 里的字节取走。软件问题，调采样节奏或缓冲。
    FE / NE        线上电平本身不对。波特率、走线、地线，改代码没用。
    abort          因上面任一标志整条重启 RX DMA 的次数。一次重启丢掉整个未消费
                   缓冲并让解析器失步，所以它本身就是 CRC 错的放大器。
    sfail          StartRxDma() 起不来的次数。**非零且持续增长 = 收链路已经死了**，
                   和"帧偶尔坏"是完全不同的故障。

用法：

    python tools/elrs_link_diag.py                  # 采两次，间隔 5 s，打速率
    python tools/elrs_link_diag.py --samples 5 --gap 10
    python tools/elrs_link_diag.py --elf build/Debug/drone-H743.elf
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

DEFAULT_ELF = ROOT / "build" / "Debug" / "drone-H743.elf"
CLT = Path("E:/ST/STM32CubeCLT_1.18.0")
PROGRAMMER = CLT / "STM32CubeProgrammer" / "bin" / "STM32_Programmer_CLI.exe"
NM = CLT / "GNU-tools-for-STM32" / "bin" / "arm-none-eabi-nm.exe"

# 符号名即接口。顺序 = 打印顺序；改名要同时改 App/Src/app_elrs.c 与本表，
# tests/test_crsf_parser_resync.py 会挡住只改一边。
COUNTERS = [
    ("rc",       "g_rc_frames",     "解出来的 RC 帧"),
    ("total",    "g_total_frames",  "CRC 通过的帧（含链路统计）"),
    ("crc_err",  "g_crc_errors",    "CRC 不符"),
    ("len_err",  "g_length_errors", "长度字节非法 = 解析器正在失步重同步"),
    ("ore",      "rx_err_overrun",  "溢出：固件取字节不及时（软件）"),
    ("fe",       "rx_err_framing",  "帧错：线上电平不对（硬件）"),
    ("ne",       "rx_err_noise",    "噪声：线上电平不对（硬件）"),
    ("pe",       "rx_err_parity",   "校验错"),
    ("abort",    "rx_aborts",       "整条重启 RX DMA（CRC 错的放大器）"),
    ("rst",      "rx_restarts",     "StartRxDma 成功次数"),
    ("evt",      "rx_events",       "HAL RxEvent 回调"),
    ("sfail",    "rx_start_fail",   "StartRxDma 起不来（非零且增长 = 链路已死）"),
]


def resolve_symbols(elf: Path) -> dict[str, int]:
    """从 ELF 里取每个计数的地址。静态变量也在符号表里，不需要改可见性。"""
    if not NM.is_file():
        sys.exit(f"找不到 arm-none-eabi-nm：{NM}")
    if not elf.is_file():
        sys.exit(f"找不到 ELF：{elf}（先 cmake --build --preset Debug）")

    output = subprocess.run([str(NM), str(elf)], check=True,
                            capture_output=True, text=True).stdout
    table: dict[str, int] = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)

    missing = [sym for _, sym, _ in COUNTERS if sym not in table]
    if missing:
        sys.exit("ELF 里没有这些符号（改名了？被优化掉了？）：" + ", ".join(missing))
    return {sym: table[sym] for _, sym, _ in COUNTERS}


def clusters(addresses: list[int], stride: int = 64) -> list[tuple[int, int]]:
    """把地址并成几段连读。

    解析器计数在 Driver 的 BSS 里、UART 计数在 App 的 BSS 里，两簇隔着十几 KB。
    从最低读到最高要搬两万字节，一次采样就得几百毫秒——采样窗口一宽，算出来的
    "每秒速率"就不是那一刻的了。分簇读把每次采样压到两小段。
    """
    ordered = sorted(set(addresses))
    spans: list[tuple[int, int]] = []
    start = previous = ordered[0]
    for address in ordered[1:]:
        if address - previous > stride:
            spans.append((start, previous - start + 4))
            start = address
        previous = address
    spans.append((start, previous - start + 4))
    return spans


def read_words(addresses: list[int]) -> dict[int, int]:
    words: dict[int, int] = {}
    for base, size in clusters(addresses):
        result = subprocess.run(
            [str(PROGRAMMER), "-q", "-c", "port=SWD", "mode=HOTPLUG",
             "-r32", hex(base), hex((size + 3) & ~3)],
            check=True, capture_output=True, text=True,
        )
        for line in result.stdout.splitlines():
            match = re.match(r"^(0x[0-9A-Fa-f]+)\s*:\s*(.+)$", line.strip())
            if not match:
                continue
            row = int(match.group(1), 16)
            for index, token in enumerate(match.group(2).split()):
                words[row + 4 * index] = int(token, 16)
    return words


def sample(symbols: dict[str, int]) -> dict[str, int]:
    words = read_words(list(symbols.values()))
    missing = [sym for sym, addr in symbols.items() if addr not in words]
    if missing:
        sys.exit("读回来的内存没覆盖到：" + ", ".join(missing))
    return {sym: words[addr] for sym, addr in symbols.items()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--samples", type=int, default=2)
    parser.add_argument("--gap", type=float, default=5.0)
    args = parser.parse_args()

    if not PROGRAMMER.is_file():
        sys.exit(f"找不到 STM32_Programmer_CLI：{PROGRAMMER}")
    if shutil.which("gcc") is None:
        pass  # 不需要 gcc，只是提醒读者本脚本不编译任何东西

    symbols = resolve_symbols(args.elf)
    print(f"ELF: {args.elf}")
    print("符号: " + "  ".join(f"{sym}@0x{addr:08X}" for sym, addr in symbols.items()))
    print()

    taken: list[tuple[float, dict[str, int]]] = []
    for index in range(max(args.samples, 2)):
        stamp = time.monotonic()
        taken.append((stamp, sample(symbols)))
        values = taken[-1][1]
        print(f"[{index}] " + "  ".join(
            f"{label}={values[sym]}" for label, sym, _ in COUNTERS))
        if index + 1 < max(args.samples, 2):
            time.sleep(args.gap)

    print("\n=== 每秒速率 ===")
    for (t0, before), (t1, after) in zip(taken, taken[1:]):
        span = t1 - t0
        rates = {label: (after[sym] - before[sym]) / span
                 for label, sym, _ in COUNTERS}
        print("  " + "  ".join(f"{label}={rate:8.1f}/s"
                               for label, rate in rates.items()))
        good, bad = rates["rc"], rates["crc_err"]
        if good + bad > 0.0:
            print(f"      真帧 {good:.0f}/s    错帧率 {100.0 * bad / (good + bad):.1f}%")
        if rates["sfail"] > 0.0:
            print("      !! sfail 在涨：StartRxDma 起不来，收链路处于失效状态")
        if rates["ore"] > 1.0:
            print("      !! ORE 在涨：固件取字节不及时，是软件侧问题")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

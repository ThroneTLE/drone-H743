"""CRSF 解析器的失步与重同步（`Driver/Src/drv_elrs.c`，宿主 gcc 直接编真实源码）。

为什么专门立一份：2026-09-06 实机取证发现遥控链路 **70% 错帧率**，而
SWD 直读 UART4 的 DMA 缓冲显示**线上字节是干净的**——连续 26 字节一帧、帧距恒为
26、抽查 8/8 帧 CRC 全过（那唯一一处 4 字节缺口是环形缓冲的接缝：256 % 26 == 22）。
射频侧也好：`lq=100`、`rssi≈20`。也就是说错帧全是飞控收侧自己造出来的。

原因是 CRSF 没有转义也没有帧定界符，全靠"地址字节"起头，而
`Crsf_IsCommonAddress()` 会把 67/256（26%）的字节值当成地址。所以**一旦失步，
解析器就在 payload 里逐字节撞运气**，一次失步能连打十几个假帧，期间真帧全被吃掉。

于是"什么时候必须主动丢掉手上的半帧"就成了这条链路的核心正确性问题，
本文件钉的就是它。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_elrs.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    char     token[32];
    unsigned rc_updates = 0U;
    uint16_t raw[CRSF_CHANNEL_COUNT];
    uint16_t us[CRSF_CHANNEL_COUNT];

    DRV_ELRS_Init();

    while (scanf("%31s", token) == 1) {
        unsigned value = 0U;

        if (strcmp(token, "RESET") == 0) {
            DRV_ELRS_ResetParser();
            continue;
        }
        if (sscanf(token, "%x", &value) != 1) {
            return 2;
        }
        rc_updates += (unsigned)DRV_ELRS_ProcessByte((uint8_t)value);
    }

    DRV_ELRS_GetChannels(raw, us);
    printf("rc_updates=%u rc=%lu total=%lu crc_err=%lu len_err=%lu ch0=%u ch1=%u\n",
           rc_updates,
           (unsigned long)DRV_ELRS_GetRcFrames(),
           (unsigned long)DRV_ELRS_GetTotalFrames(),
           (unsigned long)DRV_ELRS_GetCrcErrors(),
           (unsigned long)DRV_ELRS_GetLengthErrors(),
           (unsigned)raw[0], (unsigned)raw[1]);
    return 0;
}
"""


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


# ------------------------------------------------------------------ CRSF 造帧


def crc8(data: bytes) -> int:
    """CRSF 的 CRC-8/DVB-S2（多项式 0xD5），与 DRV_ELRS_Crc8 同源。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_channels(values: list[int]) -> bytes:
    """16 路 × 11 bit 小端紧排，即 RC_CHANNELS_PACKED 的 22 字节 payload。"""
    assert len(values) == 16
    bits = 0
    for index, value in enumerate(values):
        bits |= (value & 0x7FF) << (11 * index)
    return bits.to_bytes(22, "little")


def rc_frame(values: list[int] | None = None) -> bytes:
    payload = pack_channels(values if values is not None else [992] * 16)
    body = bytes([0x16]) + payload            # type + payload
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)])


CLEAN = rc_frame()
assert len(CLEAN) == 26, "CRSF RC 帧就是 26 字节，测试假设写死在这里"


# ------------------------------------------------------------------ 宿主编译


@pytest.fixture(scope="module")
def parser(tmp_path_factory):
    """把真实的 drv_elrs.c 编成一个吃十六进制字节流的小程序。"""
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    work = tmp_path_factory.mktemp("crsf")
    (work / "harness.c").write_text(HARNESS, encoding="ascii")
    executable = work / "crsf.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Driver" / "Src" / "drv_elrs.c"),
            str(work / "harness.c"),
            "-o", str(executable),
        ],
        check=True, capture_output=True,
    )

    def run(stream) -> dict[str, int]:
        tokens = []
        for item in stream:
            if item == "RESET":
                tokens.append("RESET")
            else:
                tokens.extend(f"{byte:02X}" for byte in item)
        result = subprocess.run(
            [str(executable)], input=" ".join(tokens).encode("ascii"),
            check=True, capture_output=True,
        )
        return {
            key: int(value)
            for key, value in (
                pair.split("=") for pair in result.stdout.decode().split()
            )
        }

    return run


# ------------------------------------------------------------------ 用例


def test_a_clean_stream_decodes_every_frame(parser) -> None:
    stats = parser([CLEAN * 20])

    assert stats["rc"] == 20
    assert stats["total"] == 20
    assert stats["crc_err"] == 0
    assert stats["len_err"] == 0


def test_channel_values_survive_the_packing(parser) -> None:
    stats = parser([rc_frame([172, 1811] + [992] * 14)])

    assert stats["rc"] == 1
    assert stats["ch0"] == 172
    assert stats["ch1"] == 1811


def test_one_corrupt_payload_byte_costs_exactly_one_frame(parser) -> None:
    """坏一个 payload 字节只该丢一帧。

    帧长没被破坏，解析器消费的字节数就还是对的，下一个字节仍是真正的帧头——
    这是"失步代价有界"的基线。
    """
    broken = bytearray(CLEAN)
    broken[10] ^= 0xFF
    stats = parser([CLEAN * 3 + bytes(broken) + CLEAN * 3])

    assert stats["crc_err"] == 1
    assert stats["rc"] == 6, "坏帧前后的 6 帧都该正常解出来"


def test_a_truncated_frame_poisons_the_next_one_unless_the_parser_is_reset(parser) -> None:
    """DMA 重启会在字节流上留下断点，手上的半帧必须丢掉。

    这是 `APP_ELRS_Step()` 里 `DRV_ELRS_ResetParser()` 那两处调用的判据：
    半帧后面接上新流的字节，一定拼出一个长度合法、CRC 必错的假帧，而且假帧吃掉的
    字节数是错的，**后面的真帧会跟着一起错位**。
    """
    truncated = CLEAN[:10]          # DMA 重启时被丢掉后半截的那一帧

    poisoned = parser([truncated + CLEAN * 4])
    assert poisoned["crc_err"] >= 1, "不重置就必然多出坏帧"
    assert poisoned["rc"] < 4, "而且后面的真帧会被连累"

    healed = parser([truncated, "RESET", CLEAN * 4])
    assert healed["crc_err"] == 0
    assert healed["rc"] == 4, "重置之后下一个字节就是真正的帧头，一帧都不该丢"


def test_resync_after_a_desync_costs_at_most_one_frame(parser) -> None:
    """从任意位置失步之后，重新锁上的代价必须有界。

    这是 2026-09-06 那个 30% 错帧率的**真正放大器**：CRSF 全靠地址字节起头，
    原来的判据（`0x00/0x10/0x80/>=0xC0`）有 67/256 = 26% 的字节值会被当成地址，
    于是解析器在 payload 里逐字节撞运气，一次失步平均连打七八个假帧。实测
    abort≈30/s 就打出 crc_err≈215/s，而真坏字节只有 fe+ne≈35/s。

    这里穷举所有可能的失步相位：把干净流的前 k 个字节吃掉（k = 1..25），每个相位
    都要求最多丢一帧就重新锁上。
    """
    worst = 0
    for skip in range(1, len(CLEAN)):
        stats = parser([CLEAN[skip:] + CLEAN * 6])
        assert stats["rc"] >= 6, f"相位 {skip}：后面 6 帧应当全解出来，实际 {stats['rc']}"
        worst = max(worst, stats["crc_err"] + stats["len_err"])
    assert worst <= 1, f"最坏相位下产生了 {worst} 个假帧/长度错，重同步代价失控"


def test_only_named_crsf_addresses_start_a_frame(parser) -> None:
    """地址判据的宽度就是失步代价，必须是具名地址而不是范围。

    实测（SWD 直读 DMA 缓冲、全缓冲扫描 + CRC 校验）50/50 帧地址都是 0xC8。
    """
    source = read("Driver/Src/drv_elrs.c")
    body = source[source.index("static uint8_t Crsf_IsCommonAddress"):
                  source.index("static uint16_t Crsf_ReadPackedChannel")]

    assert "CRSF_ADDRESS_FLIGHT_CONTROLLER" in body
    assert "CRSF_ADDRESS_BROADCAST" in body
    assert ">= 0xC0" not in body and ">=0xC0" not in body, (
        "范围判断会把 64 个字节值当成地址，失步后就是逐字节撞运气"
    )

    # 未被接受的地址不许起头：拿一整帧改掉地址字节，应当一帧都解不出来。
    for address in (0x10, 0x80, 0xC0, 0xEA, 0xEE):
        mutated = bytearray(CLEAN)
        mutated[0] = address
        stats = parser([bytes(mutated)])
        assert stats["rc"] == 0, f"0x{address:02X} 不该被当成帧头"

    for address in (0xC8, 0x00):
        mutated = bytearray(CLEAN)
        mutated[0] = address
        stats = parser([bytes(mutated)])
        assert stats["rc"] == 1, f"0x{address:02X} 是本链路的合法目的地址"


def test_reset_keeps_counters_and_channel_values(parser) -> None:
    """重置只丢半帧，不许把已经解出来的东西也一并清掉。

    计数是诊断依据，通道值是飞行数据——重置若把它们抹掉，一次 DMA 重启就等于
    遥控输入瞬间跳回中位。
    """
    stats = parser([rc_frame([172] + [992] * 15), "RESET", "RESET"])

    assert stats["rc"] == 1
    assert stats["ch0"] == 172


def test_step_resets_the_parser_at_both_stream_discontinuities() -> None:
    """两处断点都必须重置：DMA 重启，以及帧间空隙。

    源码级机检——这两处是运行时行为，宿主编不出 HAL/DMA 来。
    """
    source = read("App/Src/app_elrs.c")

    start_dma = source[source.index("static void StartRxDma"):
                       source.index("static uint8_t DmaNeedsRestart")]
    assert "DRV_ELRS_ResetParser();" in start_dma, (
        "DMA 重启后字节流断了一截，解析器手上的半帧必须丢掉"
    )

    step = source[source.index("void APP_ELRS_Step"):
                  source.index("void APP_ELRS_GetChannels")]
    assert "if (consumed == 0U) {" in step
    assert "DRV_ELRS_ResetParser();" in step.split("if (consumed == 0U) {")[1], (
        "一拍一个新字节都没来 = 落在帧间空隙里，此时的半帧永远等不到剩下的字节"
    )


def test_start_rx_dma_can_recover_on_its_own() -> None:
    """`StartRxDma()` 起不来时必须自己把 HAL 状态机收尾。

    实测每次上电都有 `sfail` 1~5 次。HAL 的 RxState 不在 READY 时重试永远是
    HAL_BUSY，没有这一句就是"开机头几拍没起来 = 遥控链路整条死掉"，而且
    再也不会自己恢复（2026-09-06 实机复现过一次）。
    """
    source = read("App/Src/app_elrs.c")
    body = source[source.index("static void StartRxDma"):
                  source.index("static uint8_t DmaNeedsRestart")]
    failure = body[body.index("if (status != HAL_OK) {"):body.index("__HAL_DMA_DISABLE_IT")]

    # 句柄名从源码里推出来，不写死：ELRS 2026-09-10 从 UART4 搬到了板载 RC 口
    # USART6，而本条守的是"起不来时必须 abort"这个性质，跟它挂在哪个串口无关。
    handle = re.search(r"HAL_UARTEx_ReceiveToIdle_DMA\(&(huart\d+),", body)
    assert handle is not None, "找不到 ELRS 的 DMA 接收启动调用"

    assert f"HAL_UART_AbortReceive(&{handle.group(1)});" in failure
    assert "rx_start_fail++;" in failure, "起不来的次数要看得见，否则查不出是这里"


def test_rx_diagnostics_separate_the_physical_layer_from_the_parser() -> None:
    """诊断必须能分层，否则"错帧率高"永远定位不到是哪一层。

    ORE 是固件取字节不及时（软件），FE/NE 是线上电平不对（波特率/走线，改代码
    没用）。两者处置相反，混在一个 `crc_err` 里就只能靠猜。
    """
    header = read("App/Inc/app_elrs.h")
    source = read("App/Src/app_elrs.c")

    for field in ("overrun", "framing", "noise", "parity",
                  "aborts", "restarts", "events", "start_fail"):
        assert field in header, field

    clear_errors = source[source.index("static void ClearErrors"):
                          source.index("static void StartRxDma")]
    for flag, counter in (("USART_ISR_ORE", "rx_err_overrun"),
                          ("USART_ISR_FE", "rx_err_framing"),
                          ("USART_ISR_NE", "rx_err_noise"),
                          ("USART_ISR_PE", "rx_err_parity")):
        assert flag in clear_errors and counter in clear_errors, flag
    assert clear_errors.index("rx_err_overrun++") < clear_errors.index("__HAL_UART_CLEAR_FLAG"), (
        "必须先分类再清标志，清完就分不出是哪一层坏的"
    )


def test_the_counter_names_are_the_swd_read_interface() -> None:
    """这些计数不进命令回包，符号名就是接口。

    作者裁决 2026-09-06：唯一能挂的几个报告函数（`app_control_report_rc_live`、
    `app_control_report_uart_stats` 等）都被 D2/D4 的 SHA256 钉死，那些哈希证明的
    是"当年从 app_control.c 逐字节搬过来、一个字符没动"，不值得为一组排查用的
    计数去破坏。所以访问路径是 `nm` 找地址 + SWD 读内存。

    于是改名就是改接口：固件侧和 `tools/elrs_link_diag.py` 必须同时改，
    这条机检挡的就是只改一边。
    """
    firmware = read("App/Src/app_elrs.c") + read("Driver/Src/drv_elrs.c")
    tool = read("tools/elrs_link_diag.py")

    names = re.findall(r'\(\s*"[a-z_]+",\s*"(\w+)",', tool)
    assert len(names) == 12, f"工具里应当有 12 个计数，实际 {len(names)}：{names}"

    for name in names:
        assert re.search(rf"^static\s+(?:volatile\s+)?uint32_t\s+{name}\s*;",
                         firmware, re.MULTILINE), (
            f"{name} 在固件里不是文件级 uint32_t 静态变量，SWD 直读的地址会对不上"
        )

    # 读内存必须是 HOTPLUG：飞控可能正在通电工作，不许顺手复位。
    assert "mode=HOTPLUG" in tool
    assert "mode=UR" not in tool, "诊断脚本一个字节都不写目标，不该用会复位的模式"

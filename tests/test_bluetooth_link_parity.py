"""板载蓝牙（UART8）必须和 USB、数传是同一条链路，不是半条。

MicoAir743V2 的 UART8（PE1/PE0，115200）接的是板载蓝牙模块。命令面本来就是同一套
——维护口收到的行直接交给 `APP_Control_ProcessLine`，和 USB 走的是同一个解析器。
缺的一直是另外两半：

  1. **波形出不去**。遥测流只有 `uart`（数传）和 `usb` 两个出口，蓝牙连上以后
     只能敲命令、看不了曲线，"和 USB 一样"就是句空话。
  2. **异步输出收不到**。结构化文本无条件镜像到 USB、走队列到数传，而蓝牙只在
     "正在处理一条蓝牙命令"期间才有输出——于是蓝牙上永远看不到 `READY`，
     也看不到任何没人问也该来的回报。

本文件钉住这两半补齐了，以及补齐的方式没有引入新的坑：二进制帧不能走按字符串
处理的路径（遥测帧里含 0x00），空闲时不能每条文本都去阻塞一个没人听的串口。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_bluetooth_is_a_telemetry_sink_like_usb_and_the_radio() -> None:
    header = read("App/Inc/app_telem_stream.h")
    policy = read("App/Src/app_telem_stream.c")
    command = read("App/Src/app_cmd_telem.c")

    assert "APP_TELEM_SINK_BT" in header
    # 出口三选一必须在同一个 switch/分支里各有落点，不能有哪一个掉进 else。
    assert "APP_TelemStream_PortSendBt" in header
    assert "sink == APP_TELEM_SINK_BT" in policy
    assert 'return "bt";' in policy

    # 命令面与用法串同步：用法串里没有 bt 的话，这条出口等于没人知道。
    assert 'strcmp(tokens[2], "bt") == 0' in command
    assert command.count("TELEM SINK usb|uart|bt|auto") >= 3
    assert "TELEM SINK usb|uart|auto" not in command


def test_a_command_from_bluetooth_selects_bluetooth_for_auto_sink() -> None:
    """`SINK auto` 必须认得蓝牙，否则在蓝牙上开流会把波形发去数传。

    那是最难查的一种：命令回了 OK、流也确实开着，只是数据去了另一条链路，
    而那条链路上没人接。
    """
    maint = read("App/Src/app_maint_uart.c")
    policy = read("App/Src/app_telem_stream.c")

    assert "APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_BT)" in maint
    note = policy.split("void APP_TelemStream_NoteCommandSource", 1)[1]
    note = note.split("\n}", 1)[0]
    assert "APP_TELEM_SINK_BT" in note


def test_binary_frames_do_not_go_through_a_string_path() -> None:
    """遥测帧里含 0x00，按 C 字符串处理会被截断。

    `APP_MaintUART_Write` 的形参是 `const char *`，强转着用能编过、也能发出去
    大部分字节，坏在哪一帧取决于数据内容——所以另开一个二进制入口，让"用错了"
    在编译期就是类型错误，而不是在某次波形上出现莫名其妙的截断。
    """
    header = read("App/Inc/app_maint_uart.h")
    source = read("App/Src/app_maint_uart.c")
    port = read("App/Src/app_telem_port.c")

    assert "uint8_t APP_MaintUART_WriteRaw(const uint8_t *data, uint16_t length);" in header
    # 二进制入口要走 BSP 的字节级写，不能退回任何 const char* 的文本路径。
    assert "BSP_UART_MaintWrite(data, length)" in source

    send_bt = port.split("uint8_t APP_TelemStream_PortSendBt", 1)[1].split("\n}", 1)[0]
    assert "APP_MaintUART_WriteRaw" in send_bt
    assert "APP_MaintUART_Write(" not in send_bt
    # 发不出去要如实回 0，不能无条件 return 1：上层的 drop= 全靠这个返回值。
    assert "return APP_MaintUART_WriteRaw(frame, length);" in send_bt


def test_async_text_reaches_bluetooth_only_while_that_link_is_in_use() -> None:
    """异步文本要镜像到蓝牙，但只在蓝牙确实在用的时候。

    发送改成 DMA 队列之后代价小了（不再每条文本阻塞一次 UART 任务），但没有消失：
    没连蓝牙时队列会被一条没人收的链路慢慢填满，随后连命令回包都要排队等位。
    """
    core = read("App/Src/app_control_core.c")
    maint = read("App/Src/app_maint_uart.c")

    mirror = core.split("void app_control_queue_proto_text", 1)[1].split("\n}", 1)[0]
    assert "APP_MaintUART_IsLinkActive()" in mirror
    assert "APP_MaintUART_Write(tx_message.text, tx_message.length);" in mirror

    # 判据必须是"最近收到过命令"，不是某个开机就置位的标志。
    assert "maint_last_command_ms = SVC_Timestamp_Ms();" in maint
    idle = re.search(r"#define APP_MAINT_UART_LINK_IDLE_MS\s+(\d+)U", maint)
    assert idle is not None
    # 比一问一答宽得多（不抖），比一次会话短得多（断开后不再空发）。
    assert 5000 <= int(idle.group(1)) <= 120000


def test_bluetooth_shares_the_radio_frame_budget_not_the_usb_one() -> None:
    """蓝牙 115200 与数传同量级，不能套用 USB 那条放宽的上限。

    给它 USB 的大帧只会在 40 Hz 下丢帧，而不是传得更多——而丢帧表现成波形
    断断续续，很容易被当成蓝牙模块不稳定去查。
    """
    port = read("App/Src/app_telem_port.c")
    body = port.split("uint16_t APP_TelemStream_PortMaxPayload", 1)[1].split("\n}", 1)[0]

    assert "sink == APP_TELEM_SINK_USB" in body
    assert "APP_TELEM_FRAME_MAX_PAYLOAD" in body
    assert "APP_UART_TX_TEXT_SIZE - APP_TELEM_FRAME_OVERHEAD" in body
    # 没有为 BT 单开一条更宽的分支。
    assert "APP_TELEM_SINK_BT" not in body


def test_the_board_doc_records_which_uart_the_bluetooth_module_is_on() -> None:
    """接错串口是这类改动最容易出的错，出处必须写在板级文档里。"""
    doc = read("doc/micoair743v2/README.md")
    assert "| UART8 | PE1 | PE0 | 蓝牙模块 |" in doc


def test_the_board_doc_identifies_the_module_by_mac_not_by_com_port() -> None:
    """COM 口号会变，MAC 不会。

    实测这块板的蓝牙是经典 SPP（不是 BLE），设备名 `MicoAir743v2-99806`、
    MAC `1CBE4DC4A41A`。文档里只写"在 COM35"的话，换台电脑或重新配对就作废，
    而下一个人会以为是固件坏了——所以必须同时给出按 MAC 反查串口的办法。
    """
    doc = read("doc/micoair743v2/README.md")

    assert "MicoAir743v2-99806" in doc
    assert "1CBE4DC4A41A" in doc
    # SPP 的 UUID：认成 BLE 会一直找不到串口。
    assert "00001101-0000-1000-8000-00805F9B34FB" in doc
    # 按 MAC 反查串口的命令，而不是写死一个口号。
    assert "PNPDeviceID -like '*1CBE4DC4A41A*'" in doc

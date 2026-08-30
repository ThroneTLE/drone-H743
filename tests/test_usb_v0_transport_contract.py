"""USB V0."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "App" / "Src" / "app_control.c"
USB_APP = ROOT / "App" / "Src" / "app_usb_cdc.c"
USB_HEADER = ROOT / "App" / "Inc" / "app_usb_cdc.h"
USB_IF = ROOT / "USB_DEVICE" / "App" / "usbd_cdc_if.c"
IOC = ROOT / "drone-H743.ioc"
PANEL = ROOT / "tools" / "drone_tcp_panel.py"
PANEL_TRANSPORT = ROOT / "tools" / "panel_lib" / "transport.py"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def c_function_body(source: str, signature: str) -> str:
    start = source.rindex(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_usb_cdc_is_cubemx_configured_and_routes_received_lines_to_control() -> None:
    ioc = read(IOC)
    interface = read(USB_IF)
    app = read(USB_APP)

    assert "USB_DEVICE.CLASS_NAME_FS=CDC" in ioc
    assert "USB_OTG_FS.VirtualMode=Device_Only" in ioc
    assert "APP_USB_CDC_OnReceive(Buf" in interface
    receive = c_function_body(app, "void APP_USB_CDC_OnReceive(")
    task = c_function_body(app, "void APP_USB_CDC_Task_Step(void)")
    process = c_function_body(app, "static void app_usb_cdc_process_line(void)")
    assert "app_usb_cdc_rx_ring[head] = data[index];" in receive
    assert "app_usb_cdc_process_line();" in task
    assert "APP_Control_ProcessLine(app_usb_cdc_line);" in process


def test_structured_v0_replies_are_mirrored_to_usb_and_uart() -> None:
    source = read(CONTROL)
    body = c_function_body(
        source,
        "static void app_control_queue_proto_text(uint16_t function, const char *format, ...)",
    )

    gate_at = body.index("APP_IMU_Capture_IsExportActive() == 0U")
    usb_at = body.index("APP_USB_CDC_Write")
    uart_at = body.index("osMessageQueuePut")
    assert gate_at < usb_at < uart_at
    assert "(void)APP_USB_CDC_Write" in body
    assert "APP_UART_NotifyTxPending();" in body
    assert "return" not in body[usb_at:uart_at]


def test_imucap_binary_export_owns_usb_stream_while_active() -> None:
    control = read(CONTROL)
    assert (
        "IMUCAP export\n     * owns the CDC byte stream while active" in control
    )
    proto_body = c_function_body(
        control,
        "static void app_control_queue_proto_text(uint16_t function, const char *format, ...)",
    )
    assert "if (APP_IMU_Capture_IsExportActive() == 0U)" in proto_body


def test_usb_text_buffers_cover_protocol_lines_and_panel_can_select_serial() -> None:
    usb_header = read(USB_HEADER)
    panel = read(PANEL)
    transport = read(PANEL_TRANSPORT)

    assert "#define APP_USB_CDC_TX_SIZE 1536U" in usb_header
    assert 'values=("tcp", "udp", "serial")' in panel
    assert "class SerialTransport" in transport
    assert "SerialTransport = _panel_transport.SerialTransport" in panel
    assert 'self._send_proto_silent(PROTO_REQ_IMU, "IMU?")' in panel
    assert "source=stabilizer_snapshot" in read(CONTROL)

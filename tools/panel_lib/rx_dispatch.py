"""Queue-to-Tk boundary: reject foreign sessions before any line handler runs."""

import queue

from .connection_state import ReceivedMessage, snapshot_is_current


def _text_of(item):
    """队列里同一行文本有两种壳：裸 `str` 和 `("proto", function, text)`。

    电源页必须两种都认。只认裸 `str` 的话，结构化协议链路上的 `CURRENT ...`
    会安静地永远到不了页面——不报错、不崩，只是诊断区永远空着。
    """
    if isinstance(item, str):
        return item
    if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto":
        text = item[2]
        return text if isinstance(text, str) else None
    return None


def drain_rx(panel, batch_size, busy_ms, idle_ms):
    processed = 0
    try:
        while processed < batch_size:
            item = panel.rx_queue.get_nowait()
            processed += 1
            context = None
            if isinstance(item, ReceivedMessage):
                context = item.context
                if not context.is_current(panel.transport):
                    panel.rx_stale_discarded = getattr(panel, "rx_stale_discarded", 0) + 1
                    if item.diagnostic:
                        panel._append(str(item.payload))
                    continue
                item = item.payload
            panel._rx_context = context
            try:
                if isinstance(item, tuple) and len(item) == 2 and item[0] == "sysid_bin":
                    sysid = getattr(panel, "sysid_page", None)
                    if sysid is not None:
                        sysid.accept(item[1], context)
                    continue
                if isinstance(item, tuple) and len(item) == 2 and item[0] == "battery_bin":
                    power = getattr(panel, "power_page", None)
                    if power is not None:
                        power.accept(item[1], context)
                    continue
                overview = getattr(panel, "overview_page", None)
                if isinstance(item, tuple) and len(item) == 2 and item[0] == "component_bin":
                    if overview is not None:
                        overview.accept(item[1], context)
                    continue
                if overview is not None and isinstance(item, str):
                    overview.handle_line(item)
                power = getattr(panel, "power_page", None)
                if power is not None:
                    text = _text_of(item)
                    if text is not None:
                        power.handle_line(text)
                if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto":
                    _tag, function, text = item
                    panel._handle_proto_frame(int(function), str(text))
                elif isinstance(item, tuple) and len(item) == 4 and item[0] == "udp_raw":
                    _tag, host, port, size = item
                    panel.udp_raw_hidden_count += 1
                    panel.udp_raw_last_note = f"hidden UDP binary frames={panel.udp_raw_hidden_count} last={host}:{port} {size}B"
                    panel.ident_link_var.set(panel.udp_raw_last_note)
                else:
                    line = str(item)
                    panel._append(line)
                    if context is not None and line.startswith("[上位机] 串口已连接"):
                        panel._save_panel_state()
                        panel._refresh_serial_selection_lock()
                    panel._handle_board_line(line)
            finally:
                panel._rx_context = None
    except queue.Empty:
        pass
    delay_ms = busy_ms if not panel.rx_queue.empty() else idle_ms
    panel.after(delay_ms, panel._drain_rx)

# Ai-WB2-12F PC Tools

This folder contains PC-side helpers for testing the Ai-WB2-12F module before
the STM32 firmware owns the link.

Print the PC IP used to reach the module:

```bash
python3 tools/aiwb2_net_tool.py ip --target 192.168.223.181
```

Listen for UDP data from the module:

```bash
python3 tools/aiwb2_net_tool.py udp-server --port 6666
```

Listen for UDP data and type replies back to the module:

```bash
python3 tools/aiwb2_net_tool.py udp-reply-console --port 6666
```

For the current Ai-WB2 auto transparent UDP-client mode, the PC must reply to
the module's source port. This command remembers the last source address and
uses it for replies.

Send one UDP packet to the module:

```bash
python3 tools/aiwb2_net_tool.py udp-send --module-ip 192.168.223.181 --module-port 7777 --message PING1234
```

Run a two-way UDP console:

```bash
python3 tools/aiwb2_net_tool.py udp-bridge --local-port 6666 --module-ip 192.168.223.181 --module-port 7777
```

Run a TCP server for the module to connect:

```bash
python3 tools/aiwb2_net_tool.py tcp-server --port 6666
```

## drone-H743 TCP panel

After the Ai-WB2 is configured as TCP client transparent mode to the PC, run
the GUI panel:

```bash
python3 tools/drone_tcp_panel.py
```

Default panel behavior:

- listens as a TCP server on `0.0.0.0:6666`
- shows a minimal ground-station layout with module overview/detail pages for
  GD25Q32 Flash, SPL06 barometer, ICM42688 IMU, and USART1 link diagnostics
- shows a dedicated SPL06 page with realtime/diagnostic fields, temporary data
  capture, CSV export, and optional plotting
- sends line commands such as `STATUS?`, `CONFIG?`, `PARAM?`, `PID?`,
  `BARO?`, `SERVO MOVE 0 1500 500`
- controls two Zhongling bus servos, mapped by default to IDs `1` and `2`
- can ask the board to save/load servo config in the onboard GD25Q32 flash
- keeps unknown board lines in the raw command log so firmware commands can be
  added incrementally without breaking the UI

SPL06 plotting uses `matplotlib` when it is installed. The panel still starts
without it and shows an install hint in the plot area:

```bash
python3 -m pip install matplotlib
```

The parameter/PID page is intentionally line-based first. It currently sends
`CONFIG?`, `PARAM?`, `PID?`, `PARAM SET <name> <value>`,
`PID SET <axis> kp=<v> ki=<v> kd=<v>`, `SAVE`, and `LOAD`; firmware that does
not yet implement every command should reply with its normal `ERR` line, which
the panel logs without crashing.

## drone-H743 flash diagnostics

When the board UART is flooded by periodic IMU sample lines, use the filtered
diagnostic runner instead of pasting commands into a serial terminal:

```bash
python3 tools/flash_diag_test.py --list-ports
python3 tools/flash_diag_test.py --serial COM18 --baud 115200 --final-rtos
```

The default test sends `RTOS?`, `FLASH?`, and the standard `FLASH VERIFY`
ranges one at a time. It prints only matching RTOS/FLASH responses, filters
unrelated IMU stream lines, and returns a non-zero exit code if any required
check fails.

Optional read throughput test:

```bash
python3 tools/flash_diag_test.py --serial COM18 --baud 115200 --bench --final-rtos
```

Optional serial auto-configuration through CH340 requires `pyserial`:

```bash
python3 -m pip install pyserial
python3 tools/aiwb2_net_tool.py configure-at \
  --serial-port /dev/cu.wchusbserialXXXX \
  --ssid YOUR_WIFI_SSID \
  --password YOUR_WIFI_PASSWORD \
  --local-port 7777
```

For local convenience, `configure-at` and the loop test also read
`AIWB2_WIFI_SSID`, `AIWB2_WIFI_PASSWORD`, and `AIWB2_TCP_PORT` from the
environment. Do not commit real WiFi credentials.

## VOFA UDP direct mode

The Ai-WB2 is configured as an auto-transparent UDP server on module port
`7777`, so VOFA can talk to it directly without the Python bridge.

Suggested VOFA settings:

- `数据接口`: `UDP`
- `远程IP`: the Ai-WB2 module IP shown by the router or `WIFI?`
- `远程端口`: `7777`
- `本地端口`: any free local UDP port, for example `6668`

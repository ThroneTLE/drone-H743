from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_aiwb2_prompt_symbol_is_preserved_for_transparent_entry() -> None:
    app_uart = read("App/Src/app_uart.c")
    app_aiwb2 = read("App/Src/app_aiwb2.c")

    assert 'line[start_index] == \'>\'' not in app_uart
    assert 'APP_AiWB2_ProcessLine(normalized);' in app_uart
    assert 'line[0] == \'>\'' in app_aiwb2
    assert 'aiwb2_is_transparent_prompt(line)' in app_aiwb2


def test_wifi_diag_escapes_transparent_mode_before_at_queries() -> None:
    app_aiwb2 = read("App/Src/app_aiwb2.c")

    assert 'aiwb2_provision_commands[2] = (APP_AiWB2Command){ "AT+WJAP?"' in app_aiwb2
    assert 'aiwb2_provision_commands[4] = (APP_AiWB2Command){ "AT+WAP?"' in app_aiwb2
    assert 'aiwb2_provision_commands[5] = (APP_AiWB2Command){ "AT+WAPDHCP?"' in app_aiwb2
    assert 'aiwb2_provision_commands[8] = (APP_AiWB2Command){ "AT+SOCKETTT"' in app_aiwb2
    assert "aiwb2_state = APP_AIWB2_STATE_ESCAPE_BEFORE;" in app_aiwb2
    assert 'APP_AiWB2_SendRawCommand(commands[i])' not in app_aiwb2


def test_aiwb2_default_wifi_mode_is_softap_udp_server() -> None:
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    app_control = read("App/Src/app_control.c")

    assert '#define APP_AIWB2_SOFTAP_SSID "DroneH743"' in app_aiwb2
    assert '{ "AT+WMODE=2,1", 2500U, 0U, 0U }' in app_aiwb2
    assert '"AT+WAP=" APP_AIWB2_SOFTAP_SSID' in app_aiwb2
    assert '"AT+WAPDHCP=1," APP_AIWB2_SOFTAP_DHCP_START' in app_aiwb2
    assert '{ "AT+SOCKET=1," APP_AIWB2_UDP_SERVER_PORT, 2500U, 0U, 0U }' in app_aiwb2
    assert 'aiwb2_starts_with(command->text, "AT+SOCKET=1,")' in app_aiwb2
    assert 'APP_AiWB2_StartSoftAp(tokens[2], tokens[3], channel, local_port)' in app_control


def test_usart1_wifi_udp_pid_tuning_path_is_text_line_based() -> None:
    usart = read("Core/Src/usart.c")
    app_uart = read("App/Src/app_uart.c")
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    app_control = read("App/Src/app_control.c")
    panel = read("tools/drone_tcp_panel.py")

    assert "huart1.Init.Mode = UART_MODE_TX_RX;" in usart
    assert "DMA_REQUEST_USART1_RX" in usart
    assert "DMA_REQUEST_USART1_TX" in usart
    assert "HAL_UARTEx_ReceiveToIdle_DMA(&huart1" in app_uart
    assert "HAL_UART_Transmit_DMA(&huart1" in app_uart

    assert 'strcmp(line, "PARAM?") == 0' in app_aiwb2
    assert 'strcmp(line, "PID?") == 0' in app_aiwb2
    assert 'aiwb2_starts_with(line, "PARAM ")' in app_aiwb2
    assert 'aiwb2_starts_with(line, "PID ")' in app_aiwb2
    assert "APP_Control_ProcessLine(normalized);" in app_uart
    assert "DRV_COAX_CTRL_SetParam(kp_name, kp)" in app_control
    assert "DRV_COAX_CTRL_SetParam(name, value)" in app_control

    assert "class UdpTransport" in panel
    assert 'values=("tcp", "udp", "serial")' in panel
    assert "return self.send_line(text)" in panel
    assert "self.structured_protocol_supported = False" in panel


def test_vofa_pid_slider_colon_lines_are_control_payloads() -> None:
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    app_control = read("App/Src/app_control.c")

    assert "aiwb2_is_pid_slider_payload" in app_aiwb2
    assert '"roll_angle_kp"' in app_aiwb2
    assert '"pitch_angle_kp"' in app_aiwb2
    assert '"roll_rate_kd"' in app_aiwb2
    assert '"pitch_rate_kd"' in app_aiwb2
    assert '"Pitch_kp"' not in app_aiwb2
    assert '"Roll_kp"' not in app_aiwb2
    assert '"vel_x_kd"' not in app_aiwb2
    assert '"vel_y_kd"' not in app_aiwb2
    assert '"vel_loop_x_kp"' in app_aiwb2
    assert "(aiwb2_is_pid_slider_payload(line) != 0U)" in app_aiwb2
    assert '"roll_angle_kp",  "coax.roll_angle_kp"' in app_control
    assert '"pitch_angle_kp", "coax.pitch_angle_kp"' in app_control
    assert '"roll_rate_kd",   "coax.roll_rate_kd"' in app_control
    assert '"pitch_rate_kd",  "coax.pitch_rate_kd"' in app_control
    assert '"Pitch_kp"' not in app_control
    assert '"Roll_kp"' not in app_control
    assert "app_control_after_param_separator" in app_control

cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME drone-H743)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and existing C++ tooling
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set(ESC_PROTOCOL "DSHOT300" CACHE STRING
    "ESC output protocol: DSHOT300, DSHOT300_BIDIR or PWM")
set_property(CACHE ESC_PROTOCOL PROPERTY STRINGS DSHOT300 DSHOT300_BIDIR PWM)
if(ESC_PROTOCOL STREQUAL "DSHOT300")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BSP_ESC_PROTOCOL=1)
elseif(ESC_PROTOCOL STREQUAL "DSHOT300_BIDIR")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BSP_ESC_PROTOCOL=2)
elseif(ESC_PROTOCOL STREQUAL "PWM")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BSP_ESC_PROTOCOL=0)
else()
    message(FATAL_ERROR "ESC_PROTOCOL must be DSHOT300, DSHOT300_BIDIR or PWM")
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Driver layer
    Driver/Src/drv_dshot.c
    Driver/Src/drv_sysid_rig.c
    Driver/Src/drv_sysid_record.c
    Driver/Src/drv_sysid_excitation.c
    Driver/Src/drv_dshot_telemetry.c
    Driver/Src/drv_dshot_bitbang.c
    Driver/Src/drv_current.c
    Driver/Src/drv_current_filter.c
    Driver/Src/drv_battery.c
    BSP/Src/bsp_current.c
    App/Src/app_current.c
    App/Src/app_battery.c
    App/Src/app_battery_led.c
    App/Src/app_battery_proto.c
    App/Src/app_battery_telemetry.c
    App/Src/app_diag_binary.c
    Driver/Src/drv_component_proto.c
    BSP/Src/bsp_component_catalog.c
    App/Src/app_components.c
    App/Src/app_components_board.c
    App/Src/app_components_transport.c
    BSP/Src/bsp_dshot.c
    BSP/Src/bsp_dshot_bitbang.c
    BSP/Src/bsp_dshot_rx.c
    App/Src/app_esc_diag.c
    App/Src/app_esc_log.c
    Driver/Src/drv_imu.c
    Driver/Src/drv_bmi088.c
    Driver/Src/drv_bmi088_tables.c
    Driver/Src/drv_bmi270.c
    Driver/Src/drv_bmi270_tables.c
    Driver/Src/drv_bmi270_config.c
    Driver/Src/drv_imu_calibration.c
    Driver/Src/drv_baro.c
    Driver/Src/drv_gd25q32.c
    Driver/Src/drv_gd25q32_timing_probe.c
    Driver/Src/drv_intflash.c
    Driver/Src/drv_sdblock.c
    Driver/Src/drv_mag.c
    Driver/Src/drv_mag_calibration.c
    Driver/Src/drv_gps.c
    Driver/Src/drv_tx_ring.c
    Driver/Src/drv_servo.c
    Driver/Src/drv_rgb_led.c
    Driver/Src/drv_motor.c
    Driver/Src/drv_elrs.c
    Driver/Src/drv_optical_flow.c
    Driver/Src/drv_airframe_params.c
    Driver/Src/drv_prop_map.c
    Driver/Src/drv_coax_ctrl.c
    Driver/Src/drv_flight_throttle.c
    Driver/Src/drv_thrust_lut.c
    Driver/Src/drv_position_control.c
    Driver/Src/drv_z_estimator.c
    Driver/Src/drv_hover_thrust_est.c
    Driver/Src/drv_attitude_control.c
    Driver/Src/drv_rate_control.c
    Driver/Src/drv_rpm_notch.c
    Driver/Src/drv_servo_backlash.c
    Driver/Src/drv_nav_ekf.c
    Driver/Src/drv_attitude_fusion.c
    ThirdParty/Fusion/FusionAhrs.c
    # Services layer
    Services/Src/svc_flow_nav.c
    Services/Src/svc_flow_capture.c
    Services/Src/svc_led.c
    Services/Src/svc_imu.c
    Services/Src/svc_mag.c
    Services/Src/svc_mag_heading.c
    Services/Src/svc_param.c
    Services/Src/svc_timestamp.c
    # BSP layer
    BSP/Src/bsp.c
    BSP/Src/bsp_board.c
    BSP/Src/bsp_cache.c
    BSP/Src/bsp_gpio.c
    BSP/Src/bsp_rgb_led.c
    BSP/Src/bsp_system.c
    BSP/Src/bsp_uart.c
    BSP/Src/bsp_uart_tx.c
    BSP/Src/bsp_uart_events.c
    BSP/Src/bsp_uart_link.c
    BSP/Src/bsp_rom_bootloader.c
    BSP/Src/bsp_critical.c
    BSP/Src/bsp_usb_cdc.c
    BSP/Src/bsp_i2c.c
    BSP/Src/bsp_spi.c
    BSP/Src/bsp_imu.c
    BSP/Src/bsp_baro.c
    BSP/Src/bsp_flash_bus.c
    BSP/Src/bsp_gps.c
    BSP/Src/bsp_optical_flow.c
    BSP/Src/bsp_mag.c
    BSP/Src/bsp_pwm.c
    BSP/Src/bsp_bus_servo.c
    BSP/Src/bsp_aiwb2_power.c
    # App layer
    App/Src/app.c
    App/Src/app_action.c
    App/Src/app_aiwb2.c
    App/Src/app_background.c
    App/Src/app_baro.c
    App/Src/app_boot.c
    App/Src/app_acceptance.c
    App/Src/app_control.c
    App/Src/app_control_core.c
    App/Src/app_control_config_compat.c
    App/Src/app_control_config_store.c
    App/Src/app_control_scheduler.c
    App/Src/app_cmd_servocal.c
    App/Src/app_cmd_servotype.c
    App/Src/app_cmd_imucal.c
    App/Src/app_cmd_rcmap.c
    App/Src/app_cmd_flow.c
    App/Src/app_cmd_telem.c
    App/Src/app_cmd_system.c
    App/Src/app_cmd_diag.c
    App/Src/app_cmd_imusel.c
    App/Src/app_cmd_probe.c
    App/Src/app_cmd_airframe.c
    App/Src/app_param_trial.c
    App/Src/app_cmd_arm.c
    App/Src/app_cmd_led.c
    App/Src/app_cmd_ledmap.c
    App/Src/app_led_config.c
    App/Src/app_cmd_propcal.c
    App/Src/app_cmd_thrust_bench.c
    App/Src/app_cmd_magcal.c
    App/Src/app_magxy.c
    App/Src/app_cmd_currprobe.c
    App/Src/app_cmd_hover.c
    App/Src/app_cmd_imuzero.c
    App/Src/app_rc_aux.c
    App/Src/app_hover_adapt.c
    App/Src/app_cmd_flograte.c
    App/Src/app_cmd_flogdump.c
    App/Src/app_cmd_thrmode.c
    App/Src/app_cmd_flowcap.c
    App/Src/app_hover_thrust.c
    App/Src/app_cmd_esc_edt.c
    App/Src/app_cmd_esc_kv.c
    App/Src/app_esc_command.c
    App/Src/app_cmd_sysid.c
    App/Src/app_sysid.c
    App/Src/app_sysid_alt.c
    App/Src/app_sysid_xy.c
    App/Src/app_sysid_yaw.c
    App/Src/app_prop_spin.c
    App/Src/app_thrust_bench.c
    App/Src/app_thrust_lut.c
    App/Src/app_rpm_notch.c
    App/Src/app_cmd_rpmnotch.c
    App/Src/app_servo_backlash.c
    App/Src/app_cmd_backlash.c
    App/Src/app_cmd_servohz.c
    App/Src/app_cmd_fallback.c
    App/Src/app_diag.c
    App/Src/app_flash.c
    App/Src/app_firmware_identity.c
    App/Src/app_flight_calibration.c
    App/Src/app_flight_log.c
    App/Src/app_flight_log_nav.c
    App/Src/app_imu_capture.c
    App/Src/app_imu_health.c
    App/Src/app_flash_service.c
    App/Src/app_gps.c
    App/Src/app_optical_flow.c
    App/Src/app_rc_config.c
    App/Src/app_rc_intent.c
    App/Src/app_ident.c
    App/Src/app_servo_cal.c
    App/Src/app_servo_bus_guard.c
    App/Src/app_servo_type.c
    App/Src/app_servo_jog.c
    App/Src/app_servo_feedback.c
    App/Src/app_servo_feedback_bench.c
    App/Src/app_sensor.c
    App/Src/app_stabilizer.c
    App/Src/app_led.c
    App/Src/app_mag.c
    App/Src/app_maint_uart.c
    App/Src/app_message.c
    App/Src/app_nav_estimator.c
    App/Src/app_proto.c
    App/Src/app_tasks.c
    App/Src/app_telemetry.c
    App/Src/app_telem_frame.c
    App/Src/app_telem_port.c
    App/Src/app_telem_stream.c
    App/Src/app_uart.c
    App/Src/app_usb_cdc.c
    App/Src/app_elrs.c
    App/Src/app_vofa.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Driver/Inc
    ThirdParty/Fusion
    Services/Inc
    BSP/Inc
    App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

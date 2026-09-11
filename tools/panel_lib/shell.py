"""Main panel shell assembly; page logic remains in its own modules."""
import tkinter as tk
from tkinter import ttk
from .viewport import VerticalScrolledFrame, FixedActionViewport
from .arm_banner import mount_arm_banner
from .pages.airframe import mount_airframe
from .pages.logs import mount_logs
from .pages.simulation import mount_simulation


def build_ui(self):
    root = ttk.Frame(self, padding=12, style="Shell.TFrame")
    root.pack(fill=tk.BOTH, expand=True)

    self._build_connection_bar(root)
    # 主窗口最上方留给解锁状态：这是每次上机都要先看一眼的东西。
    # 仿真启动栏原来占着这个位置，已搬进「仿真」栏目。
    mount_arm_banner(self, root)

    body = ttk.PanedWindow(root, orient=tk.VERTICAL)
    body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
    self._body_pane = body

    self.notebook = ttk.Notebook(body)
    body.add(self.notebook, weight=6)

    overview_scroll = VerticalScrolledFrame(self.notebook)
    calibration = ttk.Frame(self.notebook, padding=8, style="Page.TFrame")
    self.calibration_notebook = ttk.Notebook(calibration)
    self.calibration_notebook.pack(fill=tk.BOTH, expand=True)

    validation_scroll = VerticalScrolledFrame(self.calibration_notebook)
    metrology_scroll = VerticalScrolledFrame(self.calibration_notebook)
    rc_scroll = VerticalScrolledFrame(self.calibration_notebook)
    mechanical_scroll = VerticalScrolledFrame(self.calibration_notebook)
    flow_range_scroll = VerticalScrolledFrame(self.calibration_notebook)
    acceptance_v2_scroll = VerticalScrolledFrame(self.calibration_notebook)
    vibration_scroll = VerticalScrolledFrame(self.calibration_notebook)
    firmware_scroll = VerticalScrolledFrame(self.notebook)
    validation = validation_scroll.content
    metrology = metrology_scroll.content
    rc = rc_scroll.content
    mechanical = mechanical_scroll.content
    flow_range = flow_range_scroll.content
    acceptance_v2 = acceptance_v2_scroll.content
    vibration = vibration_scroll.content
    firmware = firmware_scroll.content
    overview = overview_scroll.content
    sensors = ttk.Frame(self.notebook, padding=8, style="Page.TFrame")
    self.sensor_notebook = ttk.Notebook(sensors)
    self.sensor_notebook.pack(fill=tk.BOTH, expand=True)
    baro_scroll = VerticalScrolledFrame(self.sensor_notebook)
    imu_scroll = VerticalScrolledFrame(self.sensor_notebook)
    gps_scroll = VerticalScrolledFrame(self.sensor_notebook)
    flow_sensor_scroll = VerticalScrolledFrame(self.sensor_notebook)
    ident_scroll = VerticalScrolledFrame(self.notebook)
    params_scroll = VerticalScrolledFrame(self.notebook)
    airframe_scroll = VerticalScrolledFrame(self.notebook)
    simulation_scroll = VerticalScrolledFrame(self.notebook)
    servos_view = FixedActionViewport(self.notebook)
    commands_scroll = VerticalScrolledFrame(self.notebook)
    baro = baro_scroll.content
    imu = imu_scroll.content
    gps = gps_scroll.content
    flow_sensor = flow_sensor_scroll.content
    ident = ident_scroll.content
    params = params_scroll.content
    airframe = airframe_scroll.content
    simulation = simulation_scroll.content
    servos = servos_view.content
    commands = commands_scroll.content
    self.airframe_tab = airframe_scroll
    self.simulation_tab = simulation_scroll
    self.baro_tab = baro_scroll
    self.imu_tab = imu_scroll
    self.calibration_group_tab = calibration
    self.sensor_group_tab = sensors
    self.flow_sensor_tab = flow_sensor_scroll
    self.validation_tab = validation_scroll
    self.v1_tab = metrology_scroll
    self.rc_tab = rc_scroll
    self.mechanical_tab = mechanical_scroll
    self.flow_range_tab = flow_range_scroll
    self.v2_tab = acceptance_v2_scroll
    self.vibration_tab = vibration_scroll
    self.firmware_tab = firmware_scroll
    self.gps_tab = gps_scroll
    self.ident_tab = ident_scroll

    self.notebook.add(overview_scroll, text="总览")
    self.notebook.add(calibration, text="校准")
    self.calibration_notebook.add(validation_scroll, text="坐标系与极性")
    self.calibration_notebook.add(metrology_scroll, text="IMU 零偏与比例")
    self.calibration_notebook.add(rc_scroll, text="遥控器")
    self.calibration_notebook.add(mechanical_scroll, text="舵机机械中心与行程")
    self.calibration_notebook.add(flow_range_scroll, text="光流与测距")
    self.calibration_notebook.add(acceptance_v2_scroll, text="无桨控制链验收")
    self.calibration_notebook.add(vibration_scroll, text="振动检测与滤波")
    self.notebook.add(firmware_scroll, text="维护 · 固件升级")
    self.notebook.add(sensors, text="传感器")
    self.sensor_notebook.add(baro_scroll, text="气压计")
    self.sensor_notebook.add(imu_scroll, text="IMU 监视（旧链）")
    self.sensor_notebook.add(gps_scroll, text="GPS / 磁力计")
    self.sensor_notebook.add(flow_sensor_scroll, text="光流")
    self.notebook.add(servos_view, text="维护 · 舵机调试")
    self.notebook.add(params_scroll, text="参数 / PID")
    self.notebook.add(airframe_scroll, text="机体模型")
    self.notebook.add(ident_scroll, text="系统辨识")
    self.notebook.add(simulation_scroll, text="仿真")
    self.notebook.add(commands_scroll, text="诊断 / 命令")

    self._build_overview_page(overview)
    self._build_validation_page(validation)
    self._build_v1_page(metrology)
    self._build_rc_page(rc)
    self._build_mechanical_calibration_page(mechanical)
    self._build_flow_range_calibration_page(flow_range)
    self._build_v2_page(acceptance_v2)
    self._build_vibration_filter_page(vibration)
    self._build_firmware_update_page(firmware)
    self._build_baro_page(baro)
    self._build_imu_page(imu)
    self._build_gps_page(gps)
    self._build_sensor_flow_page(flow_sensor)
    self._build_ident_page(ident)
    self._build_params_page(params)
    mount_airframe(self, airframe)
    mount_simulation(self, simulation)
    self._build_servo_page(servos, fixed_parent=servos_view.fixed)
    self._build_command_page(commands)
    self._dashboard_mount(self.notebook)
    mount_logs(self)

    self.log_box = ttk.LabelFrame(body, text="原始命令日志", padding=8)
    body.add(self.log_box, weight=1)
    self._build_log_area(self.log_box)
    self.after_idle(self._toggle_log_area)

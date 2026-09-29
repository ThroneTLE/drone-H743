"""Main panel shell assembly; page logic remains in its own modules."""
import tkinter as tk
from tkinter import ttk
from .viewport import VerticalScrolledFrame, FixedActionViewport
from .arm_banner import mount_arm_banner
from .pages.airframe import mount_airframe
from .pages.led_map import mount_led_map
from .pages.mag_cal import MAG_CAL_TAB_TEXT, mount_mag_cal
from .pages.prop_map import mount_prop_map
from .pages.logs import mount_logs
from .pages.simulation import mount_simulation
from .pages.power import POWER_TAB_TEXT, mount_power
from .pages.sysid import SYSID_TAB_TEXT, mount_sysid


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
    prop_map_scroll = VerticalScrolledFrame(self.calibration_notebook)
    flow_range_scroll = VerticalScrolledFrame(self.calibration_notebook)
    acceptance_v2_scroll = VerticalScrolledFrame(self.calibration_notebook)
    vibration_scroll = VerticalScrolledFrame(self.calibration_notebook)
    mag_cal_scroll = VerticalScrolledFrame(self.calibration_notebook)
    # 固件升级 / 舵机调试 / 状态灯原来在顶层各占一格，前缀都写着“维护 · ”——
    # 三个同级页签拼同一个前缀，就是在用文字模拟一层本该存在的结构。收进分组。
    maintenance = ttk.Frame(self.notebook, padding=8, style="Page.TFrame")
    self.maintenance_notebook = ttk.Notebook(maintenance)
    self.maintenance_notebook.pack(fill=tk.BOTH, expand=True)
    firmware_scroll = VerticalScrolledFrame(self.maintenance_notebook)
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
    power_scroll = VerticalScrolledFrame(self.sensor_notebook)
    sysid_group = ttk.Frame(self.notebook, padding=8, style="Page.TFrame")
    params_scroll = VerticalScrolledFrame(self.notebook)
    airframe_scroll = VerticalScrolledFrame(self.notebook)
    simulation_scroll = VerticalScrolledFrame(self.notebook)
    servos_view = FixedActionViewport(self.maintenance_notebook)
    commands_scroll = VerticalScrolledFrame(self.notebook)
    led_map_scroll = VerticalScrolledFrame(self.maintenance_notebook)
    baro = baro_scroll.content
    imu = imu_scroll.content
    gps = gps_scroll.content
    flow_sensor = flow_sensor_scroll.content
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
    self.maintenance_group_tab = maintenance
    self.flow_sensor_tab = flow_sensor_scroll
    # 「电流计」与「电池电压」曾是两个页签，各自 2 秒轮询、都显示电流。合并成
    # 一个「电源」页之后只剩一个页签属性（R-PWR-1）。
    self.power_tab = power_scroll
    self.validation_tab = validation_scroll
    self.v1_tab = metrology_scroll
    self.rc_tab = rc_scroll
    self.mechanical_tab = mechanical_scroll
    self.prop_map_tab = prop_map_scroll
    self.flow_range_tab = flow_range_scroll
    self.v2_tab = acceptance_v2_scroll
    self.vibration_tab = vibration_scroll
    self.mag_cal_tab = mag_cal_scroll
    self.firmware_tab = firmware_scroll
    self.gps_tab = gps_scroll
    self.sysid_tab = sysid_group
    self.led_map_tab = led_map_scroll

    self.notebook.add(overview_scroll, text="总览")
    self.notebook.add(calibration, text="校准")
    self.calibration_notebook.add(validation_scroll, text="坐标系与极性")
    self.calibration_notebook.add(metrology_scroll, text="IMU 零偏与比例")
    self.calibration_notebook.add(rc_scroll, text="遥控器")
    self.calibration_notebook.add(mechanical_scroll, text="舵机机械中心与行程")
    self.calibration_notebook.add(prop_map_scroll, text="桨叶与电机方向")
    self.calibration_notebook.add(flow_range_scroll, text="光流与测距")
    self.calibration_notebook.add(acceptance_v2_scroll, text="无桨控制链验收")
    self.calibration_notebook.add(vibration_scroll, text="振动检测与滤波")
    self.calibration_notebook.add(mag_cal_scroll, text=MAG_CAL_TAB_TEXT)
    self.notebook.add(maintenance, text="维护")
    self.maintenance_notebook.add(firmware_scroll, text="固件升级")
    self.maintenance_notebook.add(servos_view, text="舵机调试")
    self.maintenance_notebook.add(led_map_scroll, text="状态灯")
    self.notebook.add(sensors, text="传感器")
    self.sensor_notebook.add(baro_scroll, text="气压计")
    self.sensor_notebook.add(imu_scroll, text="IMU 监视（旧链）")
    self.sensor_notebook.add(gps_scroll, text="GPS / 磁力计")
    self.sensor_notebook.add(flow_sensor_scroll, text="光流")
    self.sensor_notebook.add(power_scroll, text=POWER_TAB_TEXT)
    self.notebook.add(params_scroll, text="参数 / PID")
    self.notebook.add(airframe_scroll, text="机体模型")
    self.notebook.add(sysid_group, text=SYSID_TAB_TEXT)
    self.notebook.add(simulation_scroll, text="仿真")
    self.notebook.add(commands_scroll, text="诊断 / 命令")

    self._build_overview_page(overview)
    self._build_validation_page(validation)
    self._build_v1_page(metrology)
    self._build_rc_page(rc)
    self._build_mechanical_calibration_page(mechanical)
    mount_prop_map(self, prop_map_scroll.content)
    self._build_flow_range_calibration_page(flow_range)
    self._build_v2_page(acceptance_v2)
    self._build_vibration_filter_page(vibration)
    mount_mag_cal(self, mag_cal_scroll.content)
    self._build_firmware_update_page(firmware)
    self._build_baro_page(baro)
    self._build_imu_page(imu)
    self._build_gps_page(gps)
    self._build_sensor_flow_page(flow_sensor)
    mount_sysid(self, sysid_group)
    self._build_params_page(params)
    mount_airframe(self, airframe)
    mount_led_map(self, led_map_scroll.content)
    mount_simulation(self, simulation)
    self._build_servo_page(servos, fixed_parent=servos_view.fixed)
    self._build_command_page(commands)
    # 电源页要向工作台的链路仲裁器登记订阅，所以必须在它之后挂载。
    self._dashboard_mount(self.notebook)
    mount_power(self, power_scroll.content)
    mount_logs(self)

    self.log_box = ttk.LabelFrame(body, text="原始命令日志", padding=8)
    body.add(self.log_box, weight=1)
    self._build_log_area(self.log_box)
    self.after_idle(self._toggle_log_area)

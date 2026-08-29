# drone-H743 专属上位机与飞控自检架构规划

> **定位**：面向 `drone-H743` 飞控与测控系统的跨平台、高颜值、多协议可视化调试地面站与自检工作台（基于 [Serial-Studio](https://github.com/Serial-Studio/Serial-Studio) 二次开发）。

---

## 🏛️ 一、核心工程架构思想

### 1. 系统边界与控制权原则
* **上位机定位**：负责**状态展示、参数配置、任务编排发起与结果呈现**。上位机原则上不直接运行高危的延时控制逻辑（如在脚本中通过 `sleep(5)` 分步控制电机或舵机）。
* **固件端定位**：负责**物理执行器的闭环安全、状态互锁、过程看门狗与异常急停**。任何涉及物理执行器持续运转或改变飞控基准的操作，其安全生命周期必须闭环在固件端。

```text
[ 上位机 (Serial-Studio / UI) ]
   │
   ├─ 1. Query / Command (查询状态 / 原子指令)
   ├─ 2. Action: Start / Cancel (发起任务 / 取消任务)
   └─ 3. Action: Status Poll / Event Stream (监听进度与结果展示)
   │
[ 通信链路 (USB CDC / Wi-Fi UDP / UART) ]
   │
[ 飞控端 (STM32H743 FreeRTOS) ]
   ├─ 状态互锁校验 (Armed 状态禁止任何自检 Action)
   ├─ 硬件级时间看门狗 (通信中断自动超时复位)
   └─ 任务状态机闭环 (IDLE -> RUNNING -> COMPLETED / FAILED / CANCELLED)
```

---

## 🧩 二、API 分类与生命周期契约规范

根据行为语义与风险等级，飞控端接口严格划分为四类：

### 1. Query（查询类 API）
* **特征**：只读、无副作用、无时间跨度、幂等。
* **调用条件**：随时可调，不影响飞控运行。
* **典型接口**：
  * `SYS:STATE?`：查询系统主状态（`DISARMED` / `ARMED` / `IN_FLIGHT` / `ERROR`）。
  * `SENS:IMU?`：读取当前 IMU 原始样本与 Fusion 姿态角。
  * `SYS:BATT?`：读取电池电压、电流与剩余电量。

### 2. Command（基础原子指令）
* **特征**：瞬时完成、原子操作、无持续性物理危险。
* **典型接口**：
  * `CMD:BEEP [freq_hz] [duration_ms]`：蜂鸣器提示音。
  * `CMD:LED_SET [id] [color_rgb]`：状态指示灯配置。
  * `CMD:PARAM_SET [key] [val]`：修改内存参数（非持久化）。

### 3. Action（任务 API / 复合多步骤任务）
* **特征**：**跨越时间、多步骤、中途意外停止会导致危险、独占硬件执行器**。
* **生命周期状态机**：
  $$\text{IDLE} \xrightarrow{\text{START}} \text{RUNNING} \xrightarrow{\text{DONE}} \text{COMPLETED} \quad \Big/ \quad \xrightarrow{\text{ERR/TIMEOUT}} \text{FAILED} \quad \Big/ \quad \xrightarrow{\text{CANCEL}} \text{CANCELLED}$$
* **统一任务契约字段**：
  * `action_id`：任务唯一标识。
  * `state`：当前状态（`IDLE / RUNNING / COMPLETED / FAILED / CANCELLED`）。
  * `progress_percent`：进度百分比（$0 \sim 100\%$）。
  * `timeout_ms`：固件端强制硬件看门狗时间（超时无条件复位硬件）。
  * `error_code`：失败原因（$0$: 正常, 负数: 错误码）。
  * `hard_reset_fn`：任务中止或超时时的安全复位回调（如电机断能、舵机回中）。

### 4. Stream & Telemetry（连续流与遥测）
* **特征**：持续周期推送、可适度丢包、带时间戳。
* **典型协议**：
  * **VOFA+ JustFloat**：高频 28 通道 32 位浮点流（小端 IEEE 754 + `0x00, 0x00, 0x80, 0x7F` 尾帧）。
  * **VOFA+ FireWater**：文本逗号分隔流（`ch1:1.23,ch2:4.56\n`），便于简易 `printf`。
  * **Compact Telemetry**：定长紧凑二进制遥测包。

---

## 🛠️ 三、典型自检与校准 Action 模型

### 任务 1：IMU 静态零偏自检与校准 (`ACT:IMU_CALIB`)
* **解决的问题**：在地面静止状态下采集若干组陀螺仪/加速度计样本，计算零偏方差。
* **前置互锁**：系统必须处于 `DISARMED` 且机身静止。
* **运行机制**：采样 1000 组数据，固件内部检测振动方差。若检测到机身晃动，立即终止并返回 `ERR_IMU_VIBRATION`；若平稳收敛，计算均值并返回 `COMPLETED`。
* **数据持久化**：校准完成后不直接刷写 Flash，需由上位机下发独立的 `FLASH:SAVE` 确认指令。

### 任务 2：舵机全行程自检与回中响应 (`ACT:SERVO_SWEEP`)
* **解决的问题**：测试共轴双桨倾转舵机行程是否卡死、反馈是否正常。
* **安全约束**：固件内置硬看门狗（如限时 4000ms）。
* **异常处理**：上位机如果在第 2 秒断线，飞控在 4000ms 超时后**无条件驱动舵机回中位并释放 PWM 输出**，禁止舵机堵转卡在极限位。

### 任务 3：电机受控点动安全测试 (`ACT:MOTOR_SPIN`)
* **解决的问题**：装机排查电机转向、电调通信及桨叶平衡。
* **安全约束**：
  * 最大油门输出硬限幅（如最大允许 $15\%$）。
  * 最大运转时间硬限幅（如单次点动最长允许 $2000\text{ ms}$）。
  * 急停打断：收到任何非法指令或取消指令，立即关闭 PWM。

### 任务 4：气压计 / ToF / 光流传感器健康度自检 (`ACT:SENSOR_HEALTH`)
* **解决的问题**：检查各 I2C/SPI/UART 外设通信连通性与底噪数据合理性。
* **输出结果**：各项传感器连通状态、采样率统计、丢包率与自检通过标志。

---

## 🛡️ 四、安全互锁与异常处理矩阵

| 场景 / 异常 | 风险描述 | 固件端处理策略 (Firmware Action) | 上位机表现 (UI/SDK) |
| :--- | :--- | :--- | :--- |
| **已解锁状态触发自检** | 飞行中误触导致执行器失控或传感器归零 | 互锁拦截：直接返回 `ERR_STATE_ARMED`，拒绝启动任何 Action | 按钮置灰，弹出危险拦截提示 |
| **通信中断 / 上位机崩溃** | 舵机扫频或电机测试中途断线 | Action 内部定时器超时，触发 `hard_reset_fn`，电机停机、舵机回中 | UI 显示 Disconnected 并超时判定任务失效 |
| **校准中机体晃动** | 产生严重错误的姿态基准导致翻机 | 固件方差超标拦截，标记 `ACT_FAILED`，不应用该组零偏 | 提示“校准失败：机身晃动，请保持静止后重试” |
| **紧急中止 (Abort/E-Stop)** | 现场出现险情需要立即停止所有动作 | 立即终止当前 Action，强制所有执行器进入安全断能态 | 提供显著的红色急停 (E-Stop) 按钮 |

---

## 🖥️ 五、上位机 (Serial-Studio 二次开发) 规划

### 1. 软件架构设计
```text
[ Qt Quick / QML 用户界面层 ]
   ├── 3D 姿态与航向仪表盘 (OpenGL / 3D Model)
   ├── 实时多通道曲线绘图 (QCustomPlot / OpenGL)
   ├── 飞控自检与校准向导面板 (Action Orchestration)
   └── PID 在线调参台与参数管理器 (PID Tuner)
         │
[ 统一 Device SDK / 协议分发层 ]
   ├── Action Manager (任务发起、状态轮询、超时看门狗、进度事件)
   ├── Protocol Engine (JustFloat / FireWater / ASCII Command / Telemetry)
   └── Flash Parameter Cache (参数读取/差异比对/一键写入)
         │
[ 传输层 (SerialPort / Network Socket / UDP) ]
```

### 2. 核心功能页面规划
1. **飞控仪表与 3D 姿态监控面板**：
   - 导入四旋翼/共轴双桨 3D 机架模型（`models/drone_frame.glb`），实时绑定 Fusion 姿态角。
   - 姿态地平仪、升降速度计、高度曲线、电机 PWM 条形图。
2. **一键自检与校准向导 (Self-Test Wizard)**：
   - 组合调用固件端 Action：IMU 静态校准 $\to$ 舵机全行程测试 $\to$ 传感器健康度扫描。
   - 输出完整的自检健康度报告（Pass/Fail 矩阵）。
3. **PID 在线调参面板 (PID Tuner)**：
   - 姿态角环、角速度环 PID 参数滑动条实时发送与生效。
   - 支持读取飞控当前参数、比对差异、一键烧录到飞控 Flash（`FLASH:SAVE`）。
4. **飞行日志回放与波形分析 (Log Player)**：
   - 对接 `data/flight_logs/`，载入离线日志实现时间轴拖拽播放与特征分析。

---

## 🧪 六、验收测试标准 (Given / When / Then)

* **[TEST-01] 互锁保护测试**：
  * *Given*: 飞控处于解锁状态（`ARMED`）。
  * *When*: 上位机发送 `ACT:IMU_CALIB:START`。
  * *Then*: 固件在 5ms 内返回错误码 `ERR_NOT_PERMITTED_IN_ARMED`，执行器与传感器基准不发生任何变化。
* **[TEST-02] 断线看门狗测试**：
  * *Given*: 启动舵机自检任务（持续时间 4 秒）。
  * *When*: 运行至第 1.5 秒时拔掉 USB 数据线。
  * *Then*: 飞控在 4 秒硬件超时到达时刻，自动驱动舵机回归中位并释放 PWM 输出。
* **[TEST-03] 取消操作测试**：
  * *Given*: 任务处于 `RUNNING` 状态。
  * *When*: 上位机发送 `ACT:CANCEL`。
  * *Then*: 固件在 10ms 内停止动作，进入 `CANCELLED` 状态并执行安全复位。

---

## 🚀 五、快速上手与使用指南

我们已将原 `tools/drone_tcp_panel.py` 的核心监控、控制指令、校准流程与数据显示完整迁移至专用地面站项目：[`tools/ground_station/Drone-H743-GCS.ssproj`](file:///d:/stm32hal/drone-H743/tools/ground_station/Drone-H743-GCS.ssproj)。

### 1. 一键启动方式

| 启动脚本 | 说明 | 适用场景 |
| :--- | :--- | :--- |
| **`run_sim.bat`** | **一键启动地面站 + 50Hz 高保真飞行仿真器** | 无需连接飞控硬件，立即查看全部 3D 姿态、航向罗盘、高度计、GPS 地图、频谱及双向指令交互 |
| **`run_gcs.bat`** | **直接启动专用地面站** | 连接真实 STM32H743 飞控（USB 串口 / Wi-Fi TCP / UDP）进行实机调试 |

### 2. 专用地面站内置监控面板与功能

1. **✈️ 3D姿态与角速度 (Attitude & Gyro)**：
   - 3D 姿态球/人工地平仪（Roll 横滚、Pitch 俯仰、Yaw 偏航）。
   - 3 轴角速度实时动态曲线。
2. **📈 3轴加速度与振动频谱 (Accelerometer & FFT)**：
   - $Ax, Ay, Az$ 3 轴加速度波形与实时快速傅里叶变换（FFT）振动频谱分析。
3. **⛰️ 气压高度与升降速率 (Barometer & Variometer)**：
   - SPL06 相对高度曲线、大气压强仪表盘、垂直升降速率计。
4. **🛰️ GPS 导航地图与电子罗盘 (GPS Map & Compass)**：
   - 经纬度航迹地图跟踪、地速表盘、卫星颗数、3D Fix 状态指示灯、磁航向罗盘。
5. **⚡ 电源与系统健康监视 (Power & System Health)**：
   - 动力电池电压（3S/4S 电平监控与低压告警）、总电流、已消耗电量、CPU 占用率、加解锁（Armed）安全指示。
6. **🎛️ 共轴双电机与4舵面输出 (Coaxial Motors & Servos)**：
   - 上/下双电机实时油门开度百分比、4 舵机实时舵偏角反馈柱状图。
7. **🎛️ 顶部一键交互动作栏 (Actions Toolbar)**：
   - `🔔 PING`：链路心跳检测
   - `🛡️ STATUS?`：查询主状态
   - `🔒 ARM`：解锁电机
   - `⛔ DISARM`：紧急停机加锁
   - `🧭 CALIB IMU`：零偏校准 IMU
   - `⛰️ CALIB BARO`：校准气压基准
   - `🧪 SERVO TEST`：舵机行程扫频自检
   - `💾 CONFIG SAVE`：持久化配置写入 Flash
   - `🔄 REBOOT`：远程软重启飞控

---

## 📦 七、仓库与环境配置

* **上游仓库**：[Serial-Studio/Serial-Studio](https://github.com/Serial-Studio/Serial-Studio) (MIT License)
* **本工程 Fork 仓库**：[ThroneTLE/Serial-Studio](https://github.com/ThroneTLE/Serial-Studio)
* **子模块路径**：`tools/上位机/Serial-Studio/`
* **推荐编译环境**：Qt 6.5+ (MSVC 2019/2022 64-bit 或 MinGW 64-bit), CMake 3.20+, Ninja

```powershell
# 子模块初始化与更新命令
cd d:\stm32hal\drone-H743
git submodule update --init --recursive
```

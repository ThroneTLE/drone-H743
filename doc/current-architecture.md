# drone-H743 当前软件架构

> 本文只描述当前模块边界和数据流，不保存易变参数、测试数量或阶段进度。
> 当前进度只看 `PIPELINE.md`，具体符号和值以源码和契约测试为准。

## 分层

```text
App：任务、命令、运行时行为、安全与控制装配
  ↓
Services：同步、无任务的数据与领域服务
  ↓
Driver：器件协议和可复用算法
  ↓
BSP：板级外设绑定、锁、DMA 回调和 cache 维护
  ↓
Core/HAL：CubeMX 生成的时钟、引脚、外设和 RTOS 对象
```

依赖只能向下。`Core/Src/main.c`、`freertos.c`、外设初始化和 `USB_DEVICE/*` 由 CubeMX
拥有；手写行为放在 `App/*`、`Services/*`、`Driver/*`、`BSP/*`。

## 主要运行链

### IMU 与姿态

```text
ICM42688 Driver/BSP
  → app_sensor（采样、校准、机体方向发布）
  → app_stabilizer（Fusion 适配、状态装配、控制周期）
  → drv_attitude_fusion
```

规范机体系由 `Driver/Inc/drv_frame_contract.h` 唯一定义。运行时仍存在具名 legacy
适配，不能因为某个出口已经是 FLU 就宣称全链迁移完成。

### 光流与水平导航

```text
drv_optical_flow（只取帧）
  → app_optical_flow（设备生命周期和健康状态）
  → svc_flow_nav（高度/速度滤波、质量门、EKF、位置与位移积分）
  → app_stabilizer（姿态/旋转补偿、控制器边界）
```

滤波、质量判定和水平估计只允许由 `Services/svc_flow_nav` 拥有，App 不保留副本。

### RC、控制器与执行器

```text
RC Driver → app_rc_config → app_stabilizer
  → drv_coax_ctrl
  → 电机 PWM
  → 舵机输出类型选择：BUS 或 PWM
```

安全门、RC 意图、控制器坐标适配、机械 `pulse_sign` 和舵机输出类型是不同边界，
不得通过负增益或发射机反向选项互相补偿。

### 参数与慢操作

```text
App 请求
  → backgroundReqQueue / backgroundTask
  → svc_param 或 app_flash_service
  → drv_gd25q32
  → bsp_flash_bus
```

控制环不得直接执行 Flash 擦写或阻塞 USB 文本发送。Param 使用双槽后台持久化。

### 遥测与上位机

```text
app_telemetry（通道表）
  → app_telem_stream / app_telem_port
  → USB CDC 或 UART 的 $X 帧
  → panel_lib.transport
  → panel_lib.telem_stream
  → dashboard tiles / CSV
```

线上格式见 `doc/telemetry-protocol.md`。状态监视对外采用带来源标记的机体 FLU；
这只规范观察边界，不自动迁移控制器内部表示。

## 安全边界

- 解锁至少受 IMU frame、IMU 健康、RC 链路和低油门上升沿约束。
- 标定 RAM candidate 保持 arm-lock。
- 危险动作的超时和安全复位必须闭环在固件端，上位机退出不能留下危险状态。
- 历史校准证据 `data/calibration/**` 不修改、不覆盖。
- M7 带桨工作以 `PIPELINE.md` 状态为准，不能由软件测试代替实机验收。

## 当前代码导航

先运行仓库索引检查，再读最小分片：

```powershell
python .agents/skills/drone-h743-project/scripts/update_repository_index.py --check
```

索引入口：`.agents/skills/drone-h743-project/references/repository-index/README.md`。

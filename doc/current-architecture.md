# drone-H743 当前软件架构

> 本文维护当前模块边界、数据流与已采纳设计理由，不保存易变参数、测试数量或阶段进度。
> 当前进度只看 `PIPELINE.md`，具体符号和值以规范契约头及核实过的源码为准；未审实现不能自行改写已采纳设计。

## 已采纳设计与延续

以下登记既有工程约定，状态为“采用中”，不表示实现或实机已经验收。条款详细定义仍在所链接的规范中，不复制第二份技术标准。

| 决定 | 为什么采用 | 允许调整 / 必须保持 | 依据与重新讨论条件 |
|---|---|---|---|
| AD-01 分层与硬件配置归属 | 隔离器件、板级资源和应用，避免生成代码覆盖业务 | 模块内实现可调整；依赖向下，生成配置由 CubeMX 拥有 | 下节分层及 [解耦规范](../.agents/skills/drone-h743-project/references/decoupling-spec.md)；新硬件需求无法由现有边界表达时由主控提案，生成归属不自行改变 |
| AD-02 导航估计单一归属 | 避免 App 与 Service 各自滤波、门控，形成不一致状态 | 导航估计由 `svc_flow_nav` 拥有；App 管设备/装配，页面消费结果，监控统计不得替代控制状态 | 本文光流链；更换估计方案需明确状态/重置语义与全部调用方迁移 |
| AD-03 控制节拍与慢操作隔离 | 阻塞存储/通信会丢控制拍，名义时间掩盖真实延迟 | 模块实现可优化；控制高频路径不执行阻塞 I/O，调度使用约定时间来源 | 本文控制/参数链及 [runtime-services](../.agents/skills/drone-h743-project/references/runtime-services.md)；调度职责变化需列清时间域和最坏阻塞证据 |
| AD-04 超限入口只减不增 | 避免重复状态、难审的大文件和多 Agent 争用入口 | 新功能在模块实现；两个冻结增长入口只减不增，不复制旧实现形成双轨 | [技术规范 §11](technical-spec.md#11-巨文件永久约束)；职责迁移先由主控确定唯一所有者，取消此约束需作者决定 |
| AD-05 共轴台架观测与飞行控制隔离 | 用电转速eRPM、DShot分路电流、称重与带载电压描述实际安装的双桨，避免 KV 推算、异步拼接或不可辨识参数冒充实测模型 | 原 `pressure_rs485_gui.py` 是单页工作流，保留称重/砝码标定，H743手动与扫描直接替代旧ESP控制；手动/扫描与既有PROPCAL逐通道点转共用连接且互斥；用户观察后选择上下桨及俯视旋向，整表确认后提交RAM，显式保存；停止代次屏障禁止迟到结果恢复输出；`thrust_bench.records` v2明确电转速域与DShot电流来源，板ADC仅诊断，不需极对数、不混入旧机械RPM；采集保留来源时刻/有效性，默认原窗口使用自动补数（明确油门/换电电压/有界时间和点数；用户换电显式续采）→本地SQLite实验库→整轮训练/选择验证，记录与版本固定、连续实测V候选和仅eRPM基线比较；原电压分层模型保留兼容，模型离线且不自动改飞行推力表或 Flash；独立 TBENCH 窗口由固件执行互锁/超时，旧 PROPCAL 与 SYSID 职责保持 | 作者 2026-09-20 授权的 R-THRUST-2；[本批接口约定](thrust-bench-contract.md)。新模型接入飞行控制或替换物理标定需另行协调并取得对应实机证据 |
| AD-06 光杆辨识的油门归属与单摆模型 | 作者只负责解锁，程序在杆上自动升降油门，才能一键完成可重复的辨识；杆在质心上方时重力是已知力矩，可以不依赖固件力矩模型标定惯量 | `app_sysid` 只计算电机脉宽（`APP_SysId_GetMotorPulse`），实际写电调只在稳定环“已解锁 + 链路正常”分支；开始要求已解锁且油门杆最低；推杆、上锁、失联、停止或任一门限触发时当拍交还遥控器；目标合推力走分配器同一张推力表，按最高油门 % 封顶；辨识占用期间不清控制调度器；断点只按真实丢样判定；杆到质心距离 d 为尺量输入，拟合给出力矩模型系数 κ，整定用 I_质心/κ；只写 RAM、不自动保存 Flash。阶段时长、门限和默认激励可调 | 作者 2026-09-26 授权（“解锁交给我，我解锁之后你可以操控油门进行辨识”）；[操作说明](system-identification.md)。改为自由飞行辨识或放宽交还条件，须作者另行授权 |

主控派单时附相关 AD 编号、文档基线提交与接口约定，执行者只读相关条款。后续任务先继承，再在允许范围内修改。
新增重要决定由主控在此登记“决定/理由/边界/重新讨论条件”，注明授权来源或采用依据；未确认方案标作“提议”，不能强加给下游。
更改已采纳决定先说明实际问题、替代方案及受影响调用方，由主控在权限内协调；推翻作者确认过的设计或硬边界仍由作者决定。
采纳替代方案时更新当前条款并标明替代关系；旧理由保留在 Git 或明确标作已替代的历史记录。通知受影响 Agent 对齐，不能让旧约定的交付直接合并。
相关接口与关键行为用必要测试保护，修改这些保护须一并审查设计变化；不为本表措辞增设测试，也不以“测试绿”替代职责检查。

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
  → app_control_scheduler（500/250/100/50 Hz，真实时间戳与导航新样本门）
  → drv_position_control（位置 P → 速度 PID）
  → drv_coax_ctrl（合力、重力与期望姿态）
  → drv_attitude_control（SO(3) 姿态 P → ω_sp）
  → drv_rate_control（角速度 PID → M_cmd）
  → drv_coax_ctrl（同轴分配、M_achieved 与逐方向饱和反馈）
  → 电机 PWM
  → 舵机输出类型选择：BUS 或 PWM
```

慢环未到期时保持上次目标，不重复积分；位置/速度环只在新的导航样本到达后推进。
速度 D 使用滤波后的测量加速度，角速度 D 使用滤波后的测量角加速度。积分器由解锁、
链路、IMU/导航有效性、模式切换和实际执行器饱和共同管理。控制器内部 legacy 适配仍在
`drv_coax_ctrl` 的具名边界，R-S5-1 不改变 FLU 迁移掩码或执行器极性。

时间域保持分离：HAL tick 负责 RC、IMU、光流等既有新鲜度比较；TIM17 微秒时间只计算
控制器真实 dt。导航样本时刻作为不透明更新 token 进入调度器，不与 TIM17 比较；各环
只有在 App 实际执行控制拍时才提交调度状态。Z 速度 D 的测量加速度来自同一 Z-down
测距速度的变步长差分。`M_achieved` 使用最终电机分配推力和机械脉宽反解后的倾角计算，
因此包含电机与舵机最终裁剪，而非限位前请求值。

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
级联参数由 `app_control_config_store` 以 CFG V19 保存，并继续读取 V18/V17/V16/V15；
旧参数先按原物理语义换算，不能把旧数值直接套入新量纲。

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

FlightLog V10 记录真实级联中间量、分配实现力矩和饱和方向；V7/V8/V9 仍按各自
`record_size` 与参数布局解析，历史文件的 frame provenance 不被重解释。

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

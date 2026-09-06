# R-F6 工单：FLU 运行时迁移（六 seam 收口）

类别模式：`frame-migration`（先读
[`.agents/skills/drone-h743-project/references/modes/frame-migration.md`](../.agents/skills/drone-h743-project/references/modes/frame-migration.md)
与 [`flu-coordinate-contract.md`](../.agents/skills/drone-h743-project/references/flu-coordinate-contract.md)）。

## 0. 这张工单要解决什么

飞控当前**无法解锁**。实机读数（2026-09-05，数传链路）：

```
IMUFRAME  active=-x,-y,+z active_code=3 persisted_code=3
          frame=canonical_flu_persisted   arm_lock=1
IMUCAL    arm_lock=0
SERVOCAL  arm_lock=0
RC norm   throttle=-1000 thr01=0  arm=+1000     ← 拨杆和油门都对
RC link   fresh=1 lq=100 age_ms=1  armed=0
```

原因不是遥控、不是 IMU 健康、不是拨杆边沿，而是
[`app_stabilizer.c:926`](../App/Src/app_stabilizer.c#L926) 的半迁移互锁：

```c
(APP_Sensor_IsFluOrientationActive() != 0U) && (DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U)
```

seam 0（传感器）已经持久化成 FLU（朝向码 3），seam 1~5 仍是 legacy 口径。
前后不一致起飞即翻机，固件因此物理禁飞。

**R-F0~F5 交付的是契约与证据基线，不是运行时迁移**（PIPELINE 那一行的括号
写明了这点，证据栏还特地注了"运行时掩码仍为 0x00，不得解读成全链 FLU"）。
本工单要做的是 R-F6：把剩下的运行时表述真正迁移过去，逐位置
`DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK`，最终 `0x00 → 0x3F`。

**作者决策（2026-09-05）：不退回 legacy 朝向。目标是归零到规范坐标系之后再试飞。**

## 1. 现状盘点

| 位 | seam | 现状 | 测试 | 剩余工作 |
|---|---|---|---|---|
| bit0 `SENSOR_TO_FLU` | 传感器 | **已迁移**，朝向码 3 已持久化 | `test_flu_seam0_sensor_frame.py` | 仅置位 |
| bit1 `ESTIMATOR_ADAPTER` | 估计器适配 | 无运行时改动（作者 2026-08-30 裁定 option C） | `test_flu_seam1_estimator_frame.py` | 仅置位 |
| bit2 `NAVIGATION` | 导航 | **口径未成文，无测试** | ❌ 缺失 | R-F6-1 |
| bit3 `CONTROLLER` | 控制器 | 内部仍 X前/Y右/Z下，适配藏在驱动内部常量里 | `test_flu_seam3_controller_frame.py`（钉现状） | R-F6-2 |
| bit4 `RC_ACTUATOR` | RC/执行器 | 横向 body-right-positive，跟随控制器 | `test_flu_seam4_rc_actuator_frame.py`（钉现状） | R-F6-3 |
| bit5 `TELEMETRY_LOG` | 遥测/日志 | 遥测出口已适配为 FLU；FlightLog 与回放几何仍 legacy | `test_flu_seam5_telemetry_frame.py`（钉残余） | R-F6-4 |

仍在运行的 legacy 适配器（迁移完成的标志是它们**被删掉**，不是被改值）：

- [`drv_coax_ctrl.c:42-47`](../Driver/Src/drv_coax_ctrl.c#L42-L47)
  `FORCE_FRAME_ROLL_SIGN=-1` / `FORCE_FRAME_PITCH_SIGN=+1` / `RATE_FRAME_*=+1`
- [`app_stabilizer.c:1606`](../App/Src/app_stabilizer.c#L1606) `velocity_z_m_s = -range_velocity_m_s`
- [`app_stabilizer.c:1662`](../App/Src/app_stabilizer.c#L1662) `attitude.z_m = -relative_height_m`
- [`app_telem_port.c:117-120`](../App/Src/app_telem_port.c#L117-L120) 每帧 `DRV_FRAME_FrdToFlu()`

## 2. 子工单

每张按"架构三件套"派发。**一次一个 seam，不许合并提交。**

---

### R-F6-0 收口 seam 0 / seam 1 掩码位

前置：审核者先复核这两份测试是否确实满足置位条件（两份 docstring 都自称是对应
bit 的 executable evidence）。复核不通过则本子工单退回，不得自行置位。

- **Contract**：只改 `Driver/Inc/drv_frame_contract.h` 的
  `DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK`（`0U` → `0x03U`），同步更新
  `flu-coordinate-contract.md` 的 *Current Migration Status* 段。
- **Boundary**：禁止改任何运行时代码；禁止碰 arm lock 判定；禁止动 seam 2~5。
- **Test Seam**：`test_flu_seam0_*` + `test_flu_seam1_*` + `test_flu_frame_contract.py`。
  另在 `test_flu_frame_contract.py` 追加一条机检：`DONE_MASK` 里每个置位的 bit
  都必须存在对应的 `tests/test_flu_seam<N>_*.py`。

⚠️ 置这两位**不会解锁**（`COMPLETE` 要求 `== 0x3F`）。这是记账，不是进度。

---

### R-F6-1 seam 2 导航口径成文 + 可执行测试

当前导航层完全没有坐标系文档，也没有 seam 测试——六个 seam 里唯一的空白。

- **Contract**：给 `Services/Inc/svc_flow_nav.h`、`Driver/Inc/drv_nav_ekf.h`、
  `App/Inc/app_nav_estimator.h` 的每个公开接口补齐**坐标系 / 单位 / 符号 /
  时间戳来源**；新建 `tests/test_flu_seam2_navigation_frame.py`。
  必须显式命名导航/世界坐标系（不许从机体系推断 NED/ENU）。
- **Boundary**：本子工单**只钉现状、只命名，不改数值行为**（与 seam 1 的
  option C 同形）。禁止顺手改高度符号——那是 R-F6-2 的事。禁止引入新 IPC。
- **Test Seam**：宿主 gcc 直接编真实 `svc_flow_nav.c` / `drv_nav_ekf.c`；
  判据数据取 `data/` 里已有的光流+测距实录，不许自造输入下算法结论。

---

### R-F6-2 seam 3 控制器内部表述迁移到 FLU ★核心

- **Contract**：`DRV_COAX_CTRL_Reference` 的 `x_m/y_m/z_m`、
  `vx_m_s/vy_m_s/vz_m_s` 改为规范 FLU（+X前 / +Y左 / +Z上）；
  删除 `FORCE_FRAME_ROLL_SIGN` / `FORCE_FRAME_PITCH_SIGN` /
  `RATE_FRAME_ROLL_SIGN` / `RATE_FRAME_PITCH_SIGN` 四个常量；
  调用侧 `app_stabilizer.c` 的两处 Z 取反随之删除；
  同步改 `drv_coax_ctrl.h` 里那段声明"X forward, Y right"的注释块。
- **Boundary**：**禁止用增益抵消符号**（本类模式硬规矩 2）。分配器与舵机极性
  属 seam 4，本子工单不动。CFG 持久化格式若必须变，单独提 REQ 并保证向后兼容。
- **Test Seam**：`test_flu_seam3_*` 从"钉现状"改成"钉迁移后语义"；
  `test_cascade_controller_contract.py` / `test_coax_sign_convention.py` /
  `test_coax_yaw_so3_contract.py` 同步；**追加迁移前后逐样本对拍**。
- 交付**软件证据即可，不置位**。bit3 由 R-F6-5 的实机证据触发。

> ⚠ **本节已被重发工单取代，以那份为准：**
> [`req-rf6-2-controller-flu-migration.md`](req-rf6-2-controller-flu-migration.md)
>
> 上面这条 Test Seam 原本写的是"力矩输出应在浮点容差内一致"，**已于 2026-09-06
> 被实测证伪**：四个符号常量不是惰性的（翻 `FORCE_FRAME_ROLL_SIGN` 会让 beta 变
> 67.5 mrad，对照舵机角容差 0.8 mrad），照原判据做不出来。重发工单把判据改成
> "同一**物理**姿态下舵机指令一致"，并已验证可精确达成（0.000 mrad）。
> 重发工单还带了核心矩阵重导的结论、以及一条**必须先由持机会话定性**的前置
> （光流 Y 到底是机体左还是右）。

---

### R-F6-3 seam 4 RC / 执行器跟随

依赖 R-F6-2 合入。

- **Contract**：RC 横向意图由 body-right-positive 改为 body-left-positive，与
  迁移后的控制器一致；舵机极性继续走运行时 `ServoCalibration`
  （`pulse_sign` / `center_us` / `min_us` / `max_us`），**不许回退成编译期符号宏**。
- **Boundary**：禁止改 RCMAP 的 Flash ABI；禁止改 failsafe 行为；禁止改
  `app_control.c`（只减不增）。
- **Test Seam**：`test_flu_seam4_*` 更新；新增一条端到端 host 测试，从"摇杆
  往左打"一路断言到"期望力矩符号"，中间不许有测试替身。

---

### R-F6-4 seam 5 残余：FlightLog 与回放几何

- **Contract**：FlightLog 记录头写入 `DRV_FRAME_CONTRACT_VERSION` + 显式
  frame 标识；`tools/flight_log_rerun_replay.py` 的 `rpy_body_to_local_down()`
  换成 FLU 几何，并**按记录里的 frame 字段分派**——历史 FRD 记录仍按 FRD 回放。
- **Boundary**：**绝不重解释历史数据**（本类模式硬规矩 6）。
  `tools/drone_tcp_panel.py` 只减不增。
- **Test Seam**：`test_flu_seam5_*` 更新；取 `data/` 里一份迁移前的 FRD 记录
  和一份迁移后的 FLU 记录，断言各自回放几何正确且互不串用。

---

### R-F6-5〔机〕拆桨物理方向验证 + 一次性置位 0x3F

**归审核者（持实机），不归执行工程师。** 这是真正解锁的那一步。

按 FLU 契约的 *V2 end-to-end actuation*，**拆桨**下逐项确认物理方向：
导航变换 → 控制器误差符号 → RC 意图 → 舵机/电机极性 → 混控响应 → failsafe；
通过后做系留通电测试。全部匹配才把 `DONE_MASK` 一次置到 `0x3FU`。

置位后 `IsImuFrameArmLocked()` 自然返回 0，飞控恢复可解锁。

## 3. 派发顺序与门

```
R-F6-0 ──┐
R-F6-1 ──┼─→ R-F6-2 ─→ R-F6-3 ─→ R-F6-4 ─→ R-F6-5〔机〕─→ 掩码 0x3F ─→ 可解锁
         │   (核心)                                    ↑
         └── 可与 R-F6-1 并行                    M5 / M6 门
```

**需要作者拍板的两件事**（PIPELINE 现状与本工单的目标有冲突，不能由我代决）：

1. PIPELINE 上 R-F6 挂着"等待 **M6** 与作者批准的迁移方案"，而 M5（光流与测距
   校准）证据栏写着"缺实机采样与判定"，尚未完成。**R-F6-1 的判据数据依赖 M5
   的实录。** 是先补 M5，还是允许 R-F6-1 用现有实录先做？
2. **M7（带桨动力、振动与首飞）目前是冻结状态**，其前置写着"M0～M6 全部完成；
   运行时坐标迁移完成；另行批准带桨测试方案"。R-F6-5 只解开解锁互锁，**不等于
   解冻 M7**。带桨试飞仍需你单独批准测试方案。

## 4. 顺带发现的缺陷（独立于本工单，建议单独修）

`stabilizer_publish_led()` 的 LED 原因链
（[`app_stabilizer.c:1525-1545`](../App/Src/app_stabilizer.c#L1525-L1545)）
**没有 frame arm lock 这一档**。arm lock 把 `rc_armed` 强制清零后，判定落到
`APP_LED_ARM_BLOCK_ARM_SWITCH`（LED_3 闪 3 下），把"坐标迁移未完成"误报成
"解锁拨杆没打"。

讽刺的是同一段里 IMU 健康那一档旁边就写着注释：*"必须排在 rc_armed 判定之前：
采样链失效会把 rc_armed 强制清零，否则这里会误报成'拨杆没打'，把真正原因藏
起来。"* ——作者防住了那一个，漏了这一个。修法同源：在 `rc_armed == 0` 判定
之前插入 frame arm lock 分支，并给 `APP_LED_ArmBlockReason` 增加一个原因码。

走修 bug 模式，单独 `fix(...)` 提交、单独一行证据，不要混进 R-F6 任何一张子工单。

# R-F6-2〔重发〕seam 3 控制器内部表述迁移到 FLU

类别模式：`frame-migration`（先读
[`modes/frame-migration.md`](../.agents/skills/drone-h743-project/references/modes/frame-migration.md)
与 [`flu-coordinate-contract.md`](../.agents/skills/drone-h743-project/references/flu-coordinate-contract.md)）。
父工单：[`req-rf6-flu-runtime-migration.md`](req-rf6-flu-runtime-migration.md)。

## 0. 为什么重发这张

原工单的验收判据是：

> 用 `data/` 实录喂进新旧两版控制器，力矩输出应在浮点容差内一致
>（纯表述变更不该改变数值，对不上就是掺了别的改动）

**括号里的意图完全正确，方法错了，照做必然做不出来。**

判据背后的假设是"这四个符号常量是惰性的"——这来自仓库里被复述了五遍的一句话：
*"此符号同时作用于实测姿态和目标姿态，因此在姿态误差中相消"*。**那句话是错的。**

姿态误差走 SO(3) 的 `e_R = 0.5*vee(R_d^T R_a − R_a^T R_d)`，是矩阵乘出来的非线性量。
把 `Rz(ψ)Ry(θ)Rx(φ)` 里 Rx 一个因子的参数反号**不是相似变换**（三个角同时反号才
是），所以它不会在误差里掉出去，还会串到 pitch/yaw 通道。

实测（2026-09-06，编译两份真 `drv_coax_ctrl.c`、只差 `FORCE_FRAME_ROLL_SIGN` 一个
常量，625 组直接姿态模式 + 悬停推力逐样本对拍）：

| | 差异 | 对照 |
|---|---|---|
| beta | **67.5 mrad** | 舵机角容差 `SERVO_ANGLE_TOL_RAD` = 0.8 mrad |
| alpha | **20.4 mrad** | 519/625 组输出符号翻转 |

所以"删掉常量而力矩不变"是不可能事件。判据必须改。

## 1. 新判据：同一**物理**姿态下舵机指令一致

迁移不是重构，是**换标**——同一个数字在新旧两版里指的是不同的物理量。拿同一份
实录直接喂两版，比的是两个不同的物理状态，当然对不上，而且**应该**对不上。

正确的做法：把实录按换标关系折算之后再喂。

```
旧版实录 ──(按下表折算)──> 新版输入
                                   两边跑完，舵机指令应逐样本相等
旧版实录 ─────────────────> 旧版输入
```

**已验证这条判据可精确达成**：拿 `FORCE_FRAME_ROLL_SIGN` 单独做实验（−1 → +1，
等价于 roll 表述反向），

```
旧判据（两版喂同一份数字）      最大差  67.544 mrad  → 对不上
新判据（两版喂同一个物理姿态）  最大差   0.000 mrad  → 精确相等
```

不是"放宽到某个容差"，是**精确为零**。对不上就真的是掺了别的改动——原工单那句
括号的意图，到这里才真正立得住。

### 换标表

| 字段 | 旧（legacy local-level） | 新（规范 FLU） | 折算 |
|---|---|---|---|
| `Reference` / `AttitudeInput` 的 `x_m`、`vx_m_s`、`ax_m_s2` | 前 | 前 | **不变** |
| 同上的 `y_m`、`vy_m_s`、`ay_m_s2` | **右**正 | **左**正 | **取反** |
| 同上的 `z_m`、`vz_m_s`、`az_m_s2` | **下**正 | **上**正 | **取反** |
| `roll_rad` / `pitch_rad` / `yaw_rad` | 已是规范 FLU | 同 | **不变** |
| `gyro_*_rad_s` | 已是规范 FLU | 同 | **不变** |
| 输出 `alpha_rad` / `beta_rad` / 电机推力 | 物理量 | 物理量 | **必须逐样本相等** |

姿态与角速率**本来就已经是 FLU**（seam 0/1 已迁，见
[`drv_coax_ctrl.h`](../Driver/Inc/drv_coax_ctrl.h) 的 seam 3 frame map），
所以这次动的只有平动那六个字段和它们的调用侧。

## 2. 已经替你做完的部分，别重做

核心矩阵重导由持机会话在 2026-09-06 完成，结论落在
[`tests/test_flu_seam3_force_frame_derivation.py`](../tests/test_flu_seam3_force_frame_derivation.py)
（4 条，都是真编译真控制器，不是文本断言）。三条结论：

1. **符号常量是实的**，不是惰性适配器（上表）。
2. **内环问不出口径**。姿态误差对实测与目标一视同仁，所以无论口径怎么变它都自洽。
   想验证口径，只能看外环。
3. **外环被物理钉死**——目标姿态由加速度指令经力矢量算出（`atan2(F_前, F_上)`），
   加速度指令是物理量。实测控制器认哪个姿态叫"到位"：

   ```
   向前加速（物理上要机头下俯）→ 最安静在 pitch = +0.19 → FLU 机头下俯  ✓ 自洽
   向右加速（物理上要右翼下沉）→ 最安静在 roll  = −0.20 → FLU 左翼下沉  ✗ 矛盾
   ```

   pitch 自洽。**roll 与"local Y 是机体右"这条全链注释矛盾。**

## 3. 开工前置：roll 那条矛盾必须先定性〔机，不归你〕

第 2 节第 3 条的 roll 矛盾有两种可能，**主机判不了**：

- **(A)** local Y 真是机体右（四处注释都这么写），那么外环 roll 反了 —— 是真缺陷；
- **(B)** local Y 其实是机体左，四处注释都写错了 —— 那么闭环自洽，只是文档错。

四处注释分别在 `drv_coax_ctrl.h`、`drv_coax_ctrl.c`、
`app_stabilizer.c:106`（`STABILIZER_VELOCITY_MEAS_Y_SIGN` 那行）、
以及 R-F6-1 刚补的 `svc_flow_nav.h`。它们**都只是注释**，不构成证据：光流 Y 的物理
方向从没被实测钉过（R-F6-1 因此正确地没有置 bit2）。

判定方法：**拆桨、通电、把机体沿地面向右平移**，读遥测里的 `vy`。一次两分钟。
归持机会话（审核者），做完把结论写进本节。

**这条不定性，本工单不许开工**——(A) 和 (B) 会导出相反的 `y` 折算方向，
在错的方向上做完对拍，只会得到一个自洽但反向的控制器，而且对拍全绿。

### 定性结论（2026-09-06，持机会话）

**(B) 成立**：拆桨、通电，把机体沿地面向右平移，`FLOW?` 读回的原始 `flow_vy`
（未经任何软件符号处理，`APP_OpticalFlow` 原样透传）为**负**。即向右平移时
原始光流 Y 为负 → 光流传感器的原始 Y 轴方向本来就是**左正**，而非四处注释
声称的"右正"。此结论与第 2 节的外环实测互相独证：若 local Y 本来就是左正，
"向右加速"这一说法本身就是误标——`ay_m_s2=+2` 实际命令的是"向左加速"，
其物理正确响应正是左翼下沉（FLU roll 为负），与实测完全吻合，不存在矛盾。

**结论**：local Y（`Reference`/`AttitudeInput` 的 `y_m`/`vy_m_s`/`ay_m_s2`，
以及 `svc_flow_nav`/`app_stabilizer` 里所有喂给它们的量）从传感器原始读数
起就一直是**左正**，从未真的是"右正"。四处（`drv_coax_ctrl.h`、
`drv_coax_ctrl.c`、`app_stabilizer.c:106`、`svc_flow_nav.h`）以及新发现的
第五处（`drv_position_control.h:18`）声称"Y 右正"的注释**全部是错的**，
运行时数值本身不需要为 Y 做任何改动。第 4 节 Contract 与第 1 节换标表中
"y_m/vy_m_s/ay_m_s2：右正→左正，取反" 一律按本节结论**反转为"不变，仅改
注释"**。`STABILIZER_VELOCITY_MEAS_Y_SIGN` 保持 `+1`，不改为 `-1`。

不是 FLU 迁移引入的问题，也不是缺陷，是五处文档注释与实际物理方向不符，
本工单一并改正措辞、不改数值。

> ⚠ 这条差异**不是 FLU 迁移引入的**：roll 口径在 FRD 与 FLU 下同为"右翼下沉为正"。
> 它是既有问题，R-F6-2 也不会顺手修好它。若定性为 (A)，单独立 REQ 修，不要混进来。

## 4. Contract

- `DRV_COAX_CTRL_Reference` 与 `DRV_COAX_CTRL_AttitudeInput` 的
  `x_m/y_m/z_m`、`vx_m_s/vy_m_s/vz_m_s`、`ax_m_s2/ay_m_s2/az_m_s2`
  改为规范 FLU（+X 前 / +Y 左 / +Z 上）。
  **`AttitudeInput` 必须与 `Reference` 同步迁移**——两者喂进同一个误差计算
  （`coax_ctrl_compute_accel_cmd`），只改一边算出来的误差物理上不自洽。
- 删除 `FORCE_FRAME_ROLL_SIGN` / `FORCE_FRAME_PITCH_SIGN` /
  `RATE_FRAME_ROLL_SIGN` / `RATE_FRAME_PITCH_SIGN` 四个常量，并**重导**
  `coax_ctrl_attitude_matrix` / `coax_ctrl_local_down_to_body` /
  `target_roll/pitch` 三处的几何，使其直接吃规范 FLU。
- 调用侧同步（[`app_stabilizer.c:1666-1675`](../App/Src/app_stabilizer.c#L1666-L1675)）：
  - `attitude.z_m = -relative_height_m` → 去掉取反；
  - `attitude.vz_m_s = -range_velocity_m_s` → 去掉取反；
  - `STABILIZER_VELOCITY_MEAS_Y_SIGN` 由 `+1` 改为 **`−1`**（seam 2 仍是 legacy
    右正，这个常量就是 seam2→seam3 的适配器；**留着它、改它的值**，不要删）。
    ——若第 3 节定性为 (B)，此项相反，以第 3 节结论为准。
- 同步改 `drv_coax_ctrl.h` 里那段 seam 3 frame map 注释与本文件的换标表。

## 5. Boundary

- **禁止用增益抵消符号**（本类模式硬规矩 2）。增益一律保持非负。
- **不许碰解锁互锁**：`APP_Stabilizer_IsImuFrameArmLocked()` 与
  `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` 的判定式一个字不动。
  半迁移期间全靠它兜底，它不是碍事的东西。
- **不置位**。`DONE_MASK` 保持 `0x03`，bit3 由 R-F6-5 的实机证据触发。
- 分配器与舵机极性属 seam 4（R-F6-3），本工单不动。
- 不许在控制律里加"按 `APP_Sensor_IsFluOrientationActive()` 切口径"的分支——
  运行时分叉会让两种朝向都缺乏证据。现有机检会拦（第 6 节）。
- `App/Src/app_control.c` 与 `tools/drone_tcp_panel.py` 只减不增。
- CFG 持久化格式若必须变，单独提 REQ 并保证向后兼容。

## 6. Test Seam

- `tests/test_flu_seam3_controller_frame.py`：从"钉现状"改成"钉迁移后语义"。
- `tests/test_flu_seam3_force_frame_derivation.py`：4 条都会因本次改动而失效，
  **必须连同论证一起更新，不许直接删**。其中
  `test_the_cancellation_claim_is_not_reasserted` 是防复发守卫（那句错话曾在五处
  出现），**这条要留着**。
- `tests/test_coax_sign_convention.py` / `test_coax_yaw_so3_contract.py` /
  `test_cascade_controller_contract.py` 同步。
- **新增迁移前后逐样本对拍**，按第 1 节的换标表折算输入，判据是**精确相等**
  （浮点容差取 `SERVO_ANGLE_TOL_RAD = 8e-4 rad` 的百分之一即可，实测能到 0）。
  数据取 `data/` 实录，不许自造输入下算法结论。

## 7. 顺带必须查的一条：yaw

`+yaw` 在 FLU 是**机头左转**，在 legacy FRD 是**机头右转**——和 pitch 一样，
seam 0/1 迁移时翻了，而控制器里**没有对应的符号常量**。
内环自洽（实测与目标同源），但 `coax_ctrl_local_down_to_body` 用 `psi` 把世界系
力矢量转到机体系，那里 yaw 是和物理咬合的。

用第 2 节同一套方法查：给一个固定的世界系水平加速度指令，扫 `att.yaw_rad`，
看目标姿态随偏航的旋转方向对不对。**查出来是什么就写什么，不要顺手改。**

### 查证结果（2026-09-06）

`att.yaw_rad` 只在两处出现：

1. `coax_ctrl_attitude_matrix`（`actual_r`）与 `desired_body_r`
   （`reference->yaw_rad`）——姿态误差的真实路径。实测（悬停推力 + 直接姿态
   目标，`reference.yaw_rad` 钉死为 0，扫 `attitude.yaw_rad`）：yaw 误差为正
   （测得比目标更偏"机头左转"）时，`moment_cmd_n_m[2]` 为**负**；yaw 误差为
   负时为正——始终把偏航拉回目标。FLU 里 +yaw 是机头左转，"测得更左转"要拉回
   就必须朝右转，对应 +Z 轴负向的力矩——负号自洽，且这里**没有任何符号常量**
   （`drv_coax_ctrl.c` 里搜不到"YAW_SIGN"），是姿态融合早在 seam 0/1 就已经把
   yaw 迁到 FLU 的自然结果。见
   `tests/test_flu_seam3_force_frame_derivation.py::test_yaw_moment_restores_toward_target_the_flu_consistent_way`。
2. `coax_ctrl_local_down_to_body` 的 `psi`——但这条路径的输出
   `desired_force_local_n`（旋转后即 `desired_force_body_n`）**只喂
   `DRV_COAX_CTRL_Debug.force_cmd_n`**（遥测），不参与 `target_roll_rad`/
   `target_pitch_rad` 的计算——那两个量直接从未经偏航旋转的
   `desired_force_local_n` 算 `atan2`。也就是说外环把"world X/Y"当成"当前
   航向下的 X/Y"，并不随测得的偏航重新旋转成真正随偏航稳定的世界系。

**这是本工单之前就存在的架构特征，不是 R-F6-2 引入或应当顺手修的缺陷**——
按本节要求，只记录，不改动。

## 8. 交付

按 `AGENTS.md` 完成协议：状态改**待审核**（执行者无权置 ✅），PIPELINE 证据表追加
一行，交付变更说明 + 测试输出原文 + 证据文件路径。措辞禁止"校准完成 / 验收通过 /
可以飞"。

本工单只交付**软件证据**。物理方向验证与掩码置位归 R-F6-5〔机〕。

> **改这四个常量会让存档的 `ServoCalibration` 极性失效。** 那份记录是在旧符号链下
> 实测出来的，`pulse_sign[]` 已经把旧符号吸收进去了。迁移合入后必须重新拆桨实测
> 舵机方向——这也是绝对物理方向主机上判不了的原因：最后一环不在源码里。

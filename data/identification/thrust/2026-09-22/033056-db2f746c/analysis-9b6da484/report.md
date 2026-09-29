# 共轴双桨实际安装状态离线模型报告

> 两片桨均保持安装。本报告拟合称重得到的总推力；未驱动桨可能被动风车，因此不能把任何系数解释为孤立单桨推力或扭矩。

## 数据身份与适用范围

- 样本 schema：`2`
- 速度域：`electrical_erpm`（不做极对数换算）
- 固件/配置身份：`未提供`
- 两桨安装：`未提供`
- 未驱动桨状态：`未知；可能静止或风车`
- 稳态工作点：18
- 上桨电转速范围：[7455.111111111111, 29820.666666666668] eRPM
- 下桨电转速范围：[7316.0, 29481.88888888889] eRPM
- 电压层：[11.5, 12.3] V

## 总推力静态模型

- **双驱动模型不可用**；原因：至少一个电压层内部的 eRPM 模型不可辨识（`voltage_layer_fit_unavailable`）。
- 这不影响下方各自满足条件的 upper/lower 单驱动安装状态模型。
- 双驱动需要至少两个稳定且分离的真实带载电压层，并在相邻层具备有面积的 eRPM 凸包交集。

- 本次电压层默认/配置判据：`{'max_span_v': 0.5, 'min_adjacent_gap_v': 0.1}`。标签只用于分组，稳定性和分离性按真实带载电压范围判断。
- 分组策略：`{'labeled_samples': 'group_by_voltage_layer_v_but_measure_center_and_range_from_loaded_voltage', 'unlabeled_samples': 'group_by_run_id', 'continuous_discharge_auto_binning': False}`。未标层按 run 分组，不自动把连续放电曲线分箱。

### eRPM-only 基线

- 不可用：两路 eRPM 没有独立变化（`inputs_not_independently_varied`）。

### 单驱动安装状态模型

- upper：unavailable；原因 真实带载电压层少于两个（`fewer_than_2_measured_voltage_layers`）。
- lower：unavailable；原因 真实带载电压层少于两个（`fewer_than_2_measured_voltage_layers`）。
- 共保留 0 个单驱动工作点。结果是两片桨均安装时的总测力，未驱动桨可能风车，不能称为孤立单桨推力。

### 数据质量

- 输入样本：230
- 接受的稳态点：18
- 排除原因：`{'dual_undriven_zero_erpm_boundary': 43, 'scale_not_synchronised': 2}`

### 使用方法

- 调用 `predict_thrust(analysis, upper_erpm=..., lower_erpm=..., voltage_v=..., mode=...)`。
- eRPM 必须直接来自有效的双向 DShot 回传；不除极对数，不使用 KV 估算兜底。
- `dual` 需要两路实测 eRPM；`upper/lower` 只要求主动路实测 eRPM。
- 输入超出共同 eRPM 覆盖或相邻实测电压层中心范围时函数会拒绝，不做外推。

## 输入功耗

- 总状态：`unavailable`；来源：`dshot_esc_current`。
- 上路电流模型：`unavailable`；`I_upper=f(eRPM_upper,eRPM_lower,V_loaded)`。
- 下路电流模型：`unavailable`；`I_lower=f(eRPM_upper,eRPM_lower,V_loaded)`。
- 同步组合功耗模型：`unavailable`；`P=V_loaded×(I_upper+I_lower)`，只有两路电流与电压均新鲜且对齐才形成样本。
- 当前源分辨率：1.0 A（TBENCH v1 whole-amp ESC current fields）；DShot ESC 电流外部校验：`False`。未外校验不阻断观察模型，但不宣称正式安培标定。
- 分路预测调用 `predict_current(analysis, upper_erpm=..., lower_erpm=..., voltage_v=..., role='upper'|'lower')`；覆盖范围外拒绝外推。
- 板总 ADC 电流只作诊断，不参与拟合，也不在 DShot 电流缺失时回退。

- dual 扫描：upper I `unavailable`，lower I `unavailable`，组合功耗 `unavailable`。
- upper 扫描：upper I `unavailable`，lower I `unavailable`，组合功耗 `unavailable`。
- lower 扫描：upper I `unavailable`，lower I `unavailable`，组合功耗 `unavailable`。

## 升降速动态

- 状态：unavailable；原因：`no_dynamic_segments`
- 拒绝项：`[]`

## 独立验证

- 状态：training_only；策略：`whole_run_and_voltage_layer_holdout`
- 可用留出预测点：0；指标：`None`
- 按实际带载电压分组误差：`[]`
- 不可用留出组：`[{'group': 'run', 'value': '072b10edce6c', 'reason': 'fewer_than_2_measured_voltage_layers', 'test_points': 11, 'explanation_zh': '该留出组无法由剩余训练组建立并预测。'}, {'group': 'run', 'value': '14309671afe4', 'reason': 'fewer_than_2_measured_voltage_layers', 'test_points': 7, 'explanation_zh': '该留出组无法由剩余训练组建立并预测。'}]`
- 同一 segment 的相邻样本不会随机拆到训练集和验证集。独立 run/电压层不足时，只报告训练拟合，不能宣称泛化。

## 数据充分性

- 结果：`{'status': 'insufficient', 'summary': '双驱动聚合稳态点 18，每个电压组最少 4 个唯一非零指令组合，实际电压覆盖组 2；整轮留出可用/拒绝 0/2，中间电压层留出可用 0；仍有 4 项缺口。', 'reference': {'kind': 'starting_reference_only', 'advisory_only': True, 'recommended_unique_nonzero_command_combinations_per_voltage_group': 25, 'recommended_actual_voltage_coverage_count': 3, 'recommended_independent_validation_rounds': 1, 'rmse_full_scale_fraction': 0.05, 'p95_full_scale_fraction': 0.1, 'does_not_authorize_flight': True}, 'aggregated_steady_points': 18, 'nonzero_steady_points': 18, 'command_grid': {'status': 'available', 'meaning': 'Exact recorded command grid counts; not unique eRPM operating points.', 'groups': [{'mode': 'dual', 'voltage_group': 'layer:11.5', 'steady_points': 11, 'missing_command_points': 0, 'unique_command_combinations': 6, 'unique_nonzero_command_combinations': 6, 'command_combinations': [[5.0, 5.0], [10.0, 10.0], [15.0, 15.0], [20.0, 20.0], [25.0, 25.0], [30.0, 30.0]]}, {'mode': 'dual', 'voltage_group': 'layer:12.3', 'steady_points': 7, 'missing_command_points': 0, 'unique_command_combinations': 4, 'unique_nonzero_command_combinations': 4, 'command_combinations': [[5.0, 5.0], [10.0, 10.0], [15.0, 15.0], [20.0, 20.0]]}], 'missing_command_points': 0}, 'minimum_unique_nonzero_command_combinations_per_dual_voltage_group': 4, 'independent_runs': 2, 'actual_voltage_coverage': [{'label': 'layer:11.5', 'center_v': 12.327867424242426, 'range_v': [12.138, 12.473]}, {'label': 'layer:12.3', 'center_v': 12.385553571428572, 'range_v': [12.282, 12.462]}], 'input_independently_varied': False, 'adjacent_domain_intersections': None, 'holdout_available_groups': 0, 'full_scale_thrust_n': 3.0265209372384936, 'holdout_metrics': None, 'gaps': ['fewer_than_25_unique_nonzero_command_combinations_per_voltage_group_reference', 'fewer_than_3_actual_voltage_coverage_groups_reference', 'voltage_layer_fit_unavailable', 'no_independent_holdout_validation'], 'zero_or_not_spinning_points_counted_separately': 0, 'mode_point_counts': {'dual': 18}, 'modes_observed': ['dual']}`
- 已采模式及聚合稳态点：`{'dual': 18}`
- 指令网格：`{'status': 'available', 'meaning': 'Exact recorded command grid counts; not unique eRPM operating points.', 'groups': [{'mode': 'dual', 'voltage_group': 'layer:11.5', 'steady_points': 11, 'missing_command_points': 0, 'unique_command_combinations': 6, 'unique_nonzero_command_combinations': 6, 'command_combinations': [[5.0, 5.0], [10.0, 10.0], [15.0, 15.0], [20.0, 20.0], [25.0, 25.0], [30.0, 30.0]]}, {'mode': 'dual', 'voltage_group': 'layer:12.3', 'steady_points': 7, 'missing_command_points': 0, 'unique_command_combinations': 4, 'unique_nonzero_command_combinations': 4, 'command_combinations': [[5.0, 5.0], [10.0, 10.0], [15.0, 15.0], [20.0, 20.0]]}], 'missing_command_points': 0}`。该数量来自实际记录的 command 组合，不代表唯一 eRPM 工况；实际覆盖看 eRPM/V 图。
- 25 个非零有效组合 × 3 个实际电量覆盖 + 独立验证轮只是起步采集建议，不是硬数量门。
- raw 行数、聚合稳态点、唯一指令组合和独立验证轮是不同数量；零点/不转点另计。
- 默认 5% 满量程 RMSE、10% 满量程 P95 只是起步参考，不代表飞行放行。

### 电压层留出限制

- 整层留出只允许在剩余电压层之间插值，不外推；最低/最高边界层通常需用同层独立重复轮验证。
- 两个电压层不会执行整层留出；至少三个电压层才可能验证中间层。

## 待实测与使用边界

- 在计划使用的转速、电压和双桨组合范围内补齐二维覆盖；窄的同油门线不能辨识二维模型。
- 用独立 run 和独立电压层复测误差；确认称重滤波、采样新鲜度、丢帧与间断。
- 电池放电时建议交错/正反扫描、缩短一组，并跨电量重复相同有效组合；当前分层约束不表示任意连续放电曲线都能直接建模。
- DShot 电流未外部校验时只称遥测尺度观察模型，不称正式安培标定。
- 本报告不会自动下发模型、写 Flash 或替换 C 控制推力表。

## 警告

- This is a total-thrust model of the actual two-propeller installed assembly.
- Speed inputs are raw electrical eRPM; no pole-pair or mechanical-RPM conversion is applied.
- DShot ESC current is not externally calibrated and remains a telemetry-scale observation.
- Board ADC current is diagnostic only and is never used as a fitting fallback.
- It cannot uniquely decompose upper/lower thrust or torque; a nominally undriven propeller may windmill.
- Results are valid only inside the reported eRPM and voltage coverage.
- No model is automatically sent to the flight controller, written to Flash, or substituted for a control thrust table.
- Quantitative input-power calibration is unavailable: None
- Independent generalisation evidence is unavailable; training fit alone is not validation.
- The primary voltage-dependent thrust calibration is unavailable: voltage_layer_fit_unavailable

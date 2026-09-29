# 共轴推力台本批接口约定（R-THRUST-2）

作者 2026-09-20 在讨论后授权分工实现：保留上下桨安装，直接记录两路DShot电转速eRPM、分路ESC反馈电流、带载电压及RS485称重，建立安装状态下的总推力、功耗和动态模型。未经新授权不连接、烧录、复位或发送实机命令。交付最多待审核。

继承 AD-01/03/04。旧 SYSID 的 RC 电机归属、RAM 参数策略不变；旧 PROPCAL SPIN 单路 20%/300 ms 语义不变。新台架控制使用独立 `TBENCH` 窗口：未解锁、映射已标定、各既有执行器排斥门成立时才可开启，显式 `confirm=bench` 和本次 `max_pct`（1..100）必填；ARM 不产生转动；SET 两路原子更新；STOP 幂等；合法 SET 续命，300 ms 超时由 500 Hz 提交路径停机；错误/抢占关闭且不自动恢复。普通飞行与 SYSID 不获得新输出能力。不自动 SAVE。

## 在用推力模型

（先读，2026-09-24）飞控与台架验证**只用推力查补表**（`tools/thrust_bench/thrust_lut.py`；固件 `drv_thrust_lut` + `app_thrust_lut`，默认模式 PAIR）。当前版本以 `data/identification/thrust/models/lut/current.json` 为准（现为 `1281c31206bf`），固件表 `Driver/Src/drv_thrust_lut_table.inc` 由 `thrust_lut_export` 从它生成。`thrust_lut.latest()` 只是最新候选，`thrust_lut.current()` 才是飞控在用的表。

下列都不是现行飞控换算，不得用来生成飞控表、当作对比基准或新验证依据：
- `models/throttle/*` 多项式油门模型：已被查补表取代；`throttle_model.py` 仍提供数据选择函数。
- `models/<日期>/training-*` 与 `dataset_model.py`：实验库和“生成模型报告”用的整轮模型。
- `drv_coax_ctrl.c` 的 21 点旧曲线、`emit.py`：旧 ESP 台架流程，只在 `THRUSTLUT MODE LEGACY` 下生效。
- 机体参数 `max_total_thrust_g = 1595.342`：旧曲线末点，未按新表更新（作者自有参数）。

各版本与换表流程见 `data/identification/thrust/models/README.md`。

## 文件归属

本轮eRPM/DShot电流改动仅上位机；作者另一agent负责底层回读，固件和fc_protocol.py保持不动。

- 主控：本契约、records.py、整体文档/PIPELINE/索引、集成审查、最终回归。
- firmware 执行者：Driver/BSP DShot 遥测、独立 App 台架模块/命令与最小装配、公用协议 ID/CMake、tools/thrust_bench/fc_protocol.py、对应真实 C/协议测试。
- acquisition 执行者：tools/thrust_bench 的 plans.py/acquisition.py/session.py/connection.py/ui.py/driver.py/scale.py/__main__.py 与采集/UI 测试；不改 records.py/fc_protocol.py 或拟合模块。
- modelling 执行者：tools/thrust_bench/model.py/model_report.py 及模型测试；可修旧 fit.py/emit.py/sweep.py 中与分层/迟滞/导出证据直接相关的问题，保持已有接口。

## 采集与存档

唯一共享类型为 `tools.thrust_bench.records.FcSnapshot` 和 `BenchSample`；主机BenchSample和分析JSON使用v2电转速契约，FcSnapshot仍对接既有v1线协议。两路 DShot 元组按物理 ESC 通道，转换为上/下桨须依固件确认映射。直接使用eRPM，不配置极对数、不换算机械RPM。来源时钟不能直接相减；FC 32 位毫秒回绕须展开。每次查询包含 nonce，过期或代次错误回复不得配到新的指令。保留每个来源的原始时间/年龄、CRC/解析/超时事件以及连接代次；无效值 None，禁止填 0 或沿用旧值伪装新样本。

默认数传档的快照查询、回复、100 ms SET 心跳与 ACK 的完整双向带宽合计须小于 3456 B/s；USB 可选更高速并明确有效源更新率，动态分析不靠插值伪造采样率。固件回应一帧，最大 payload 200 B，加 $X 外层不超过 UART 256 B 实际缓存。新 function ID 为 `APP_PROTO_MSG_THRUST_BENCH=0x2235`，两端同时登记；上行仍是文本命令，不启用二进制解析器。

`fc_protocol.py` 提供 `decode_snapshot(payload: bytes) -> FcSnapshot` 和 `snapshot_command(nonce: int) -> str`（`TBENCH? <nonce>`）。v1 最终载荷为 68 B，末尾追加飞控全局最后接受的 `last_request_id`；计数在窗口关闭时保留。该模块提供 `arm_command(max_percent, *, request_id)`、`set_command(upper_percent, lower_percent, *, request_id, window_token)`、`stop_command()`。新连接先取得当前快照，从飞控序号继续；固件以32位串行号顺序拒绝跨窗口重放和倒序，不依赖只记住最近一个关闭token。ARM使用request_id作window_token，SET要求当前token且新序号；STOP无条件且不需要token。两端以真实 C 编码器完整 $X 黄金帧验证，ACK回显request_id/token，接入者不得自写另一份decoder。

保存位置通过 tools.project_paths.THRUST_IDENT_DIR，日期/唯一 session 子目录，不覆盖既有 CSV/报告。文件含 metadata.json、samples.csv、各源 raw/events.jsonl；metadata 保存版本、固件/配置身份、电转速/电流来源、上下通道、桨规格/间距、两桨均安装、未驱动桨状态（未知/静止/风车）、称重标定和电流是否外部校准。一个 session 可追加不同 run_id 的重复/电压层，分析按整轮分组验证，不把同一稳态窗口拆到训练和验证。

## 模型 API

`model.py` 提供 `analyze_samples(samples: Sequence[BenchSample], metadata: Mapping | None = None) -> dict`，结果可 JSON 序列化，包含 schema_version、static、power、dynamics、validation、warnings。不足数据时明确 unavailable/reason，不造参数。

`model_report.py` 提供 `write_analysis(samples, metadata, output_dir) -> dict[str, Path]`，返回 report/model 等产物；调用者传唯一新输出目录。报告标明安装状态模型、覆盖范围/来源质量、验证误差及无法拆分的单桨力/扭矩。模型不下发到飞控、不自动修改当前推力表。

作者随后明确验收为“推力=f(转速,电压)，P4双向DShot eRPM回传”。主要产物须显式接受带载电压：双驱动总推力 f(eRPM上,eRPM下,V)，单驱动分别为安装状态总测力 f(eRPM主动,V)；可按真实电压层建低阶eRPM模型后在相邻层插值，真实V覆盖不足时拒绝。电压层标签不能替代实测值，未知/范围外不外推、不固定乘V²。仅eRPM模型保留作基线，禁止仅同油门线拟出伪二维结果。分路电流与回归eRPM/V使用同一有效对齐子集；组合功耗逐样本计算V*(I上+I下)再聚合。DShot遥测尺度允许拟合，未外部校验则明确标注，不宣称正式安培标定；板ADC只作诊断。静态按segment核实稳态/散差/来源独立更新。未驱动桨可能风车，不能假造转速或孤立桨C_T。动态按run/segment与上/下路分离拟合升降速一阶+纯延迟；不足采样/大间断拒绝。合成数据只用于明确标注的测试，不能充当台架实测。

## 本批验证

执行者仅定向真实 C、主机、真实 Tk（隔离临时数据，禁止物理连接）。主控按验证策略执行全量pytest；包含固件修改时执行Debug/DSHOT300_BIDIR/PWM所需构建，检查两个超限入口净行数不增、保留旧行为、协议容错、生命周期/异常停机和最终使用流程。硬件/带桨/飞行结论留给作者授权的实机验收。

## 原程序扩展（作者 2026-09-21 纠正）

统一用户入口为 tools/pressure_rs485_gui.py，包入口无子命令也转到该窗口。旧称重/去皮/砝码标定保留，新实测辨识与H743手动控制直接接替原ESP控制区，单页呈现；不要求手工导出/选择标定 JSON。原单点比例和多点分段线性标定通过不可变 LegacyCalibration 快照复用；采集统一调用 LoadCell.grams_from_raw(raw)，不得直接读取 gain 或二次读传感器。原始值与换算值来自同一样本；去皮为校准后重量差。旧硬件地址、通道与超时必须继承，新旧串口操作互斥。已连接会话不因原页面编辑点而改变标定快照。

电机型号按用户提供记录 AEO CRM2413-KV1300；型号/KV 不证明磁极数。缺少确认时不生成机械 RPM 或启动定量扫描，称重标定仍可独立使用。旧CSV查看器保留离线读取兼容；原ESP控制和旧油门扫描从当前窗口移除，不混入实测模型。

## H743手动控制接替旧ESP（2026-09-21）

ManualControl只调用现有AcquisitionEngine的arm/set_targets/stop，固件安全门、300ms看门狗、全局序号与令牌、ACK和心跳保持。engine.active_operation统一manual/plan所有权，手动解锁和保持期间扫描不能开始。保持时间限定大于0且不超过60秒，到期停止；ARM未确认也先停止再释放。STOP升会话代次、排空在途ARM/SET/监视线程后才释放，超时则保留占用；迟到成功回包补STOP但不能复活输出。健康停止（包括解锁但未应用目标时）走相同屏障。

latest_observation复用已有缓存，不新建轮询线程；即时核验连接代次、断连、负持有时间及source age+held freshness，过期eRPM/V/I置None。手动观测留在原始来源记录，自动扫描的样本继续由既有采集器生成。全局停止按钮无论手动服务是否持有窗口都调用engine.stop，从而也能中止plan。

## 作者确认电转速与DShot电流（2026-09-21，替代前述机械RPM/总ADC电流路径）

运行链直接使用FcSnapshot.erpm，不要求pole pairs，不换算机械RPM。上位机BenchSample/schema_version=2采用upper_erpm/lower_erpm及各自age；upper_esc_current_a/lower_esc_current_a及各自age按固件上下通道映射。board_current_a/age/calibrated仅诊断，绝不回退或参与电流/功耗拟合。speed_domain=electrical_erpm、speed_source=dshot_erpm、current_source=dshot。esc_current_calibrated独立且默认False，板ADC标志不得证明ESC已外部校准。

ESC电流fresh限1000ms，沿用线上协议允许年龄；eRPM/V限250ms。CSV来源age保持FC快照原始口径；UI latest_observation所有age统一加主机held时间，过期值缺失。电流求和/功耗使用current_sources共享helper，默认跨源skew≤50ms，两路缺任一路不得推断合计；功率还需新鲜对齐V。分路电流模型允许在遥测尺度拟合，明确外部校验未知；不得因为单路电流缺失丢掉仍有效的推力样本。

自动采集的250ms是**单条测量能否入库**的阈值，不是立即停机阈值。短暂缺失电压、eRPM或称重时保持最后已确认油门、继续SET心跳，不发新目标、不把缺失区间插值或写入稳态样本；恢复后在同一轮重建完整稳态段。读数连续缺失达到2秒才STOP并等待人工恢复；收到有效的低电压读数、控制ACK超时、断连或主动停止仍立即STOP。板端300ms命令看门狗不变。原始FC/称重接收时刻及来源年龄保留，用于离线质量筛选和对齐。

v1机械RPM记录拒绝直接进入v2，不做隐式重标。板端TBENCH帧版本和decoder不变；DShot底层回读由作者另一agent负责，本轮不写固件/配置/硬件。当前整数A字段分辨率1A写metadata，未来协议变更必须显式更新解码契约。

## 自动补数：同一电量下开环扫油门网格（作者 2026-09-23 要求提速）

替代此前的“追eRPM目标 + 电量段测完后保持油门等电压降0.3V”。模型按实测eRPM/V回归，一个格只需一次稳态测量，不要求落在指定eRPM；实际到达的转速原样入库。

- 网格：油门2%～本次上限，嵌套分层。第0层5×5含四角（保证凸包支持域），细层逐次步长减半，步长不小于5%：上限20%为25格，50%为81格，100%为289格。
- 选点：当前电量段（补偿电压±0.15V，即作者认可的0.3V粒度）缺测的格优先；同为缺测时先粗层后细层，再按两路油门最大变化量就近。整段已覆盖时改测最久未测的格，不空转等降压。实验库同装配的历史合格点计入覆盖；某格在时限内不稳定时，本电量段内跳过。
- 电量坐标：带载电压 + k·Σ(eRPM/1e4)³，只用于排程；存档和模型仍用原始带载电压。k默认0.0035（2026-09-22/23实录拟合0.0030～0.0042），本轮稳态点≥8且负载跨度足够时在线重拟合，限幅0.001～0.012。
- 每格：每条SET的油门变化≤10%，仍逐条ACK并按100ms节拍发送；自适应稳定≤3s；稳定后另采独立稳态段，要求≥5次独立称重和eRPM来源更新，平稳判据与model._steady_points默认值一致（推力CV≤5%，eRPM CV≤2%）。漂移的头部样本丢弃，1.5s内不成则该格本段跳过。
- 停止：换电电压仍是主要停止条件。每轮默认上限60分钟/3000点（允许3600s/5000点）。读数短缺、低压、ACK失败、断连和主动停止的处理同上一节。
- 汇报：结束或换电时报告本轮稳态点数，以及其中补在此前缺失格子的数量。run元数据记录网格、电量带宽、k和补缺数。

## 报告：推力—油门曲线（作者 2026-09-23 要求）

整轮建模报告新增推力—油门视图：只取上下桨同油门（相差≤0.6%）的稳态点，按电池电量（带载电压+k·Σ(eRPM/1e4)³，与自动补数同一口径）每0.3V分段；超过5段时段宽加倍，以保持有序色阶可分辨。线为所选模型在实测工况上的预测，域外不画，下幅为误差和±50克目标线。HTML附明细表。模型本身不分电压段。

## 最大推力测试（作者 2026-09-23 要求）

采集区“最大推力测试”：双桨按每条SET≤10%升到100%（按钮标明100%，作者2026-09-23要求不受“最高油门%”限制；ARM上限随之为100），等待稳定≤4s后记录一个与模型同判据的独立稳态段，随后STOP。报告稳态平均/窗口峰值/波动、两路eRPM、带载及电量电压；稳态段写入会话，run名`max_thrust_test`。未稳定时只报告1s实时读数，不写稳态行。租约、读数短缺、换电阈值、ACK失败、全局停止与自动补数相同。

## 油门模型与实机反推验证（作者 2026-09-23 要求）

油门模型：推力 = poly3(x上, x下)，x = 油门/100 × 电量电压/12；电量电压定义同自动补数。训练只用 `esc_config.json` 记录的 KV 写入会话之后、且低油门（≤3%）推力绝对值 ≤10 克（零点核对）的会话，不用 `model_validation` 轮。评估按时间连续块留出。预测和反推只在训练油门与电量范围内给出，域外返回 None，不外推。实机验证：自动去皮，按当前电量反推同油门，稳定后记录，报告“实测−目标”，限值 50 克；该轮只作独立测试数据。自动采集、最大推力测试、模型验证开始前，若两路 eRPM<100 持续 0.5 s，则自动去皮。

## 推力查补表（作者 2026-09-23 要求）

`thrust_lut`：两张固定网格表，数值单位为克，双线性插值，数据选择同油门模型。转速表的轴为上/下桨 eRPM，0～80k，每 5k 一格；油门表的轴为 e = 油门% × 电量电压 / 12，0～110，每 5 一格。节点值用最小二乘加二阶差分平滑（λ=0.1）求得。查询点在实测凸包外、超出电量或油门的实测范围时，返回 None。支撑权重 < 0.1 的节点记为无实测支撑，作为补测对象。实测验证以查补表为后端：按油门表反查同油门，同时按实测 eRPM 核对转速表。建表和验证结束时各生成一页结果（`thrust_lut.html`、`validation-<run>.html`），“查看拟合效果”打开最新一页。

## 换电判据：电量电压加带载底线（2026-09-23 实测修正）

本节取代前文“收到有效的低电压读数即 STOP”中对带载电压的用法。满电 3S 在 80% 油门时带载约 10.7 V，所以换电电压（界面 10.5～12.0 V，默认 10.8 V）与电量电压比较：带载 V + k·Σ(eRPM/1e4)³，取最近 3 次读数的中位数。另设带载硬底线 9.0 V（每节 3.0 V），任一读数（含自适应稳定等待期间）≤ 9.0 V 立即 STOP。eRPM 短缺期间沿用上次电量估计。结束的控制器 `close()` 不再 STOP，以免打断之后开始的操作；“立即停止”不受影响。

## 查补表写入飞控（作者 2026-09-23 要求）

- 生成：`python -m tools.thrust_bench.thrust_lut_export [thrust_lut.json]` 写出 `Driver/Src/drv_thrust_lut_table.inc`（勿手改），由 `drv_thrust_lut.c` 包含。当前为实测验证过的 `c493194dbb35`（054551）。新表生效需重新导出、编译、烧录；`test_thrust_lut_firmware.py` 检查生成文件与来源 JSON 一致。
- 换算：控制器仍给单桨推力 T，沿用旧曲线口径（两桨同油门合推力 = 2T）。查同油门对角线得等效油门 e（生成器预采样，步长 0.5，单调包络），油门% = e × 12 / 电量电压，脉宽 = 1100 + % × 8.4 us（与推力台一致）。超出实测最大推力时饱和。
- 注入：`DRV_COAX_CTRL_SetThrustMap()`，`APP_Init` 在任务开始前装入；传 NULL 时回到内置 21 点旧曲线。`THRUSTLUT MODE LUT|LEGACY` 做 A/B，仅未解锁时可切换，重启恢复 LUT。
- 电量电压：messageTask 每 20 ms 取带载电压 + 0.0035·Σ(eRPM/1e4)³，一阶低通 τ=2 s。电压超过 500 ms 或任一路 eRPM 超过 100 ms 不新鲜时，保持上次估计；从未有过估计时不补偿（按 12 V）。补偿用的电压夹在 [实测最低 − 0.75 V, 12.6 V]；每次建表都以“高电量建表、预测最低 0.75 V”复核一次。
- 只读查询：`THRUSTLUT ?`、`THRUSTLUT MAP <合推力 g> [电量 V]`、`THRUSTLUT PAIR <上 g> <下 g> [电量 V]`、`THRUSTLUT SPEED <上 eRPM> <下 eRPM>`，数值以定点整数输出。
- 差速分配（2026-09-24）：先逐桨查同油门线，再把两桨同时平移 c（|c|≤10%），使二维油门表的合推力等于逐桨推力之和；油门差不变。任一桨饱和或低于实测最低油门时不平移。默认模式为 PAIR，`THRUSTLUT MODE PAIR|LUT|LEGACY` 可切换（仅未解锁时）。上位机 `thrust_lut.pair_throttle` 与固件逐项一致；推力台“差速实测验证”每组新旧分配各测一次（plan `split_validation`，不进训练）。

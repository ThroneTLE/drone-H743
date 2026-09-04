# drone-H743 遥测协议

> 当前工作树的线上规范。当前板上烧录版本、实机结果和审核状态只看 `PIPELINE.md`；
> 本文存在不等于相应固件已经烧录或验收。

## 事实源

- 通道编号：`App/Inc/app_telemetry.h`
- 通道元数据与 SchemaHash：`App/Src/app_telemetry.c`
- 帧编码：`App/Inc/app_telem_frame.h`、`App/Src/app_telem_frame.c`
- 命令：`App/Src/app_cmd_telem.c`
- 主机解码：`tools/panel_lib/telem_stream.py`

## Schema v3

`TELEM?` 返回表头，当前必须包含：

```text
TELEM ver=3 n=<count> rate=<hz> page=<count> hash=<8hex> frame=body_flu contract=1
```

`TELEM CH from=<n>` 分页返回：

```text
TELEM CH idx=<n> name=<name> unit=<unit> min=<min> max=<max> grp=<group> param=<key-or-->
TELEM PAGE from=<n> count=<count> next=<next-or--1>
```

SchemaHash 使用 FNV-1a，覆盖版本、通道数、标称速率、frame、坐标契约版本及全部通道
元数据。frame、contract 或任一通道元数据变化都必须使 hash 变化。

主机对 schema v3 缺失或错误的 frame/contract 整表拒绝；v1/v2 只按
`legacy_unspecified` 兼容读取，绝不倒推成 FLU。

## `$X` 遥测帧

function：`APP_PROTO_MSG_TELEM_FRAME = 0x2230`。payload 小端：

```text
off  0  u8   ver       = 1
off  1  u8   count     样本数，至少 1
off  2  u16  seq       帧序号，自然回绕
off  4  u32  schema    APP_Telemetry_SchemaHash()
off  8  u32  t_us      首样本时间戳低 32 位
off 12  u16  dt_us     批内样本间隔；count=1 时为 0
off 14  u16  flags     bit0=全量刷新
off 16  u64  mask      bit i 表示通道 i 存在
off 24  f32  data[]    通道索引升序，再按样本序排列
```

必须满足：

```text
payload_len = 24 + 4 * count * popcount(mask)
```

长度、版本、掩码或 schema 不一致时整帧拒绝并计数，不能部分解码或静默截断。

## 坐标与单位

对外轴向数据遵守机体 FLU：`+X` 前、`+Y` 左、`+Z` 上，契约版本来自
`Driver/Inc/drv_frame_contract.h`。控制器 legacy X前/Y右的速度和位置必须在
`app_telem_port.c` 通过显式适配后发布。

每个通道必须声明单位。历史捕获不得因为当前 schema 已是 FLU 而被重新解释。

## 命令

```text
TELEM?
TELEM CH from=<n>
TELEM STREAM on|off
TELEM RATE <hz>
TELEM MASK <hex>
TELEM REFRESH <s>
TELEM FORMAT bin|jf
TELEM SINK usb|uart|auto
```

流配置只存 RAM。USB/UART 两个出口发送相同帧；USB 拔出、IMUCAP/FLOG 导出等互斥
条件按固件实现进入安全状态。任何速率/掩码组合超出出口能力必须回复 ERR。

## Dashboard 契约

- 通道按名称绑定，不能按固定数组位置硬编码。
- mask 是当前工作区所有组件绑定通道与参数通道的并集。
- 收线程解码并写环形缓冲，Tk 线程只读快照。
- 参数卡的“已确认”只能来自飞控回显，发送成功不能冒充应用成功。
- 页面明确显示 `FLU v1（X前 / Y左 / Z上）`。
- CSV 保存 schema hash 和通道名；本帧未携带的通道留空，不补零。

## 必要验证

```powershell
python -m pytest tests/test_telem_stream_contract.py -q
python -m pytest tests/test_telem_stream_decoder.py -q
python -m pytest tests/test_telemetry_schema_contract.py -q
python -m pytest tests/test_dashboard_page.py -q
```

黄金向量：`tests/golden/telem_frames_v1.bin`。线上帧布局变化必须同步更新固件、主机、
本文和黄金向量；通道表只允许追加，不允许重排历史编号。

## 实机验证门

- UART/数传 40 Hz 与 USB 40 Hz 各持续 60 秒：seq 无缺口、拒帧为零，命令回复不被流挤掉。
- `SINK auto` 随发起 `STREAM on` 的链路选择出口；切换出口后旧出口不再继续发流。
- 参数写入到飞控回显的面板往返延迟小于 150 ms；`LOAD/DEFAULTS` 后控件收敛到飞控值。
- 固件通道表或 frame provenance 变化后，旧 hash 被拒绝，面板重拉 schema 且通道不串位。
- USB 高速档若由独立 REQ 启用：500 Hz × 8 路持续 60 秒无 seq 缺口，导出互斥前后流能安全停/恢复。

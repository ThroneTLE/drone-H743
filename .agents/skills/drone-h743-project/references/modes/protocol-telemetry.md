# 类别模式：协议 / 遥测改动

适用于**改线上字节格式、命令族、通道表、上位机解码**的 REQ（R-T1-x、R-T2-x 及后续同类）。
线上格式在 [`doc/telemetry-protocol.md`](../../../../../doc/telemetry-protocol.md)，本文只写授权与判据。

## 授权

- 新建固件模块（`App/Src/app_telem_stream.c`、`App/Src/app_cmd_telem.c` 等）与上位机模块（`tools/panel_lib/telem_stream.py`、`scope.py`、`pages/scope.py`）。
- 解禁 `app_proto.c` 里的 `APP_Proto_BuildFrame` 及其 CRC；**不**解禁解析器。
- 在 `App/Inc/app_proto.h` 与 `tools/panel_lib/proto.py` **同一提交**登记新 function ID（历史教训：0x1022/0x1023 曾单方定义）。
- 改 `tools/panel_lib/transport.py::_consume_buffer` 增加二进制分支；改 `app_telemetry.c/h` 表结构与版本号。
- 早期遥测迁移中的生成入口搬移不是后续任务的常驻授权；涉及 `Core/Src/freertos.c` 等生成代码仍走当前 CubeMX/作者授权流程。

## 判据

1. **帧格式以 `doc/telemetry-protocol.md` 的当前线上规范为准**；格式变化先由主控明确约定，再协调两端实现和文档作为完整单元集成，不能交付半套协议。
2. 两端**逐字节一致**由黄金向量钉住：固件 host 装置产出的字节 == pytest 解码器的期望输入。不允许各写各的"看起来一样"。
3. 上位机解码器必须过**模糊测试**：任意截断/插入/翻转，永不产出错长度样本、一帧内重同步、hash 不符不入缓冲。
4. **静默失败零容忍**：长度不符、hash 不符、超限配置都必须可观测（计数器 + `ERR` 回复），不能悄悄截断、悄悄丢弃。
5. 带宽算给审核者看：每个默认配置写明 B/s 与链路占比；数传档任何默认配置不得超过 5760 B/s 的 60%。
6. 通道表**只追加不重排**；升 `APP_TELEM_SCHEMA_VERSION` 时同步更新 pytest 里的 hash 复算。

## 禁止（未经作者单独批准）

- 改 `$X` 帧头、方向字节、CRC 算法；改既有 function ID 的语义。
- 改 `PARAM SET` / `PID` 的现有回复文本；改任何参数写入的钳位或安全门。
- 在控制环任务上下文里调用 `APP_Control_QueueText`/`queue_proto_text`/`APP_USB_CDC_Write`（spec §5 上下文契约）。
- 向 `drone_tcp_panel.py`、`app_control.c` 追加内容；两入口只减不增，新功能通过模块接入。
- 持久化流配置到 Param blob。

## 交付物

按改动附真实编码器/黄金向量、解码容错输出、相关带宽计算及两端 ID 登记差异；公共文件由主控指定唯一负责人。触及超限入口时证明其未增长。开发阶段定向检查，最终跨端组合及全量时机统一按 [验证策略](../validation-policy.md)，不要求每个子 Agent 重复整套。

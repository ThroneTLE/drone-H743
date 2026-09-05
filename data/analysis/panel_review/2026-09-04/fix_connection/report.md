# 上位机连接基础返修（H1 / H2 / H4）

状态：待审核。执行基线 `dcf18faf`；问题来源是作者提供的
`data/analysis/panel_review/2026-09-04/review.md`（审核基线 `6f7a440e`）。
本批只修连接基础，不声称关闭 H3/H5/H6/H7，也不作为实机验收证据。

## 根因与修复

- H1：发送线程持有 UI 状态锁执行串口 write/flush。现在每次打开独立持有端口、
  线程、队列与取消事件；状态锁只保护状态快照。写出和输出排空共用 0.5 s 期限；
  Windows 无期限 flush 改为可取消的 out_waiting 轮询。期限到或取消进行中的写入时
  立即撤下该会话，后台 cancel_read/cancel_write/close，不重放待发动作。
- H2：原队列没有接收时间和连接代次，Tk 合并时重新打时间戳。现在文本及二进制文本
  回复附带不可变 ReceiveContext（transport 对象、generation、接收边界 monotonic 秒）；
  Tk 先拒收旧会话，再分发。IMU 同一 seq 分行合并保留最早一行接收时间，跨会话不拼接，
  新的半帧不借用上一份完整快照的新鲜度；断线立即使原快照失效。原 1.5 s 门不变。
  TCP/UDP 文本入口也附带来源和代次，避免切换链路后旧队列进入当前页面。
- H4：原读异常只 break，不记录原因且不显式关闭端口。现在读写/排空错误及收尾错误
  保留异常类型、errno/winerror、端口、代次、最后收发时间和发生时刻；同一会话只收尾一次。
  旧读写线程晚退出不影响新会话。慢关闭时，重新打开在后台等待旧句柄释放（有界），
  用户再次停止会取消等待；旧连接通知仍进入日志，但不再触发新会话探测或重置协议状态。

为什么旧测试未挡住：正常收发/显式停止与源码锚点测试没有模拟阻塞 I/O、队列积压、
同 seq 跨会话拼接、旧线程晚返回及延迟连接通知。新回归直接调用真实 SerialTransport
和真实 DronePanel；IMU 测试报文从 `App/Src/app_control.c` 的实际格式串生成。

## 模块契约与边界

- `serial_session.py`：SerialSessionMixin 的 start/stop/send_line/send_frame 保留公开入口；
  SerialSession 只管理一份端口资源。无 Tk 调用，不读取校准历史，不自动重放命令。
- `connection_state.py`：ReceiveContext / ReceivedMessage 和 snapshot_receipt；
  主机单调秒只用于接收新鲜度，固件 ts_ms 仍用于样本时间；无硬件 I/O。
- `rx_dispatch.py`：只在 Tk 线程按既有批量上限处理队列，旧数据在处理器前丢弃并计数，
  保留诊断日志。回调异常恢复和时间预算属于 H3/U4，仍待后续批次。
- `tests/conftest.py`：收集前阻止真实 serial.Serial.open；隔离测试默认面板状态和日志，
  构造时关闭自动连接/历史自动加载，显式持久化测试仍使用真实方法及临时目录。

## 验证记录

- `baseline.txt`：首轮复现含 Tk 清理装置错误，原文保留。
- `baseline_rerun.txt`：修正装置后 `6 failed, 1 skipped in 7.46s`，对应慢 I/O、缺失收尾、
  快照时间/拼接失败；另一跨代次用例由提供的审核记录及后续实际 Tk 回归补齐。
- `focused_initial.txt` / `focused_second.txt`：保留开发中源码锚点、测试隔离调整的记录。
- `focused_extended.txt`：`20 passed in 5.51s`。
- `full_tests.txt`：首次全量 `2 failed, 1024 passed in 105.21s`；一项旧源码锚点随封装更新，
  一项画布基准 5.98 ms 超过原 5 ms；未放宽性能限值。后续聚焦复跑见下。
- `reconnect_red.txt`：提交前检查新增的重开句柄和旧通知竞态均红灯。
- `reconnect_green.txt`：重连修复、画布和遥测解码聚焦 `49 passed in 7.56s`。
- `full_tests_final.txt`：最终 `1030 passed in 92.84s (0:01:32)`。
- `build.txt`：`cmake --build --preset Debug` → `ninja: no work to do.`，零警告。

所有受保护测试的真实串口打开尝试为 0。没有烧录、复位或发送目标板命令。
`data/calibration/**`、CubeMX、固件源码均未改动；超预算 panel 入口净减少 20 行。
只更新与修复有关的旧测试源码锚点及三个快照方法的 AST 基线，行为由新增契约测试覆盖。

工作模式：横切修 bug（过期/异代数据可能被当作有效安全状态与证据）；diagnose
复现→最小根因修复→回归；protocol-telemetry（主机接收来源和兼容检查，线上字节未变）。
执行需求清单没有本批专属 REQ，因此不改任何既有 REQ 状态，只追加独立修复证据行并交审。

# Serial Studio / Qt 上位机资料

> [!CAUTION]
> 本目录是历史调研和原型，不是当前真机工作台。当前副线状态只看仓库根目录
> `PIPELINE.md`；若该副线仍冻结，Agent 不得依据本文自行恢复开发。

## 当前可确认的内容

- `Serial-Studio/` 是 vendored 的 Serial Studio 4.0.3 源码，使用 Qt/QML。
- 本仓库保留 Qt 6.7/MinGW 兼容补丁和历史构建产物。
- `extensions/org.drone-h743.control-panel/` 是 QML Widget Extension 原型，曾验证可通过
  `ApiTerminalBridge`/`io.writeData` 向回环链路发送字节。
- `Drone-H743-GCS.ssproj` 与 `drone_simulator.py` 是仿真演示，不对应当前真机协议和页面。
- Serial Studio 为 GPL-3.0-or-later 或商业双授权；它与固件代码保持目录和链接边界。

## 当前不应假设的内容

- 不要假设 `.ssproj` 已接通当前 `$X` 遥测或 schema v3。
- 不要把演示按钮、通道 ID、旧 JustFloat 解析或旧通信状态当成固件事实。
- 不要 fork C++ 核心来实现项目功能；如未来解冻，优先使用 frame parser、控制脚本、
  QML Widget Extension 或外部 plugin。
- 不要引用旧调研中的命令数量、测试数量、Qt 上游要求或“下一步”作为当前结论。

## Agent 使用历史信息的规则

需要复用本目录的实验、配置或接口结论时，先问用户：

1. 本次要继续 Serial Studio 路线，还是只把它当性能参考？
2. 使用哪个固件提交和遥测 schema？
3. 目标是只读监视、参数调节，还是完整校准工作台？
4. 使用 GPL 构建还是已有商业授权？

用户未回答前，只能做只读调查，不能修改工程、安装扩展或启动实机连接。旧的详细路线、
能力探底和推进记录保存在 Git 历史中；恢复时必须与当前源码和 `doc/telemetry-protocol.md`
重新核对。

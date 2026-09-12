# PX4 DShot 单向移植基线

- Upstream: PX4/PX4-Autopilot，tag `v1.16.0`
- Commit: `6ea3539157ca358c70a515878b77077af7d4611d`
- Source: `platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c`
- URL: https://raw.githubusercontent.com/PX4/PX4-Autopilot/6ea3539157ca358c70a515878b77077af7d4611d/platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c
- Snapshot: `upstream_dshot.c.txt`（原始下载内容，不编入目标固件）
- SHA256: `fa428eec81cbfdf70d89684eb182fbb2d023ca64c8db4cb82133598d2c5fbbaa`
- License: BSD-3-Clause，完整版权/免责声明在快照开头及移植源文件中；二进制分发时一并附带。

## 实际移植与本地差异

`drv_dshot.c` 的编码、XOR nibble 校验和 MSB-first 双通道交错取自上游
`dshot_motor_data_set()`。移植为无全局状态的纯 Driver，拒绝特殊命令、禁用遥测位。
时序按本地 300 kbit/s 的 3/8、6/8 占空比精确计算；不照搬上游的 7/14 常数。
显式增加两个低电平尾槽以供后续 BSP 处理预装载收尾。增加项目等效微秒指令映射。

测试从原始快照提取**未改函数体**，以最小板级数据表桩编译，穷举全部正常油门和停止码
比较两路 16-bit 位流。黄金向量独立按协议中的 throttle/telemetry/checksum 字段定义。
测试不下载网络资源，不依赖 PX4/NuttX 构建。

当前只完成纯协议 Driver；上游的 DMA 启动接口及回调尚未移植到本项目 BSP。
波形数组测试不等于真实定时器预装载/DMA 或实机电调验证。

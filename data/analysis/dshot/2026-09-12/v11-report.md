# R-DSHOT-2 解锁日志逐条 DShot 诊断

工作树 D:/stm32hal/drone-H743-dshot；基线 ebf973f0。模式：默认 REQ + dshot-esc + protocol-telemetry。
联合上位机回归出现的 Tk 销毁问题单独 fix，见 current 同日期 tk-lifetime-report.md。

FlightLog V11每条808字节：保留V10前772字节，追加32字节诊断，最后4字节为全记录CRC。
字段为 present、enabled_mask、busy、fault、upper/lower_code、submitted、completed、busy_rejected、errors、cancelled、timer_clock_hz。
独立 app_esc_log 适配 BSP 快照；实际 Observe 构建记录时复制，排队后不读取后续状态。
记录依然由原解锁录制门和125Hz采样控制；没有控制律或安全判据修改。
原 motor_upper_us/motor_lower_us 是等效PWM指令；DShot码是最后接受的发送码，计数自显式初始化起累计。
DMA completed不表示电调确认，首版无转速/遥测回读。

PWM后端 present=0，新诊断CSV列为空；V10及更早新增列也为空，不根据当前协议补零或重解释。
256字节扇区头、原CRC机制、协议标记及旧日志扫描保留；V10原776字节继续有真实C迁移检查。
主机支持V11解码/CSV，错误V11记录可观测；波形工具按数值列自动提供这些新增通道。

`v11-tests-second.txt`：63 passed。新增装置编译真实结构/记录构建/Observe、app_esc_log，
仅RTOS/BSP为seam，覆盖PWM/DShot、未录制无读/入队、普通/故障状态、排队快照不变、CRC、截断、版本不匹配及旧记录兼容。
调整原固定776/V10断言为808/V11，同时新增V10大小/读取保留检查，没有删掉原控制字段断言。
最终联合全量：data/analysis/current/2026-09-12/readback-full-verified.txt，1551 passed in 232.70s，无警告。
Debug原文：v11-build.txt（DSHOT300）、v11-build-pwm.txt（PWM），实际编译链接零警告。
每扇区仍4条记录；队列64条多2048字节，实际D1 RAM总量增加2240字节；DTCM/D2大小未变。

软件交审、待审核；未操作目标板。解锁后的SD日志落盘、并发负载与实际电调响应仍需实机复核。

# 原工程合并准备

作者要求把 codex/dshot300 合回 D:/stm32hal/drone-H743。
原工程 HEAD=d5ed69d3，存在正在被另一任务编辑的 LED 映射/参数存储/界面改动；作者已确认并发编辑尚未结束。
因此只在干净的 D:/stm32hal/drone-H743-integrate（codex/integrate-dshot）整合已提交基线，尚不改动原目录。
父版本为930f7a35与d5ed69d3。模式：默认REQ合并 + dshot-esc/protocol-telemetry/tk-ui复核。

冲突处置：索引由生成器重建；Core/Src/freertos.c整文件取原工程d5ed69d3版本，保留LED任务启动，不手写生成代码；
遥测优先级测试保留读取实际字段和相对生产者优先级的检查，其余两侧变更保留。
ADC/DShot生成文件、TIM1 DMA与电流后台挂载不与RGB LED引脚/任务覆盖。
全量所需的既有rm1_3_block_queue CSV从dshot工作树逐字节复制并核验SHA256，源文件未改动，不生成替代数据。

构建：merge-configure.txt/merge-build.txt、merge-configure-pwm.txt/merge-build-pwm.txt，两种Debug实际编译链接零警告。
全量原文merge-full.txt：2 failed,1555 passed,15 warnings；Dashboard重绘11.77ms>10ms；另一个错误实际来自Image.__del__跨线程回收，之前只覆盖了Variable。
本提交只是保留完整合并父关系的候选节点，后续修复独立提交；全量门尚未闭合，未同步原工程，不作合并完成或实机结论。

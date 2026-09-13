# 电流采样周期修复：原工程交付

状态：软件自审完成，R-CURRENT-1/2仍待审核；未操作目标板。
原工程：`D:\stm32hal\drone-H743`，代码提交`7c8a8f10`，保留原LED/参数/界面改动。

用户截图的回包已解析成功，但样本数只有1、样本年龄51282ms，所以有效位为否。ADC原始值为0本身不是失效原因。
本次修复了电流采样与慢存储共用backgroundTask的调用链缺陷。真实C任务链在约51秒存储占用中复现samples=1/age=51305ms；复用已有messageTask后，2565次采样、age=25ms、valid=1。没有放宽250ms过期判据。

验证原文：

```text
1612 passed in 371.25s (0:06:11)
Physical serial open attempts: 0
Flashing/probe tool invocation attempts: 0
```

- 全量：[main-full.txt](main-full.txt)。两种固件构建均零警告：[DSHOT300](main-build-dshot.txt)、[PWM](main-build-pwm.txt)。
- 真实C负例/修复后：[red.txt](red.txt)、[green-trace.txt](green-trace.txt)；旧测试仅直接调用Current_Step，漏掉存储任务阻塞。
- 原目录实际ARM ELF的连续任务验证：[main-task-arm.txt](main-task-arm.txt)：模拟60秒/3000次采样/最后年龄20ms；后台从未调度。ADC=0为模拟输入，不是用户实测。
- 初始化/采样只有一个任务所有者；零值可有效、超时可恢复、过期仍失效、报告不做ADC I/O。未新建任务、未改CubeMX、优先级、DShot、控制律或标称电流比例。
- 原39个改动文件已备份恢复：33个内容不变，6个文档/索引共同合并，见[main-preservation.json](main-preservation.json)。备份`.tmp/current-cadence-integration/`及stash均保留，其他任务的工作仍未提交。
- 固件SHA256和受保护入口检查：[main-build-review.json](main-build-review.json)。源码修复已在原目录，默认`build/Debug/drone-H743.elf`为DSHOT300。

本次模式：默认REQ、protocol-telemetry（保持CURRENT字段/有效性语义）、横切修bug（修复真实任务链阻塞）。遵循runtime-services与分层边界；修复为单独fix提交。

实机边界：截图没有提供任务PC，本次没有读取目标板，不能断言板上具体卡在哪个存储调用；已经修复并复现验证的是这条确定存在的软件故障路径。
作者/审核者更新本次固件后，确认采样次数持续增加、样本年龄保持小于250ms、有效位为是。若实际原始ADC持续为0，应显示有效的0.000 A；这不代表电流比例已实测校准。Curr信号电压不是电池母线电压。

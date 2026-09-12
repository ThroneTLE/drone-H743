# 原工程整合完成记录

作者确认原目录已停止并发编辑后执行。目录：D:/stm32hal/drone-H743；分支feat/micoair743v2。
已从d5ed69d3快进到815450a7，包含DShot、电流采样/传感器页、V11日志及Tk资源清理修复。
这是原目录的验证，不再只是独立整合工作树的结果。

原工程还有39个未提交/未跟踪改动文件，先用stash完整备份，再合并并恢复：
e7a5c6433576d9f171ec186ee1d14f689c196a64（保留未删除）。
状态灯映射、配置V21和维护分组保持为原有未提交工作，没有把它们悄悄丢弃或混入本次证据提交。
备份比对：24个内容不变（忽略Git正常换行转换），15个完成双边合并或索引重建，没有缺失文件。
具体快照与比对保存在 .tmp/dshot-main-integration/before.json / preservation.json；原始恢复输出stash-apply.txt保留。

合并处置：
- shell同时保留“维护 / 固件升级、舵机调试、状态灯”与“传感器 / 电流计”；没有重复挂载舵机页。
- 两侧页面增加各一页，叶页总数25、三尺寸报告75；维护3页、传感器5页，所有可达性判据保留。
- PIPELINE保留两侧新增证据；索引合并既有说明压缩96字符及入口4项，上限未变。
- 生成代码沿用已提交整合结果，没有手改Core/USB_DEVICE或修改安全阈值。

验证原文：main-merge-focused.txt，77 passed；main-merge-full.txt，1593 passed in 365.76s，无warnings/skips。
Debug DSHOT300：main-merge-build-fresh.txt，实际编译链接零警告；PWM：main-merge-build-pwm.txt，实际编译链接零警告。
首次配置遇到旧缓存混用编译器/Windows链接参数，失败原文main-merge-configure.txt/main-merge-build.txt保留。
校验路径后将原build/Debug移到 .tmp/dshot-main-integration/Debug-pre-merge-cache，重新按工程工具链配置，无源码绕过。
主Debug缓存固定ESC_PROTOCOL=DSHOT300；PWM验证目录build/PWM-merge。

本结论绑定“815450a7已提交代码 + 恢复后的原有未提交工作”的组合；未提交源码哈希见main-merge-validation.json。
工作模式：默认REQ整合、tk-ui、protocol-telemetry、dshot-esc；本轮只有冲突/构建缓存处理，没有新增控制行为。
没有烧录、复位或操作目标板；REQ仍待审核。原上位机进程需要关闭重开才能加载新页面。

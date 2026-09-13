# 电流文本回读缺值：软件根因与修复

状态：软件自审候选；实机复核归审核者。作者确认AM32原配8PIN包含Curr/GND，本次未怀疑或更改接线。

固件使用newlib-nano且未链接`_printf_float`，原`CURRENT`用`%.5f`/`%.3f`格式化。实际ARM ELF指令仿真输出`adc_v= current_a=`，上位机正确拒绝了空字段。
旧宿主测试链接PC的完整C库，能正常处理浮点printf，所以未挡住目标库差异。不能从宿主格式串对拍推导板上格式化成功。

最小修复：仅CURRENT两个浮点字段用有界整数定点格式化，保持原命令、键、精度与未校准语义；非有限/越界输出nan且valid=0。
没有全局启用浮点printf，没有改ADC配置、标称比例、安全门或控制律。

证据：`arm-before.txt`、`arm-after.txt`和`arm_current_probe.py`。
修后同样注入raw=12000，实际ARM格式化输出`adc_v=0.60426 current_a=47.393`。
仿真执行目标库真实格式化指令，ADC和传输出口被替身替代；未连接目标板，未烧录/复位/驱动电机。

进入模式：默认REQ、protocol-telemetry；由于回包缺字段阻塞R-CURRENT-2回读，进入横切修bug模式。该修复独立fix提交。

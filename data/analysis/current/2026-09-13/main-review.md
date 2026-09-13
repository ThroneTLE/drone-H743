# 电流回读与飞控注册总览：原工程整合、自审交付

软件自审通过，R-MODULES-1及电流相关REQ仍待审核。交付目录：`D:\stm32hal\drone-H743`。
验证对象是代码提交`224a9d7f`加原工程保留的LED/CFG V21/界面工作；不是只验证独立工作树。

## 改动与根因

截图数值说明：0.60426 V是模拟Curr信号的ADC电压，不是12 V电池母线；47.393 A是模拟raw=12000的换算结果，不是实际空载电流。本次没有读取目标板，也尚未新增电池电压回读。

- CURRENT空字段：实际目标newlib-nano未包含浮点printf；原`adc_v= current_a=`导致上位机拒绝。现用有界整数定点格式保持原有CURRENT字段与nan/valid语义，未全局引入浮点printf。相同模拟raw=12000得到`adc_v=0.60426 current_a=47.393`。
- 总览：固件注册元件、型号、接口、状态和最多4个关键读数，上位机仅按类型渲染。`REGISTRY? nonce`查询，`0x2231`二进制BEGIN/RECORD/END事务；序号、CRC、版本、连接代次和接收时间都验证，收到完整表才替换。
- 本板13个已配置元件；未初始化和故障仍可注册，注册不等于检测成功。GPS在其初始化入口执行后加入；没有旧板GD25Q32/I2C1/M9N等上位机固定行。IMU型号来自实际选择结果。
- 进入总览读取一次，可手动刷新；无周期整表流量。默认UART遥测仍3356 B/s（58.3%）。典型查询1770 B/次，避免最初1Hz草案把并发稳态流量推到89%。
- 未支持协议、未收齐、断连、错误和过期明确显示；当前ADC仍为标称比例/未实测校准，DShot显示发送码和软件状态，不是电调确认。

## 自审与回归

```text
1611 passed in 405.01s (0:06:45)
Physical serial open attempts: 0
Flashing/probe tool invocation attempts: 0
```

- 原文：[main-full.txt](main-full.txt)。全量没有失败、警告或跳过。
- 双协议Debug零警告：[main-debug-dshot.txt](main-debug-dshot.txt)、[main-debug-pwm.txt](main-debug-pwm.txt)。默认构建仍是DSHOT300；镜像校验和在[main-elf-sha256.json](main-elf-sha256.json)。
- 真实C编解码/独立黄金字节、2000例截断插入翻转、缓冲边界、晚注册、预算及发送失败、USB/UART/维护字节路由、重连和旧固件均覆盖。
- 真实DronePanel三尺寸×三缩放、选择/滚动/最后一行、单次查询、拒绝/超时/恢复。原目录的[离线截图](main-overview-offline.png)使用ARM仿真帧和模拟ADC，图中已注明。
- [main-current-arm.txt](main-current-arm.txt)、[main-registry-arm.txt](main-registry-arm.txt)执行原目录实际ELF；ADC/出口替身，外设地址未映射，不是实机测量。PWM候选ELF也验证了注册协议名为PWM。
- 初轮全量[full-dshot.txt](full-dshot.txt)为2 failed / 1574 passed；两项为新协议常量未从旧入口导出及目录计数未更新。补齐兼容导出后专项17 passed，最终原目录全量结果如上；未删除旧保障。

## 原有工作与边界

原40个改动文件已备份。规范化换行后27个内容一致，13个与新功能共同合并；具体文件和哈希见[main-preservation.json](main-preservation.json)。
手工合并点是命令兜底链同时保留REGISTRY与LEDMAP；8份索引按当前所有文件重生成，其余由Git合并并经全量验证。
原LED配置、CFG V21与V20读取器、维护分组、舵机页和参数界面继续保留未提交；本次没有将它们混进功能提交。
备份在`.tmp/current-overview-integration/`，保留stash `29f0821b3e83d9c25b7bb256cd8f1458f8f4f1ac`；之前的备份stash也未删除。

对比整合前：`drone_tcp_panel.py`净减少119行；`app_control.c`、CubeMX生成入口和`.ioc`不变。控制律、门限、舵机校准、DShot/PWM输出链和历史校准证据未改。静态结果与代码输入指纹见[main-validation.json](main-validation.json)。

进入模式：默认REQ；protocol-telemetry用于注册协议/回读；tk-ui用于总览；横切修bug用于CURRENT目标库格式化缺陷及索引超容量阻塞。两个修bug独立提交，REQ状态最多待审核。

## 审核者下一步

重启原目录上位机，并由有授权的作者/审核者烧录本次固件后，确认实际CURRENT、元件注册清单及连接重建行为。
本次未烧录、复位、发送目标板命令或驱动电机；电流实测校准与硬件验收未代办。旧固件不提供REGISTRY时，新总览会明确提示，不能仅更新上位机来获得新固件字段。

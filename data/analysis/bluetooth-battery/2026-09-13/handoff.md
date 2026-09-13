# 蓝牙与3S电池保护：原工程自审交付

状态：软件自审完成，R-BT-1/R-BATT-1待审核；未进行实机连接、烧录或复位。
交付目录：`D:\stm32hal\drone-H743`。验证对象为`670024ac`代码加原有LED/CFG V21/界面工作，完整输入指纹见[final-code-inputs.json](final-code-inputs.json)。

## 使用方式与结果

- 通道选择“蓝牙”，自动按MicoAir设备地址匹配Windows出站SPP口；点击“启动连接”。优先上次成功设备，多设备不盲选，COM号变化仍按地址追踪。首次需Windows已配对。
- 打开“传感器→电池电压”，回读总压、平均单节、当前电流、采样有效性/年龄/次数/错误、低压门及ELRS DMA接纳/拒绝统计。总览也增加了飞控注册的电池电压项。
- 默认3S：低于10.5V告警并拒绝新的解锁，恢复线10.8V；拒绝后仍需重新拨杆低→高。已有解锁锁存不会因低压或电压失效而被自动清除；RC失联、IMU等原安全门照旧。
- 低压/失效通过上位机和状态灯8闪告警。原16项LED颜色配置与252B Flash结构保留，新电池告警走独立固定图案，避免旧配置编号错位。
- 阈值和串数可在确认未解锁时修改、回读；**仅本次运行RAM配置，重启恢复默认**。平均单节来自总压/串数，未逐节测量，21.12分压比为标称值。
- ELRS每500ms发送最新电压/电流。未知容量、剩余比例、失效电压/电流用全FF，不伪造百分比；接纳计数不是遥控器确认。

协议与设计见[doc/bluetooth-battery.md](../../../../doc/bluetooth-battery.md)，CubeMX交接见[battery-cubemx-checklist.md](../../../../doc/battery-cubemx-checklist.md)。

## 验证原文

```text
1634 passed in 418.51s (0:06:58)
Physical serial open attempts: 0
Flashing/probe tool invocation attempts: 0
```

- 最终全量：[main-full-final.txt](main-full-final.txt)。整合首轮也通过1634项；之后补ADC停止失败锁存，再运行上述完整回归。
- 双协议Debug零警告：[main-build-dshot-final.txt](main-build-dshot-final.txt)、[main-build-pwm-final.txt](main-build-pwm-final.txt)。默认镜像`build/Debug/drone-H743.elf`仍是DSHOT300。
- 真实C电池回包：[battery-diagnostic.golden.bin](battery-diagnostic.golden.bin)，69B；实际ELRS打包/CRC：[battery-crsf.golden.bin](battery-crsf.golden.bin)，12B。均为测试输入，不是用户实测。
- 真实解锁函数测试覆盖低压/过期拒绝、恢复不自动解锁、已解锁不被低压切断、原RC/油门/IMU/坐标/模型门；ADC两rank、抢占时间、失败丢弃、停止失败锁存、明确配置ACK与拒绝均覆盖。
- 真实DronePanel测试覆盖设备歧义/换号/取消/异步打开、USB记录保留、当前连接确认、配置草稿与回读、三尺寸×三缩放和全部26个叶页。测试原文见feature-tests.txt、regression-fixes.txt、ui-final.txt及最终全量。
- [改动前界面](connection-before.png)、[蓝牙界面](bluetooth-offline.png)、[电池页](battery-offline.png)都是离线布局。蓝牙图只读取Windows已有配对记录，电池图没有实测数字；屏幕抓取不可用时只渲染本任务自己的Tk窗口。
- 初次候选[full-first.txt](full-first.txt)有9项失败：旧位置/页面数量契约、样式和索引未刷新；修正后相关专项及原目录完整回归通过，失败原文保留。

## 自审修复与保留边界

1. 索引超出98304B：缩短用途摘要96→80，未放宽8/32/96KiB预算。
2. 总览RC接口仍写UART4：按BSP实际huart6修正为USART6，仅修显示元数据。
3. ADC停止失败会丢失下一rank位置保证：旧提交负例[adc-stop-negative.txt](adc-stop-negative.txt)失败，新实现锁存不可用，显式初始化前不再转换，见adc-stop-guard.txt。普通采样超时仍可恢复。

原39个工作文件已备份恢复：20个内容保持、19个共同合并/适配，见[main-preservation.json](main-preservation.json)。保留`.tmp/bt-battery-integration/`和stash `b92b8b2b507c3714a7e662f46a930b1ff432e72f`。
原LED/CFG V21/维护分组工作仍未提交；本次没有把这些工作混入功能提交。LED合同中旧7个可配置原因仍逐一保证，新8闪通路由真实C另行验证。
作者完成了ADC生成：PC1/CH11/rank1、PC0/CH10/rank2、discontinuous=1；生成器只改变ADC配置与删去freertos.c的一段注释，任务优先级未变。没有手改生成代码，没有修改历史校准证据或控制律。

工作模式：默认REQ、tk-ui、protocol-telemetry；横切修bug用于上述索引容量、RC元数据、ADC停止失败边界。低压新解锁门由作者明确授权（3S、告警并禁止低压解锁，飞行中不自动切电机）。

## 实机复核

重启原目录上位机以加载蓝牙选项；电池功能需作者/审核者更新本次固件。ADC配置已生成，无需再Generate。
实机核对总压与万用表、采样持续更新、ELRS电池字段更新，以及拆桨条件下低压拒绝/恢复重新拨杆的行为。本次软件测试不替代上述验收。
Windows当前只证明存在MicoAir配对记录，未建立实际蓝牙连接；也没有读取用户当前电池电压。

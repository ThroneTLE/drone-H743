# R-BATT-1：ADC双通道生成交接

**已完成，无需再Generate。** 作者已在本仓库的`drone-H743.ioc`上生成，`Core/Src/adc.c`的结果已提交。
本文件保留为验收清单：下面每一条都可以直接对着当前树核对。交接当时用的独立目录已合并回本仓库，
不要再去`D:\stm32hal\drone-H743-bt-power`重新生成——那会覆盖本仓库已验证的生成结果。

- PC1保持ADC1_INP11，regular rank 1：电流。
- 新增PC0为ADC1_INP10，regular rank 2：电池电压。
- NbrOfConversion=2，ScanConvMode=ENABLE。
- DiscontinuousConvMode=ENABLE，NbrOfDiscConversion=1：每次软件触发只转换一项，CPU读完后再触发下一项，避免任务抢占时后一项覆盖前一项DR。
- 16位、单端、387.5周期、异步时钟/4、EOC_SINGLE_CONV、软件触发、DataPreserved、非DMA和非连续模式均保留。
- 不改DShot、UART8、USART6或现有任务/中断配置。

生成后检查Core/Src/adc.c中：PC0与PC1均为模拟输入；CH11/rank1与CH10/rank2；2个regular conversion与1个discontinuous conversion。
Driver/BSP将在两次Start/Poll/GetValue后统一Stop；转换失败丢弃整个采样对。完整一对读完是序列结束，下一对必然从rank1开始；中途失败则在下次取数前重跑校准（HAL校准会先`ADC_Disable`）把状态机复位，不依赖Stop对discontinuous子组指针的行为。Stop或复位失败则锁存不可用并报ERROR，显式初始化前不再启动转换，避免未知sequencer位置导致两路互换。

板级来源：doc/micoair743v2/vendor/ardupilot-hwdef.dat的PC0/PC1与HAL_BATT_VOLT_SCALE=21.12；电流仍沿用AM32的12.75mV/A，不能误套其另一电调的40.2A/V。

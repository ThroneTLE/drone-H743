# R-BATT-1：ADC双通道生成交接

已在本工作树的`drone-H743.ioc`配置完成，作者只需打开这份工程并Generate Code。
独立目录：`D:\stm32hal\drone-H743-bt-power`。不需要烧录。

- PC1保持ADC1_INP11，regular rank 1：电流。
- 新增PC0为ADC1_INP10，regular rank 2：电池电压。
- NbrOfConversion=2，ScanConvMode=ENABLE。
- DiscontinuousConvMode=ENABLE，NbrOfDiscConversion=1：每次软件触发只转换一项，CPU读完后再触发下一项，避免任务抢占时后一项覆盖前一项DR。
- 16位、单端、387.5周期、异步时钟/4、EOC_SINGLE_CONV、软件触发、DataPreserved、非DMA和非连续模式均保留。
- 不改DShot、UART8、USART6或现有任务/中断配置。

生成后检查Core/Src/adc.c中：PC0与PC1均为模拟输入；CH11/rank1与CH10/rank2；2个regular conversion与1个discontinuous conversion。
Driver/BSP将在两次Start/Poll/GetValue后统一Stop；转换失败丢弃整个采样对，Stop成功后下一轮从rank1重新开始。若Stop失败则锁存不可用，显式初始化前不再启动转换，避免未知sequencer位置导致两路互换。

板级来源：doc/micoair743v2/vendor/ardupilot-hwdef.dat的PC0/PC1与HAL_BATT_VOLT_SCALE=21.12；电流仍沿用AM32的12.75mV/A，不能误套其另一电调的40.2A/V。

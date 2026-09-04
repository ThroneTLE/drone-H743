# drone-H743 硬件参考

> 本文只放稳定的板级事实。具体 COM 号、当前烧录固件、接线改动和测量批次会变化；
> Agent 在执行任何实机动作前必须向用户确认当前设备、端口、固件和授权范围。

## 权威边界

- 引脚、时钟、外设实例、DMA、NVIC、RTOS 对象：`drone-H743.ioc`
- 链接内存区：`STM32H743XX_FLASH.ld`
- 机体坐标：`Driver/Inc/drv_frame_contract.h`
- 当前实机验收状态：`PIPELINE.md`

CubeMX 生成文件不得手改。硬件安装或接线与本文冲突时，先向用户确认，再更新 `.ioc`
并重新生成。

## 已登记器件

| 功能 | 器件/接口 | 软件入口 |
|---|---|---|
| IMU | ICM-42688 | `Driver/Inc/drv_imu.h`、`BSP/Inc/bsp_imu.h` |
| 气压计 | SPL06 | `Driver/Inc/drv_baro.h`、`BSP/Inc/bsp_baro.h` |
| 外部 Flash | GD25Q32，JEDEC `C8 40 16` | `app_flash_service → drv_gd25q32 → bsp_flash_bus` |
| 光流 | MicoLink 光流帧 | `drv_optical_flow → app_optical_flow → svc_flow_nav` |
| 测距 | 光流组合测距与独立 rangefinder 路径 | 对应 App/Driver/BSP |
| 舵机 | 运行时选择 BUS 或 PWM | `app_servo_type`、`app_stabilizer` |
| 电机 | 双 ESC PWM | `drv_motor`、`bsp_pwm` |

外部 Flash 当前板级绑定以 `.ioc` 和 BSP 为准；历史记录中的 SPI1/PA4..PA7 只可作为
线索，不能替代当前工程配置核对。

## 主机连接

应用 USB CDC 的 VID/PID 为 `0483:5740`。Windows COM 号可能变化，工具应按 USB
指纹匹配，不能写死端口或选择列表中的第一个串口。

实机默认归审核者。即使本机能看到串口或调试器，没有 REQ 明文授权也不得烧录、复位、
擦写 Flash 或发送目标板命令。

#ifndef BSP_USB_CDC_H
#define BSP_USB_CDC_H

#include <stdint.h>

/*
 * USB CDC 发送口的板级薄壳。
 *
 * 为什么要这层壳：`CDC_Transmit_FS()` 与 `USBD_OK` 来自 CubeMX 生成的
 * `USB_DEVICE/` 目录（`usbd_cdc_if.h`），那是工具管辖的代码。App 直接包它就同时
 * 绑上了 ST 的 USB 设备栈和这一版的生成结果；换芯片、换 USB 栈、或者在 PC 上
 * 编译这个模块时，都要从 App 里一处处挖出来改。
 *
 * 壳很薄是故意的：这里**不加**重试、不加排队、不加状态机。那些是策略，
 * 属于 App/Src/app_usb_cdc.c；这里只负责"把字节交给 USB 栈，并如实回报收没收下"。
 *
 * 返回 0 = 没交出去（栈忙或未配置），调用方自行决定重试还是计丢。
 */
uint8_t BSP_UsbCdc_Transmit(uint8_t *data, uint16_t length);

/*
 * 停掉并反初始化 USB 设备栈。
 *
 * 复位前先跟主机说一声"这个设备要走了"，主机才会干净地重新枚举；不说的话，
 * 上位机那边会留着一个已经没人应答的串口，看起来像是飞控挂了。
 */
void BSP_UsbCdc_Teardown(void);

#endif /* BSP_USB_CDC_H */

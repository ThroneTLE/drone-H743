#ifndef DRV_BMI270_CONFIG_H
#define DRV_BMI270_CONFIG_H

/*
 * BMI270 上电必须由主机灌入的初始化配置数据。
 *
 * BMI270 内部有一颗可编程协处理器，出厂时**不带**数据通路配置：不灌这份数据
 * 芯片就永远停在 INTERNAL_STATUS != 1，读寄存器都成功，数据却恒为 0。
 * 这是 BMI270 相对 BMI088 最主要的额外工作量。
 *
 * 数据体在 drv_bmi270_config.c，原样取自 Bosch 官方 BSD-3-Clause 实现。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_BMI270_CONFIG_SIZE 328U

extern const uint8_t drv_bmi270_config_file[DRV_BMI270_CONFIG_SIZE];

#ifdef __cplusplus
}
#endif

#endif

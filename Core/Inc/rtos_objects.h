#ifndef RTOS_OBJECTS_H
#define RTOS_OBJECTS_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 功能：声明由 CubeMX/Core/Src/freertos.c 创建的 RTOS 对象。
 * 作用：让 BSP/Driver/App 可以引用调度器对象，同时避免底层模块反向 include App 头文件。
 */
extern osMutexId_t flashBusMutexHandle;

#ifdef __cplusplus
}
#endif

#endif

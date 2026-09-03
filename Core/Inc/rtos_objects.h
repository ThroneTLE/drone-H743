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
/*
 * 姿态/传感器快照队列（深度 1）。Sensor_Task 写，遥测流读。
 * R-T1-1 把消费者从 freertos.c 的 VOFA_task 搬到 App/Src/app_telem_port.c 之后
 * 需要一个跨模块声明，按本文件的既定用途登记在这里。
 */
extern osMessageQueueId_t vofaLogQueueHandle;

#ifdef __cplusplus
}
#endif

#endif

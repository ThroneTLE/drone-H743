/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_diag.h"
#include "app_sensor.h"
#include "app_messages.h"
#include "app_tasks.h"
#include <math.h>

#include "app_vofa.h"
#include "bsp_baro.h"
#include "bsp_imu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t imuDataReadySemaphore;
volatile uint8_t vofaStreamActive = 1U;
/* USER CODE END Variables */
/* Definitions for Stabilizer */
osThreadId_t StabilizerHandle;
const osThreadAttr_t Stabilizer_attributes = {
  .name = "Stabilizer",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for messageTask */
osThreadId_t messageTaskHandle;
const osThreadAttr_t messageTask_attributes = {
  .name = "messageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for backgroundTask */
osThreadId_t backgroundTaskHandle;
const osThreadAttr_t backgroundTask_attributes = {
  .name = "backgroundTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for VOFA_Task */
osThreadId_t VOFA_TaskHandle;
const osThreadAttr_t VOFA_Task_attributes = {
  .name = "VOFA_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTxQueue */
osMessageQueueId_t uartTxQueueHandle;
const osMessageQueueAttr_t uartTxQueue_attributes = {
  .name = "uartTxQueue"
};
/* Definitions for SensorSampleQueue */
osMessageQueueId_t SensorSampleQueueHandle;
const osMessageQueueAttr_t SensorSampleQueue_attributes = {
  .name = "SensorSampleQueue"
};
/* Definitions for backgroundReqQueue */
osMessageQueueId_t backgroundReqQueueHandle;
const osMessageQueueAttr_t backgroundReqQueue_attributes = {
  .name = "backgroundReqQueue"
};
/* Definitions for backgroundRespQueue */
osMessageQueueId_t backgroundRespQueueHandle;
const osMessageQueueAttr_t backgroundRespQueue_attributes = {
  .name = "backgroundRespQueue"
};
/* Definitions for vofaLogQueue */
osMessageQueueId_t vofaLogQueueHandle;
const osMessageQueueAttr_t vofaLogQueue_attributes = {
  .name = "vofaLogQueue"
};
/* Definitions for flashBusMutex */
osMutexId_t flashBusMutexHandle;
const osMutexAttr_t flashBusMutex_attributes = {
  .name = "flashBusMutex",
  .attr_bits = osMutexRecursive,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StabilizerTask(void *argument);
void Sensor_Task(void *argument);
void message_push(void *argument);
void UART_fun(void *argument);
void BackgroundTask(void *argument);
void VOFA_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   APP_Diag_RecordStackOverflow(pcTaskName);
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
   APP_Diag_RecordMallocFailed();
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the recursive mutex(es) */
  /* creation of flashBusMutex */
  flashBusMutexHandle = osMutexNew(&flashBusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  imuDataReadySemaphore = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartTxQueue */
  uartTxQueueHandle = osMessageQueueNew (32, sizeof(APP_UART_TxMessage), &uartTxQueue_attributes);

  /* creation of SensorSampleQueue */
  SensorSampleQueueHandle = osMessageQueueNew (8, sizeof(APP_Sensor_SampleMessage), &SensorSampleQueue_attributes);

  /* creation of backgroundReqQueue */
  backgroundReqQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundRequest), &backgroundReqQueue_attributes);

  /* creation of backgroundRespQueue */
  backgroundRespQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundResponse), &backgroundRespQueue_attributes);

  /* creation of vofaLogQueue */
  vofaLogQueueHandle = osMessageQueueNew (1, sizeof(APP_Sensor_SampleMessage), &vofaLogQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Stabilizer */
  StabilizerHandle = osThreadNew(StabilizerTask, NULL, &Stabilizer_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(Sensor_Task, NULL, &SensorTask_attributes);

  /* creation of messageTask */
  messageTaskHandle = osThreadNew(message_push, NULL, &messageTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(UART_fun, NULL, &UARTTask_attributes);

  /* creation of backgroundTask */
  backgroundTaskHandle = osThreadNew(BackgroundTask, NULL, &backgroundTask_attributes);

  /* creation of VOFA_Task */
  VOFA_TaskHandle = osThreadNew(VOFA_task, NULL, &VOFA_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StabilizerTask */
/**
  * @brief  Function implementing the Stabilizer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StabilizerTask */
void StabilizerTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StabilizerTask */

  /*
   * Stabilizer 任务：姿态融合 + 控制输出。
   *
   * 数据流：
   *   Sensor_Task → SensorSampleQueue → 这里
   *     ① 互补滤波算出 roll/pitch/yaw
   *     ② 写入 vofaLogQueue（VOFA_task 50Hz 发送到上位机）
   *     ③ TODO: 控制输出
   */

  APP_Sensor_SampleMessage msg;
  float  roll  = 0.0f;
  float  pitch = 0.0f;
  float  yaw   = 0.0f;
  uint32_t last_tick_ms = 0U;
  uint32_t stabilizer_seq = 0U;

  for(;;)
  {
    if (osSemaphoreAcquire(imuDataReadySemaphore, osWaitForever) == osOK) {
      if (osMessageQueueGet(SensorSampleQueueHandle, &msg, 0U, 0U) == osOK) {
        stabilizer_seq++;

        /* ① 互补滤波（加速度 + 陀螺仪融合） */
        APP_IMU_UpdateAttitude(&msg.imu, &roll, &pitch, &yaw,
                               &last_tick_ms, stabilizer_seq);

        /* ② 写入 VOFA 日志队列（带姿态，覆盖模式） */
        msg.roll_deg  = roll;
        msg.pitch_deg = pitch;
        msg.yaw_deg   = yaw;

        if (osMessageQueuePut(vofaLogQueueHandle, &msg, 0U, 0U) != osOK) {
          APP_Sensor_SampleMessage drop;
          (void)osMessageQueueGet(vofaLogQueueHandle, &drop, 0U, 0U);
          (void)osMessageQueuePut(vofaLogQueueHandle, &msg, 0U, 0U);
        }

        /* ③ TODO: 控制输出（电机/PWM） */
      }
    }
  }
  /* USER CODE END StabilizerTask */
}

/* USER CODE BEGIN Header_Sensor_Task */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sensor_Task */
void Sensor_Task(void *argument)
{
  /* USER CODE BEGIN Sensor_Task */
  BSP_IMU_Invalidate();

  /* APP_Task_GPS_Init();  暂时停止 GPS */
  APP_Task_MAG_Init();

  /* ---- 初始化 IMU（重试直到成功） ---- */
  while (BSP_IMU_Init() != DRV_IMU_OK) {
    osDelay(50);
  }
  BSP_BARO_Init();   /* 气压计初始化，失败时在读数据时重试 */

  DRV_IMU_RawData    raw;
  DRV_IMU_ScaledData scaled;
  uint32_t sample_count = 0U;
  uint32_t baro_cnt     = 0U;
  float    baro_pa      = 0.0f;   /* 保持旧值直到下次气压计更新 */
  float    baro_temp    = 0.0f;

  /* 陀螺仪零偏校准状态 */
  APP_Sensor_GyroBias gyro_bias = {0};

  /* 低通滤波器：陀螺 80Hz，加速度 30Hz (dt = 0.001s @ 1kHz) */
  APP_Sensor_Lpf gyro_lpf[3], acc_lpf[3];
  for (uint32_t i = 0U; i < 3U; i++) {
    APP_Sensor_LpfInit(&gyro_lpf[i], 80.0f, 0.001f);
    APP_Sensor_LpfInit(&acc_lpf[i], 30.0f, 0.001f);
  }

  osDelay(10);

  for(;;)
  {
    /* ---- 等待 PC0 数据就绪中断，超时后轮询 ---- */
    bool imu_ready = false;
    if ((osThreadFlagsWait(0x0001U, osFlagsWaitAny, 10U) & 0x0001U) == 0U) {
      (void)BSP_IMU_IsDataReady(&imu_ready);
      if (!imu_ready) { continue; }
    }
    imu_ready = true;

    /* ---- 读取 IMU 原始值，转换为物理单位 ---- */
    if (BSP_IMU_ReadRaw(&raw) != DRV_IMU_OK) {
      BSP_IMU_Invalidate();
      osDelay(20);
      continue;
    }

    sample_count++;
    APP_IMU_RawToScaled(&raw, &scaled);

    /* ---- 陀螺仪零偏校准（前 1000 个样本取均值） ---- */
    {
      float g[3] = {scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps};
      if (APP_Sensor_CalibrateGyroBias(g[0], g[1], g[2], &gyro_bias)) {
        /* 校准刚完成，第一次减零偏自然生效 */
      }
      if (gyro_bias.ready) {
        g[0] -= gyro_bias.bias[0];
        g[1] -= gyro_bias.bias[1];
        g[2] -= gyro_bias.bias[2];
      }
      scaled.gyro_x_dps = g[0];
      scaled.gyro_y_dps = g[1];
      scaled.gyro_z_dps = g[2];
    }

    /* ---- 坐标系对齐 + 低通滤波 ---- */
    {
      float a[3] = {scaled.accel_x_g, scaled.accel_y_g, scaled.accel_z_g};
      float g[3] = {scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps};
      float a_align[3], g_align[3];

      APP_Sensor_AlignToAirframe(a, a_align);
      APP_Sensor_AlignToAirframe(g, g_align);
      APP_Sensor_LpfApply3f(gyro_lpf, g_align, g_align);
      APP_Sensor_LpfApply3f(acc_lpf,  a_align, a_align);

      scaled.accel_x_g  = a_align[0];  scaled.accel_y_g = a_align[1];  scaled.accel_z_g = a_align[2];
      scaled.gyro_x_dps = g_align[0];  scaled.gyro_y_dps = g_align[1]; scaled.gyro_z_dps = g_align[2];
    }

    /* ---- 气压计：1kHz IMU 触发中每 32 次读一次（≈32Hz） ---- */
    uint8_t baro_fresh = 0U;

    if (++baro_cnt >= 32U) {
      baro_cnt = 0U;
      uint8_t buf[6];

      if (BSP_BARO_ReadRawRegisters(0x00U, buf, 6U) == DRV_BARO_OK) {
        /* SPL06-007：24 位补码，MSB 在前 */
        int32_t prs_raw = ((int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8))) >> 8;
        int32_t tmp_raw = ((int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8))) >> 8;

        APP_IMU_ConvertBaro(prs_raw, tmp_raw, &baro_pa, &baro_temp);
        baro_fresh = 1U;
      } else {
        BSP_BARO_Invalidate();
        BSP_BARO_Init();
      }
    }

    /* ---- 推送传感器数据给 StabilizerTask ---- */
    APP_Sensor_SampleMessage msg;

    msg.base.timestamp_us    = SVC_Timestamp_Us();
    msg.base.type            = APP_SENSOR_TYPE_IMU;
    msg.base.sequence        = sample_count;
    msg.imu                  = scaled;
    msg.roll_deg             = 0.0f;   /* Stabilizer 填入 */
    msg.pitch_deg            = 0.0f;
    msg.yaw_deg              = 0.0f;
    msg.baro_updated         = baro_fresh;
    msg.baro_pressure_pa     = baro_pa;
    msg.baro_temperature_c   = baro_temp;
    msg.imu_data_ready_count = 0U;

    if (osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U) != osOK) {
      APP_Sensor_SampleMessage drop;
      (void)osMessageQueueGet(SensorSampleQueueHandle, &drop, 0U, 0U);
      (void)osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U);
    }

    (void)osSemaphoreRelease(imuDataReadySemaphore);

    /* MAG 按自己的节奏运行（非阻塞步进） */
    /* APP_Task_GPS_Step();  暂时停止 GPS */
    APP_Task_MAG_Step();
  }
  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_message_push */
/**
* @brief Function implementing the messageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_message_push */
void message_push(void *argument)
{
  /* USER CODE BEGIN message_push */
  APP_Task_Message_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Message_Step();
  }
  /* USER CODE END message_push */
}

/* USER CODE BEGIN Header_UART_fun */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_fun */
void UART_fun(void *argument)
{
  /* USER CODE BEGIN UART_fun */
  APP_Task_UART_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_UART_Step();
  }
  /* USER CODE END UART_fun */
}

/* USER CODE BEGIN Header_BackgroundTask */
/**
* @brief Function implementing the backgroundTask thread.
* @param argument: Not used
* @retval None
*
* 功能：后台低优先级任务。
* 这个任务负责执行不应该阻塞控制环的慢操作，例如参数从 FLASH 自动加载、
* 参数异步保存、维护模式 FLASH 块读写。它是任务执行体；真正的数据规则
* 放在 Services 层的 Param/Background API 中。
*/
/* USER CODE END Header_BackgroundTask */
void BackgroundTask(void *argument)
{
  /* USER CODE BEGIN BackgroundTask */
  APP_Task_Background_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Background_Step();
  }
  /* USER CODE END BackgroundTask */
}

/* USER CODE BEGIN Header_VOFA_task */
/**
* @brief Function implementing the VOFA_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA_task */
void VOFA_task(void *argument)
{
  /* USER CODE BEGIN VOFA_task */

  /*
   * 高度：开机后气压计数据先累积 50 帧做平均作为地面气压 p0_pa，
   * 之后用相对高度公式估算。
   * TODO: 若需要更精确的高度，可以在 Stabilizer 里做卡尔曼融合。
   */
  APP_Sensor_SampleMessage msg;
  #define VOFA_DATA_SIZE 11U
  float vofa_data[VOFA_DATA_SIZE];

  /* 地面气压平均 */
  #define VOFA_P0_SAMPLES 50U
  float  p0_sum = 0.0f;
  uint32_t p0_cnt = 0U;

  for(;;)
  {
    osDelay(200);  /* 10Hz 发送（WiFi 透传不积包） */

    if (!vofaStreamActive) {
      continue;
    }

    if (osMessageQueueGet(vofaLogQueueHandle, &msg, 0U, 0U) == osOK) {
      /* 姿态 */
      vofa_data[0] = msg.roll_deg;             /* 横滚  °   */
      vofa_data[1] = msg.pitch_deg;            /* 俯仰  °   */
      vofa_data[2] = msg.yaw_deg;              /* 偏航  °   */

      /* 高度（开机前 50 帧做地面气压平均） */
      if (p0_cnt < VOFA_P0_SAMPLES) {
        if (msg.baro_pressure_pa > 10000.0f && msg.baro_pressure_pa < 110000.0f) {
          p0_sum += msg.baro_pressure_pa;
          p0_cnt++;
        }
        vofa_data[3] = 0.0f;
      } else {
        float p0 = p0_sum / (float)VOFA_P0_SAMPLES;
        float ratio = msg.baro_pressure_pa / p0;
        if (ratio > 0.0f) {
          vofa_data[3] = 44330.0f * (1.0f - powf(ratio, 0.190295f));
        } else {
          vofa_data[3] = 0.0f;
        }
      }

      /* 加速度原始值 */
      vofa_data[4] = msg.imu.accel_x_g;        /* ax [g] */
      vofa_data[5] = msg.imu.accel_y_g;        /* ay [g] */
      vofa_data[6] = msg.imu.accel_z_g;        /* az [g] */

      /* 陀螺仪原始值（已减零偏 + 低通） */
      vofa_data[7] = msg.imu.gyro_x_dps;       /* gx [dps] */
      vofa_data[8] = msg.imu.gyro_y_dps;       /* gy [dps] */
      vofa_data[9] = msg.imu.gyro_z_dps;       /* gz [dps] */
      vofa_data[10] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;  /* 时间戳 [s] */

      APP_VOFA_SendFloats(vofa_data, VOFA_DATA_SIZE);
    }
  }
  /* USER CODE END VOFA_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


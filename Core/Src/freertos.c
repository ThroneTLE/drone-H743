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
/*
 * ============================================================================
 * 模块依赖说明
 * ============================================================================
 * App 层（应用逻辑）：
 *   app_diag.h       — 诊断记录（栈溢出 / malloc 失败时记录到 FLASH）
 *   app_sensor.h     — 传感器数据处理（零偏校准、坐标系对齐、低通滤波）
 *   app_messages.h   — 消息协议编解码（JSON / 二进制帧封装）
 *   app_tasks.h      — 各任务 Init/Step 函数声明
 *   app_telem_stream.h — 遥测流 v2（通道装配、掩码帧、双出口）
 *   app_elrs.h       — ELRS 遥控器链路（通道值读取）
 *
 * BSP 层（板级支持包——硬件抽象）：
 *   bsp_baro.h       — 气压计 SPL06-007 驱动
 *   bsp_imu.h        — IMU (ICM-42688-P) 驱动
 *   bsp_pwm.h        — TIM2 PWM 输出（CH1/2 电调，CH3/4 普通舵机）
 *   bsp_aiwb2_power.h — Ai-WB2 WiFi 模块电源控制
 *
 * Driver 层（外设驱动——算法 / 协议）：
 *   drv_coax_ctrl.h  — 同轴倾转旋翼控制器（论文控制分配）
 */
#include "app_diag.h"
#include "app_flight_log.h"
#include "app_imu_capture.h"
#include "app_imu_health.h"
#include "app_led.h"
#include "app_nav_estimator.h"
#include "app_sensor.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_telemetry.h"
#include "app_ident.h"
#include "app_servo_cal.h"
#include "app_servo_jog.h"
#include "app_servo_feedback.h"
#include "app_servo_feedback_bench.h"
#include "app_stabilizer.h"
#include <math.h>
#include <string.h>

#include "app_telem_stream.h"
#include "app_elrs.h"
#include "app_optical_flow.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "bsp_bus_servo.h"
#include "bsp_pwm.h"
#include "bsp_aiwb2_power.h"
#include "drv_attitude_fusion.h"
#include "drv_coax_ctrl.h"
#include "drv_nav_ekf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * ============================================================================
 * 传感器相关常量
 * ============================================================================
 * IMU 数据就绪优先由 PC0 外部中断触发，中断中设置 Thread Flag。
 * 若短时间未等到中断，只读取一次 IMU ready 状态作为兜底；坏帧/短暂丢帧会被跳过。
 */
#define SENSOR_IMU_DATA_READY_FLAG     0x0001U  /* 位 0：IMU 数据就绪事件标志          */
#define SENSOR_IMU_DRDY_TIMEOUT_MS     20U      /* 单次等待 DRDY 的超时时间            */
#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT 50U    /* 连续 1s 无 DRDY/ready 才锁存故障    */
#define SENSOR_IMU_READ_FAIL_LIMIT     25U      /* 连续读失败次数，超过后锁存故障      */
#define SENSOR_MAG_PERIOD_US           50000ULL /* 磁力计步进间隔 50ms = 20Hz          */
/* 稳定器/导航/舵机常量已移至 App/Src/app_stabilizer.c */
/* 遥测帧的周期、长度、通道装配已移至 App/Src/app_telem_stream.c + app_telem_port.c */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*
 * ============================================================================
 * 全局同步对象
 * ============================================================================
 * imuDataReadySemaphore — IMU 数据就绪信号量
 *   生产者：Sensor_Task（每次 IMU 采样完成后 release）
 *   消费者：StabilizerTask（acquire 后开始消费 SensorSampleQueue）
 *   初始值 0，最大值 1（二进制信号量，只做事件通知不做计数）
 *
 * 遥测流开关 vofaStreamActive 已移至 App/Src/app_telem_stream.c。
 */
osSemaphoreId_t imuDataReadySemaphore;

/* USER CODE END Variables */
/* Definitions for Stabilizer */
osThreadId_t StabilizerHandle;
const osThreadAttr_t Stabilizer_attributes = {
  .name = "Stabilizer",
  .stack_size = 2048 * 4,
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
  /*
   * 2026-09-11 由 osPriorityLow 提到 BelowNormal。
   *
   * Low 比 messageTask / backgroundTask（都是 BelowNormal）还低，于是只有在
   * 它们和 1 kHz 的 Stabilizer 全都让出 CPU 时才轮得到。实测这条件不成立：
   * `RTOS?` 报 TELEM state=1(Ready)，而任务内第一条语句的计数器从上电起一直是
   * 0——**一整拍都没跑过**，遥测流表现为 stream=1 却一个字节都不来。
   *
   * 遥测是要按固定节拍产出的任务，不该靠捡剩余时间片。放到 BelowNormal 与
   * 消息/后台同级，仍然低于控制环（Normal），不会跟稳定环抢时间。
   * 改这里必须同步改 drone-H743.ioc 的 FREERTOS.Tasks01（VOFA_Task 优先级 8→16）。
   */
  .priority = (osPriority_t) osPriorityBelowNormal,
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
/*
 * vApplicationStackOverflowHook — 任务栈溢出钩子
 *
 * 触发条件：configCHECK_FOR_STACK_OVERFLOW 设置为 1 或 2 时，
 * FreeRTOS 在每个任务切换时检查栈指针是否越界。
 *
 * 处理策略：
 *   1. 记录栈溢出事件到诊断系统（APP_Diag_RecordStackOverflow）
 *   2. 关全局中断（taskDISABLE_INTERRUPTS）
 *   3. 死循环——栈溢出是不可恢复的错误，继续运行会导致内存损坏
 *
 * 排查方法：如果某个任务反复触发此钩子，增大该任务的 .stack_size，
 * 或检查函数内的局部变量是否过大（大数组应改为 static）。
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   (void)xTask;
   APP_Diag_RecordStackOverflow(pcTaskName);
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
/*
 * vApplicationMallocFailedHook — 动态内存分配失败钩子
 *
 * 触发条件：configUSE_MALLOC_FAILED_HOOK = 1 时，
 * pvPortMalloc() 返回 NULL（堆内存耗尽）时调用。
 *
 * FreeRTOS 内部在创建任务/队列/信号量/定时器时都会调用 pvPortMalloc()。
 * 堆大小由 FreeRTOSConfig.h 中的 configTOTAL_HEAP_SIZE 定义。
 *
 * 处理策略：与栈溢出相同——记录诊断信息后死循环。
 *
 * 排查方法：检查 configTOTAL_HEAP_SIZE 是否足够；
 * 检查是否有内存泄漏（创建对象后未删除）；
 * 使用 xPortGetFreeHeapSize() 监控剩余堆空间。
 */
void vApplicationMallocFailedHook(void)
{
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
  /* 创建 IMU 数据就绪信号量：初始 0，最大 1，二进制事件通知 */
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
  APP_Task_LED_Init();
  APP_ServoCal_Init();
  APP_ServoJog_Init();
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
  * @brief  StabilizerTask —— 姿态融合 + 控制输出（核心控制线程）
  * @param  argument: Not used
  * @retval None
  *
  * 这是整个飞控的核心任务，负责：
  *   1. 等待 IMU 数据就绪（通过 imuDataReadySemaphore 信号量）
  *   2. 从 SensorSampleQueue 消费传感器数据
  *   3. x-io Fusion AHRS → roll / pitch / yaw
  *   4. 将融合后的姿态写入 vofaLogQueue（VOFA_task 在上位机显示）
  *   5. 按 25Hz 控制周期输出舵机指令
  *
  * 数据流：
  *   Sensor_Task → SensorSampleQueue → 这里
  *     ① Fusion 四元数传播、动态加速度拒绝和自动恢复
  *     ② 写入 vofaLogQueue（VOFA_task 50Hz 发送到上位机）
  *     ③ 控制器更新（500Hz，舵机总线仅在目标变化时发送）
  *
  * 舵机控制有两种模式（编译期切换）：
  *   模式 A (USE_DIRECT_ANGLE_SERVO=1)：姿态角直接映射为舵机脉宽
  *   模式 B (USE_DIRECT_ANGLE_SERVO=0)：经过 Simulink 生成的同轴控制器
  *
  * 注意：未满足安全条件时断开电调 PWM 输出（CCR=0）。
  */
/* USER CODE END Header_StabilizerTask */
void StabilizerTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StabilizerTask */

  /* 核心控制逻辑已拆到独立模块 App/Src/app_stabilizer.c */
  APP_Stabilizer_Run(imuDataReadySemaphore, SensorSampleQueueHandle,
                     vofaLogQueueHandle);

  /* USER CODE END StabilizerTask */
}

/* USER CODE BEGIN Header_Sensor_Task */
/**
  * @brief  Sensor_Task —— IMU 传感器采集与数据预处理（1kHz，中断驱动）
  * @param  argument: Not used
  * @retval None
  *
  * 这是整个飞控的数据源头，负责：
  *   1. 初始化 IMU（ICM-42688-P）、气压计（SPL06-007）、磁力计（QMC5883L）
  *   2. 等待 PC0 外部中断通知（IMU 数据就绪），超时后轮询兜底
  *   3. 读取 IMU 原始寄存器值 → 转换为物理单位（dps / g）
  *   4. 陀螺仪零偏校准（前 1000 个样本自动均值，要求静止）
  *   5. 坐标系对齐（ICM 芯片坐标系 → 机体 NED 坐标系）
  *   6. 低通滤波（陀螺 80Hz、加速度 30Hz，IIR 一阶）
  *   7. 气压计降采样读取（1kHz 中每 32 次读一次 ≈ 32Hz）
  *   8. 组装 SensorSampleMessage → 推入 SensorSampleQueue
  *   9. Release imuDataReadySemaphore 通知 StabilizerTask 消费
  *
  * 运行时序：
  *   硬件 IRQ ──→ Thread Flag ──→ 本任务唤醒 ──→ SPI 读取 ──→ 处理流水线
  *   ──→ 推送队列 ──→ Release 信号量 ──→ 再次阻塞等待 Thread Flag
  *
  * 涉及的硬件接口：
  *   SPI1 → ICM-42688-P (IMU, 加速度 + 陀螺仪)
  *   I2C1 → SPL06-007   (气压计)
  *   I2C1 → QMC5883L    (磁力计)
  *   PC0  → EXTI 中断   (IMU INT1 数据就绪引脚)
  */
/* USER CODE END Header_Sensor_Task */
void Sensor_Task(void *argument)
{
  /* USER CODE BEGIN Sensor_Task */
  BSP_IMU_Invalidate();                  /* 复位 IMU 驱动内部状态              */
  APP_ImuHealth_Init();                  /* 采样链健康监测（DRDY 静默降级检测） */

  /* APP_Task_GPS_Init();  暂时停止 GPS */
  APP_Task_OpticalFlow_Init();
  APP_Task_MAG_Init();                   /* 初始化磁力计 QMC5883L              */

  /* ---- 初始化 IMU（重试直到成功） ---- */
  while (BSP_IMU_Init() != DRV_IMU_OK) {
    osDelay(50);
  }
  BSP_BARO_Init();   /* 气压计初始化，失败时在读数据时重试 */

  DRV_IMU_RawData    raw;               /* IMU 原始 ADC 值（寄存器原始读数）    */
  DRV_IMU_ScaledData scaled;            /* IMU 物理单位值（dps / g）            */
  uint32_t sample_count = 0U;           /* 总采样帧计数（溢出回绕是安全的）     */
  uint32_t baro_cnt     = 0U;           /* 气压计降采样计数器（32 分频）        */
  uint64_t last_mag_step_us = 0ULL;     /* 磁力计上次步进时间 [μs]              */
  float    baro_pa      = 0.0f;         /* 当前气压值 [Pa]，保持旧值直到更新    */
  float    baro_temp    = 0.0f;         /* 当前气压计温度 [°C]                  */

  /* 陀螺仪零偏校准状态 */
  APP_Sensor_GyroBias gyro_bias = {0};
  APP_Sensor_RateMeter imu_rate_meter = {0}; /* IMU 实际采样率统计              */
  APP_Sensor_RateMeter imu_irq_rate_meter = {0};
  APP_Sensor_RateMeter imu_poll_rate_meter = {0};

  /*
   * 低通滤波器：二阶 Butterworth，陀螺 80Hz，加速度 40Hz，dt = 0.001s。
   *
   * 截止频率由实测定速扫描确定（data/captures/imu_vibration/）：桨叶通过频率
   * 随油门从 56Hz 线性升到悬停的 180Hz（r=0.996，确认是真实振动非混叠）。
   * 80Hz 二阶在 180Hz 给 -15.9dB，原一阶只有 -8.9dB；代价是 10Hz 处群延迟
   * 从 1.93ms 增到 2.80ms，对姿态环可接受。
   *
   * 加速度抬到 40Hz（原 30Hz）：二阶滚降已足够陡，不必再靠压低截止频率换
   * 衰减，抬高可减少重力方向的相位滞后。
   */
  APP_Sensor_Lpf gyro_lpf[3], acc_lpf[3];
  uint32_t imu_irq_ready_count = 0U;
  uint32_t imu_poll_ready_count = 0U;
  uint32_t imu_read_fail_count = 0U;
  uint32_t imu_drdy_miss_count = 0U;
  for (uint32_t i = 0U; i < 3U; i++) {
    APP_Sensor_LpfInit(&gyro_lpf[i], 80.0f, 0.001f);
    APP_Sensor_LpfInit(&acc_lpf[i], 40.0f, 0.001f);
  }

  osDelay(10);

  for(;;)
  {
    uint64_t imu_sample_timestamp_us = APP_SENSOR_TIMESTAMP_INVALID;
    uint32_t imu_ready_flags;

    /*
     * 步骤 1：等待 IMU 数据就绪
     * PC0 EXTI → HAL_GPIO_EXTI_Callback → osThreadFlagsSet → 本任务被唤醒。
     * 未等到中断时只读一次 ready 状态；若仍未 ready，则跳过本轮继续等下一帧。
     */
    imu_ready_flags =
      osThreadFlagsWait(SENSOR_IMU_DATA_READY_FLAG, osFlagsWaitAny,
                        SENSOR_IMU_DRDY_TIMEOUT_MS);
    if ((imu_ready_flags & SENSOR_IMU_DATA_READY_FLAG) != 0U) {
      imu_irq_ready_count++;
      imu_drdy_miss_count = 0U;
      APP_ImuHealth_NoteSample(1U);
      if (APP_IMU_ReadDataReadyTimestamp(&imu_sample_timestamp_us) == 0U) {
        imu_sample_timestamp_us = SVC_Timestamp_Us();
      }
    } else {
      bool imu_ready = false;

      if ((BSP_IMU_IsDataReady(&imu_ready) == DRV_IMU_OK) && imu_ready) {
        imu_poll_ready_count++;
        /*
         * 这里刻意不再清零 imu_drdy_miss_count：轮询兜底成功只说明"拿到了
         * 数据"，不说明"中断是健康的"。以前两者混为一谈，导致 DRDY 永久失效时
         * 兜底每 20ms 成功一次、miss 计数被反复清零，故障门永远不触发，整条链
         * 静默降到 50Hz。miss 计数现在只在真正收到中断时清零。
         */
        imu_drdy_miss_count++;
        APP_ImuHealth_NoteSample(0U);
        imu_sample_timestamp_us = SVC_Timestamp_Us();
        if (imu_drdy_miss_count >= SENSOR_IMU_DRDY_MISS_FAULT_LIMIT) {
          APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);
        }
      } else {
        imu_drdy_miss_count++;
        if (imu_drdy_miss_count >= SENSOR_IMU_DRDY_MISS_FAULT_LIMIT) {
          APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);
          (void)osSemaphoreRelease(imuDataReadySemaphore);
        }
        continue;
      }
    }

    /* ---- 步骤 2：SPI 读取 IMU 原始值，转换为物理单位 ---- */
    if (BSP_IMU_ReadRaw(&raw) != DRV_IMU_OK) {
      imu_read_fail_count++;
      if (imu_read_fail_count >= SENSOR_IMU_READ_FAIL_LIMIT) {
        APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_READ_FAIL);
        (void)osSemaphoreRelease(imuDataReadySemaphore);
      }
      continue;
    }
    imu_read_fail_count = 0U;
    /*
     * 只有中断链健康（本帧由 DRDY 唤醒）时才清除故障锁存。以前无条件清除，
     * 于是上一行刚锁存的 DRDY_TIMEOUT 会被下一帧立刻抹掉，故障标志形同虚设。
     */
    if (imu_drdy_miss_count == 0U) {
      APP_Stabilizer_ClearImuFault();
    }
    APP_Stabilizer_MarkImuSample(HAL_GetTick());
    APP_ImuHealth_Update(HAL_GetTick());

    sample_count++;

    /*
     * 振动谱采集钩子：必须放在缩放/对齐/低通之前。
     * 飞行日志和 VOFA 都是抽取后且已滤波的数据，看不到 150~300 Hz 的桨叶带，
     * 无法用来确定 AAF 和 IIR 的截止频率；这里取的是未经处理的原始 LSB。
     * 未启动采集时该调用直接返回，不会阻塞 1kHz 采样节奏。
     */
    APP_IMU_Capture_Push((uint32_t)imu_sample_timestamp_us,
                         &raw,
                         BSP_PWM_GetEscPulse(1),
                         BSP_PWM_GetEscPulse(2));

    APP_IMU_RawToScaled(&raw, &scaled);  /* ADC → 物理单位（dps / g）         */

    /*
     * 步骤 3：陀螺仪零偏校准
     * APP_Sensor_CalibrateGyroBias() 内部累积前 1000 个样本求均值。
     * 校准完成后（gyro_bias.ready == true），后续所有采样都减去零偏。
     * 重要：校准时飞行器必须完全静止，否则零偏不准确。
     */
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

    /*
     * 步骤 4：坐标系对齐 + 低通滤波
     * ICM-42688-P 芯片坐标系与机体坐标系不一定一致（取决于 PCB 焊接方向）。
     * APP_Sensor_AlignToAirframe() 通过轴重排和取反将传感器轴映射到机体轴。
     * 机体坐标系定义（NED 约定）：
     *   X → 机头前方,  Y → 机身右侧,  Z → 机身下方
     * 对齐后分别对陀螺和加速度施加一阶 IIR 低通滤波。
     */
    {
      float a[3] = {scaled.accel_x_g, scaled.accel_y_g, scaled.accel_z_g};
      float g[3] = {scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps};
      float a_align[3], g_align[3];

      APP_Sensor_AlignToAirframe(a, a_align);
      APP_Sensor_AlignToAirframe(g, g_align);
      APP_Sensor_LpfApply3f(gyro_lpf, g_align, g_align); /* IIR 低通：陀螺    */
      APP_Sensor_LpfApply3f(acc_lpf,  a_align, a_align); /* IIR 低通：加速度  */

      /* 记录滤波后的值，使 IIR 的实际衰减可以和同一帧的输入直接对比。 */
      APP_IMU_Capture_AnnotateFiltered(a_align, g_align, gyro_bias.ready);

      scaled.accel_x_g  = a_align[0];  scaled.accel_y_g = a_align[1];  scaled.accel_z_g = a_align[2];
      scaled.gyro_x_dps = g_align[0];  scaled.gyro_y_dps = g_align[1]; scaled.gyro_z_dps = g_align[2];
    }

    /*
     * 步骤 5：气压计降采样读取（≈32Hz）
     * 气压计不需要 1kHz 更新——大气压力变化缓慢（< 10Hz）。
     * 每 32 次 IMU 循环（= 32ms @ 1kHz）读取一次，实际频率 ≈ 31.25Hz。
     * SPI06-007 偶有上电复位后无响应的问题——读取失败时重新初始化。
     */
    uint8_t baro_fresh = 0U;

    if (++baro_cnt >= 32U) {
      baro_cnt = 0U;
      uint8_t buf[6];                    /* 3 字节压力原始值 + 3 字节温度原始值 */

      if (BSP_BARO_ReadRawRegisters(0x00U, buf, 6U) == DRV_BARO_OK) {
        /* SPL06-007：24 位补码，MSB 在前，左移后算术右移完成符号扩展 */
        int32_t prs_raw = ((int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8))) >> 8;
        int32_t tmp_raw = ((int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8))) >> 8;

        APP_IMU_ConvertBaro(prs_raw, tmp_raw, &baro_pa, &baro_temp);
        baro_fresh = 1U;
      } else {
        /* 读取失败：复位驱动状态 + 重新初始化传感器 */
        BSP_BARO_Invalidate();
        BSP_BARO_Init();
      }
    }

    /*
     * 步骤 6：组装传感器消息 → 推送队列 → 通知 StabilizerTask
     * SensorSampleMessage 包含：
     *   - 时间戳（SVC_Timestamp_Us：32 位微秒硬件计数器）
     *   - IMU 数据（加速度 + 陀螺仪，已滤波 + 坐标对齐 + 零偏校正）
     *   - 气压计数据（压力 + 温度，baro_updated 标志是否本帧新鲜）
     *   - IMU 采样率统计
     *
     * 队列满处理：丢弃最旧的一帧再写入（覆盖模式），保证 Stabilizer 拿到最新数据。
     *
     * 顺序关键：必须先 Push 队列，再 Release 信号量！
     * 否则 StabilizerTask 被唤醒后发现队列空，白跑一趟。
     */
    APP_Sensor_SampleMessage msg;

    msg.base.timestamp_us    = imu_sample_timestamp_us;
    msg.base.type            = APP_SENSOR_TYPE_IMU;
    msg.base.sequence        = sample_count;
    msg.raw_imu              = raw;
    msg.imu                  = scaled;
    msg.roll_deg             = 0.0f;   /* Stabilizer 填入 */
    msg.pitch_deg            = 0.0f;
    msg.yaw_deg              = 0.0f;
    msg.baro_updated         = baro_fresh;
    msg.baro_pressure_pa     = baro_pa;
    msg.baro_temperature_c   = baro_temp;
    msg.imu_sample_rate_hz   = APP_SensorRateMeter_Update(&imu_rate_meter,
                                                          msg.base.timestamp_us,
                                                          sample_count);
    msg.imu_irq_sample_rate_hz  = APP_SensorRateMeter_Update(&imu_irq_rate_meter,
                                                              msg.base.timestamp_us,
                                                              imu_irq_ready_count);
    msg.imu_poll_sample_rate_hz = APP_SensorRateMeter_Update(&imu_poll_rate_meter,
                                                              msg.base.timestamp_us,
                                                              imu_poll_ready_count);
    msg.imu_age_ms = 0.0f;
    memset(&msg.attitude_debug, 0, sizeof(msg.attitude_debug));
    msg.gyro_bias_ready = gyro_bias.ready;
    msg.imu_data_ready_count = imu_irq_ready_count;
    msg.imu_poll_ready_count = imu_poll_ready_count;
    msg.fusion_acceleration_error_deg = 0.0f;
    msg.fusion_acceleration_recovery_trigger = 0.0f;
    msg.fusion_accel_correction_count = 0U;
    msg.fusion_accelerometer_ignored = 0U;
    msg.fusion_acceleration_recovery = 0U;
    msg.fusion_angular_rate_recovery = 0U;
    msg.fusion_accel_norm_rejected = 0U;

    if (osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U) != osOK) {
      APP_Sensor_SampleMessage drop;
      (void)osMessageQueueGet(SensorSampleQueueHandle, &drop, 0U, 0U);
      (void)osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U);
    }

    /* 步骤 7：Release 信号量 → 唤醒 StabilizerTask */
    (void)osSemaphoreRelease(imuDataReadySemaphore);

    /*
     * 步骤 8：磁力计步进（非阻塞，20Hz 独立节奏）
     * 磁力计挂在 I2C 总线上，读取速度慢（~100Hz max），且数据不需要 1kHz 更新。
     * 使用独立的时间间隔 SENSOR_MAG_PERIOD_US (50ms = 20Hz) 来控制步进。
     */
                             /* APP_Task_GPS_Step();  暂时停止 GPS */
    APP_Task_OpticalFlow_Step();
    {
      uint64_t now_us = msg.base.timestamp_us;

      if ((last_mag_step_us == 0ULL) ||
          ((now_us - last_mag_step_us) >= SENSOR_MAG_PERIOD_US)) {
        last_mag_step_us = now_us;
        APP_Task_MAG_Step();
      }
    }
  }
  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_message_push */
/**
  * @brief  messageTask —— 消息协议处理（JSON 解析 / 二进制帧编解码）
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   处理来自上位机 / 遥控器的消息协议，包括：
  *   - JSON 消息解析与应答
  *   - 二进制帧封包 / 解包
  *   - 参数读写请求的协议层处理
  *   - WiFi 模块（Ai-WB2）的命令交互
  *
  * 优先级 BelowNormal：消息处理不影响飞控实时性。
  * 栈 1024×4：JSON 处理和字符串操作需要较大栈空间。
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
  * @brief  UARTTask —— UART 收发调度
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   统一管理所有 UART 外设的数据收发：
  *   - UART4 (PD.1) → 舵机总线（半双工串行舵机协议）
  *   - UART7 → ELRS 遥控器接收机（Crossfire 协议）
  *   - UART8 → Ai-WB2 WiFi 模块（AT 命令 + 数据透传）
  *
  * 从 uartTxQueue 中取出待发送数据，分别投递到对应的 UART 外设。
  * 各 UART 的接收在中断（IDLE / RXNE）中完成，此任务做分发调度。
  *
  * 优先级 Normal：UART 发送不及时会导致舵机丢帧或 WiFi 阻塞。
  * 栈 1024×4：三个 UART 通道的缓冲区和协议栈需要较大空间。
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
  * @brief  BackgroundTask —— 后台低优先级任务（FLASH 参数管理）
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   执行不应该阻塞控制环（SensorTask / StabilizerTask）的慢操作：
  *   - 参数从 FLASH 自动加载（上电后首次运行）
  *   - 参数异步保存（修改参数后延迟写入 FLASH，减少擦写次数）
  *   - 维护模式 FLASH 块读写（坏块管理、磨损均衡等）
  *
  * 交互方式：请求-响应模式
  *   - 请求者（任意任务）→ backgroundReqQueue → 本任务处理
  *   - 本任务 → backgroundRespQueue → 请求者取走结果
  *
  * 所有 FLASH 操作前必须获取 flashBusMutex（递归互斥锁）。
  *
  * 优先级 BelowNormal：只在所有实时任务空闲时执行。
  * 栈 1024×4：FLASH 页缓冲需要较大栈空间（一页 = 128 字节，需双缓冲）。
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
  * @brief  VOFA_Task —— 遥测流发送任务
  * @param  argument: Not used
  * @retval None
  *
  * 任务体整体位于 App/Src/app_telem_stream.c（策略）与 app_telem_port.c
  * （平台实现）。这里只剩调用：通道装配、掩码帧编码、出口选择、导出互斥
  * 都不再写在 CubeMX 生成的文件里。
  *
  * 优先级 Low：可视化数据允许延迟或丢帧，不影响飞行安全。
  */
/* USER CODE END Header_VOFA_task */
void VOFA_task(void *argument)
{
  /* USER CODE BEGIN VOFA_task */
  for(;;)
  {
    APP_TelemStream_Tick();
  }
  /* USER CODE END VOFA_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


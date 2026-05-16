/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "app.h"
#include "svc_timestamp.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAIN_UART_BOOT_DIAG_ENABLED 0U
#define MAIN_IMU_SPI_WAVEFORM_ONLY  0U
#define MAIN_IMU_I2C_PROBE_ONLY     0U

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Main_StagePulse(uint32_t stage)
{
  if (osKernelGetState() != osKernelInactive)
  {
    return;
  }

  for (uint32_t i = 0; i < stage; ++i)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(80);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(80);
  }

  HAL_Delay(220);
}

static void Main_DebugUartPrint(const char *text)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U) || (MAIN_UART_BOOT_DIAG_ENABLED == 0U)
  (void)text;
  return;
#else
  if (text == NULL)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart1,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          100U);
#endif
}

#if MAIN_IMU_SPI_WAVEFORM_ONLY
static void Main_ImuWaveDelay(void)
{
  for (volatile uint32_t i = 0U; i < 8000U; ++i)
  {
    __NOP();
  }
}

static void Main_ImuWavePinsInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
}

static void Main_ImuWaveByte(uint8_t value)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,
                      ((value & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Main_ImuWaveDelay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    Main_ImuWaveDelay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    Main_ImuWaveDelay();
  }
}

static void Main_ImuWaveLoop(void)
{
  Main_ImuWavePinsInit();

  while (1)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(5U);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    Main_ImuWaveDelay();
    Main_ImuWaveByte(0xF5U);
    Main_ImuWaveByte(0x00U);
    Main_ImuWaveDelay();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  }
}
#endif

#if MAIN_IMU_I2C_PROBE_ONLY
#define MAIN_IMU_I2C_SCL_PORT GPIOA
#define MAIN_IMU_I2C_SCL_PIN  GPIO_PIN_9
#define MAIN_IMU_I2C_SDA_PORT GPIOC
#define MAIN_IMU_I2C_SDA_PIN  GPIO_PIN_1

static void Main_ImuI2cDelay(void)
{
  for (volatile uint32_t i = 0U; i < 1200U; ++i)
  {
    __NOP();
  }
}

static void Main_ImuI2cSetScl(uint8_t high)
{
  HAL_GPIO_WritePin(MAIN_IMU_I2C_SCL_PORT,
                    MAIN_IMU_I2C_SCL_PIN,
                    high != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Main_ImuI2cSetSda(uint8_t high)
{
  HAL_GPIO_WritePin(MAIN_IMU_I2C_SDA_PORT,
                    MAIN_IMU_I2C_SDA_PIN,
                    high != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Main_ImuI2cReadSda(void)
{
  return (HAL_GPIO_ReadPin(MAIN_IMU_I2C_SDA_PORT, MAIN_IMU_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Main_ImuI2cPinsInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAIN_IMU_I2C_SCL_PORT, MAIN_IMU_I2C_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAIN_IMU_I2C_SDA_PORT, MAIN_IMU_I2C_SDA_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MAIN_IMU_I2C_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_IMU_I2C_SCL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MAIN_IMU_I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_IMU_I2C_SDA_PORT, &GPIO_InitStruct);

  Main_ImuI2cSetScl(1U);
  Main_ImuI2cSetSda(1U);
}

static void Main_ImuI2cStart(void)
{
  Main_ImuI2cSetSda(1U);
  Main_ImuI2cSetScl(1U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetSda(0U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetScl(0U);
  Main_ImuI2cDelay();
}

static void Main_ImuI2cStop(void)
{
  Main_ImuI2cSetSda(0U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetScl(1U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetSda(1U);
  Main_ImuI2cDelay();
}

static uint8_t Main_ImuI2cWriteByte(uint8_t value)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    Main_ImuI2cSetSda((value & mask) != 0U ? 1U : 0U);
    Main_ImuI2cDelay();
    Main_ImuI2cSetScl(1U);
    Main_ImuI2cDelay();
    Main_ImuI2cSetScl(0U);
    Main_ImuI2cDelay();
  }

  Main_ImuI2cSetSda(1U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetScl(1U);
  Main_ImuI2cDelay();
  uint8_t ack = (Main_ImuI2cReadSda() == 0U) ? 1U : 0U;
  Main_ImuI2cSetScl(0U);
  Main_ImuI2cDelay();
  return ack;
}

static uint8_t Main_ImuI2cReadByte(uint8_t ack)
{
  uint8_t value = 0U;

  Main_ImuI2cSetSda(1U);
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    value <<= 1U;
    Main_ImuI2cDelay();
    Main_ImuI2cSetScl(1U);
    Main_ImuI2cDelay();
    value |= Main_ImuI2cReadSda();
    Main_ImuI2cSetScl(0U);
    Main_ImuI2cDelay();
  }

  Main_ImuI2cSetSda(ack != 0U ? 0U : 1U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetScl(1U);
  Main_ImuI2cDelay();
  Main_ImuI2cSetScl(0U);
  Main_ImuI2cSetSda(1U);
  Main_ImuI2cDelay();

  return value;
}

static uint8_t Main_ImuI2cReadReg(uint8_t addr7, uint8_t reg, uint8_t *value)
{
  uint8_t ok = 1U;

  Main_ImuI2cStart();
  ok &= Main_ImuI2cWriteByte((uint8_t)(addr7 << 1U));
  ok &= Main_ImuI2cWriteByte(reg);
  Main_ImuI2cStart();
  ok &= Main_ImuI2cWriteByte((uint8_t)((addr7 << 1U) | 0x01U));
  *value = Main_ImuI2cReadByte(0U);
  Main_ImuI2cStop();

  return ok;
}

static void Main_ImuI2cProbeLoop(void)
{
  uint8_t id68 = 0U;
  uint8_t id69 = 0U;
  uint8_t ok68;
  uint8_t ok69;

  Main_ImuI2cPinsInit();
  HAL_Delay(350U);

  while (1)
  {
    ok68 = Main_ImuI2cReadReg(0x68U, 0x75U, &id68);
    HAL_Delay(2U);
    ok69 = Main_ImuI2cReadReg(0x69U, 0x75U, &id69);

    if ((ok68 != 0U && id68 == 0x47U) || (ok69 != 0U && id69 == 0x47U))
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      HAL_Delay(80U);
    }
    else
    {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      HAL_Delay(250U);
    }
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* clock configured */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM8_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  SVC_Timestamp_Init();
  Main_DebugUartPrint("BOOT user2_begin\r\n");
  Main_StagePulse(2U);
  BSP_UART_Release_USART1_ForExternalDebug();
  Main_DebugUartPrint("BOOT bsp_release_done\r\n");
  BSP_Init();
  Main_DebugUartPrint("BOOT bsp_init_done\r\n");
  Main_StagePulse(3U);
  APP_Init();
  Main_DebugUartPrint("BOOT app_init_done\r\n");
  Main_StagePulse(4U);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM17) {
      SVC_Timestamp_Tick();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

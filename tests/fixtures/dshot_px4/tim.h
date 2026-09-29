/* Host register seam: constants are checked against the real CMSIS/HAL headers. */
#ifndef TEST_DSHOT_TIM_H
#define TEST_DSHOT_TIM_H
#include <stdint.h>
#include <stddef.h>
typedef struct { volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR,DCR,DMAR; } TIM_TypeDef;
typedef struct { volatile uint32_t CR, NDTR, PAR, M0AR, FCR; } DMA_Stream_TypeDef;
/* DMA 控制器级寄存器（区别于单条流的 DMA_Stream_TypeDef）：双向档接收相
 * 结束时要清 Stream2 的中断标志位，只有这一个寄存器被用到。 */
typedef struct { volatile uint32_t LIFCR; } DMA_TypeDef;
/* DMAMUX 通道：接收相在发送/接收之间切换 DMA 请求源时改的就是这个。 */
typedef struct { volatile uint32_t CCR; } DMAMUX_Channel_TypeDef;
typedef struct { uint32_t Request,Direction,Mode,MemDataAlignment,PeriphDataAlignment,MemInc,PeriphInc; } DMA_InitTypeDef;
typedef struct DMA_HandleTypeDef DMA_HandleTypeDef;
struct DMA_HandleTypeDef {
    void *Instance; DMA_InitTypeDef Init; uint32_t State, Lock, ErrorCode, StreamIndex;
    uintptr_t StreamBaseAddress;
    void (*XferCpltCallback)(DMA_HandleTypeDef*);
    void (*XferErrorCallback)(DMA_HandleTypeDef*);
    void (*XferHalfCpltCallback)(DMA_HandleTypeDef*);
    void (*XferAbortCallback)(DMA_HandleTypeDef*);
    void (*XferM1CpltCallback)(DMA_HandleTypeDef*);
    void (*XferM1HalfCpltCallback)(DMA_HandleTypeDef*);
};
typedef struct { TIM_TypeDef *Instance; DMA_HandleTypeDef *hdma[7]; } TIM_HandleTypeDef;
typedef struct { volatile uint32_t CFGR; } RCC_TypeDef;
typedef struct { uint32_t APB2CLKDivider; } RCC_ClkInitTypeDef;
extern TIM_TypeDef test_tim1, test_tim4;
extern DMA_Stream_TypeDef test_stream2;
extern RCC_TypeDef test_rcc;
extern TIM_HandleTypeDef htim1, htim4;
extern DMA_TypeDef test_dma1;
extern DMAMUX_Channel_TypeDef test_dmamux1_channel2;
#define TIM1 (&test_tim1)
#define DMA1_Stream2 (&test_stream2)
#define RCC (&test_rcc)
#define DMA1 (&test_dma1)
#define DMAMUX1_Channel2 (&test_dmamux1_channel2)
#define TIM_DMA_ID_UPDATE 0
#define TIM_CCMR1_OC1M 0x10070U
#define TIM_CCMR1_OC2M 0x1007000U
#define TIM_CCMR1_OC1PE 0x8U
#define TIM_CCMR1_OC2PE 0x800U
/* 双向档接收相：CCMR1 同一批位在输出比较/输入捕获之间复用，见 stm32h743xx.h。 */
#define TIM_CCMR1_OC1M_2 0x40U
#define TIM_CCMR1_OC2M_2 0x4000U
#define TIM_CCMR1_CC1S 0x3U
#define TIM_CCMR1_CC1S_0 0x1U
#define TIM_CCMR1_CC2S 0x300U
#define TIM_CCMR1_CC2S_0 0x100U
#define TIM_CCMR1_IC1F 0xF0U
#define TIM_CCMR1_IC1F_Pos 4U
#define TIM_CCMR1_IC1PSC 0xCU
#define TIM_CCMR1_IC2F 0xF000U
#define TIM_CCMR1_IC2F_Pos 12U
#define TIM_CCMR1_IC2PSC 0xC00U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCMODE_FORCED_INACTIVE 0x40U
#define TIM_BDTR_MOE 0x8000U
#define TIM_BDTR_OSSI 0x400U
#define TIM_BDTR_OSSR 0x800U
#define TIM_DIER_UDE 0x100U
#define TIM_DIER_CC1DE 0x200U
#define TIM_DIER_CC2DE 0x400U
#define TIM_CR1_CEN 1U
#define TIM_CR1_ARPE 0x80U
#define TIM_CR1_URS 0x4U
#define TIM_CR2_OIS1 0x100U
#define TIM_CR2_OIS2 0x400U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 0x10U
#define TIM_CCER_CC1P 0x2U
#define TIM_CCER_CC1NP 0x8U
#define TIM_CCER_CC2P 0x20U
#define TIM_CCER_CC2NP 0x80U
#define TIM_EGR_UG 1U
#define TIM_DMABASE_CCR1 13U
#define TIM_DMABURSTLENGTH_2TRANSFERS 0x100U
#define DMA_SxCR_EN 1U
#define DMA_SxCR_TCIE 0x10U
#define DMA_SxCR_HTIE 0x8U
#define DMA_SxCR_TEIE 0x4U
#define DMA_SxCR_DMEIE 0x2U
#define DMA_SxCR_PL_1 0x20000U
#define DMA_SxCR_MSIZE_1 0x4000U
#define DMA_SxCR_PSIZE_1 0x1000U
#define DMA_SxCR_MINC 0x400U
#define DMA_SxFCR_DMDIS 0x4U
#define DMA_REQUEST_TIM1_UP 15U
#define DMA_REQUEST_TIM1_CH1 11U
#define DMA_REQUEST_TIM1_CH2 12U
#define DMA_MEMORY_TO_PERIPH 0x40U
#define DMA_NORMAL 0U
#define DMA_MDATAALIGN_WORD 0x4000U
#define DMA_PDATAALIGN_WORD 0x1000U
#define DMA_MINC_ENABLE 0x400U
#define DMA_PINC_DISABLE 0U
#define RCC_CFGR_TIMPRE 0x8000U
#define RCC_APB2_DIV1 0U
#define RCC_APB2_DIV2 0x400U
#define RCC_APB2_DIV4 0x500U
#define HAL_DMA_STATE_READY 1U
/* bitbang 后端手写寄存器启流，所以要自己把 HAL 的状态摆成 BUSY，
 * 否则 HAL_DMA_IRQHandler 不会派发完成回调。值见 stm32h7xx_hal_dma.h。 */
#define HAL_DMA_STATE_BUSY 2U
#define HAL_DMA_STATE_ERROR 3U
#define HAL_DMA_STATE_ABORT 4U
#define HAL_DMA_ERROR_NONE 0U
#define HAL_DMA_ERROR_TE 1U
#define HAL_DMA_ERROR_FE 2U
#define HAL_DMA_ERROR_DME 4U
#define DMA_SxCR_DBM 0x40000U
#define DMA_SxCR_CT 0x80000U
#define DMA_SxCR_CIRC 0x100U
#define DMA_IT_TC DMA_SxCR_TCIE
#define DMA_IT_TE DMA_SxCR_TEIE
#define DMA_IT_HT DMA_SxCR_HTIE
#define DMA_IT_DME DMA_SxCR_DMEIE
#define DMA_IT_FE 0x80U
#define DMA_FLAG_FEIF0_4 1U
#define DMA_FLAG_DMEIF0_4 4U
#define DMA_FLAG_TEIF0_4 8U
#define DMA_FLAG_HTIF0_4 16U
#define DMA_FLAG_TCIF0_4 32U
/* Stream2 专属的 LIFCR 清标志位；接收相收尾（rx_stop_stream）用来清 DMA1 侧的挂起标志。 */
#define DMA_LIFCR_CTCIF2 0x200000U
#define DMA_LIFCR_CHTIF2 0x100000U
#define DMA_LIFCR_CTEIF2 0x80000U
#define DMA_LIFCR_CDMEIF2 0x40000U
#define DMA_LIFCR_CFEIF2 0x10000U
#define DMAMUX_CxCR_DMAREQ_ID 0xFFU
#define __IO volatile
#define __HAL_UNLOCK(d) ((d)->Lock=0U)
#define __HAL_DMA_DISABLE(d) (((DMA_Stream_TypeDef*)(d)->Instance)->CR &= ~DMA_SxCR_EN)
#define __HAL_DMA_GET_IT_SOURCE(d,it) (((it)==DMA_IT_FE ? ((DMA_Stream_TypeDef*)(d)->Instance)->FCR : ((DMA_Stream_TypeDef*)(d)->Instance)->CR) & (it))
typedef struct { volatile uint32_t ISR, IFCR; } DMA_Base_Registers;
#define HAL_UNLOCKED 0U
#define HAL_OK 0
#define HAL_ERROR 1
#define DMA1_Stream2_IRQn 13
#define MODIFY_REG(reg,clear,set) ((reg)=((reg)&~(clear))|(set))
#define __HAL_DMA_GET_TC_FLAG_INDEX(d) 1U
#define __HAL_DMA_GET_HT_FLAG_INDEX(d) 2U
#define __HAL_DMA_GET_TE_FLAG_INDEX(d) 4U
#define __HAL_DMA_GET_DME_FLAG_INDEX(d) 8U
#define __HAL_DMA_GET_FE_FLAG_INDEX(d) 16U
#define __HAL_DMA_CLEAR_FLAG(d,f) test_clear_flags(f)
void test_clear_flags(uint32_t);
void HAL_NVIC_ClearPendingIRQ(int);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef*,uint32_t*);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
int HAL_DMA_Start_IT(DMA_HandleTypeDef*,uint32_t,uint32_t,uint32_t);
/* 双向档接收相记录回包时间戳用；host 侧没有真实 SysTick，见 bsp_harness.c 里的桩。 */
uint32_t HAL_GetTick(void);
typedef int HAL_StatusTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define __HAL_TIM_SET_AUTORELOAD(t,v) ((t)->Instance->ARR=(v))
#define __HAL_TIM_SET_COMPARE(t,c,v) (*((volatile uint32_t*)&(t)->Instance->CCR1 + (c)/4U)=(v))
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel) {
    (void)channel; t->Instance->CR1|=TIM_CR1_CEN; return HAL_OK;
}
/*
 * 电调信号脚所在的 GPIO 端口影子。`BSP_DShot_ReadEscPinLevels()` 读 IDR 来量
 * 引脚**实际电平**（PE9=通道1、PE11=通道2）——那是把"我们以为寄存器设对了"和
 * "线上真的是这样"分开的唯一手段，所以它的位映射必须能在宿主上判对错。
 * 只有 IDR：这个装置不需要也不该模拟整个 GPIO 端口。
 */
typedef struct { uint32_t IDR; uint32_t BSRR; } GPIO_TypeDef;
extern GPIO_TypeDef test_gpioe;
#define GPIOE (&test_gpioe)

/*
 * bitbang 后端用的 GPIO 一组。值按 stm32h7xx_hal_gpio.h / stm32h743xx.h：
 * PIN_9=0x0200、PIN_11=0x0800；MODE_INPUT=0、MODE_OUTPUT_PP=1；
 * NOPULL=0、PULLUP=1；SPEED_FREQ_VERY_HIGH=3。
 * 注意 `test_register_constants_match_actual_st_headers` 的正则只认
 * TIM/DMA/RCC/HAL_DMA 前缀，所以 GPIO_* 这批**不会**被自动核对——它们的正确性
 * 靠"写错了这个装置里的引脚断言就会红"来兜（harness 的 HAL_GPIO_Init 桩会
 * 对 Pin 掩码 abort），而 DMA_SxCR_DIR* 在前缀内，会被自动核对。
 */
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_11 0x0800U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
#define DMA_SxCR_DIR 0xC0U
#define DMA_SxCR_DIR_0 0x40U
#endif

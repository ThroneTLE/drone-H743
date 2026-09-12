/* Host register seam: constants are checked against the real CMSIS/HAL headers. */
#ifndef TEST_DSHOT_TIM_H
#define TEST_DSHOT_TIM_H
#include <stdint.h>
#include <stddef.h>
typedef struct { volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR,DCR,DMAR; } TIM_TypeDef;
typedef struct { volatile uint32_t CR, FCR; } DMA_Stream_TypeDef;
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
#define TIM1 (&test_tim1)
#define DMA1_Stream2 (&test_stream2)
#define RCC (&test_rcc)
#define TIM_DMA_ID_UPDATE 0
#define TIM_CCMR1_OC1M 0x10070U
#define TIM_CCMR1_OC2M 0x1007000U
#define TIM_CCMR1_OC1PE 0x8U
#define TIM_CCMR1_OC2PE 0x800U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCMODE_FORCED_INACTIVE 0x40U
#define TIM_BDTR_MOE 0x8000U
#define TIM_BDTR_OSSI 0x400U
#define TIM_BDTR_OSSR 0x800U
#define TIM_DIER_UDE 0x100U
#define TIM_CR1_CEN 1U
#define TIM_CR1_ARPE 0x80U
#define TIM_CR1_URS 0x4U
#define TIM_CR2_OIS1 0x100U
#define TIM_CR2_OIS2 0x400U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 0x10U
#define TIM_EGR_UG 1U
#define TIM_DMABASE_CCR1 13U
#define TIM_DMABURSTLENGTH_2TRANSFERS 0x100U
#define DMA_SxCR_EN 1U
#define DMA_SxCR_TCIE 0x10U
#define DMA_SxCR_HTIE 0x8U
#define DMA_SxCR_TEIE 0x4U
#define DMA_SxCR_DMEIE 0x2U
#define DMA_REQUEST_TIM1_UP 15U
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
#endif

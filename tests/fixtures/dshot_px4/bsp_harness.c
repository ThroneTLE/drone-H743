#include "tim.h"
#include "bsp_dshot.h"
#include "drv_dshot.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
TIM_TypeDef test_tim1, test_tim4;
DMA_Stream_TypeDef test_stream2;
RCC_TypeDef test_rcc;
static DMA_HandleTypeDef dma;
TIM_HandleTypeDef htim1 = {&test_tim1,{&dma}}, htim4 = {&test_tim4,{0}};
static uint32_t primask, starts, cleaned, irq_cleared, flags_cleared;
static int start_fail, interrupt_during_clean;
uint32_t BSP_Critical_Enter(void) { uint32_t old=primask; primask=1; return old; }
void BSP_Critical_Exit(uint32_t old) { primask=old; }
void BSP_Critical_MemoryBarrier(void) {}
void test_clear_flags(uint32_t flags) { flags_cleared=flags; }
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; irq_cleared++; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c,uint32_t *l) { c->APB2CLKDivider=RCC_APB2_DIV2; *l=0; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 60000000; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 240000000; }
void BSP_Cache_CleanDCache(const void *p,uint32_t len) {
    if (((uintptr_t)p&31)!=0 || len!=160) { abort(); } cleaned++;
    if(interrupt_during_clean) { interrupt_during_clean=0; BSP_DShot_Disable(3); }
}
int HAL_DMA_Start_IT(DMA_HandleTypeDef *d,uint32_t src,uint32_t dst,uint32_t len) {
    (void)src; (void)dst;
    if(len!=36 || !cleaned || !flags_cleared || !irq_cleared) { abort(); }
    if(start_fail) { return HAL_ERROR; }
    d->State=2; d->ErrorCode=HAL_DMA_ERROR_NONE;
    ((DMA_Stream_TypeDef*)d->Instance)->CR|=DMA_SxCR_EN|DMA_IT_TC|DMA_IT_TE|DMA_IT_DME;
    starts++; return HAL_OK;
}
/* Compile the real BSP. File-local frame buffer is observed only by this test. */
#include "bsp_dshot.c"
#include "bsp_pwm.c"
#include "arbitration_seam.h"
#include "app_esc_diag.c"
#include "hal_irq_seam.h"
static char report[1024];
void APP_Control_QueueText(const char *fmt, ...) {
    va_list ap; va_start(ap,fmt); vsnprintf(report,sizeof(report),fmt,ap); va_end(ap);
}

static int setup(void) {
    memset(&test_tim1,0,sizeof(test_tim1)); memset(&test_stream2,0,sizeof(test_stream2));
    memset(&dma,0,sizeof(dma)); memset(&state,0,sizeof(state)); initialized=0;
    dma.Instance=DMA1_Stream2; dma.Init.Request=DMA_REQUEST_TIM1_UP;
    dma.Init.Direction=DMA_MEMORY_TO_PERIPH; dma.Init.Mode=DMA_NORMAL;
    dma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD; dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    dma.Init.MemInc=DMA_MINC_ENABLE; dma.Init.PeriphInc=DMA_PINC_DISABLE;
    primask=starts=cleaned=irq_cleared=flags_cleared=0; start_fail=interrupt_during_clean=0;
    test_tim4.ARR=19999; test_tim4.PSC=119;
    CHECK(BSP_DShot_Init()==BSP_DSHOT_OK);
    CHECK(test_tim1.ARR==399 && test_tim1.PSC==0);
    CHECK(test_tim1.DCR==(TIM_DMABASE_CCR1|TIM_DMABURSTLENGTH_2TRANSFERS));
    CHECK(!(test_tim1.BDTR&TIM_BDTR_MOE));
    return 0;
}
static void complete(void) { test_stream2.CR&=~DMA_SxCR_EN; dma.State=HAL_DMA_STATE_READY; dma.Lock=0; dma.XferCpltCallback(&dma); }

static int waveform(void) {
    CHECK(setup()==0); uint16_t code[2]={49,2047};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    uint16_t expected[2]; DRV_DShot_Encode(code[0],&expected[0]); DRV_DShot_Encode(code[1],&expected[1]);
    /* Model update: active <- preload; DMA then writes preload. Both CCRs began
     * at 0 and were latched by UG before CEN. Two zero slots drain the pipeline. */
    uint32_t preload[2]={0,0}, active[2]={0,0}; uint16_t decoded[2]={0,0};
    for(unsigned slot=0;slot<18;slot++) {
        for(unsigned ch=0;ch<2;ch++) {
            active[ch]=preload[ch]; preload[ch]=dma_words[slot*2+ch];
            if(slot>=1 && slot<=16) {
                CHECK(active[ch]==150 || active[ch]==300);
                decoded[ch]=(uint16_t)((decoded[ch]<<1)|(active[ch]==300));
            }
        }
    }
    CHECK(decoded[0]==expected[0] && decoded[1]==expected[1]);
    CHECK(active[0]==0 && active[1]==0 && preload[0]==0 && preload[1]==0);
    complete(); CHECK(!state.busy && state.completed==1);
    CHECK(!(test_tim1.CR1&TIM_CR1_CEN)); CHECK(!(test_tim1.DIER&TIM_DIER_UDE));
    CHECK(test_tim4.ARR==19999 && test_tim4.PSC==119);
    return 0;
}
static int races(void) {
    CHECK(setup()==0); uint16_t code[2]={48,2047};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    uint32_t old[40]; memcpy(old,dma_words,sizeof(old));
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_BUSY); CHECK(memcmp(old,dma_words,sizeof(old))==0);
    CHECK(BSP_DShot_Init()==BSP_DSHOT_BUSY); CHECK(state.enabled_mask==3);
    CHECK(BSP_DShot_Disable(1)==BSP_DSHOT_OK);
    CHECK((test_tim1.CCMR1&TIM_CCMR1_OC1M)==TIM_OCMODE_FORCED_INACTIVE);
    fprintf(stderr,"CCMR1=%08x mask=%08x expected=%08x\n",(unsigned)test_tim1.CCMR1,(unsigned)TIM_CCMR1_OC2M,(unsigned)(TIM_OCMODE_PWM1<<8)); CHECK((test_tim1.CCMR1&TIM_CCMR1_OC2M)==(TIM_OCMODE_PWM1<<8));
    complete(); CHECK(state.enabled_mask==2); CHECK((test_tim1.CCMR1&TIM_CCMR1_OC1M)==TIM_OCMODE_FORCED_INACTIVE);
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    primask=1; CHECK(BSP_DShot_Disable(3)==BSP_DSHOT_OK); CHECK(primask==1); primask=0;
    CHECK(!state.enabled_mask && !(test_tim1.BDTR&TIM_BDTR_MOE));
    uint32_t done=state.completed; complete(); CHECK(state.completed==done); CHECK(!state.enabled_mask);
    interrupt_during_clean=1; uint32_t count=starts;
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_BUSY); CHECK(starts==count); CHECK(!state.busy);
    CHECK(!(test_tim1.BDTR&TIM_BDTR_MOE));
    return 0;
}
static int faults(void) {
    CHECK(setup()==0); uint16_t code[2]={48,2047};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK); dma.XferErrorCallback(&dma);
    CHECK(state.fault && !state.enabled_mask && !state.busy);
    CHECK(!(test_tim1.BDTR&TIM_BDTR_MOE));
    uint32_t count=starts; CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_ERROR); CHECK(starts==count);
    complete(); CHECK(state.fault && state.completed==0);
    CHECK(BSP_DShot_Init()==BSP_DSHOT_OK); start_fail=1;
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_ERROR); CHECK(state.fault);
    CHECK(BSP_DShot_Disable(3)==BSP_DSHOT_OK); CHECK(state.fault);
    CHECK(setup()==0); dma.Init.Request=0; CHECK(BSP_DShot_Init()==BSP_DSHOT_ERROR);
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_ERROR);
    CHECK(setup()==0); code[1]=47; CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_INVALID); CHECK(!starts);
    CHECK(BSP_DShot_Disable(4)==BSP_DSHOT_INVALID);
    return 0;
}
static int arbitration(void) {
    CHECK(setup()==0); pwm_started=0; CHECK(BSP_PWM_Init()==BSP_PWM_OK);
    ArbFrame f={0}; f.rc_link_ok=1; f.rc_link_seen=1;
    run_motor_arbitration(&f);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    CHECK(state.code[0]==0 && state.code[1]==0 && state.enabled_mask==3 && starts==1);
    complete();
#else
    CHECK(test_tim1.PSC==119 && test_tim1.ARR==2499);
    CHECK(test_tim1.CCR1==1100 && test_tim1.CCR2==1100);
#endif
    f.rc_armed=1; f.rc_control_motor_mix_allowed=1; f.imu_control_valid=1;
    f.ctrl_out.motor_upper_us=1500; f.ctrl_out.motor_lower_us=1600;
    run_motor_arbitration(&f);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    uint16_t first,second; DRV_DShot_FromPulseUs(1500,&first); DRV_DShot_FromPulseUs(1600,&second);
    CHECK(state.code[0]==first && state.code[1]==second && starts==2);
    complete();
#else
    CHECK(test_tim1.CCR1==1500 && test_tim1.CCR2==1600);
#endif
    f.rc_link_ok=0; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    CHECK(!state.enabled_mask && !(test_tim1.BDTR&TIM_BDTR_MOE));
#else
    CHECK(test_tim1.CCR1==0 && test_tim1.CCR2==0);
#endif
    f.rc_link_ok=1; f.rc_armed=0; f.servo_cal_active=1; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==1100 && BSP_PWM_GetEscPulse(2)==1100);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    CHECK(state.code[0]==0 && state.code[1]==0); complete();
#endif
    acceptance_active=1; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
    CHECK(test_tim4.ARR==19999 && test_tim4.PSC==119);
    APP_EscDiag_Report();
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    CHECK(strstr(report,"protocol=DSHOT300 command_unit=pwm_equivalent_us ack=unavailable"));
    CHECK(strstr(report,"enabled=0"));
#else
    CHECK(strstr(report,"protocol=PWM command_unit=us ack=unavailable"));
#endif
    return 0;
}
static int vendor_errors(void) {
    const uint32_t flags[]={DMA_FLAG_TEIF0_4,DMA_FLAG_DMEIF0_4,DMA_FLAG_FEIF0_4};
    uint16_t code[2]={48,2047};
    for(unsigned i=0;i<3;i++) {
        for(unsigned with_tc=0;with_tc<2;with_tc++) {
            CHECK(setup()==0);
            DMA_Base_Registers regs={0};
            dma.StreamBaseAddress=(uintptr_t)&regs;
            dma.StreamIndex=16; /* Stream2 */
            CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
            test_stream2.FCR |= DMA_IT_FE;
            regs.ISR=(flags[i] | (with_tc ? DMA_FLAG_TCIF0_4 : 0U)) << 16;
            vendor_stream_irq(&dma);
            CHECK(state.fault && state.errors==1 && state.completed==0);
            CHECK(!state.enabled_mask && !state.busy);
            CHECK(!(test_tim1.BDTR&TIM_BDTR_MOE));
            CHECK(!(test_tim1.DIER&TIM_DIER_UDE));
            CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_ERROR);
            CHECK(BSP_DShot_Init()==BSP_DSHOT_OK);
            CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
            regs.ISR=DMA_FLAG_TCIF0_4 << 16;
            vendor_stream_irq(&dma);
            CHECK(!state.fault && state.completed==1);
        }
    }
    return 0;
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    int n=atoi(argv[1]); int rc=n==0?waveform():(n==1?races():(n==2?faults():(n==3?arbitration():vendor_errors())));
    if(!rc) puts("real BSP host state/register seam passed (not hardware validation)");
    return rc;
}

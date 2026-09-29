#include "tim.h"
#include "bsp_dshot.h"
#include "drv_dshot.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
TIM_TypeDef test_tim1, test_tim4;
GPIO_TypeDef test_gpioe;
DMA_Stream_TypeDef test_stream2;
RCC_TypeDef test_rcc;
DMA_TypeDef test_dma1;
DMAMUX_Channel_TypeDef test_dmamux1_channel2;
static DMA_HandleTypeDef dma;
TIM_HandleTypeDef htim1 = {&test_tim1,{&dma}}, htim4 = {&test_tim4,{0}};
static uint32_t primask, starts, cleaned, irq_cleared, flags_cleared;
static int start_fail, interrupt_during_clean;
/* 双向档接收相用的桩：host 上没有真实 SysTick，给一个单调递增的假计数就够——
 * 当前 5 个 case 都没有断言具体的 ms 值，只要求它能被调用、不崩。 */
uint32_t HAL_GetTick(void) { static uint32_t ticks; return ++ticks; }
/* 与 BSP_Cache_CleanDCache 同一个"老实"标准：真实约束只有 cache 行对齐，
 * 接收缓冲区大小不像发送缓冲区那样是这个装置里唯一的一次用法，不额外锁长度。 */
static uint32_t rx_invalidated;
void BSP_Cache_InvalidateDCache(const void *p, uint32_t len) {
    if (((uintptr_t)p & 31) != 0) { abort(); }
    (void)len; rx_invalidated++;
}
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
/* 双向档才有实体；非双向档整份文件只剩 BSP_DShotRx_GetSnapshot 一个空快照函数
 * （bsp_dshot_rx.h 的 #else 分支已经把其余四个调用点做成内联空实现）。 */
#include "bsp_dshot_rx.c"
#include "bsp_pwm.c"
#include "arbitration_seam.h"
#include "app_esc_diag.c"
#include "hal_irq_seam.h"
static char report[1024];
void APP_Control_QueueText(const char *fmt, ...) {
    va_list ap; va_start(ap,fmt); vsnprintf(report,sizeof(report),fmt,ap); va_end(ap);
}

static void complete(void);

static int setup(void) {
    memset(&test_tim1,0,sizeof(test_tim1)); memset(&test_stream2,0,sizeof(test_stream2));
    memset(&test_dma1,0,sizeof(test_dma1)); memset(&test_dmamux1_channel2,0,sizeof(test_dmamux1_channel2));
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
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    /*
     * 跑满开机自检宽限期。期内 `BSP_DShotRx_Start()` 故意不翻面（线驻在空闲高
     * 让电调自检出双向模式，见 bsp_dshot_rx.c 文件头），所以凡是要验接收相行为
     * 的 case 都必须先把它耗掉——否则验的是"还没开始收"，不是"收不到"。
     * 宽限期本身由 `grace_period()` 单独验，不在这里顺带断言。
     */
    {
        uint16_t warm[2]={48,48};
        for (unsigned i=0;i<BSP_DSHOT_RX_DETECT_GRACE_FRAMES;i++) {
            CHECK(BSP_DShot_Submit(warm,3)==BSP_DSHOT_OK);
            complete();
        }
        /* 计数归零：预热帧不属于任何一条 case 的被测行为，留着会让
         * `state.completed==1` 这类断言变成在数预热次数。 */
        state.submitted=0; state.completed=0; starts=0; cleaned=0;
    }
#endif
    return 0;
}
static void complete(void) { test_stream2.CR&=~DMA_SxCR_EN; dma.State=HAL_DMA_STATE_READY; dma.Lock=0; dma.XferCpltCallback(&dma); }

static int waveform(void) {
    CHECK(setup()==0); uint16_t code[2]={49,2047};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    /*
     * 参考帧必须按**本档协议**算。两档 DShot 的 12 bit 载荷相同、4 bit 校验互为反码
     * （drv_dshot_telemetry.h 开头那三点差别的第二点），所以同一个油门码在两档下
     * 是两串不同的位：throttle=49 单向 0x0624、双向 0x062b。
     * 拿单向编码器当双向的参考，测的就不再是"DMA 流水线把帧按位送出去了吗"，
     * 而是"两个协议碰巧一样吗"——后者恒为假，前者才是这条 case 的本意。
     * 这里的分流与 bsp_dshot.c::encode_packets() 内部那一处严格对应。
     */
    uint16_t expected[2];
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    DRV_DShotTelem_EncodeRequest(code[0],0U,&expected[0]);
    DRV_DShotTelem_EncodeRequest(code[1],0U,&expected[1]);
#else
    DRV_DShot_Encode(code[0],&expected[0]); DRV_DShot_Encode(code[1],&expected[1]);
#endif
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
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    /*
     * 双向档发完立刻进接收相，所以 CR1.CEN **仍然开着**——那是捕获在跑，不是
     * 还在发。"发送已经结束"这件事的判据在 UDE 上（下一行），不在 CEN 上；
     * 顺带钉一下捕获 DMA 请求确实开了，否则"CEN 开着"也可能只是没停干净。
     */
    CHECK(test_tim1.CR1&TIM_CR1_CEN);
    CHECK(test_tim1.DIER&(TIM_DIER_CC1DE|TIM_DIER_CC2DE));
#else
    CHECK(!(test_tim1.CR1&TIM_CR1_CEN));
#endif
    CHECK(!(test_tim1.DIER&TIM_DIER_UDE));
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
    complete(); CHECK(state.enabled_mask==2);
    fprintf(stderr,"post-complete CCMR1=%08x CCER=%08x\n",(unsigned)test_tim1.CCMR1,(unsigned)test_tim1.CCER);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    /*
     * 双向档"发完就听"：dma_complete() 里会调 BSP_DShotRx_Start()，把本轮的
     * rx_channel（首轮是通道 1）从输出比较翻成输入捕获。**它不看 enabled_mask**，
     * 所以刚被安全禁用的那一路同样会被翻过去。
     *
     * 这不违反"禁用的通道不许出油门"：输入捕获是高阻，根本不驱动任何电平，
     * 比强制无效更彻底。所以这里钉的是同一条安全属性在本档下的形态。
     */
    CHECK((test_tim1.CCMR1&TIM_CCMR1_CC1S)==TIM_CCMR1_CC1S_0);
#else
    CHECK((test_tim1.CCMR1&TIM_CCMR1_OC1M)==TIM_OCMODE_FORCED_INACTIVE);
#endif
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    /*
     * 而且翻过去必须是**有界**的：下一次提交里的 Harvest 要把这一路交回输出模式
     * （CC1S=0），否则被禁用过一次的通道就永远发不出帧了——那是一条"禁用之后
     * 再也恢复不了"的静默故障，比不禁用更糟。
     */
    CHECK((test_tim1.CCMR1&TIM_CCMR1_CC1S)==0U);
#endif
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
/* 接线标定：通道 1 = 上桨、通道 2 = 下桨，与 2026-09-13 改造前写死的顺序一致，
 * 所以下面每一条既有断言的期望值一个都不用改。 */
static void calibrate_props(uint8_t upper_channel) {
    DRV_PropMap m; DRV_PropMap_Defaults(&m);
    m.channel[upper_channel-1U].role=(uint8_t)DRV_PROP_ROLE_UPPER;
    m.channel[upper_channel-1U].spin_sense=DRV_PROP_SPIN_CCW;
    m.channel[2U-upper_channel].role=(uint8_t)DRV_PROP_ROLE_LOWER;
    m.channel[2U-upper_channel].spin_sense=DRV_PROP_SPIN_CW;
    m.calibrated=1U;
    if(!DRV_PropMap_PublishActive(&m)) { abort(); }
}
static int arbitration(void) {
    CHECK(setup()==0); pwm_started=0; CHECK(BSP_PWM_Init()==BSP_PWM_OK);
    APP_PropSpin_Reset(); calibrate_props(1);
    ArbFrame f={0}; f.rc_link_ok=1; f.rc_link_seen=1;
    run_motor_arbitration(&f);
    /*
     * 两档 DShot 共用同一条 BSP_DShot_Submit/state 快照路径（发送侧只有极性/校验
     * 不同，见 bsp_esc_protocol.h 的 BSP_ESC_PROTOCOL_IS_DSHOT），所以这里跟
     * app_esc_diag.c、bsp_pwm.c 的 Init/SetEscPulse 分支一样按"是不是 DShot"分组，
     * 不是按具体哪一档 DShot 分组。PWM 才走 #else 的裸 CCR/PSC/ARR 断言。
     */
#if BSP_ESC_PROTOCOL_IS_DSHOT
    CHECK(state.code[0]==0 && state.code[1]==0 && state.enabled_mask==3 && starts==1);
    complete();
#else
    CHECK(test_tim1.PSC==119 && test_tim1.ARR==2499);
    CHECK(test_tim1.CCR1==1100 && test_tim1.CCR2==1100);
#endif
    f.rc_armed=1; f.rc_control_motor_mix_allowed=1; f.imu_control_valid=1;
    f.ctrl_out.motor_upper_us=1500; f.ctrl_out.motor_lower_us=1600;
    run_motor_arbitration(&f);
#if BSP_ESC_PROTOCOL_IS_DSHOT
    uint16_t first,second; DRV_DShot_FromPulseUs(1500,&first); DRV_DShot_FromPulseUs(1600,&second);
    CHECK(state.code[0]==first && state.code[1]==second && starts==2);
    complete();
#else
    CHECK(test_tim1.CCR1==1500 && test_tim1.CCR2==1600);
#endif
    f.rc_link_ok=0; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
#if BSP_ESC_PROTOCOL_IS_DSHOT
    CHECK(!state.enabled_mask && !(test_tim1.BDTR&TIM_BDTR_MOE));
#else
    CHECK(test_tim1.CCR1==0 && test_tim1.CCR2==0);
#endif
    f.rc_link_ok=1; f.rc_armed=0; f.servo_cal_active=1; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==1100 && BSP_PWM_GetEscPulse(2)==1100);
#if BSP_ESC_PROTOCOL_IS_DSHOT
    CHECK(state.code[0]==0 && state.code[1]==0); complete();
#endif
    acceptance_active=1; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
    /* 舵机 TIM4 仍是 BSP_PWM_Init 写下的上电帧（2026-09-28 起 333 Hz），电调仲裁一下都没碰。 */
    CHECK(test_tim4.ARR==BSP_PWM_SERVO_FRAME_US-1U && test_tim4.PSC==119);

    /*
     * 接线反过来标定时，控制器的"上桨推力"必须真的落到通道 2 上。
     * 这条走的是真 BSP，所以它验的是整条 上/下桨 -> 通道 -> 寄存器/DShot 码 的路，
     * 不是一个只比较变量的替身。
     */
    acceptance_active=0; f.servo_cal_active=0;
    calibrate_props(2);
    f.rc_link_ok=1; f.rc_armed=1; f.rc_control_motor_mix_allowed=1; f.imu_control_valid=1;
    f.ctrl_out.motor_upper_us=1500; f.ctrl_out.motor_lower_us=1600;
    run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==1600 && BSP_PWM_GetEscPulse(2)==1500);
    calibrate_props(1);

    /*
     * 点电机窗口：一次只转一路，而且**停不下来必须不可能**。心跳超时的判定
     * 就在这条仲裁链里，所以这里能验到真实行为——上位机不再发命令、时间往前
     * 走过 APP_PROP_SPIN_TIMEOUT_MS，下一拍两路都回到零油门。
     */
    f.rc_armed=0; f.rc_control_motor_mix_allowed=0; f.imu_control_valid=0;
    f.now_ms=10000;
    CHECK(APP_PropSpin_Open(f.now_ms, APP_PROP_SPIN_DEFAULT_MAX_PERCENT)==1);
    CHECK(APP_PropSpin_Command(f.now_ms,2,APP_PROP_SPIN_DEFAULT_MAX_PERCENT)==1);
    run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(1)==BSP_PWM_ESC_MIN_US);
    CHECK(BSP_PWM_GetEscPulse(2)==BSP_PWM_PercentToPulse(APP_PROP_SPIN_DEFAULT_MAX_PERCENT));
    CHECK(f.motor_output_reason==APP_FLIGHT_LOG_MOTOR_REASON_PROP_SPIN_TEST);
    /* 还没到超时：照转。 */
    f.now_ms+=APP_PROP_SPIN_TIMEOUT_MS-1U; run_motor_arbitration(&f);
    CHECK(BSP_PWM_GetEscPulse(2)==BSP_PWM_PercentToPulse(APP_PROP_SPIN_DEFAULT_MAX_PERCENT));
    /* 越过超时：停，而且窗口关掉，不是"油门归零但还开着"。 */
    f.now_ms+=1U; run_motor_arbitration(&f);
    CHECK(!APP_PropSpin_IsActive());
    CHECK(APP_PropSpin_LastStopReason()==APP_PROP_SPIN_STOP_HEARTBEAT);
    CHECK(BSP_PWM_GetEscPulse(1)==BSP_PWM_ESC_MIN_US);
    CHECK(BSP_PWM_GetEscPulse(2)==BSP_PWM_ESC_MIN_US);
    /* 解锁抢占：窗口当场关闭，执行器交回给飞行链路。 */
    CHECK(APP_PropSpin_Open(f.now_ms, APP_PROP_SPIN_DEFAULT_MAX_PERCENT)==1);
    CHECK(APP_PropSpin_Command(f.now_ms,1,5)==1);
    f.rc_armed=1; run_motor_arbitration(&f);
    CHECK(!APP_PropSpin_IsActive());
    CHECK(APP_PropSpin_LastStopReason()==APP_PROP_SPIN_STOP_INHIBIT);
    f.rc_armed=0;

    /* TBENCH stage failure: feed an invalid pulse through the real BSP setter.
     * The same arbitration pass must close the window and physically disable
     * both staged channels rather than carrying one successful half-update. */
    thrust_bench_active=1U; thrust_bench_stop_reason=0U;
    memset(&thrust_bench_output,0,sizeof(thrust_bench_output));
    thrust_bench_output.active=1U;
    thrust_bench_output.pulse_us[0]=BSP_PWM_ESC_MAX_US+1U;
    thrust_bench_output.pulse_us[1]=BSP_PWM_ESC_MIN_US;
    run_motor_arbitration(&f);
    CHECK(!thrust_bench_active);
    CHECK(thrust_bench_stop_reason==APP_THRUST_BENCH_STOP_OUTPUT_ERROR);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
#if BSP_ESC_PROTOCOL_IS_DSHOT
    CHECK(!state.enabled_mask);
    CHECK(setup()==0); pwm_started=0; CHECK(BSP_PWM_Init()==BSP_PWM_OK);
    thrust_bench_active=1U; thrust_bench_stop_reason=0U;
    memset(&thrust_bench_output,0,sizeof(thrust_bench_output));
    thrust_bench_output.active=1U;
    thrust_bench_output.pulse_us[0]=1200U;
    thrust_bench_output.pulse_us[1]=1300U;
    start_fail=1;
    run_motor_arbitration(&f);
    CHECK(!thrust_bench_active);
    CHECK(thrust_bench_stop_reason==APP_THRUST_BENCH_STOP_OUTPUT_ERROR);
    CHECK(BSP_PWM_GetEscPulse(1)==0 && BSP_PWM_GetEscPulse(2)==0);
    CHECK(!state.enabled_mask);
#endif

    acceptance_active=1; f.servo_cal_active=1; run_motor_arbitration(&f);
    APP_EscDiag_Report();
    /*
     * 三档各自的协议名互不相让（bsp_pwm.c::BSP_PWM_EscProtocol() 原话："不能把
     * 双向档并进 DSHOT300"），所以这里也拆成三支，不是把 BIDIR 塞进 DSHOT300 分支
     * 或放任它落进 PWM 的 #else。command_unit/ack 两个字段两档 DShot 共用
     * （app_esc_diag.c 按 BSP_ESC_PROTOCOL_IS_DSHOT 分支，见该文件）。
     */
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    CHECK(strstr(report,"protocol=DSHOT300 command_unit=pwm_equivalent_us ack=unavailable"));
    CHECK(strstr(report,"enabled=0"));
#elif BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    CHECK(strstr(report,"protocol=DSHOT300_BIDIR command_unit=pwm_equivalent_us ack=unavailable"));
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
/*
 * 开机自检宽限期：期内**一次都不许翻成输入捕获**。
 *
 * 这是 2026-09-21 加的：AM32 靠"未解锁时每收一帧采样一次引脚、读到高才计数、
 * 累计过 100 才切双向"来自检（`AM32/Src/dshot.c` computeDshotDMA）。接收相会把
 * 通道翻成高阻输入约 2 ms，两路轮流就等于把能计数的帧打了对折。宽限期把这段
 * 让出来，线 100% 驻在空闲高。
 *
 * 期内翻了面，这条设计就等于没有——而症状只是"电调还是不回话"，和没加之前
 * 一模一样，从外面根本看不出来是哪一侧的问题。所以必须逐帧钉。
 *
 * 非双向档没有接收相，本 case 退化成"确认压根没有输入模式"，同样成立。
 */
/*
 * 引脚电平读取的位映射：PE9 -> bit0（通道1）、PE11 -> bit1（通道2）。
 *
 * 这条必须钉死。这个函数存在的全部意义是"量出来的电平"，把两位接反或者读错
 * 位号，得到的仍然是一个看起来很合理的数字——2026-09-21 就是靠它把"我们以为
 * 设对了"和"线上真的是高"分开的，位错了这个结论就整个反过来，而没有任何别的
 * 地方能发现。
 */
static int pin_levels(void) {
    test_gpioe.IDR = 0U;
    CHECK(BSP_DShot_ReadEscPinLevels()==0U);
    test_gpioe.IDR = (1UL<<9);
    CHECK(BSP_DShot_ReadEscPinLevels()==1U);
    test_gpioe.IDR = (1UL<<11);
    CHECK(BSP_DShot_ReadEscPinLevels()==2U);
    test_gpioe.IDR = (1UL<<9)|(1UL<<11);
    CHECK(BSP_DShot_ReadEscPinLevels()==3U);
    /* 邻位不许串进来：PE8/PE10/PE12 动了不影响结果。 */
    test_gpioe.IDR = (1UL<<8)|(1UL<<10)|(1UL<<12);
    CHECK(BSP_DShot_ReadEscPinLevels()==0U);
    return 0;
}

static int grace_period(void) {
    uint16_t code[2]={48,2047};
    CHECK(pin_levels()==0);
    /* 注意：不能用 setup()——它自己就会把宽限期跑满。 */
    memset(&test_tim1,0,sizeof(test_tim1)); memset(&test_stream2,0,sizeof(test_stream2));
    memset(&test_dma1,0,sizeof(test_dma1)); memset(&test_dmamux1_channel2,0,sizeof(test_dmamux1_channel2));
    memset(&dma,0,sizeof(dma)); memset(&state,0,sizeof(state)); initialized=0;
    dma.Instance=DMA1_Stream2; dma.Init.Request=DMA_REQUEST_TIM1_UP;
    dma.Init.Direction=DMA_MEMORY_TO_PERIPH; dma.Init.Mode=DMA_NORMAL;
    dma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD; dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    dma.Init.MemInc=DMA_MINC_ENABLE; dma.Init.PeriphInc=DMA_PINC_DISABLE;
    primask=starts=cleaned=irq_cleared=flags_cleared=0; start_fail=interrupt_during_clean=0;
    CHECK(BSP_DShot_Init()==BSP_DSHOT_OK);

#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    for (unsigned i=0;i<BSP_DSHOT_RX_DETECT_GRACE_FRAMES;i++) {
        BSP_DShotRxSnapshot s;
        CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
        complete();
        /* 两路都必须仍是输出比较（CCxS=0），线才是被驱动的高电平。 */
        CHECK((test_tim1.CCMR1&TIM_CCMR1_CC1S)==0U);
        CHECK((test_tim1.CCMR1&TIM_CCMR1_CC2S)==0U);
        /* 期内不记超时：本拍根本没在听，记成超时是在编故障。 */
        BSP_DShotRx_GetSnapshot(&s);
        CHECK(s.timeouts[0]==0U && s.timeouts[1]==0U);
        /* 剩余帧数必须**可见**且单调递减，否则这段等待从外面看是一片黑。 */
        CHECK(s.detect_grace==(uint16_t)(BSP_DSHOT_RX_DETECT_GRACE_FRAMES-1U-i));
    }
    {
        BSP_DShotRxSnapshot s;
        BSP_DShotRx_GetSnapshot(&s);
        CHECK(s.detect_grace==0U);
        /* 期满之后恢复轮流捕获：下一帧完成时必须真的翻面了。 */
        CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
        complete();
        CHECK((test_tim1.CCMR1&TIM_CCMR1_CC1S)!=0U);
    }
#else
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    complete();
    CHECK((test_tim1.CCMR1&TIM_CCMR1_CC1S)==0U);
    CHECK((test_tim1.CCMR1&TIM_CCMR1_CC2S)==0U);
#endif
    return 0;
}

int main(int argc,char **argv) {
    if(argc!=2) return 2;
    int n=atoi(argv[1]);
    if (n==5) { int rc=grace_period();
        if(!rc) puts("real BSP host state/register seam passed (not hardware validation)");
        return rc; }
    int rc=n==0?waveform():(n==1?races():(n==2?faults():(n==3?arbitration():vendor_errors())));
    if(!rc) puts("real BSP host state/register seam passed (not hardware validation)");
    return rc;
}

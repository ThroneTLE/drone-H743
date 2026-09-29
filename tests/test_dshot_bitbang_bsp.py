"""bitbang 后端的寄存器时序（`BSP/Src/bsp_dshot_bitbang.c`），宿主真编译。

**为什么单独开一个装置而不是塞进 `test_dshot_bsp.py`。** 那个装置是围绕
"定时器输出比较 + DMA burst"建的：它的 `setup()` 断言 `DCR==DMABASE_CCR1|...`、
`ARR==399`，整套判据都属于 TIMER 后端。bitbang 把定时器退化成纯时钟、把引脚
改成普通 GPIO，没有一条那样的断言仍然成立——硬塞进去只会让两边的守卫互相
让路，最后谁也没守住。

**这里钉的是什么。** Driver 层（子槽占空、GCR 解码）已经由
`tests/test_dshot_bitbang.py` 的 27 条用例判对错，本文件只管**绑定**：

  1. 发送相真的是"内存 -> GPIOE->BSRR"，接收相真的是"GPIOE->IDR -> 内存"；
     方向位写反的话 DMA 会把 BSRR 读进内存，而现象只是"电调不回话"——
     和我们 2026-09-21 追了一整天的那个症状一模一样，从外面分不出来。
  2. 引脚在两相之间确实翻了面，且接收相带上拉（双向 DShot 空闲为高，
     没有任何一方驱动的那几十微秒必须仍然是高，否则电调自检不出双向模式）。
  3. 空闲电平是**高**。这条错了电调根本进不了双向模式。
  4. 时钟不整除时拒绝初始化，而不是带着固定相位误差发一串没人认的帧。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/dshot_px4"

HARNESS = r"""
#include "tim.h"
#include "bsp_dshot.h"
#include "bsp_dshot_rx.h"
#include "drv_dshot.h"
#include "drv_dshot_bitbang.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)

TIM_TypeDef test_tim1, test_tim4;
GPIO_TypeDef test_gpioe;
DMA_Stream_TypeDef test_stream2;
RCC_TypeDef test_rcc;
DMA_TypeDef test_dma1;
DMAMUX_Channel_TypeDef test_dmamux1_channel2;
static DMA_HandleTypeDef dma;
TIM_HandleTypeDef htim1 = {&test_tim1,{&dma}}, htim4 = {&test_tim4,{0}};
static uint32_t primask;

/* 引脚配置的影子：记录最后一次 HAL_GPIO_Init 的模式与上下拉，
   这是"两相之间引脚确实翻了面"唯一能在宿主上观测的地方。 */
static uint32_t gpio_mode, gpio_pull;
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    if (port != GPIOE) { abort(); }
    if (init->Pin != (GPIO_PIN_9 | GPIO_PIN_11)) { abort(); }
    gpio_mode = init->Mode; gpio_pull = init->Pull;
}

uint32_t HAL_GetTick(void) { static uint32_t t; return ++t; }
uint32_t BSP_Critical_Enter(void) { uint32_t old=primask; primask=1; return old; }
void BSP_Critical_Exit(uint32_t old) { primask=old; }
void BSP_Critical_MemoryBarrier(void) {}
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
/* `tim.h` 的 __HAL_DMA_CLEAR_FLAG 宏最终落到这里（装置里 DMA 标志寄存器是影子，
   不需要真的清）。abort 路径会用到它，缺了就链不上。 */
static uint32_t flags_cleared;
void test_clear_flags(uint32_t flags) { flags_cleared = flags; }
static uint32_t apb2_div = RCC_APB2_DIV2;
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c,uint32_t *l) { c->APB2CLKDivider=apb2_div; *l=0; }
static uint32_t pclk2_hz = 60000000U;
uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2_hz; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 240000000U; }
/* 缓冲对齐是真实约束（cache 行），照旧检查；长度不锁死。 */
void BSP_Cache_CleanDCache(const void *p,uint32_t len) {
    if (((uintptr_t)p&31)!=0) { abort(); } (void)len;
}
void BSP_Cache_InvalidateDCache(const void *p,uint32_t len) {
    if (((uintptr_t)p&31)!=0) { abort(); } (void)len;
}

#include "bsp_dshot_bitbang.c"

/* DMA 完成：照搬真实回调的触发方式，不自己伪造成功。 */
static void complete(void) {
    test_stream2.CR &= ~DMA_SxCR_EN;
    dma.State = HAL_DMA_STATE_READY; dma.Lock = 0;
    dma.XferCpltCallback(&dma);
}

static int setup(void) {
    memset(&test_tim1,0,sizeof(test_tim1)); memset(&test_stream2,0,sizeof(test_stream2));
    memset(&test_gpioe,0,sizeof(test_gpioe)); memset(&dma,0,sizeof(dma));
    primask=0; gpio_mode=0xFFFFFFFFU; gpio_pull=0xFFFFFFFFU;
    apb2_div = RCC_APB2_DIV2; pclk2_hz = 60000000U;   /* -> 120 MHz 定时器时钟 */
    dma.Instance = DMA1_Stream2;
    CHECK(BSP_DShot_Init()==BSP_DSHOT_OK);
    return 0;
}

/* 1. 时钟推导与整除守卫。 */
static int clock_guard(void) {
    CHECK(setup()==0);
    /* 120 MHz / 2.4 MHz = 50 -> ARR = 49。除不尽就不该起来。 */
    CHECK(test_tim1.ARR==49);
    CHECK(test_tim1.PSC==0);

    memset(&test_tim1,0,sizeof(test_tim1)); memset(&dma,0,sizeof(dma));
    dma.Instance = DMA1_Stream2;
    pclk2_hz = 55000000U;   /* -> 110 MHz，110/2.4 除不尽 */
    CHECK(BSP_DShot_Init()==BSP_DSHOT_ERROR);
    return 0;
}

/* 2. 空闲电平必须是高，而且是在 Init 里就立起来的。 */
static int idle_high(void) {
    CHECK(setup()==0);
    /* BSRR 低 16 位 = 置位 = 拉高；两路都要。 */
    CHECK((test_gpioe.BSRR & (GPIO_PIN_9|GPIO_PIN_11)) == (uint32_t)(GPIO_PIN_9|GPIO_PIN_11));
    /* 且绝不能同时出现复位位——BSRR 里复位优先，那会静默把线拉低。 */
    CHECK((test_gpioe.BSRR & (((uint32_t)GPIO_PIN_9<<16)|((uint32_t)GPIO_PIN_11<<16))) == 0U);
    CHECK(gpio_mode==GPIO_MODE_OUTPUT_PP);
    return 0;
}

/* 3. 发送相：方向、目标寄存器、长度、引脚模式。 */
static int tx_phase(void) {
    CHECK(setup()==0);
    uint16_t code[2]={48,1000};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    /* 内存 -> 外设：DIR=01。写反会把 BSRR 读进内存，而现象只是"电调没反应"。 */
    CHECK((test_stream2.CR & DMA_SxCR_DIR) == DMA_SxCR_DIR_0);
    CHECK(test_stream2.PAR == (uint32_t)(uintptr_t)&GPIOE->BSRR);
    CHECK(test_stream2.NDTR == DRV_DSHOT_BB_FRAME_WORDS);
    CHECK(test_stream2.CR & DMA_SxCR_MINC);
    CHECK(test_stream2.CR & DMA_SxCR_EN);
    /* 发送相引脚必须是推挽输出，不能带上下拉（我们自己在驱动）。 */
    CHECK(gpio_mode==GPIO_MODE_OUTPUT_PP);
    CHECK(gpio_pull==GPIO_NOPULL);
    CHECK(test_tim1.CR1 & TIM_CR1_CEN);
    CHECK(test_tim1.DIER & TIM_DIER_UDE);
    return 0;
}

/* 4. 接收相：发完立刻翻面，方向反过来，且带上拉。 */
static int rx_phase(void) {
    CHECK(setup()==0);
    uint16_t code[2]={48,1000};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    complete();
    /* 外设 -> 内存：DIR=00。 */
    CHECK((test_stream2.CR & DMA_SxCR_DIR) == 0U);
    CHECK(test_stream2.PAR == (uint32_t)(uintptr_t)&GPIOE->IDR);
    CHECK(test_stream2.CR & DMA_SxCR_MINC);
    CHECK(test_stream2.CR & DMA_SxCR_EN);
    /*
     * 输入 + 上拉。上拉不是"读得更稳"，是让没有任何一方驱动的那几十微秒里
     * 线仍然停在空闲高——AM32 正是靠读到高电平来自检双向模式的。
     */
    CHECK(gpio_mode==GPIO_MODE_INPUT);
    CHECK(gpio_pull==GPIO_PULLUP);
    return 0;
}

/* 5. 两相之间方向位不许残留：下一拍发送必须重新变回内存->外设。 */
static int direction_does_not_leak(void) {
    CHECK(setup()==0);
    uint16_t code[2]={48,1000};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    complete();
    CHECK((test_stream2.CR & DMA_SxCR_DIR) == 0U);   /* 现在是接收相 */
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    CHECK((test_stream2.CR & DMA_SxCR_DIR) == DMA_SxCR_DIR_0);
    CHECK(test_stream2.PAR == (uint32_t)(uintptr_t)&GPIOE->BSRR);
    CHECK(gpio_mode==GPIO_MODE_OUTPUT_PP);
    return 0;
}

/* 6. 禁用：立刻停流并把线钉回空闲高，不依赖下一拍。 */
static int disable_holds_idle_high(void) {
    CHECK(setup()==0);
    uint16_t code[2]={48,1000};
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK);
    test_gpioe.BSRR = 0U;
    CHECK(BSP_DShot_Disable(3)==BSP_DSHOT_OK);
    CHECK((test_gpioe.BSRR & (GPIO_PIN_9|GPIO_PIN_11)) == (uint32_t)(GPIO_PIN_9|GPIO_PIN_11));
    CHECK((test_stream2.CR & DMA_SxCR_EN) == 0U);
    CHECK(!(test_tim1.CR1 & TIM_CR1_CEN));
    return 0;
}

int main(int argc,char **argv) {
    if(argc!=2) return 2;
    int n=atoi(argv[1]);
    int rc = n==0?clock_guard():(n==1?idle_high():(n==2?tx_phase():
             (n==3?rx_phase():(n==4?direction_does_not_leak():disable_holds_idle_high()))));
    if(!rc) puts("bitbang bsp seam passed (not hardware validation)");
    return rc;
}
"""


def _compile_harness(build, content):
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")
    harness = build / "harness.c"
    harness.write_text(content, encoding="utf-8")
    path = build / "bsp.exe"
    cmd = [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
           "-DBSP_ESC_PROTOCOL=2",
           "-I", str(FIXTURES), "-I", str(ROOT / "BSP/Inc"), "-I", str(ROOT / "BSP/Src"),
           "-I", str(ROOT / "Driver/Inc"),
           str(harness),
           str(ROOT / "Driver/Src/drv_dshot.c"),
           str(ROOT / "Driver/Src/drv_dshot_telemetry.c"),
           str(ROOT / "Driver/Src/drv_dshot_bitbang.c"),
           "-o", str(path)]
    # 本机 Python 默认用 GBK 读文本，而编译器会把源码里的中文注释原样带进
    # 错误信息里；不显式指定 utf-8 会在读取阶段抛 UnicodeDecodeError，
    # 把一个普通的编译失败变成看不懂的装置崩溃。
    r = subprocess.run(cmd, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    return path


@pytest.fixture(scope="module")
def executable(tmp_path_factory):
    return _compile_harness(tmp_path_factory.mktemp("dshot-bitbang-bsp"), HARNESS)


@pytest.mark.parametrize("case", [0, 1, 2, 3, 4, 5],
                         ids=["clock-guard", "idle-high", "tx-phase", "rx-phase",
                              "direction-no-leak", "disable-holds-idle"])
def test_bitbang_bsp(executable, case):
    r = subprocess.run([str(executable), str(case)], capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr


# Inject decoded payloads at the codec seam; harvest, classification and snapshot
# are real production C. Wire/GCR correctness is covered by separate golden tests.
TELEMETRY_SEAM = r"""
static uint16_t injected_payload;
static DRV_DShotTelemStatus injected_samples(const uint32_t *s,size_t n,uint32_t m,uint32_t q,uint32_t *o) {
    (void)s;(void)n;(void)m;(void)q;*o=0;return DRV_DSHOT_TELEM_OK;
}
static DRV_DShotTelemStatus injected_decode(uint32_t r,uint16_t *p) {
    (void)r;*p=injected_payload;return DRV_DSHOT_TELEM_OK;
}
#define DRV_DShotBitbang_BitsFromSamples injected_samples
#define DRV_DShotTelem_DecodeRaw injected_decode
#include "bsp_dshot_bitbang.c"
#undef DRV_DShotBitbang_BitsFromSamples
#undef DRV_DShotTelem_DecodeRaw
"""
TELEMETRY_MAIN = r"""
static int deliver(uint16_t payload) {
    uint16_t code[2]={0,0}; injected_payload=payload;
    CHECK(BSP_DShot_Submit(code,3)==BSP_DSHOT_OK); complete();
    test_stream2.NDTR=BB_RX_SAMPLES-2U; bb_rx_harvest();
    return 0;
}
int main(int argc,char **argv) {
    CHECK(argc==2); CHECK(setup()==0);
    int scenario=atoi(argv[1]); BSP_DShotRxSnapshot before,after;
    if(scenario==0) {
        CHECK(deliver(0x60AU)==0); BSP_DShotRx_GetSnapshot(&after);
        for(unsigned i=0;i<2;i++) {
            CHECK(after.frames[i]==1U && after.current_valid[i] && after.current_a[i]==10U);
            CHECK(after.current_sample_ms[i]!=0U);
            CHECK(!after.valid[i] && after.sample_ms[i]==0U);
        }
    } else if(scenario==1) {
        CHECK(deliver(0x100U)==0); BSP_DShotRx_GetSnapshot(&before);
        CHECK(deliver(0x60AU)==0); CHECK(deliver(0x22AU)==0); CHECK(deliver(0x450U)==0);
        BSP_DShotRx_GetSnapshot(&after);
        for(unsigned i=0;i<2;i++) {
            CHECK(before.valid[i] && before.erpm[i]==234375U);
            CHECK(after.valid[i] && after.erpm[i]==before.erpm[i]);
            CHECK(after.sample_ms[i]==before.sample_ms[i] && after.age_ticks[i]==3U);
            CHECK(after.frames[i]==4U && after.current_valid[i] && after.current_a[i]==10U);
        }
    } else {
        CHECK(deliver(0x22AU)==0); BSP_DShotRx_GetSnapshot(&before);
        CHECK(!before.valid[0] && !before.valid[1]);
        CHECK(deliver(0xFFFU)==0); BSP_DShotRx_GetSnapshot(&after);
        for(unsigned i=0;i<2;i++) {
            CHECK(after.valid[i] && after.not_spinning[i] && after.erpm[i]==0U);
            CHECK(after.sample_ms[i]!=0U && after.age_ticks[i]==0U);
        }
    }
    puts("EDT/eRPM freshness contract passed (host injection, not hardware)");return 0;
}
"""


@pytest.fixture(scope="module")
def telemetry_executable(tmp_path_factory):
    source=HARNESS.replace('#include "bsp_dshot_bitbang.c"', TELEMETRY_SEAM)
    source=source.replace('int main(int argc,char **argv)', 'int original_main(int argc,char **argv)')
    return _compile_harness(tmp_path_factory.mktemp("dshot-bitbang-freshness"),source+TELEMETRY_MAIN)


@pytest.mark.parametrize("scenario", [0,1,2], ids=["edt-cannot-create-erpm", "edt-cannot-refresh-erpm", "stopped-is-valid-erpm"])
def test_bitbang_telemetry_freshness(telemetry_executable,scenario):
    result=subprocess.run([str(telemetry_executable),str(scenario)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr

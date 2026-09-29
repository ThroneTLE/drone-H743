"""Actual BSP in a host register/DMA seam; tests do not open hardware."""
from pathlib import Path
import shutil
import subprocess
import re
import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/dshot_px4"


#
# 三档都要进这个装置。2026-09-21 之前这里只有 [0, 1]，于是 DSHOT300_BIDIR 档
# **整条输出接入都没接**却能全绿：bsp_pwm.c 里每一个判断点写的都是
# `== BSP_ESC_PROTOCOL_DSHOT300`，双向档在每一处都掉进"不是 DShot"的分支，
# BSP_DShot_Init() 从来没被调用过。编译得过不等于接得上——而这是唯一一个
# 拿真实 bsp_pwm.c 跑仲裁链的装置，它不覆盖的档位就是没人覆盖。
#
@pytest.fixture(scope="module", params=[0, 1, 2],
                ids=["PWM", "DSHOT300", "DSHOT300_BIDIR_TIMER"])
def executable(tmp_path_factory, request):
    build = tmp_path_factory.mktemp("dshot-bsp")
    path = build / "bsp.exe"
    source = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    # 起点取点电机窗口的节拍调用，不是下面第一个 if：心跳超时的判定就在那一句里，
    # 从 if 开始切会把它留在秒外，于是"超时必停"这条在本 seam 里永远测不到——
    # 而它恰恰是这条链上唯一一条由时间驱动的安全属性。
    start = source.index("  APP_PropSpin_Step(frame->now_ms,")
    end = source.index("  if (APP_Acceptance_IsActive() != 0U) {\n    APP_AcceptanceObservation", start)
    body = source[start:end]
    # 上下桨 -> ESC 通道的路由也照原样编进来：它和仲裁链是一个整体，
    # 在这里换成一个"直接写通道 1/2"的替身，等于把接线标定这一层测没了。
    routing_start = source.index(
        "static void stabilizer_commit_rotor_pulses(uint16_t upper_us, uint16_t lower_us)")
    routing = source[routing_start:source.index("\n}\n", routing_start) + 3]
    reasons = re.findall(r"(APP_FLIGHT_LOG_MOTOR_REASON_\w+)\s*=\s*(\d+)",
                         (ROOT / "App/Inc/app_flight_log.h").read_text(encoding="utf-8"))
    seam = "\n".join(f"#define {name} {value}" for name, value in reasons)
    seam += """
    #include "app_prop_spin.h"
    #include "app_esc_command.h"
    #include "drv_prop_map.h"
    typedef struct { uint16_t pulse_us[2]; uint16_t percent_x100[2];
        uint32_t window_token,last_request_id,map_generation;
        uint8_t upper_channel,lower_channel,active,max_percent; } APP_ThrustBenchOutput;
    #define APP_THRUST_BENCH_STOP_OUTPUT_ERROR 5U
    static uint8_t thrust_bench_active,thrust_bench_stop_reason;
    static APP_ThrustBenchOutput thrust_bench_output;
    static uint8_t APP_ThrustBench_IsActive(void) { return thrust_bench_active; }
    static void APP_ThrustBench_Step(uint32_t now_ms,uint8_t inhibit) { (void)now_ms;(void)inhibit; }
    static void APP_ThrustBench_GetOutput(APP_ThrustBenchOutput *out) { *out=thrust_bench_output; }
    static void APP_ThrustBench_Close(uint32_t now_ms,uint8_t reason) {
        (void)now_ms; thrust_bench_active=0U; thrust_bench_stop_reason=reason;
        memset(&thrust_bench_output,0,sizeof(thrust_bench_output));
    }
typedef struct {
    uint8_t rc_link_ok,rc_armed,rc_link_seen,servo_cal_active,rc_control_motor_mix_allowed;
    uint8_t ident_running,imu_control_valid,rc_attitude_debug_mode,motor_output_reason;
    uint8_t sysid_running;
    uint16_t rc_throttle_motor_us;
    uint32_t now_ms;
    struct { uint16_t motor_upper_us,motor_lower_us; } ctrl_out;
} ArbFrame;
static int acceptance_active;
static int APP_Acceptance_IsActive(void) { return acceptance_active; }
/* 辨识自动油门（app_sysid）：本装置不跑辨识，sysid_running 恒 0，替身永不接管。 */
static uint8_t APP_SysId_GetMotorPulse(uint16_t *pulse_us) { (void)pulse_us; return 0U; }
""" + routing + """
static void run_motor_arbitration(ArbFrame *frame) {
""" + body + "\n}\n"
    (build / "arbitration_seam.h").write_text(seam, encoding="utf-8")
    # Compile the vendor's DMA1/2 IRQ branch verbatim, including callback order.
    # BDMA and the unused BDMA preamble are outside this board binding's seam.
    hal = (ROOT / "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c").read_text()
    irq = hal.split("void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)", 1)[1]
    branch = irq.split("if(IS_DMA_STREAM_INSTANCE(hdma->Instance) != 0U)", 1)[1]
    branch = branch.split("else if(IS_BDMA_CHANNEL_INSTANCE", 1)[0]
    (build / "hal_irq_seam.h").write_text(
        "static void vendor_stream_irq(DMA_HandleTypeDef *hdma) {\n"
        "DMA_Base_Registers *regs_dma=(DMA_Base_Registers*)hdma->StreamBaseAddress;\n"
        "uint32_t tmpisr_dma=regs_dma->ISR, count=0, timeout=480000000U/9600U;\n"
        + branch + "\n}\n", encoding="utf-8")
    cmd = [shutil.which("gcc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
           "-I", str(FIXTURES), "-I", str(ROOT / "BSP/Inc"), "-I", str(ROOT / "BSP/Src"),
           f"-DBSP_ESC_PROTOCOL={request.param}",
           # 本装置验的是**TIMER 后端**（DMA burst + 输入捕获）：它的判据里
           # `DCR==DMABASE_CCR1|...`、`ARR==399`、通道翻面全都是那个实现独有的。
           # 双向档默认已切到 BITBANG（见 bsp_esc_protocol.h），所以这里必须显式
           # 钉死后端，否则整份被测实现不参与编译，装置会变成在验空气。
           # BITBANG 后端有自己的绑定装置：tests/test_dshot_bitbang_bsp.py。
           "-DBSP_ESC_BIDIR_BACKEND=0", "-I", str(build),
           "-I", str(ROOT / "App/Inc"), "-I", str(ROOT / "App/Src"),
           "-I", str(ROOT / "Driver/Inc"), str(FIXTURES / "bsp_harness.c"),
           str(ROOT / "Driver/Src/drv_dshot.c"),
           # DSHOT300_BIDIR 才用得到：bsp_dshot.c 的 encode_packets() 和
           # bsp_dshot_rx.c 的 Harvest 都调 DRV_DShotTelem_*（纯解码层，
           # 由 tests/test_dshot_telemetry.py 单独判对错）。PWM/DSHOT300 下
           # 链接器多摆一个未调用符号的目标文件，无害。
           str(ROOT / "Driver/Src/drv_dshot_telemetry.c"),
           str(ROOT / "Driver/Src/drv_prop_map.c"),
           # 仲裁 seam 从 app_stabilizer.c 原样切出来，所以电调特殊命令那条支路
           # 也跟着进来了：命令帧会顶掉这一拍的油门帧，那是执行器仲裁的一部分，
           # 必须在这个装置里真跑，不能用替身糊过去。
           str(ROOT / "App/Src/app_esc_command.c"),
           str(ROOT / "App/Src/app_prop_spin.c"), "-o", str(path)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    return path


@pytest.mark.parametrize("case", [0, 1, 2, 3, 4, 5], ids=["preload-tail", "busy-disable-races", "fault-latch", "real-arbitration", "vendor-irq-errors", "rx-detect-grace"])
def test_real_bsp(executable, case):
    r = subprocess.run([str(executable), str(case)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_abort_is_bounded_and_does_not_depend_on_interrupts():
    source = (ROOT / "BSP/Src/bsp_dshot.c").read_text(encoding="utf-8")
    abort = source.split("static uint8_t abort_dma(void)")[1].split("static void latch_fault")[0]
    assert "tries < 256U" in abort
    assert "HAL_DMA_Abort(" not in abort and "HAL_GetTick(" not in abort
    assert abort.index("~TIM_DIER_UDE") < abort.index("~(DMA_SxCR_EN")
    assert "HAL_NVIC_ClearPendingIRQ" in abort
    assert 'section(".dma_buffer"), aligned(32)' in source


def test_register_constants_match_actual_st_headers(tmp_path):
    mock = (FIXTURES / "tim.h").read_text(encoding="utf-8")
    constants = re.findall(r"^#define ((?:TIM|DMA|RCC|HAL_DMA)_\w+) (0x[\da-fA-F]+U?|\d+U?)$", mock, re.M)
    text = '#include "stm32h7xx_hal.h"\n' + "\n".join(
        f'_Static_assert({name} == {value}, "{name} mismatch");' for name, value in constants)
    src = tmp_path / "register_constants.c"
    src.write_text(text)
    command = [shutil.which("arm-none-eabi-gcc"), "-std=c11", "-mcpu=cortex-m7", "-mthumb",
               "-DSTM32H743xx", "-DUSE_HAL_DRIVER"]
    for inc in ("Core/Inc", "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/CMSIS/Include",
                "Drivers/CMSIS/Device/ST/STM32H7xx/Include"):
        command += ["-I", str(ROOT / inc)]
    r = subprocess.run(command + ["-c", str(src), "-o", str(tmp_path / "regs.o")], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr

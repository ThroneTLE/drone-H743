"""Actual BSP in a host register/DMA seam; tests do not open hardware."""
from pathlib import Path
import shutil
import subprocess
import re
import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/dshot_px4"


@pytest.fixture(scope="module", params=[0, 1], ids=["PWM", "DSHOT300"])
def executable(tmp_path_factory, request):
    build = tmp_path_factory.mktemp("dshot-bsp")
    path = build / "bsp.exe"
    source = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    start = source.index("  if (APP_Acceptance_IsActive() != 0U) {\n    BSP_PWM_DisableEsc(1U);")
    end = source.index("  if (APP_Acceptance_IsActive() != 0U) {\n    APP_AcceptanceObservation", start)
    body = source[start:end]
    reasons = re.findall(r"(APP_FLIGHT_LOG_MOTOR_REASON_\w+)\s*=\s*(\d+)",
                         (ROOT / "App/Inc/app_flight_log.h").read_text(encoding="utf-8"))
    seam = "\n".join(f"#define {name} {value}" for name, value in reasons)
    seam += """
typedef struct {
    uint8_t rc_link_ok,rc_armed,rc_link_seen,servo_cal_active,rc_control_motor_mix_allowed;
    uint8_t ident_running,imu_control_valid,rc_attitude_debug_mode,motor_output_reason;
    uint16_t rc_throttle_motor_us;
    struct { uint16_t motor_upper_us,motor_lower_us; } ctrl_out;
} ArbFrame;
static int acceptance_active;
static int APP_Acceptance_IsActive(void) { return acceptance_active; }
static void run_motor_arbitration(ArbFrame *frame) {
""" + body + "\n}\n"
    (build / "arbitration_seam.h").write_text(seam, encoding="utf-8")
    cmd = [shutil.which("gcc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
           "-I", str(FIXTURES), "-I", str(ROOT / "BSP/Inc"), "-I", str(ROOT / "BSP/Src"),
           f"-DBSP_ESC_PROTOCOL={request.param}", "-I", str(build),
           "-I", str(ROOT / "App/Inc"), "-I", str(ROOT / "App/Src"),
           "-I", str(ROOT / "Driver/Inc"), str(FIXTURES / "bsp_harness.c"),
           str(ROOT / "Driver/Src/drv_dshot.c"), "-o", str(path)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    return path


@pytest.mark.parametrize("case", [0, 1, 2, 3], ids=["preload-tail", "busy-disable-races", "fault-latch", "real-arbitration"])
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
    mock = (FIXTURES / "tim.h").read_text()
    constants = re.findall(r"^#define ((?:TIM|DMA|RCC)_\w+) (0x[\da-fA-F]+U?|\d+U?)$", mock, re.M)
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

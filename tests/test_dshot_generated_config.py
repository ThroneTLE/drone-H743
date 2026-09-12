"""R-DSHOT-1: gate BSP integration on actual CubeMX-generated DMA ownership."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
REGENERATE = "Regenerate D:/stm32hal/drone-H743-dshot/drone-H743.ioc in CubeMX; do not hand-edit Core."


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_dshot_ioc_dma_owner_and_timer_groups():
    values = dict(line.split("=", 1) for line in read("drone-H743.ioc").splitlines() if "=" in line)
    stream2 = [key for key, value in values.items() if key.endswith(".Instance") and value == "DMA1_Stream2"]
    assert len(stream2) == 1 and "TIM1_UP" in stream2[0]
    prefix = stream2[0].removesuffix("Instance")
    for key, value in {
        "Direction": "DMA_MEMORY_TO_PERIPH", "Mode": "DMA_NORMAL",
        "MemDataAlignment": "DMA_MDATAALIGN_WORD", "PeriphDataAlignment": "DMA_PDATAALIGN_WORD",
        "MemInc": "DMA_MINC_ENABLE", "PeriphInc": "DMA_PINC_DISABLE",
    }.items():
        assert values[prefix + key] == value
    assert "Dma.UART8_RX" not in read("drone-H743.ioc")
    assert "PE9.Signal=S_TIM1_CH1" in read("drone-H743.ioc")
    assert "PE11.Signal=S_TIM1_CH2" in read("drone-H743.ioc")
    assert values["TIM4.Period"] == "19999"
    assert values["TIM4.Prescaler"] == "120-1"


def test_dshot_generated_dma_matches_ioc():
    timers = read("Core/Src/tim.c")
    uart = read("Core/Src/usart.c")
    irq = read("Core/Src/stm32h7xx_it.c")
    assert "hdma_tim1_up.Instance = DMA1_Stream2;" in timers, REGENERATE
    assert "hdma_tim1_up.Init.Request = DMA_REQUEST_TIM1_UP;" in timers, REGENERATE
    assert re.search(r"__HAL_LINKDMA\(\s*\w+\s*,\s*hdma\[TIM_DMA_ID_UPDATE\]\s*,\s*hdma_tim1_up\s*\)", timers), REGENERATE
    assert "HAL_DMA_IRQHandler(&hdma_tim1_up);" in irq, REGENERATE
    assert "hdma_uart8_rx" not in uart + irq, REGENERATE
    assert "hdma_uart8_tx.Instance = DMA1_Stream3;" in uart
    assert "HAL_UART_Receive_IT(BSP_UART_MAINT_HANDLE, byte, 1U)" in read("BSP/Src/bsp_uart.c")


def test_dshot_generated_bit_clock_matches_ioc():
    timers = read("Core/Src/tim.c")
    assert "htim1.Init.Prescaler = 0;" in timers, REGENERATE
    assert "htim1.Init.Period = 399;" in timers, REGENERATE
    assert "htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;" in timers, REGENERATE

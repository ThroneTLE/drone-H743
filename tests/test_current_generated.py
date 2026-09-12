"""ADC must really be generated, not merely look configured in the pin list."""
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT/path).read_text(encoding="utf-8")


def test_pc1_shared_signal_activates_the_adc_mode():
    ioc=read("drone-H743.ioc")
    assert "PC1.Signal=ADCx_INP11" in ioc
    assert "SH.ADCx_INP11.0=ADC1_INP11,IN11-Single-Ended" in ioc
    assert "ADC1.Channel-0\\#ChannelRegularConversion=ADC_CHANNEL_11" in ioc
    assert "ADC1.ConversionDataManagement=ADC_CONVERSIONDATA_DR" in ioc
    assert not re.search(r"^Dma\.ADC",ioc,re.M)
    assert "Dma.TIM1_UP.2.Instance=DMA1_Stream2" in ioc  # DShot resource preserved


def test_generated_adc_matches_the_current_driver():
    assert (ROOT/"Core/Src/adc.c").exists(), "CubeMX must generate ADC1 before integration"
    adc=read("Core/Src/adc.c")
    for text in ("hadc1.Instance = ADC1;", "hadc1.Init.Resolution = ADC_RESOLUTION_16B;",
                 "hadc1.Init.NbrOfConversion = 1;", "hadc1.Init.ContinuousConvMode = DISABLE;",
                 "sConfig.Channel = ADC_CHANNEL_11;", "sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;",
                 "hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;"):
        assert text in adc, text
    assert "hadc1.Init.OversamplingMode = DISABLE;" in adc
    assert "hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;" in adc
    conf=re.sub(r"/\*.*?\*/", "", read("Core/Inc/stm32h7xx_hal_conf.h"),flags=re.S)
    assert re.search(r"^\s*#define\s+HAL_ADC_MODULE_ENABLED\b",conf,re.M)
    assert "MX_ADC1_Init();" in read("Core/Src/main.c")


def test_current_monitor_is_wired_to_background_and_status():
    tasks=read("App/Src/app_tasks.c")
    init=tasks.split("void APP_Task_Background_Init(void)")[1].split("\n}")[0]
    step=tasks.split("void APP_Task_Background_Step(void)")[1].split("\n}")[0]
    assert "APP_Current_Init();" in init
    assert "APP_Current_Step();" in step
    status=read("App/Src/app_cmd_system.c").split("void app_control_report_status(void)")[1].split("\n}")[0]
    assert "APP_Current_Report();" in status

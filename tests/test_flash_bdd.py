from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_bdd_flash_diagnostics_flow_is_app_service_to_gd25q32_driver() -> None:
    """BDD: given flash DMA is stable, diagnostics use service -> driver -> BSP bus."""

    control = (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")
    service = (ROOT / "App/Src/app_flash_service.c").read_text(encoding="utf-8")
    bus = (ROOT / "BSP/Inc/bsp_flash_bus.h").read_text(encoding="utf-8")

    assert "FLASH verify" in control
    assert "APP_FlashService_ReadData(" in control
    assert "APP_FlashService_ReadDataFast(" in control
    assert "BSP_FLASH_" not in control

    assert "DRV_GD25Q32_ReadData(" in service
    assert "DRV_GD25Q32_ReadDataFast(" in service
    assert "BSP_FlashBus_Acquire" in bus
    assert "BSP_FlashBus_Release" in bus


def test_bdd_h743_cache_setting_matches_runtime_cache_enable() -> None:
    """BDD: given H743 DMA is used, CubeMX cache settings match BSP runtime."""

    ioc = (ROOT / "drone-H743.ioc").read_text(encoding="utf-8")
    system = (ROOT / "BSP/Src/bsp_system.c").read_text(encoding="utf-8")
    cache = (ROOT / "BSP/Src/bsp_cache.c").read_text(encoding="utf-8")

    assert "CORTEX_M7.CPU_ICache=Enabled" in ioc
    assert "CORTEX_M7.CPU_DCache=Enabled" in ioc
    assert "CPU_ICache,CPU_DCache" in ioc
    assert "BSP_Cache_Enable();" in system
    assert "SCB_EnableICache();" in cache
    assert "SCB_EnableDCache();" in cache

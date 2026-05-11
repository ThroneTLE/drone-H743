from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_app_uses_flash_service_not_bsp_flash_api() -> None:
    app_sources = list((ROOT / "App").glob("**/*.[ch]"))

    offenders = []
    for path in app_sources:
        text = path.read_text(encoding="utf-8")
        if '#include "bsp_flash.h"' in text or "BSP_FLASH_" in text:
            offenders.append(path.relative_to(ROOT).as_posix())

    assert offenders == []


def test_bsp_flash_bus_has_no_device_level_flash_api() -> None:
    header = read("BSP/Inc/bsp_flash_bus.h")

    forbidden = (
        "ReadData",
        "ReadDataFast",
        "EraseSector",
        "PageProgram",
        "WriteData",
        "ProbeJedecId",
    )

    assert "BSP_FlashBus_" in header
    for token in forbidden:
        assert token not in header


def test_gd25q32_driver_has_chip_specific_name() -> None:
    cmake = read("CMakeLists.txt")
    header = read("Driver/Inc/drv_gd25q32.h")
    source = read("Driver/Src/drv_gd25q32.c")

    assert "Driver/Src/drv_gd25q32.c" in cmake
    assert "Driver/Src/drv_flash.c" not in cmake
    assert "DRV_GD25Q32_ReadData" in header
    assert "DRV_FLASH_ReadData" not in header
    assert "DRV_GD25Q32_ReadDataFast" in source


def test_flash_service_is_public_app_boundary() -> None:
    header = read("App/Inc/app_flash_service.h")
    source = read("App/Src/app_flash_service.c")
    cmake = read("CMakeLists.txt")

    assert "App/Src/app_flash_service.c" in cmake
    assert "APP_FlashService_ReadData" in header
    assert "APP_FlashService_ReadDataFast" in header
    assert "DRV_GD25Q32_ReadData" in source
    assert "BSP_FlashBus_" in source


def test_legacy_bsp_chip_driver_removed() -> None:
    assert not (ROOT / "BSP/Inc/bsp_gd25q32.h").exists()
    assert not (ROOT / "BSP/Src/bsp_gd25q32.c").exists()

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
    """服务层仍是 App 侧唯一的持久化入口，但背后已经是多介质路由。

    MicoAir743v2 板上没有外部 SPI NOR，服务层从"GD25Q32 的薄封装"变成了
    按地址路由的门面：参数落片内 Flash、日志落 SD 裸块，NOR 专有的诊断接口
    仍打给真正的 NOR 驱动（探测会如实失败）。上层地址空间与几何一个字节没变，
    所以 svc_param 与 app_flight_log 不用改——这条边界正是本测试要守的东西。
    """
    header = read("App/Inc/app_flash_service.h")
    source = read("App/Src/app_flash_service.c")
    cmake = read("CMakeLists.txt")

    assert "App/Src/app_flash_service.c" in cmake
    assert "APP_FlashService_ReadData" in header
    assert "APP_FlashService_ReadDataFast" in header

    # 三个后端都由服务层拥有，别的地方不许直接碰。
    assert "DRV_INTFLASH_Read" in source
    assert "DRV_SDBLOCK_Read" in source
    assert "DRV_GD25Q32_ReadJedecId" in source
    assert "BSP_FlashBus_" in source

    # 路由必须存在且只有这一处；否则"参数和日志分别落在哪"就没人说得清了。
    assert "APP_FlashService_BackendFor" in header
    assert source.count("APP_FlashService_Backend APP_FlashService_BackendFor") == 1


def test_param_region_routing_matches_internal_flash_and_linker() -> None:
    """参数区的三处定义必须一致，错开一处就会擦掉代码或读到垃圾。

    - app_flash_service.h  逻辑地址空间里参数区的起点
    - drv_intflash.h       实际落在片内 Flash 的哪两个扇区
    - STM32H743XX_FLASH.ld 代码段必须让出这两个扇区
    """
    service_header = read("App/Inc/app_flash_service.h")
    intflash = read("Driver/Inc/drv_intflash.h")
    linker = read("STM32H743XX_FLASH.ld")

    assert "APP_FLASH_SERVICE_PARAM_SLOT_A_OFFSET" in service_header
    assert "APP_FLASH_SERVICE_PARAM_SLOT_B_OFFSET" in service_header

    # 两个 128 KB 扇区，各独占一个逻辑参数槽（片内擦除粒度就是 128 KB，
    # 共用一个扇区会让擦 A 把 B 一起抹掉，双槽掉电保护失效）。
    assert "0x081C0000" in intflash
    assert "0x081E0000" in intflash
    assert "DRV_INTFLASH_SECTOR_SIZE      (128UL * 1024UL)" in intflash

    # 2048K - 256K = 1792K：正好让出上面那两个扇区。
    assert "LENGTH = 1792K" in linker
    assert "LENGTH = 2048K" not in linker


def test_legacy_bsp_chip_driver_removed() -> None:
    assert not (ROOT / "BSP/Inc/bsp_gd25q32.h").exists()
    assert not (ROOT / "BSP/Src/bsp_gd25q32.c").exists()

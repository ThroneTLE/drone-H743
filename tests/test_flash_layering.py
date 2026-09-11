import re
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


def _int_literal(text: str, name: str) -> int:
    """从 C 头文件里取一个 `#define <name> <整数>`。"""
    match = re.search(rf"^#define\s+{name}\s+(0[xX][0-9a-fA-F]+|\d+)U?L?U?L?\s*$",
                      text, re.MULTILINE)
    assert match is not None, name
    return int(match.group(1), 0)


def test_param_region_routing_matches_internal_flash_and_linker() -> None:
    """参数区的四处定义必须一致，错开一处就会擦掉代码或读到垃圾。

    - app_flash_service.h  逻辑地址空间里参数区的起点与槽数
    - drv_intflash.h       实际落在片内 Flash 的哪几个扇区
    - STM32H743XX_FLASH.ld 代码段必须让出这些扇区
    - app_flight_log.h     日志区上界必须停在参数区之下

    这条测试是 2026-09-11 那个缺陷的机检：当时逻辑上留了三个扇区、物理上只映射了
    两个，配置记录（含机体模型）落在没有映射的那一个上，`SAVE` 恒返回
    INVALID_ARG。所以下面不只看"常量在不在"，而是真的把逻辑槽数和物理扇区数对上。
    """
    service_header = read("App/Inc/app_flash_service.h")
    intflash = read("Driver/Inc/drv_intflash.h")
    linker = read("STM32H743XX_FLASH.ld")
    flight_log = read("App/Inc/app_flight_log.h")

    slot_count = _int_literal(service_header, "APP_FLASH_SERVICE_PARAM_SLOT_COUNT")
    sector_count = _int_literal(intflash, "DRV_INTFLASH_PARAM_SECTOR_COUNT")
    first_sector = _int_literal(intflash, "DRV_INTFLASH_PARAM_FIRST_SECTOR")
    base = _int_literal(intflash, "DRV_INTFLASH_PARAM_BASE")

    # 一个逻辑槽独占一个物理扇区。片内擦除粒度就是 128 KB，共用一个扇区会让
    # 擦 A 把 B 一起抹掉，双槽掉电保护失效；映射不到扇区则直接写不进去。
    assert slot_count == sector_count, "逻辑槽与物理扇区必须一一对应"
    assert "DRV_INTFLASH_SECTOR_SIZE      (128UL * 1024UL)" in intflash

    sector_size = 128 * 1024
    bank2_end = 0x08100000 + 8 * sector_size
    assert base == 0x08100000 + first_sector * sector_size
    assert base + sector_count * sector_size == bank2_end, (
        "参数区必须顶到 Bank2 末尾，否则中间留出的扇区无人认领"
    )

    # 代码段必须正好让出这些扇区：2048K - sector_count*128K。
    code_k = 2048 - sector_count * 128
    assert f"LENGTH = {code_k}K" in linker
    assert "LENGTH = 2048K" not in linker

    # 五个逻辑槽全部具名，没有"留作缓冲"的无主扇区——上一轮的缺陷正是出在那里。
    for name in ("APP_FLASH_SERVICE_SCRATCH_OFFSET",
                 "APP_FLASH_SERVICE_PARAM_SLOT_A_OFFSET",
                 "APP_FLASH_SERVICE_PARAM_SLOT_B_OFFSET",
                 "APP_FLASH_SERVICE_CFG_SLOT_A_OFFSET",
                 "APP_FLASH_SERVICE_CFG_SLOT_B_OFFSET"):
        assert name in service_header, name
    named = len(re.findall(r"APP_FLASH_SERVICE_PARAM_SLOT\((\d)U\)", service_header))
    assert named == slot_count, "每个逻辑槽都必须有名字，不许留无主扇区"

    # 日志区上界必须正好落在参数区起点。头文件里写的是字面量（它要能在宿主上
    # 单独编译，不能拖进 HAL 依赖），一致性由 app_flight_log.c 的编译期断言保证。
    log_end = _int_literal(flight_log, "APP_FLIGHT_LOG_REGION_END_EXCL")
    nor_size = 4 * 1024 * 1024
    nor_sector = 4 * 1024
    assert log_end == nor_size - slot_count * nor_sector
    assert "_Static_assert(APP_FLIGHT_LOG_REGION_END_EXCL ==" in read(
        "App/Src/app_flight_log.c"
    )


def test_destructive_scratch_test_cannot_touch_real_data() -> None:
    """`FLASH SCRATCH TEST` 是破坏性的，必须落在自己的扇区上。

    它跟参数槽共用扇区的话，就是一个"跑一次诊断把标定/机体模型擦了"的陷阱——
    而且擦完还会显示成功，因为它测的就是"擦得掉、写得进"。
    """
    control = read("App/Src/app_control.c")

    assert ("#define APP_CONTROL_FLASH_SCRATCH_ADDR APP_FLASH_SERVICE_SCRATCH_OFFSET"
            in control)
    # 不许再出现自己算地址的写法（那正是它撞上参数槽的方式）。
    assert "APP_FLASH_SERVICE_SIZE_BYTES - 4U * 4096UL" not in control


def test_legacy_bsp_chip_driver_removed() -> None:
    assert not (ROOT / "BSP/Inc/bsp_gd25q32.h").exists()
    assert not (ROOT / "BSP/Src/bsp_gd25q32.c").exists()

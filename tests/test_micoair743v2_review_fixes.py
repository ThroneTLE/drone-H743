"""2026-09-10 软件审计发现的四个 P1 + 两个接线错位，逐条钉死。

每一条都属于**编译能过、测试全绿、要通电才发现**的类型，而且原来那 1404 条测试
一条都没覆盖到——因为它们要么测纯函数，要么读源码正则，都碰不到带 HAL 调用的控制流。
所以这里换一种做法：用 host gcc 把**仓库里真正参与构建的 .c 文件**连同一套假 HAL
编起来直接跑，看它到底调了谁。替身定义见 tests/_micoair_hostfakes.py。

被钉住的六条：
    1. 气压计 I2C 改到了不参与构建的 bsp_spl06.c，真正在跑的 drv_baro.c 还是纯 SPI，
       拿着空 hspi/空 CS 口调 HAL。
    2. SD 轮询读之后失效 D-Cache，把 CPU 刚写进 cache 的数据丢掉，还返回 OK。
    3. 存储路由重构时丢了互斥锁，SD 后端的全局块缓冲会在任务间串数据。
    4. 主 IMU 初始化失败后不再尝试备用 IMU，外层重试又选回同一颗，无限循环。
    5. ELRS 的 RX 上拉还配在老板子的 PD0 上，新板 UART4_RX 是 PA1。
    6. GPS 回调写死 USART2，而新板 GPS 在 USART3、USART2 成了光流口。
"""

from __future__ import annotations

import re
from pathlib import Path

from _micoair_hostfakes import CHECK_MACRO, ROOT, build_and_run, write_fakes


DRIVER_INC = ROOT / "Driver" / "Inc"
SERVICES_INC = ROOT / "Services" / "Inc"
BSP_INC = ROOT / "BSP" / "Inc"
APP_INC = ROOT / "App" / "Inc"


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


def strip_c_comments(text: str) -> str:
    """只看代码，不看注释——本仓库要求把"为什么"写在注释里，别让它触发正则。"""
    out: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
        elif text.startswith("//", i):
            end = text.find("\n", i)
            i = n if end < 0 else end
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


# ===================================================================== P1-1

BARO_HARNESS = (
    CHECK_MACRO
    + r"""
#include "drv_baro.h"

/* ---- 记录被测代码到底碰了什么 ---- */

static int spi_calls;
static int gpio_writes;
static int i2c_reads;
static int i2c_writes;
static uint16_t last_i2c_addr;
static uint8_t last_i2c_reg;
static uint8_t i2c_regs[256];

void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void) { return 0U; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int state)
{
    (void)pin; (void)state;
    /* 空指针写在真机上是 HardFault；这里记下来当作失败证据。 */
    CHECK(port != NULL, 100);
    gpio_writes++;
}

int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return 1;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    (void)data; (void)size; (void)timeout;
    CHECK(hspi != NULL, 101);
    spi_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data,
                                  uint16_t size, uint32_t timeout)
{
    (void)timeout;
    CHECK(hspi != NULL, 102);
    spi_calls++;
    if ((data != NULL) && (size > 0U)) { data[0] = DRV_BARO_ID_VALUE; }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx,
                                          uint8_t *rx, uint16_t size,
                                          uint32_t timeout)
{
    (void)tx; (void)timeout;
    CHECK(hspi != NULL, 103);
    spi_calls++;
    if ((rx != NULL) && (size >= 2U)) { rx[1] = DRV_BARO_ID_VALUE; }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                   uint16_t reg, uint16_t reg_size,
                                   uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint16_t i;

    (void)timeout;
    CHECK(hi2c != NULL, 104);
    CHECK(reg_size == I2C_MEMADD_SIZE_8BIT, 105);
    i2c_reads++;
    last_i2c_addr = addr;
    last_i2c_reg = (uint8_t)reg;
    for (i = 0U; i < len; i++) {
        data[i] = i2c_regs[(uint8_t)(reg + i)];
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                    uint16_t reg, uint16_t reg_size,
                                    uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)timeout;
    CHECK(hi2c != NULL, 106);
    CHECK(reg_size == I2C_MEMADD_SIZE_8BIT, 107);
    CHECK(len == 1U, 108);
    i2c_writes++;
    last_i2c_addr = addr;
    last_i2c_reg = (uint8_t)reg;
    i2c_regs[(uint8_t)reg] = data[0];
    return HAL_OK;
}

/* ---- 用例 ---- */

static I2C_HandleTypeDef fake_i2c;
static SPI_HandleTypeDef fake_spi;
static GPIO_TypeDef      fake_gpio;

static void check_i2c_path(void)
{
    DRV_BARO_Bus bus;
    DRV_BARO_Device dev;
    uint8_t id = 0U;
    uint8_t value = 0U;

    memset(&bus, 0, sizeof(bus));
    memset(&dev, 0, sizeof(dev));
    memset(i2c_regs, 0, sizeof(i2c_regs));
    i2c_regs[0x0DU] = DRV_BARO_ID_VALUE;      /* PROD_ID */
    spi_calls = 0; gpio_writes = 0; i2c_reads = 0; i2c_writes = 0;

    /* 与 bsp_board.c 里 MicoAir 的绑定完全一致：只有 I2C，没有 SPI、没有片选。 */
    bus.hi2c = &fake_i2c;
    bus.i2c_address = 0x77U;
    bus.timeout_ms = 100U;

    CHECK(DRV_BARO_Init(&dev, &bus) == DRV_BARO_OK, 1);

    /* 整条初始化里一次 SPI、一次 GPIO 都不该碰——碰了就是空指针。 */
    CHECK(spi_calls == 0, 2);
    CHECK(gpio_writes == 0, 3);

    /* 7 位地址 0x77 上总线要左移一位。 */
    CHECK(last_i2c_addr == (uint16_t)(0x77U << 1), 4);

    /* 四个配置寄存器都写到了，且是通过 I2C 写的。 */
    CHECK(i2c_writes == 4, 5);
    CHECK(i2c_regs[0x06U] != 0U, 6);   /* PRS_CFG */
    CHECK(i2c_regs[0x07U] != 0U, 7);   /* TMP_CFG */
    CHECK(i2c_regs[0x08U] == 0x07U, 8); /* MEAS_CFG 背景连续测量 */

    /* 周期读取也必须走 I2C。 */
    i2c_reads = 0; spi_calls = 0;
    CHECK(DRV_BARO_ReadRegister(&dev, 0x00U, &value) == DRV_BARO_OK, 9);
    CHECK(i2c_reads == 1, 10);
    CHECK(spi_calls == 0, 11);

    /* 诊断用的 TxRx 入口在 I2C 上退回普通读，不去碰空 hspi。 */
    CHECK(DRV_BARO_ReadIdTxRx(&dev, &id) == DRV_BARO_OK, 12);
    CHECK(id == DRV_BARO_ID_VALUE, 13);
    CHECK(spi_calls == 0, 14);
}

static void check_spi_path_still_works(void)
{
    DRV_BARO_Bus bus;
    DRV_BARO_Device dev;

    memset(&bus, 0, sizeof(bus));
    memset(&dev, 0, sizeof(dev));
    spi_calls = 0; gpio_writes = 0; i2c_reads = 0; i2c_writes = 0;

    /* 老板子仍然是 SPI，加 I2C 分支不能把它弄坏。 */
    bus.hspi = &fake_spi;
    bus.cs_port = &fake_gpio;
    bus.cs_pin = 1U;
    bus.timeout_ms = 100U;

    CHECK(DRV_BARO_Init(&dev, &bus) == DRV_BARO_OK, 20);
    CHECK(spi_calls > 0, 21);
    CHECK(gpio_writes > 0, 22);
    CHECK(i2c_reads == 0, 23);
    CHECK(i2c_writes == 0, 24);
}

static void check_bad_bindings_are_rejected(void)
{
    DRV_BARO_Bus bus;
    DRV_BARO_Device dev;
    uint8_t value = 0U;

    /* 两条总线都没配：必须如实拒绝，不能把空指针交给 HAL。 */
    memset(&bus, 0, sizeof(bus));
    memset(&dev, 0, sizeof(dev));
    spi_calls = 0; gpio_writes = 0;
    CHECK(DRV_BARO_Init(&dev, &bus) == DRV_BARO_INVALID_ARG, 30);
    CHECK(spi_calls == 0, 31);
    CHECK(gpio_writes == 0, 32);

    /* I2C 句柄有、地址没填，同样拒绝。 */
    bus.hi2c = &fake_i2c;
    bus.i2c_address = 0U;
    CHECK(DRV_BARO_Init(&dev, &bus) == DRV_BARO_INVALID_ARG, 33);

    /*
     * 关键一条：周期读取不经过 Init 的校验。设备结构体里两条总线都是空的时候，
     * 读寄存器必须返回 INVALID_ARG，而不是拿空 hspi/空 cs_port 去调 HAL。
     * 审计里报的"32 拍之后的周期读取把空指针传给 HAL"说的就是这里。
     */
    memset(&dev, 0, sizeof(dev));
    spi_calls = 0; gpio_writes = 0;
    CHECK(DRV_BARO_ReadRegister(&dev, 0x00U, &value) == DRV_BARO_INVALID_ARG, 34);
    CHECK(DRV_BARO_WriteRegister(&dev, 0x06U, 0x01U) == DRV_BARO_INVALID_ARG, 35);
    CHECK(DRV_BARO_ReadIdTxRx(&dev, &value) == DRV_BARO_INVALID_ARG, 36);
    CHECK(spi_calls == 0, 37);
    CHECK(gpio_writes == 0, 38);
}

int main(void)
{
    check_i2c_path();
    check_spi_path_still_works();
    check_bad_bindings_are_rejected();
    REPORT();
}
"""
)


def test_baro_i2c_path_runs_in_the_driver_that_is_actually_built(tmp_path: Path) -> None:
    """气压计的 I2C 通路必须在 Driver/Src/drv_baro.c 里，因为那才是被编译的那个。

    原来 I2C 分支写在 BSP/Src/bsp_spl06.c —— 那个文件不在 CMakeLists.txt 里，
    Sensor_Task 也不调它。于是新板上气压计必然初始化失败，而且周期读取会拿着
    绑定里被置空的 SPI 句柄和片选口去调 HAL。
    """
    fakes = write_fakes(tmp_path)
    result = build_and_run(
        tmp_path,
        "baro_i2c",
        BARO_HARNESS,
        sources=[ROOT / "Driver" / "Src" / "drv_baro.c"],
        includes=[fakes, DRIVER_INC],
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_dead_spl06_module_is_gone() -> None:
    """那个害人的死文件必须删掉，否则下次还会有人改错地方。

    它不在 CMakeLists.txt 里、头文件也只被它自己引用，看起来却和真驱动一模一样。
    """
    assert not (ROOT / "BSP" / "Src" / "bsp_spl06.c").exists()
    assert not (ROOT / "BSP" / "Inc" / "bsp_spl06.h").exists()

    # 上层用的 BSP_SPL06_* 别名来自 bsp_baro.h，删文件不能把它们带走。
    baro_h = read("BSP/Inc/bsp_baro.h")
    assert "BSP_SPL06_ID_VALUE" in baro_h
    assert "typedef DRV_BARO_Device BSP_SPL06_Device;" in baro_h


def test_baro_diagnostics_do_not_invent_levels() -> None:
    """I2C 上没有片选和 MISO，诊断要说"不适用"，不能报一个看着合理的 0。

    顺带钉住引脚不再写死：老代码用的是老板子的 CubeMX 标签 Press_cs_*，
    那个标签在新 .ioc 里根本不生成。
    """
    bsp = strip_c_comments(read("BSP/Src/bsp_baro.c"))

    assert "Press_cs" not in bsp, "还在引用老板子的 CubeMX 引脚标签"
    assert "BSP_BARO_LEVEL_NOT_APPLICABLE" in bsp
    assert "GPIOE" not in bsp, "MISO 引脚应由板级绑定给出，不写死在 BSP 里"

    # 电平只可能是 0 或 1，哨兵必须落在这两个值之外才不会被误读成真实读数。
    header = read("BSP/Inc/bsp_baro.h")
    match = re.search(r"#define\s+BSP_BARO_LEVEL_NOT_APPLICABLE\s+(\w+)", header)
    assert match is not None
    assert int(match.group(1).rstrip("Uu"), 0) not in (0, 1)


# ===================================================================== P1-2

SDBLOCK_HARNESS = (
    CHECK_MACRO
    + r"""
#include "drv_sdblock.h"

#define CARD_BLOCKS 4096U

static uint8_t card[CARD_BLOCKS * DRV_SDBLOCK_BLOCK_SIZE];
static SD_HandleTypeDef fake_sd;

void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void) { return 0U; }

uint32_t HAL_SD_GetCardState(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    return HAL_SD_CARD_TRANSFER;
}

HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *hsd,
                                     HAL_SD_CardInfoTypeDef *info)
{
    (void)hsd;
    info->LogBlockNbr = CARD_BLOCKS;
    info->LogBlockSize = DRV_SDBLOCK_BLOCK_SIZE;
    return HAL_OK;
}

/*
 * 按 ST 官方 H7 HAL 的阻塞实现建模：数据由 **CPU** 从 FIFO 逐字节写进 pData，
 * 不是 DMA 写内存。所以这里就是一次普通 memcpy。
 */
HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint8_t *data,
                                    uint32_t block, uint32_t count,
                                    uint32_t timeout)
{
    (void)hsd; (void)timeout;
    CHECK(block >= DRV_SDBLOCK_BASE_BLOCK, 200);   /* 永远不碰第 0 块（MBR） */
    CHECK(block + count <= CARD_BLOCKS, 201);
    memcpy(data, &card[block * DRV_SDBLOCK_BLOCK_SIZE],
           count * DRV_SDBLOCK_BLOCK_SIZE);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint8_t *data,
                                     uint32_t block, uint32_t count,
                                     uint32_t timeout)
{
    (void)hsd; (void)timeout;
    CHECK(block >= DRV_SDBLOCK_BASE_BLOCK, 202);
    CHECK(block + count <= CARD_BLOCKS, 203);
    memcpy(&card[block * DRV_SDBLOCK_BLOCK_SIZE], data,
           count * DRV_SDBLOCK_BLOCK_SIZE);
    return HAL_OK;
}

int main(void)
{
    DRV_SDBLOCK_Bus bus;
    uint8_t out[8];
    uint8_t payload[4] = {0xA5U, 0x5AU, 0x11U, 0x22U};
    uint32_t i;

    memset(card, 0x00, sizeof(card));
    memset(&bus, 0, sizeof(bus));
    bus.hsd = &fake_sd;
    bus.timeout_ms = 1000U;

    CHECK(DRV_SDBLOCK_Init(&bus) == DRV_SDBLOCK_OK, 1);
    CHECK(DRV_SDBLOCK_IsReady() == 1U, 2);

    /*
     * 数据通路回归：写进去什么，读回来就得是什么（含读-改-写与跨块）。
     *
     * 说清楚这条测的**不是**缓存缺陷本身：宿主上没有 D-Cache，模拟不了"invalidate
     * 把脏行丢掉"。缓存那条由 test_sd_polling_path_has_no_cache_maintenance 从源码
     * 侧钉死——修复的形式就是"这条路径上不许出现任何缓存维护，连挂钩子的字段都删掉"。
     * 两条合起来才完整：一条保证搬运逻辑对，一条保证没人再把缓存维护加回来。
     */
    CHECK(DRV_SDBLOCK_Write(0U, payload, sizeof(payload)) == DRV_SDBLOCK_OK, 3);
    memset(out, 0, sizeof(out));
    CHECK(DRV_SDBLOCK_Read(0U, out, sizeof(payload)) == DRV_SDBLOCK_OK, 4);
    CHECK(memcmp(out, payload, sizeof(payload)) == 0, 5);

    /* 不足整块的写是读-改-写，不能把同块内的邻居抹掉。 */
    {
        uint8_t neighbour = 0x77U;
        uint8_t patch = 0xEEU;

        CHECK(DRV_SDBLOCK_Write(100U, &neighbour, 1U) == DRV_SDBLOCK_OK, 6);
        CHECK(DRV_SDBLOCK_Write(101U, &patch, 1U) == DRV_SDBLOCK_OK, 7);

        memset(out, 0, sizeof(out));
        CHECK(DRV_SDBLOCK_Read(100U, out, 2U) == DRV_SDBLOCK_OK, 8);
        CHECK(out[0] == 0x77U, 9);
        CHECK(out[1] == 0xEEU, 10);
    }

    /* 跨块读写也要对得上。 */
    {
        static uint8_t big_in[1000];
        static uint8_t big_out[1000];

        for (i = 0U; i < sizeof(big_in); i++) { big_in[i] = (uint8_t)(i * 7U); }
        CHECK(DRV_SDBLOCK_Write(500U, big_in, sizeof(big_in)) == DRV_SDBLOCK_OK, 11);
        memset(big_out, 0, sizeof(big_out));
        CHECK(DRV_SDBLOCK_Read(500U, big_out, sizeof(big_out)) == DRV_SDBLOCK_OK, 12);
        CHECK(memcmp(big_in, big_out, sizeof(big_in)) == 0, 13);
    }

    /* NOR 语义的擦除 = 全写 0xFF，上层日志格式只依赖这一条。 */
    CHECK(DRV_SDBLOCK_Erase(0U, 16U) == DRV_SDBLOCK_OK, 14);
    memset(out, 0, sizeof(out));
    CHECK(DRV_SDBLOCK_Read(0U, out, sizeof(out)) == DRV_SDBLOCK_OK, 15);
    for (i = 0U; i < sizeof(out); i++) { CHECK(out[i] == 0xFFU, 16); }

    /* 第 0 块（MBR）必须原封不动。 */
    for (i = 0U; i < DRV_SDBLOCK_BLOCK_SIZE; i++) { CHECK(card[i] == 0x00U, 17); }

    REPORT();
}
"""
)


def test_sd_polling_reads_return_what_was_written(tmp_path: Path) -> None:
    """SD 读回来的必须是刚写进去的数据。

    H7 的阻塞版 HAL_SD_ReadBlocks 是 CPU 轮询 FIFO 搬运，数据经过 D-Cache。
    读完再 invalidate 会把 CPU 刚写进 cache 的脏行丢掉，于是读到旧内容、
    状态码还是 OK —— 日志回放会安静地读出垃圾。
    """
    fakes = write_fakes(tmp_path)
    result = build_and_run(
        tmp_path,
        "sdblock",
        SDBLOCK_HARNESS,
        sources=[ROOT / "Driver" / "Src" / "drv_sdblock.c"],
        includes=[fakes, DRIVER_INC],
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_sd_polling_path_has_no_cache_maintenance() -> None:
    """轮询传输不该出现任何缓存维护，连挂回调的字段都不该留着。

    留着字段等于留一个陷阱：下一个人看到 cache_invalidate 就会以为该调，
    而在 CPU 轮询路径上调一次就丢一次数据。
    """
    driver = strip_c_comments(read("Driver/Src/drv_sdblock.c"))
    header = strip_c_comments(read("Driver/Inc/drv_sdblock.h"))
    board = strip_c_comments(read("BSP/Src/bsp_board.c"))

    for text, where in ((driver, "drv_sdblock.c"), (header, "drv_sdblock.h")):
        assert "cache_invalidate" not in text, f"{where} 仍有缓存失效钩子"
        assert "cache_clean" not in text, f"{where} 仍有缓存清刷钩子"

    assert not re.search(r"sdblock_bus\.cache", board)

    # 反过来，片内 Flash 那条**必须**保留失效：那里是真的擦写后要看新内容。
    assert "intflash_bus.cache_invalidate" in board


# ===================================================================== P1-3

FLASH_LOCK_HARNESS = (
    CHECK_MACRO
    + r"""
#include "app_flash_service.h"
#include "drv_intflash.h"
#include "drv_sdblock.h"

/* ---- 锁深度记账 ---- */

static int lock_depth;
static int max_depth;
static int backend_calls;
static int calls_without_lock;

static void note_backend_call(void)
{
    backend_calls++;
    if (lock_depth != 1) { calls_without_lock++; }
}

DRV_GD25Q32_Status BSP_FlashBus_Acquire(uint32_t timeout_ms)
{
    (void)timeout_ms;
    lock_depth++;
    if (lock_depth > max_depth) { max_depth = lock_depth; }
    return DRV_GD25Q32_OK;
}

void BSP_FlashBus_Release(void) { lock_depth--; }

/* ---- 后端替身：只记录是否在持锁状态下被调到 ---- */

static DRV_INTFLASH_Bus fake_intflash_bus;
static DRV_SDBLOCK_Bus  fake_sdblock_bus;
static DRV_GD25Q32_Bus  fake_nor_bus;

void DRV_INTFLASH_SetBus(const DRV_INTFLASH_Bus *bus) { (void)bus; }

DRV_INTFLASH_Status DRV_INTFLASH_EraseSector(uint32_t address)
{
    (void)address; note_backend_call(); return DRV_INTFLASH_OK;
}

DRV_INTFLASH_Status DRV_INTFLASH_Write(uint32_t address, const uint8_t *data,
                                       uint32_t length)
{
    (void)address; (void)data; (void)length;
    note_backend_call();
    return DRV_INTFLASH_OK;
}

DRV_INTFLASH_Status DRV_INTFLASH_Read(uint32_t address, uint8_t *data,
                                      uint32_t length)
{
    (void)address;
    note_backend_call();
    memset(data, 0xA5, length);
    return DRV_INTFLASH_OK;
}

uint8_t DRV_INTFLASH_IsParamAddress(uint32_t address) { (void)address; return 1U; }

DRV_SDBLOCK_Status DRV_SDBLOCK_Init(const DRV_SDBLOCK_Bus *bus)
{
    (void)bus; return DRV_SDBLOCK_OK;
}

uint8_t DRV_SDBLOCK_IsReady(void) { return 1U; }
uint64_t DRV_SDBLOCK_GetUsableBytes(void) { return 0U; }

DRV_SDBLOCK_Status DRV_SDBLOCK_Read(uint32_t offset, uint8_t *data, uint32_t length)
{
    (void)offset;
    note_backend_call();
    memset(data, 0x5A, length);
    return DRV_SDBLOCK_OK;
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Write(uint32_t offset, const uint8_t *data,
                                     uint32_t length)
{
    (void)offset; (void)data; (void)length;
    note_backend_call();
    return DRV_SDBLOCK_OK;
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Erase(uint32_t offset, uint32_t length)
{
    (void)offset; (void)length; note_backend_call(); return DRV_SDBLOCK_OK;
}

/* ---- 其余外部依赖的最小替身 ---- */

const DRV_INTFLASH_Bus *BSP_Board_GetIntFlashBus(void) { return &fake_intflash_bus; }
const DRV_SDBLOCK_Bus  *BSP_Board_GetSdBlockBus(void)  { return &fake_sdblock_bus; }
const DRV_GD25Q32_Bus  *BSP_FlashBus_GetBus(void)      { return &fake_nor_bus; }
void BSP_FlashBus_RegisterDmaDevice(DRV_GD25Q32_Device *dev) { (void)dev; }
void BSP_FlashBus_InvalidateBinding(void) { }

DRV_GD25Q32_Status DRV_GD25Q32_Init(DRV_GD25Q32_Device *dev,
                                    const DRV_GD25Q32_Bus *bus)
{
    (void)dev; (void)bus; note_backend_call(); return DRV_GD25Q32_OK;
}

/*
 * NOR 专有接口在 MicoAir 板上注定失败（板上没这颗芯片），本用例不关心它们的返回值，
 * 只是链接需要。它们同样走 note_backend_call，顺便钉住"诊断入口也在锁内"。
 */
DRV_GD25Q32_Status DRV_GD25Q32_ReleaseFromPowerDown(DRV_GD25Q32_Device *dev)
{ (void)dev; note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_ReadJedecId(DRV_GD25Q32_Device *dev,
                                           DRV_GD25Q32_JedecId *id)
{ (void)dev; memset(id, 0, sizeof(*id)); note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_ReadStatus1(DRV_GD25Q32_Device *dev, uint8_t *v)
{ (void)dev; *v = 0U; note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_ReadStatus2(DRV_GD25Q32_Device *dev, uint8_t *v)
{ (void)dev; *v = 0U; note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_ReadStatus3(DRV_GD25Q32_Device *dev, uint8_t *v)
{ (void)dev; *v = 0U; note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_WriteEnableProbe(DRV_GD25Q32_Device *dev,
                                                uint8_t *before, uint8_t *after)
{ (void)dev; *before = 0U; *after = 0U; note_backend_call(); return DRV_GD25Q32_OK; }

DRV_GD25Q32_Status DRV_GD25Q32_ClearProtection(DRV_GD25Q32_Device *dev,
                                               uint8_t *s1b, uint8_t *s2b,
                                               uint8_t *s1a, uint8_t *s2a)
{
    (void)dev; *s1b = 0U; *s2b = 0U; *s1a = 0U; *s2a = 0U;
    note_backend_call();
    return DRV_GD25Q32_OK;
}

int main(void)
{
    uint8_t buffer[64];
    uint32_t log_addr = 0x1000U;
    uint32_t param_addr = APP_FLASH_SERVICE_PARAM_SLOT_A_OFFSET;

    memset(buffer, 0, sizeof(buffer));

    /* 两条路由都要覆盖：日志走 SD，参数走片内 Flash。 */
    CHECK(APP_FlashService_BackendFor(log_addr) == APP_FLASH_BACKEND_SDBLOCK, 1);
    CHECK(APP_FlashService_BackendFor(param_addr) == APP_FLASH_BACKEND_INTERNAL, 2);

    backend_calls = 0; calls_without_lock = 0; max_depth = 0; lock_depth = 0;

    CHECK(APP_FlashService_ReadData(log_addr, buffer, 16U) == DRV_GD25Q32_OK, 3);
    CHECK(APP_FlashService_ReadDataFast(log_addr, buffer, 16U) == DRV_GD25Q32_OK, 4);
    CHECK(APP_FlashService_WriteData(log_addr, buffer, 16U) == DRV_GD25Q32_OK, 5);
    CHECK(APP_FlashService_PageProgram(log_addr, buffer, 16U) == DRV_GD25Q32_OK, 6);
    CHECK(APP_FlashService_EraseSector(log_addr) == DRV_GD25Q32_OK, 7);
    CHECK(APP_FlashService_EraseBlock32K(log_addr) == DRV_GD25Q32_OK, 8);
    CHECK(APP_FlashService_EraseBlock64K(log_addr) == DRV_GD25Q32_OK, 9);

    CHECK(APP_FlashService_ReadData(param_addr, buffer, 16U) == DRV_GD25Q32_OK, 10);
    CHECK(APP_FlashService_WriteData(param_addr, buffer, 16U) == DRV_GD25Q32_OK, 11);
    CHECK(APP_FlashService_EraseSector(param_addr) == DRV_GD25Q32_OK, 12);

    /* 每一次后端调用都必须发生在持锁状态下。 */
    CHECK(backend_calls == 10, 13);
    CHECK(calls_without_lock == 0, 14);

    /*
     * 而且**不能嵌套加锁**：flashBusMutex 是非递归的（Core/Src/freertos.c 创建时
     * 只给了 .name，没有 osMutexRecursive）。PageProgram 复用写逻辑、ReadDataFast
     * 复用读逻辑，如果它们去调公开入口，同一个任务就会二次获取同一把锁，直接卡死。
     */
    CHECK(max_depth == 1, 15);

    /* 每次调用结束都要还锁，不能漏。 */
    CHECK(lock_depth == 0, 16);

    /* 参数区不接受块擦除（会跨槽边界），而且拒绝时也不能把锁留着。 */
    CHECK(APP_FlashService_EraseBlock32K(param_addr) == DRV_GD25Q32_INVALID_ARG, 17);
    CHECK(APP_FlashService_EraseBlock64K(param_addr) == DRV_GD25Q32_INVALID_ARG, 18);
    CHECK(lock_depth == 0, 19);

    /* 参数错误要在拿锁之后如实返回，同样不能漏还锁。 */
    CHECK(APP_FlashService_ReadData(log_addr, NULL, 16U) == DRV_GD25Q32_INVALID_ARG, 20);
    CHECK(APP_FlashService_WriteData(log_addr, buffer, 0U) == DRV_GD25Q32_INVALID_ARG, 21);
    CHECK(lock_depth == 0, 22);

    REPORT();
}
"""
)


def test_storage_entry_points_are_serialized(tmp_path: Path) -> None:
    """每一笔读/写/擦除都必须整笔持锁，且不许嵌套加锁。

    换后端之前这几个入口是打给 SPI NOR 的，每次都持 flashBusMutex；换成
    "片内 Flash + SD 裸块"之后锁被漏掉了。SD 后端比 NOR 更需要它——drv_sdblock
    内部有一个全局共享的 512 字节块缓冲，不足整块的写是读-改-写，
    后台日志任务与通信任务的 FLASH VERIFY 交错就能互相覆盖，而且两边都返回 OK。
    """
    fakes = write_fakes(tmp_path)
    result = build_and_run(
        tmp_path,
        "flash_lock",
        FLASH_LOCK_HARNESS,
        sources=[ROOT / "App" / "Src" / "app_flash_service.c"],
        includes=[fakes, DRIVER_INC, SERVICES_INC, BSP_INC, APP_INC],
    )
    assert result.returncode == 0, result.stdout + result.stderr


# ===================================================================== P1-4

IMU_FALLBACK_HARNESS = (
    CHECK_MACRO
    + r"""
#include "bsp_imu.h"
#include "drv_bmi088.h"
#include "drv_bmi270.h"
#include "drv_imu_iface.h"

/* ---- 故障注入：可以逐颗指定 probe / init 的结果 ---- */

typedef struct {
    DRV_IMU_Status probe_result;
    DRV_IMU_Status init_result;
    uint8_t        chip_id;
    int            probe_calls;
    int            init_calls;
} FakeChip;

static FakeChip bmi088 = {DRV_IMU_OK, DRV_IMU_OK, 0x1EU, 0, 0};
static FakeChip bmi270 = {DRV_IMU_OK, DRV_IMU_OK, 0x24U, 0, 0};
static FakeChip icm    = {DRV_IMU_BAD_ID, DRV_IMU_OK, 0x47U, 0, 0};

static DRV_IMU_Status fake_probe(FakeChip *chip, uint8_t *chip_id)
{
    chip->probe_calls++;
    if (chip_id != NULL) { *chip_id = chip->chip_id; }
    return chip->probe_result;
}

static DRV_IMU_Status fake_init(FakeChip *chip) { chip->init_calls++; return chip->init_result; }

static DRV_IMU_Status bmi088_probe(void *ctx, uint8_t *id) { (void)ctx; return fake_probe(&bmi088, id); }
static DRV_IMU_Status bmi088_init(void *ctx, const DRV_IMU_Config *c) { (void)ctx; (void)c; return fake_init(&bmi088); }
static DRV_IMU_Status bmi270_probe(void *ctx, uint8_t *id) { (void)ctx; return fake_probe(&bmi270, id); }
static DRV_IMU_Status bmi270_init(void *ctx, const DRV_IMU_Config *c) { (void)ctx; (void)c; return fake_init(&bmi270); }
static DRV_IMU_Status icm_probe(void *ctx, uint8_t *id) { (void)ctx; return fake_probe(&icm, id); }
static DRV_IMU_Status icm_init(void *ctx, const DRV_IMU_Config *c) { (void)ctx; (void)c; return fake_init(&icm); }

static DRV_IMU_Status stub_read_raw(void *ctx, DRV_IMU_RawData *raw)
{
    (void)ctx; memset(raw, 0, sizeof(*raw)); return DRV_IMU_OK;
}
static DRV_IMU_Status stub_read_scaled(void *ctx, DRV_IMU_ScaledData *s)
{
    (void)ctx; memset(s, 0, sizeof(*s)); return DRV_IMU_OK;
}
static DRV_IMU_Status stub_ready(void *ctx, bool *ready)
{
    (void)ctx; *ready = true; return DRV_IMU_OK;
}

static const DRV_IMU_Ops bmi088_ops = {
    DRV_IMU_CHIP_BMI088, "BMI088", bmi088_probe, bmi088_init,
    stub_read_raw, stub_read_scaled, stub_ready
};
static const DRV_IMU_Ops bmi270_ops = {
    DRV_IMU_CHIP_BMI270, "BMI270", bmi270_probe, bmi270_init,
    stub_read_raw, stub_read_scaled, stub_ready
};
static const DRV_IMU_Ops icm_ops = {
    DRV_IMU_CHIP_ICM42688, "ICM42688", icm_probe, icm_init,
    stub_read_raw, stub_read_scaled, stub_ready
};

const DRV_IMU_Ops *DRV_BMI088_GetOps(void) { return &bmi088_ops; }
const DRV_IMU_Ops *DRV_BMI270_GetOps(void) { return &bmi270_ops; }
const DRV_IMU_Ops *DRV_IMU_GetOps(void)    { return &icm_ops; }

/* ---- 其余依赖 ---- */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* bsp_imu.c 的背靠背读取自检要用到的片选端口与 SPI2 寄存器块，见 _micoair_hostfakes。 */
GPIO_TypeDef fake_gpio_d;
GPIO_TypeDef fake_gpio_a;
SPI_RegDef   fake_spi2_regs;
EXTI_Core_TypeDef fake_exti_d1;
/* 引脚快照（REQ mod=IMUSEL op=BUS）按端口取值，四个端口都要有实体。 */
GPIO_TypeDef fake_gpio_port_a;
GPIO_TypeDef fake_gpio_port_b;
GPIO_TypeDef fake_gpio_port_c;
GPIO_TypeDef fake_gpio_port_d;
SYSCFG_RegDef fake_syscfg_regs;

/* 只为让链接通过：本用例考的是选型回退，两颗 IMU 的总线行为由各自的 ops 桩决定。 */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                          uint8_t *rx, uint16_t n, uint32_t t)
{ (void)h; (void)tx; (void)rx; (void)n; (void)t; return HAL_OK; }

void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void) { return 0U; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, int s) { (void)p; (void)n; (void)s; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n) { (void)p; (void)n; return 1; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }

static DRV_BMI088_Bus bmi088_bus;
static DRV_BMI270_Bus bmi270_bus;
static DRV_IMU_Bus    icm_bus;

const DRV_BMI088_Bus *BSP_Board_GetBmi088Bus(void) { return &bmi088_bus; }
const DRV_BMI270_Bus *BSP_Board_GetBmi270Bus(void) { return &bmi270_bus; }
const DRV_IMU_Bus    *BSP_Board_GetImuBus(void)    { return &icm_bus; }

void DRV_IMU_DefaultConfig(DRV_IMU_Config *config) { memset(config, 0, sizeof(*config)); }
void DRV_IMU_ConvertRaw(const DRV_IMU_Device *d, const DRV_IMU_RawData *r,
                        DRV_IMU_ScaledData *s)
{ (void)d; (void)r; memset(s, 0, sizeof(*s)); }
void DRV_BMI088_ConvertRaw(DRV_IMU_AccelRange a, DRV_IMU_GyroRange g,
                           const DRV_IMU_RawData *r, DRV_IMU_ScaledData *s)
{ (void)a; (void)g; (void)r; memset(s, 0, sizeof(*s)); }
void DRV_BMI270_ConvertRaw(DRV_IMU_AccelRange a, DRV_IMU_GyroRange g,
                           const DRV_IMU_RawData *r, DRV_IMU_ScaledData *s)
{ (void)a; (void)g; (void)r; memset(s, 0, sizeof(*s)); }

static void reset_counters(void)
{
    bmi088.probe_calls = 0; bmi088.init_calls = 0;
    bmi270.probe_calls = 0; bmi270.init_calls = 0;
    icm.probe_calls = 0;    icm.init_calls = 0;
    BSP_IMU_Invalidate();
}

int main(void)
{
    SVC_IMU_Selection sel;
    BSP_IMU_Info info;

    /* --- 正常情况：主 IMU 就绪，它上岗，第二颗不必初始化 --- */
    reset_counters();
    bmi088.probe_result = DRV_IMU_OK;  bmi088.init_result = DRV_IMU_OK;
    bmi270.probe_result = DRV_IMU_OK;  bmi270.init_result = DRV_IMU_OK;
    CHECK(BSP_IMU_Init() == DRV_IMU_OK, 1);
    CHECK(BSP_IMU_GetChipKind() == DRV_IMU_CHIP_BMI088, 2);
    CHECK(bmi088.init_calls == 1, 3);
    CHECK(bmi270.init_calls == 0, 4);

    /*
     * DRDY 只放行上岗那颗。修之前回调用一个静态掩码同时收两颗的引脚，理由写的是
     * "另一颗没初始化就不会产生边沿"——冷启动成立，**软复位不成立**：DFU 跳转、
     * 看门狗复位都不给传感器掉电，上一轮配置过的那颗照旧按自己的 ODR 发边沿。
     * 2026-09-11 实测因此把节拍从 1000 Hz 顶到 1760 Hz，三成迭代读到重复的陀螺样本。
     */
    CHECK(BSP_IMU_GetDrdyPin() == BMI088_G_DRDY_Pin, 5);
    CHECK((fake_exti_d1.IMR1 & BMI088_G_DRDY_Pin) != 0U, 6);
    CHECK((fake_exti_d1.IMR1 & BMI270_DRDY_Pin) == 0U, 7);

    /*
     * --- 核心回归：主 IMU 探得到但配置失败 ---
     * 修之前这里会直接返回失败，采样任务外层重试又选回同一颗，无限循环，
     * 旁边那颗完好的 BMI270 一次都轮不到。
     */
    reset_counters();
    bmi088.probe_result = DRV_IMU_OK;  bmi088.init_result = DRV_IMU_ERROR;
    bmi270.probe_result = DRV_IMU_OK;  bmi270.init_result = DRV_IMU_OK;
    CHECK(BSP_IMU_Init() == DRV_IMU_OK, 10);
    CHECK(bmi088.init_calls == 1, 11);
    CHECK(bmi270.init_calls == 1, 12);
    CHECK(BSP_IMU_GetChipKind() == DRV_IMU_CHIP_BMI270, 13);

    /* 诊断必须和实际跑的芯片一致，不能还指着那颗失败的。 */
    BSP_IMU_GetInfo(&info);
    CHECK(info.kind == DRV_IMU_CHIP_BMI270, 14);

    /* 回退之后 DRDY 也必须跟着换过去，否则节拍源和数据源不是同一颗芯片。 */
    CHECK(BSP_IMU_GetDrdyPin() == BMI270_DRDY_Pin, 15);
    CHECK((fake_exti_d1.IMR1 & BMI270_DRDY_Pin) != 0U, 16);
    CHECK((fake_exti_d1.IMR1 & BMI088_G_DRDY_Pin) == 0U, 17);
    CHECK(info.chip_id == 0x24U, 15);
    CHECK(info.initialized == 1U, 16);

    /* 探测记账仍然把两颗都如实记下来，供诊断回报。 */
    BSP_IMU_GetSelection(&sel);
    CHECK(sel.selected == DRV_IMU_CHIP_BMI270, 17);
    CHECK(sel.probe_count == 3U, 18);
    CHECK(sel.probed_kind[0] == (uint8_t)DRV_IMU_CHIP_BMI088, 19);
    CHECK(sel.probed_status[0] == DRV_IMU_OK, 20);

    /* --- 两颗都配置失败：如实返回错误，且不留半吊子选中状态 --- */
    reset_counters();
    bmi088.probe_result = DRV_IMU_OK;  bmi088.init_result = DRV_IMU_ERROR;
    bmi270.probe_result = DRV_IMU_OK;  bmi270.init_result = DRV_IMU_TIMEOUT;
    CHECK(BSP_IMU_Init() != DRV_IMU_OK, 30);
    CHECK(bmi088.init_calls == 1, 31);
    CHECK(bmi270.init_calls == 1, 32);
    CHECK(BSP_IMU_GetChipKind() == DRV_IMU_CHIP_NONE, 33);
    CHECK(BSP_IMU_ReadRaw(NULL) == DRV_IMU_ERROR, 34);

    /* --- 一颗都探不到：返回 BAD_ID，一次 init 都不该发生 --- */
    reset_counters();
    bmi088.probe_result = DRV_IMU_BAD_ID;
    bmi270.probe_result = DRV_IMU_BAD_ID;
    CHECK(BSP_IMU_Init() == DRV_IMU_BAD_ID, 40);
    CHECK(bmi088.init_calls == 0, 41);
    CHECK(bmi270.init_calls == 0, 42);
    CHECK(icm.init_calls == 0, 43);

    /* --- 只有备用那颗在：它直接上岗 --- */
    reset_counters();
    bmi088.probe_result = DRV_IMU_BAD_ID;
    bmi270.probe_result = DRV_IMU_OK;  bmi270.init_result = DRV_IMU_OK;
    CHECK(BSP_IMU_Init() == DRV_IMU_OK, 50);
    CHECK(bmi088.init_calls == 0, 51);
    CHECK(BSP_IMU_GetChipKind() == DRV_IMU_CHIP_BMI270, 52);

    REPORT();
}
"""
)


def test_secondary_imu_takes_over_when_primary_init_fails(tmp_path: Path) -> None:
    """探到 ≠ 能用：主 IMU 配置失败必须让位给备用 IMU。

    BMI270 上电要刷 328 字节配置固件，BMI088 有一串带回读重试的寄存器写，
    两者都可能在 probe 通过之后失败。原来只初始化第一颗探到的芯片，失败就返回；
    Sensor_Task 外层重试又按同样顺序选回它，于是板上两颗 IMU 的冗余等于没有。
    """
    fakes = write_fakes(tmp_path)
    result = build_and_run(
        tmp_path,
        "imu_fallback",
        IMU_FALLBACK_HARNESS,
        sources=[
            ROOT / "BSP" / "Src" / "bsp_imu.c",
            ROOT / "Services" / "Src" / "svc_imu.c",
        ],
        includes=[fakes, DRIVER_INC, SERVICES_INC, BSP_INC],
    )
    assert result.returncode == 0, result.stdout + result.stderr


# ============================================================ 接线错位两条

def test_elrs_rx_bias_pin_matches_the_ioc() -> None:
    """RX 上拉必须配在 .ioc 给 ELRS 串口的 RX 分配的那个脚上。

    这里一度写着老板子的 PD0，移植时改成过 UART4 的 PA1，2026-09-10 又随 ELRS
    搬到板载 RC 口变成 USART6 的 PC7。把第二个脚也配成同一个 AF，等于让两个 GPIO
    驱动同一路外设输入——ST 参考手册要求一个 AF 输入只能由一个引脚提供，
    最坏情况是 RC 链路整条收不到，而且这种错误编译期完全看不出来。

    断言的是"两边一致"这个不变量，而且连**是哪个串口**都从代码里推，
    不写死引脚名也不写死外设名——这个位置已经搬过三次了。
    """
    ioc = read("drone-H743.ioc")
    # 2026-09-11：串口绑定与 RX 偏置整组搬到 BSP 的角色表（app_elrs.c 不再认识
    # 任何实例名）。断言的不变量没变——偏置配在哪个脚，必须与 .ioc 给该串口
    # 分配的 RX 脚一致。
    elrs = strip_c_comments(read("BSP/Src/bsp_uart_link.c"))

    handle = re.search(r"&huart(\d+), GPIO[A-K],", elrs)
    assert handle is not None, "找不到 BSP 角色表里 RC 那一行的串口绑定"
    index = handle.group(1)
    # huart1/2/3/6 对应 USARTx，huart4/5/7/8 对应 UARTx。
    peripheral = f"USART{index}" if index in {"1", "2", "3", "6"} else f"UART{index}"

    match = re.search(rf"^P([A-K])(\d+)\.Signal={peripheral}_RX$", ioc, re.MULTILINE)
    assert match is not None, f".ioc 里找不到 {peripheral}_RX 的引脚分配"
    port, pin = match.group(1), int(match.group(2))

    # BSP 角色表里 RC 那一行必须给出同一个端口和引脚。
    rc_row = re.search(r"static const BSP_UartLinkBinding rc = \{\s*([^}]*)\}", elrs)
    assert rc_row is not None, "找不到 BSP 角色表里的 RC 绑定行"
    row = rc_row.group(1)
    assert f"GPIO{port}" in row, (
        f".ioc 把 {peripheral}_RX 放在 P{port}{pin}，BSP 角色表配的却是别的端口"
    )
    assert f"GPIO_PIN_{pin}" in row, (
        f".ioc 把 {peripheral}_RX 放在 P{port}{pin}，BSP 角色表配的却是别的引脚"
    )

    # 该引脚在 .ioc 里不能同时被别的外设占用。
    others = re.findall(rf"^P{port}{pin}\.Signal=(.+)$", ioc, re.MULTILINE)
    assert others == [f"{peripheral}_RX"], others

    # 中断分发表也必须认同一个外设，否则 ELRS 收不到事件。
    # 2026-09-11：那张表在 BSP（见 test_hardware_decoupling）。
    events = strip_c_comments(read("BSP/Src/bsp_uart_events.c"))
    assert f"#define BSP_UART_EVENTS_RC_INSTANCE        {peripheral}" in events, (
        f"分发表还在认别的串口，而 ELRS 已经在 {peripheral} 上"
    )


def test_factory_rollback_images_are_present_and_intact() -> None:
    """出厂固件镜像必须在仓库里，而且字节没被动过。

    刷 0x08000000 会覆盖 PX4 的 bootloader，这两个文件是**回到出厂状态的唯一途径**。
    它们原本只存在于 .tmp/ —— 公共临时区，任何清理动作都可能抹掉，而"没了"这件事
    要等到真需要回滚的那一刻才会发现，那时已经晚了。所以纳入版本控制并在这里钉住：
    文件缺失或内容被换掉，测试当场就红。

    校验和同时抄在 doc/micoair743v2/README.md 的"烧录前基线"一节，两处必须一致。
    """
    import hashlib

    baseline = ROOT / "doc" / "micoair743v2" / "baseline"
    expected = {
        "MicoAir743v2-PX4-1.15.4-Bootloader+Firmware.bin": (
            2097152,
            "4f553745c2946eaf3ac68bf83626751df71ad710db6e1163e6831b5f545591d2",
        ),
        "MicoAir743v2_PX4-1.15.x_bootloader.bin": (
            41020,
            "6f97070a8dade37c151dd26a758c82b95b85a8811a6189901f154e93b4b05b7c",
        ),
    }

    for name, (size, digest) in expected.items():
        path = baseline / name
        assert path.is_file(), f"出厂还原镜像不见了：{path}，刷坏就回不去了"
        blob = path.read_bytes()
        assert len(blob) == size, f"{name} 大小不对：{len(blob)} != {size}"
        assert hashlib.sha256(blob).hexdigest() == digest, f"{name} 内容被换过"

    # 文档里的校验和必须和实际文件对得上，否则回滚时无从验证取到的是不是对的文件。
    readme = read("doc/micoair743v2/README.md")
    for _size, digest in expected.values():
        assert digest in readme, "README 的烧录前基线没记这个 sha256"

    # .gitignore 的 *.bin 默认会吞掉它们，白名单必须在。
    gitignore = read(".gitignore")
    assert "!doc/micoair743v2/baseline/*.bin" in gitignore, (
        "少了白名单，这两个镜像会被 *.bin 规则忽略，等于没入库"
    )


def test_gps_callbacks_follow_the_board_binding() -> None:
    """GPS 回调认哪个串口，必须从板级绑定里取，不能写死实例名。

    新板 GPS 挪到 USART3，而 USART2 成了光流口。回调里写死 USART2 的话，
    GPS 会去认光流的串口——这种"绑定改了、回调没跟着改"的错位编译期看不出来。
    """
    gps = strip_c_comments(read("BSP/Src/bsp_gps.c"))

    assert "BSP_Board_GetGpsBus()" in gps, "回调判据应当来自板级绑定"
    for hardcoded in ("USART1", "USART2", "USART3", "UART4", "UART5",
                      "USART6", "UART7", "UART8"):
        assert hardcoded not in gps, f"bsp_gps.c 仍写死了 {hardcoded}"

    # 板级绑定里 GPS 与光流必须是不同的串口，否则两个驱动会互相吃对方的数据。
    board = strip_c_comments(read("BSP/Src/bsp_board.c"))
    gps_uart = re.search(r"gps_bus\.huart\s*=\s*&(\w+);", board)
    flow_uart = re.search(r"optical_flow_bus\.huart\s*=\s*&(\w+);", board)
    assert gps_uart is not None and flow_uart is not None
    assert gps_uart.group(1) != flow_uart.group(1)

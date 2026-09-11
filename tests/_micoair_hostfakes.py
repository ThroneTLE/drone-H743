"""宿主侧编译真实固件源码用的最小 HAL 替身。

为什么要有这个文件：
    tests/ 里绝大多数契约测试要么是纯函数（drv_*_tables.c），要么是读源码正则。
    但 2026-09-10 那轮审计发现的四个缺陷都不在这两类里——它们藏在**带 HAL 调用的
    控制流**中：气压计走哪条总线、SD 读完之后动没动缓存、存储入口有没有持锁、
    主 IMU 配置失败之后还试不试第二颗。这些路径靠读源码看不出来，必须真跑一遍。

    所以这里提供一套假的 main.h / spi.h / bsp_board.h，把真实的 .c 文件用 host gcc
    编起来，再用替身记录"到底调了谁、参数是什么"。被测的是仓库里真正参与构建的源码，
    不是它的复制品——审计里那四条正是"改到了不参与构建的文件"这类错误。

    这些替身**只用于测试**，不进固件构建（CMakeLists.txt 不引用本目录）。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


FAKE_MAIN_H = r"""
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H

/* 替身 main.h：只提供被测源码真正用到的 HAL 类型与函数声明。 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef enum {
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct { int tag; } GPIO_TypeDef;
typedef struct { int tag; } I2C_HandleTypeDef;
typedef struct { int tag; } USART_TypeDef;

typedef struct {
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    /*
     * CFG2 的 AFCNTR。2026-09-11 实测：关着的时候 SPE=0 期间 SPI 交还引脚控制权，
     * SCK 不再被钉在 CPOL 电平上，下一笔事务开头就多出一个时钟沿——BMI088 因此
     * 只有第一笔读得到数据。bsp_imu.c 现在会强制打开它，所以假头必须有这个成员，
     * 否则这套 harness 编译的就不再是真实源码了。
     */
    uint32_t MasterKeepIOState;
} SPI_InitTypeDef;

#define SPI_MASTER_KEEP_IO_STATE_DISABLE 0x00000000U
#define SPI_MASTER_KEEP_IO_STATE_ENABLE  0x80000000U

typedef struct {
    int             tag;
    SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
    USART_TypeDef *Instance;
    uint32_t       ErrorCode;
} UART_HandleTypeDef;

#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1

#define SPI_POLARITY_LOW   0x00000000U
#define SPI_POLARITY_HIGH  0x00000002U
#define SPI_PHASE_1EDGE    0x00000000U
#define SPI_PHASE_2EDGE    0x00000001U

#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U

/*
 * BMI088 的两路片选与 SPI2 寄存器块。
 *
 * 这些本来只在 CubeMX 生成的 main.h / CMSIS 头里，但 bsp_imu.c 的背靠背读取自检
 * （BSP_IMU_DebugRawBmi088）要用到，而这套 harness 编译的是**真实的 bsp_imu.c**——
 * 换成桩文件就测不出真东西了。所以在假头里补出同名符号：
 * 片选给两个独立的 GPIO_TypeDef 实例（地址不同即可，本层不关心具体引脚），
 * SPI2 给一个可寻址的寄存器结构体，字段名与 CMSIS 对齐。
 */
extern GPIO_TypeDef fake_gpio_d;
#define BMI088_A_CS_GPIO_Port (&fake_gpio_d)
#define BMI088_A_CS_Pin       ((uint16_t)0x0010U)   /* PD4 */
#define BMI088_G_CS_GPIO_Port (&fake_gpio_d)
#define BMI088_G_CS_Pin       ((uint16_t)0x0020U)   /* PD5 */
extern GPIO_TypeDef fake_gpio_a;
#define BMI270_CS_GPIO_Port   (&fake_gpio_a)
#define BMI270_CS_Pin         ((uint16_t)0x8000U)   /* PA15 */

typedef struct {
    uint32_t CR1;
    uint32_t CR2;
    uint32_t CFG1;
    uint32_t CFG2;
    uint32_t IER;
    uint32_t SR;
} SPI_RegDef;

extern SPI_RegDef fake_spi2_regs;
#define SPI2 (&fake_spi2_regs)

void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int state);
int  HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data,
                                  uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx,
                                          uint8_t *rx, uint16_t size,
                                          uint32_t timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                   uint16_t reg, uint16_t reg_size,
                                   uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                    uint16_t reg, uint16_t reg_size,
                                    uint8_t *data, uint16_t len, uint32_t timeout);

/* ---- SDMMC：CubeMX 还没生成，这里给出与 ST HAL 一致的最小签名 ---- */

typedef struct { int tag; } SD_TypeDef;
typedef struct { SD_TypeDef *Instance; } SD_HandleTypeDef;

typedef struct {
    uint32_t BlockNbr;
    uint32_t BlockSize;
    uint32_t LogBlockNbr;
    uint32_t LogBlockSize;
} HAL_SD_CardInfoTypeDef;

#define HAL_SD_CARD_TRANSFER 4U

uint32_t HAL_SD_GetCardState(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *hsd,
                                     HAL_SD_CardInfoTypeDef *info);
HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint8_t *data,
                                    uint32_t block, uint32_t count,
                                    uint32_t timeout);
HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint8_t *data,
                                     uint32_t block, uint32_t count,
                                     uint32_t timeout);

#endif
"""


FAKE_SPI_H = r"""
#ifndef FAKE_SPI_H
#define FAKE_SPI_H
#include "main.h"
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
#endif
"""


def write_fakes(tmp_path: Path, extra: dict[str, str] | None = None) -> Path:
    """把替身头文件落到一个临时目录，返回该目录（用作第一个 -I）。"""
    fakes = tmp_path / "fakes"
    fakes.mkdir(exist_ok=True)
    (fakes / "main.h").write_text(FAKE_MAIN_H, encoding="utf-8")
    (fakes / "spi.h").write_text(FAKE_SPI_H, encoding="utf-8")
    for name, text in (extra or {}).items():
        (fakes / name).write_text(text, encoding="utf-8")
    return fakes


def build_and_run(
    tmp_path: Path,
    name: str,
    harness: str,
    sources: list[Path],
    includes: list[Path],
) -> subprocess.CompletedProcess[str]:
    """用 host gcc 把 harness 与真实源码编到一起并运行。"""
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required for the host harness")

    harness_c = tmp_path / f"{name}.c"
    harness_c.write_text(harness, encoding="utf-8")
    exe = tmp_path / f"{name}.exe"

    compile_cmd = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    compile_cmd += [f"-I{path}" for path in includes]
    compile_cmd += [str(path) for path in sources]
    compile_cmd += [str(harness_c), "-o", str(exe)]

    built = subprocess.run(compile_cmd, cwd=ROOT, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr

    return subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True)


CHECK_MACRO = r"""
#include <stdio.h>

static int failures;

#define CHECK(cond, id)                                                    \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("FAIL %d at line %d: %s\n", (id), __LINE__, #cond);     \
            failures++;                                                    \
        }                                                                  \
    } while (0)

#define REPORT()                                                           \
    do {                                                                   \
        if (failures != 0) {                                               \
            printf("%d checks failed\n", failures);                        \
            return 1;                                                      \
        }                                                                  \
        printf("all checks passed\n");                                     \
        return 0;                                                          \
    } while (0)
"""

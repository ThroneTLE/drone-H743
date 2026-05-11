#include "bsp_spi.h"

#include "drv_gd25q32.h"

#include <stdbool.h>

static DRV_GD25Q32_Device *bsp_spi_flash_dev;

void BSP_SPI_RegisterFlashDevice(DRV_GD25Q32_Device *dev)
{
    bsp_spi_flash_dev = dev;
}

static bool bsp_spi_is_flash_spi(SPI_HandleTypeDef *hspi)
{
    return (hspi != NULL) && (hspi->Instance == SPI1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (bsp_spi_is_flash_spi(hspi) && (bsp_spi_flash_dev != NULL)) {
        DRV_GD25Q32_DmaTxCplt(bsp_spi_flash_dev);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (bsp_spi_is_flash_spi(hspi) && (bsp_spi_flash_dev != NULL)) {
        DRV_GD25Q32_DmaRxCplt(bsp_spi_flash_dev);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (bsp_spi_is_flash_spi(hspi) && (bsp_spi_flash_dev != NULL)) {
        DRV_GD25Q32_DmaRxCplt(bsp_spi_flash_dev);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (bsp_spi_is_flash_spi(hspi) && (bsp_spi_flash_dev != NULL)) {
        DRV_GD25Q32_DmaError(bsp_spi_flash_dev);
    }
}

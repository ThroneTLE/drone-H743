#include "bsp_mag.h"
#include "bsp_board.h"

#include <string.h>

static DRV_MAG_Device mag_dev;

DRV_MAG_Status BSP_MAG_Init(void)
{
    memset(&mag_dev, 0, sizeof(mag_dev));
    return DRV_MAG_Init(&mag_dev, BSP_Board_GetMagBus());
}

DRV_MAG_Status BSP_MAG_Read(DRV_MAG_RawData *raw, DRV_MAG_ScaledData *scaled)
{
    return DRV_MAG_Read(&mag_dev, raw, scaled);
}

DRV_MAG_Status BSP_MAG_Probe(BSP_MAG_Status *status)
{
    if (status == NULL) { return DRV_MAG_INVALID_ARG; }

    memset(status, 0, sizeof(*status));
    status->type             = mag_dev.info.type;
    status->address          = mag_dev.info.address;
    status->who_am_i         = mag_dev.info.who_am_i;
    status->initialized      = mag_dev.initialized;
    status->last_status      = mag_dev.last_status;
    status->sample_count     = mag_dev.sample_count;
    status->raw              = mag_dev.raw;
    status->scaled           = mag_dev.scaled;
    status->detected_ist8310 = mag_dev.info.detected_ist8310;
    status->detected_hmc5883 = mag_dev.info.detected_hmc5883;
    status->detected_qmc5883 = mag_dev.info.detected_qmc5883;
    status->hmc_id[0]        = mag_dev.info.hmc_id[0];
    status->hmc_id[1]        = mag_dev.info.hmc_id[1];
    status->hmc_id[2]        = mag_dev.info.hmc_id[2];

    return mag_dev.last_status;
}

void BSP_MAG_GetStatus(BSP_MAG_Status *status)
{
    if (status == NULL) { return; }
    (void)BSP_MAG_Probe(status);
}

void BSP_MAG_Invalidate(void)
{
    DRV_MAG_Invalidate(&mag_dev);
}

const char *BSP_MAG_TypeName(DRV_MAG_Type type)
{
    return DRV_MAG_TypeName(type);
}

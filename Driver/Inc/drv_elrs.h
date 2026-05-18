#ifndef DRV_ELRS_H
#define DRV_ELRS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRSF protocol constants */
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8U
#define CRSF_FRAME_GPS                  0x02U
#define CRSF_FRAME_BATTERY_SENSOR       0x08U
#define CRSF_FRAME_BARO_ALTITUDE        0x09U
#define CRSF_FRAME_LINK_STATISTICS      0x14U
#define CRSF_FRAME_RC_CHANNELS_PACKED   0x16U
#define CRSF_FRAME_ATTITUDE             0x1EU
#define CRSF_FRAME_FLIGHT_MODE          0x21U
#define CRSF_MAX_LEN                    62U
#define CRSF_MIN_LEN                    2U
#define CRSF_MAX_FRAME_SIZE             (CRSF_MAX_LEN + 2U)
#define CRSF_RC_PAYLOAD_LEN             22U
#define CRSF_CHANNEL_COUNT              16U

typedef struct {
    uint8_t  valid;
    uint8_t  uplink_rssi_1;
    uint8_t  uplink_rssi_2;
    uint8_t  uplink_lq;
    int8_t   uplink_snr;
    uint8_t  active_antenna;
    uint8_t  rf_mode;
    uint8_t  tx_power;
    uint8_t  downlink_rssi;
    uint8_t  downlink_lq;
    int8_t   downlink_snr;
} DRV_ELRS_LinkStats;

void     DRV_ELRS_Init(void);
uint8_t  DRV_ELRS_ProcessByte(uint8_t byte);
uint8_t  DRV_ELRS_Crc8(const uint8_t *data, uint8_t len);

void     DRV_ELRS_GetChannels(uint16_t raw_out[CRSF_CHANNEL_COUNT],
                              uint16_t us_out[CRSF_CHANNEL_COUNT]);
const DRV_ELRS_LinkStats *DRV_ELRS_GetLinkStats(void);

uint8_t  DRV_ELRS_IsRcUpdated(void);
void     DRV_ELRS_ClearRcUpdated(void);

uint32_t DRV_ELRS_GetRcFrames(void);
uint32_t DRV_ELRS_GetCrcErrors(void);
uint32_t DRV_ELRS_GetLengthErrors(void);
uint32_t DRV_ELRS_GetFpsX10(void);

uint8_t  DRV_ELRS_BuildTelemetry(uint8_t type, const uint8_t *payload,
                                 uint8_t payload_len,
                                 uint8_t out_frame[CRSF_MAX_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ELRS_H */

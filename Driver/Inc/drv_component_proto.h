#ifndef DRV_COMPONENT_PROTO_H
#define DRV_COMPONENT_PROTO_H
#include <stdint.h>
#include <stddef.h>

#define DRV_COMPONENT_VERSION 1U
#define DRV_COMPONENT_MAX_PAYLOAD 247U
#define DRV_COMPONENT_MAX_COUNT 16U
#define DRV_COMPONENT_MAX_FIELDS 4U
enum { DRV_COMPONENT_BEGIN=1, DRV_COMPONENT_RECORD=2, DRV_COMPONENT_END=3, DRV_COMPONENT_ERROR=4 };
enum { DRV_COMPONENT_UNKNOWN=0, DRV_COMPONENT_READY=1, DRV_COMPONENT_WAITING=2,
       DRV_COMPONENT_DISABLED=3, DRV_COMPONENT_FAULT=4 };
enum { DRV_COMPONENT_U32=1, DRV_COMPONENT_I32=2, DRV_COMPONENT_F32=3, DRV_COMPONENT_HEX=4 };
enum { DRV_COMPONENT_IMU=1, DRV_COMPONENT_BARO, DRV_COMPONENT_MAG, DRV_COMPONENT_FLOW,
       DRV_COMPONENT_CURRENT, DRV_COMPONENT_PARAMS, DRV_COMPONENT_LOG,
       DRV_COMPONENT_ESC, DRV_COMPONENT_SERVO, DRV_COMPONENT_RC,
       DRV_COMPONENT_UART, DRV_COMPONENT_BT, DRV_COMPONENT_LED, DRV_COMPONENT_GPS };
typedef struct {
    const char *label, *unit;
    uint8_t type, valid;
    union { uint32_t u; int32_t i; float f; } value;
} DRV_ComponentField;
typedef struct {
    uint16_t id;
    uint8_t state, field_count;
    int32_t code;
    uint32_t age_ms, samples;
    const char *name, *model, *bus, *stage, *note;
    DRV_ComponentField fields[DRV_COMPONENT_MAX_FIELDS];
} DRV_ComponentRecord;

/* LE header <BBHIH>: version, kind, count, request nonce, record index.
 * Record: <HBBiII>, five length-prefixed UTF8 strings, then typed fields.
 * Each field: label string, unit string, <BBI> type/valid/value bits.
 * END appends CRC32 of complete RECORD payloads; BEGIN has no body.
 * Error appends a u32 reason. No HAL, allocation, I/O, or float printf.
 */
uint8_t DRV_Component_Encode(uint8_t kind, uint32_t nonce, uint16_t index,
                            uint16_t count, const DRV_ComponentRecord *record,
                            uint32_t trailer, uint8_t *out, uint16_t capacity,
                            uint16_t *length);
uint32_t DRV_Component_CrcUpdate(uint32_t crc, const uint8_t *bytes, size_t size);
#endif

/*
 * 辨识样本的线上打包。纯整数/浮点转换，宿主可逐字节对拍。
 */
#include "drv_sysid_record.h"

#include <math.h>
#include <string.h>

/*
 * 字段表就是**契约本身**：主机按它解码，schema_hash 也由它算出来。
 * 改任何一行（名字、单位、缩放、顺序）都会改变 hash，老主机会当场拒绝解码，
 * 而不是按旧布局解出一组看着正常的错值。
 *
 * scale 的含义是"定点 × scale = 物理量"。
 */
typedef struct {
    const char *name;
    const char *unit;
    float scale;
    uint8_t type;   /* DRV_SYSID_FIELD_TYPE_* */
} SysIdField;

static const SysIdField sysid_fields[] = {
    { "gx",       "rad/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },  /* mrad/s */
    { "gy",       "rad/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "gz",       "rad/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "omega_sp", "rad/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "alpha_ff", "rad/s^2", 1.0e-2f, DRV_SYSID_FIELD_TYPE_I16 },  /* ±327 */
    { "tilt_x",   "rad",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },  /* 0.1 mrad */
    { "tilt_y",   "rad",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "thrust",   "N",       1.0e-2f, DRV_SYSID_FIELD_TYPE_I16 },  /* cN */
    { "angle",    "rad",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "erpm",     "rpm",     4.0f,    DRV_SYSID_FIELD_TYPE_U16 },  /* 上桨；量程 262140 */
    { "torque",   "N*m",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "angle_sp", "rad",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "offset_us", "us",     1.0f,    DRV_SYSID_FIELD_TYPE_U16 },
    /* v3 只往后追加：前 13 个字段的顺序与缩放不动，按表取值的主机不必改偏移。 */
    { "erpm_lower", "rpm",   4.0f,    DRV_SYSID_FIELD_TYPE_U16 },  /* 下桨 */
    { "servo_tilt", "rad",   1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    /* 高度辨识（R-ALTID-1）只往后追加：前 15 个字段不动；非 ALT 轮这 7 个恒为 0。 */
    { "height",     "m",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },  /* 0.1 mm，±3.27 m */
    { "height_raw", "m",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "height_sp",  "m",     1.0e-4f, DRV_SYSID_FIELD_TYPE_I16 },
    { "vz",         "m/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "vz_sp",      "m/s",   1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "az",         "m/s^2", 1.0e-3f, DRV_SYSID_FIELD_TYPE_I16 },
    { "vbat",       "V",     1.0e-3f, DRV_SYSID_FIELD_TYPE_U16 },  /* mV */
};

#define SYSID_FIELD_COUNT \
    (uint32_t)(sizeof(sysid_fields) / sizeof(sysid_fields[0]))

/* 每个字段恒为 2 字节：表与记录长度对不上时编译期就停，不等到主机解出错位。 */
_Static_assert((sizeof(sysid_fields) / sizeof(sysid_fields[0])) * 2U ==
                   DRV_SYSID_RECORD_BYTES,
               "sysid field table and DRV_SYSID_RECORD_BYTES disagree");

uint32_t DRV_SysIdRecord_FieldCount(void)
{
    return SYSID_FIELD_COUNT;
}

const char *DRV_SysIdRecord_FieldName(uint32_t index)
{
    return (index < SYSID_FIELD_COUNT) ? sysid_fields[index].name : NULL;
}

const char *DRV_SysIdRecord_FieldUnit(uint32_t index)
{
    return (index < SYSID_FIELD_COUNT) ? sysid_fields[index].unit : NULL;
}

float DRV_SysIdRecord_FieldScale(uint32_t index)
{
    return (index < SYSID_FIELD_COUNT) ? sysid_fields[index].scale : 0.0f;
}

uint8_t DRV_SysIdRecord_FieldType(uint32_t index)
{
    return (index < SYSID_FIELD_COUNT) ? sysid_fields[index].type
                                       : (uint8_t)DRV_SYSID_FIELD_TYPE_I16;
}

/* FNV-1a over 字段名/单位/缩放的字节表示，外加版本与记录长度。 */
uint32_t DRV_SysIdRecord_SchemaHash(void)
{
    uint32_t hash = 2166136261U;

    for (uint32_t i = 0U; i < SYSID_FIELD_COUNT; ++i) {
        const char *text[2] = { sysid_fields[i].name, sysid_fields[i].unit };
        for (uint32_t part = 0U; part < 2U; ++part) {
            for (const char *cursor = text[part]; *cursor != '\0'; ++cursor) {
                hash ^= (uint32_t)(uint8_t)*cursor;
                hash *= 16777619U;
            }
            hash ^= 0x2FU; /* 分隔符，防止相邻字段拼接后产生同一串 */
            hash *= 16777619U;
        }
        /* 缩放按其 IEEE-754 位模式参与，改了缩放必须换 hash。 */
        uint32_t bits;
        memcpy(&bits, &sysid_fields[i].scale, sizeof(bits));
        for (uint32_t byte = 0U; byte < 4U; ++byte) {
            hash ^= (bits >> (8U * byte)) & 0xFFU;
            hash *= 16777619U;
        }
        /*
         * 类型也必须进 hash：i16 换成 u16 字节数没变、名字没变、缩放没变，
         * 但同一串字节的**含义**变了。漏掉它，老主机会把新数据解成看着正常的错值——
         * 正是 schema_hash 存在要挡住的那种失败。
         */
        hash ^= (uint32_t)sysid_fields[i].type;
        hash *= 16777619U;
    }
    hash ^= DRV_SYSID_RECORD_VERSION;
    hash *= 16777619U;
    hash ^= DRV_SYSID_RECORD_BYTES;
    hash *= 16777619U;
    return hash;
}

/* 饱和转换。超量程时钉在端点——回绕会把大角速度变成反号的小值。 */
static int16_t sysid_to_i16(float value, float scale)
{
    float counts;

    if (!isfinite(value)) {
        return 0;
    }
    counts = value / scale;
    if (counts >= 32767.0f) {
        return (int16_t)32767;
    }
    if (counts <= -32768.0f) {
        return (int16_t)(-32768);
    }
    return (int16_t)lrintf(counts);
}

static uint16_t sysid_to_u16(float value, float scale)
{
    float counts;

    if (!isfinite(value) || (value <= 0.0f)) {
        return 0U;
    }
    counts = value / scale;
    if (counts >= 65535.0f) {
        return (uint16_t)65535;
    }
    return (uint16_t)lrintf(counts);
}

static void sysid_put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void sysid_put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8U) & 0xFFU);
    out[2] = (uint8_t)((value >> 16U) & 0xFFU);
    out[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t sysid_get_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t sysid_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

DRV_SysIdRecordStatus DRV_SysIdRecord_Pack(const DRV_SysIdBatchHeader *header,
                                           const DRV_SysIdSample *samples,
                                           uint32_t count,
                                           uint8_t *out, size_t capacity,
                                           size_t *out_len)
{
    size_t needed;
    uint8_t *cursor;

    if ((header == NULL) || (samples == NULL) || (out == NULL) ||
        (out_len == NULL) || (count == 0U) ||
        (count > DRV_SYSID_RECORD_MAX_COUNT)) {
        return DRV_SYSID_RECORD_INVALID;
    }
    needed = DRV_SYSID_RECORD_HEADER_BYTES +
             ((size_t)count * DRV_SYSID_RECORD_BYTES);
    if (capacity < needed) {
        return DRV_SYSID_RECORD_TOO_SMALL;
    }

    out[0] = (uint8_t)DRV_SYSID_RECORD_VERSION;
    out[1] = (uint8_t)count;
    sysid_put_u16(&out[2], header->run_id);
    sysid_put_u32(&out[4], DRV_SysIdRecord_SchemaHash());
    sysid_put_u32(&out[8], header->base_t_us);
    sysid_put_u16(&out[12], header->dt_us);
    sysid_put_u16(&out[14], header->flags);

    cursor = &out[DRV_SYSID_RECORD_HEADER_BYTES];
    for (uint32_t i = 0U; i < count; ++i) {
        const DRV_SysIdSample *sample = &samples[i];
        sysid_put_u16(&cursor[0],
            (uint16_t)sysid_to_i16(sample->gyro_rad_s[0], sysid_fields[0].scale));
        sysid_put_u16(&cursor[2],
            (uint16_t)sysid_to_i16(sample->gyro_rad_s[1], sysid_fields[1].scale));
        sysid_put_u16(&cursor[4],
            (uint16_t)sysid_to_i16(sample->gyro_rad_s[2], sysid_fields[2].scale));
        sysid_put_u16(&cursor[6],
            (uint16_t)sysid_to_i16(sample->omega_sp_rad_s, sysid_fields[3].scale));
        sysid_put_u16(&cursor[8],
            (uint16_t)sysid_to_i16(sample->alpha_ff_rad_s2, sysid_fields[4].scale));
        sysid_put_u16(&cursor[10],
            (uint16_t)sysid_to_i16(sample->tilt_cmd_x_rad, sysid_fields[5].scale));
        sysid_put_u16(&cursor[12],
            (uint16_t)sysid_to_i16(sample->tilt_cmd_y_rad, sysid_fields[6].scale));
        sysid_put_u16(&cursor[14],
            (uint16_t)sysid_to_i16(sample->thrust_n, sysid_fields[7].scale));
        sysid_put_u16(&cursor[16],
            (uint16_t)sysid_to_i16(sample->angle_rad, sysid_fields[8].scale));
        sysid_put_u16(&cursor[18],
            sysid_to_u16((float)sample->erpm, sysid_fields[9].scale));
        sysid_put_u16(&cursor[20], (uint16_t)sysid_to_i16(sample->torque_n_m, sysid_fields[10].scale));
        sysid_put_u16(&cursor[22], (uint16_t)sysid_to_i16(sample->angle_sp_rad, sysid_fields[11].scale));
        if (sample->offset_us > 65535U) { return DRV_SYSID_RECORD_INVALID; }
        sysid_put_u16(&cursor[24], (uint16_t)sample->offset_us);
        sysid_put_u16(&cursor[26],
            sysid_to_u16((float)sample->erpm_lower, sysid_fields[13].scale));
        sysid_put_u16(&cursor[28],
            (uint16_t)sysid_to_i16(sample->servo_tilt_rad, sysid_fields[14].scale));
        sysid_put_u16(&cursor[30], (uint16_t)sysid_to_i16(sample->height_m, sysid_fields[15].scale));
        sysid_put_u16(&cursor[32], (uint16_t)sysid_to_i16(sample->height_raw_m, sysid_fields[16].scale));
        sysid_put_u16(&cursor[34], (uint16_t)sysid_to_i16(sample->height_sp_m, sysid_fields[17].scale));
        sysid_put_u16(&cursor[36], (uint16_t)sysid_to_i16(sample->vz_m_s, sysid_fields[18].scale));
        sysid_put_u16(&cursor[38], (uint16_t)sysid_to_i16(sample->vz_sp_m_s, sysid_fields[19].scale));
        sysid_put_u16(&cursor[40], (uint16_t)sysid_to_i16(sample->az_m_s2, sysid_fields[20].scale));
        sysid_put_u16(&cursor[42], sysid_to_u16(sample->vbat_v, sysid_fields[21].scale));
        cursor += DRV_SYSID_RECORD_BYTES;
    }

    *out_len = needed;
    return DRV_SYSID_RECORD_OK;
}

DRV_SysIdRecordStatus DRV_SysIdRecord_Unpack(const uint8_t *data, size_t length,
                                             DRV_SysIdBatchHeader *header,
                                             DRV_SysIdSample *samples,
                                             uint32_t *out_count)
{
    uint32_t count;
    const uint8_t *cursor;

    if ((data == NULL) || (header == NULL) || (samples == NULL) ||
        (out_count == NULL) || (length < DRV_SYSID_RECORD_HEADER_BYTES)) {
        return DRV_SYSID_RECORD_INVALID;
    }
    if (data[0] != (uint8_t)DRV_SYSID_RECORD_VERSION) {
        return DRV_SYSID_RECORD_INVALID;
    }
    count = data[1];
    if ((count == 0U) || (count > DRV_SYSID_RECORD_MAX_COUNT)) {
        return DRV_SYSID_RECORD_INVALID;
    }
    if (sysid_get_u32(&data[4]) != DRV_SysIdRecord_SchemaHash()) {
        /* 布局变了就明确失败；按旧布局硬解会得到一组看着正常的错值。 */
        return DRV_SYSID_RECORD_INVALID;
    }
    if (length < (DRV_SYSID_RECORD_HEADER_BYTES +
                  ((size_t)count * DRV_SYSID_RECORD_BYTES))) {
        return DRV_SYSID_RECORD_TOO_SMALL;
    }

    header->run_id = sysid_get_u16(&data[2]);
    header->base_t_us = sysid_get_u32(&data[8]);
    header->dt_us = sysid_get_u16(&data[12]);
    header->flags = sysid_get_u16(&data[14]);

    cursor = &data[DRV_SYSID_RECORD_HEADER_BYTES];
    for (uint32_t i = 0U; i < count; ++i) {
        DRV_SysIdSample *sample = &samples[i];
        sample->gyro_rad_s[0] =
            (float)(int16_t)sysid_get_u16(&cursor[0]) * sysid_fields[0].scale;
        sample->gyro_rad_s[1] =
            (float)(int16_t)sysid_get_u16(&cursor[2]) * sysid_fields[1].scale;
        sample->gyro_rad_s[2] =
            (float)(int16_t)sysid_get_u16(&cursor[4]) * sysid_fields[2].scale;
        sample->omega_sp_rad_s =
            (float)(int16_t)sysid_get_u16(&cursor[6]) * sysid_fields[3].scale;
        sample->alpha_ff_rad_s2 =
            (float)(int16_t)sysid_get_u16(&cursor[8]) * sysid_fields[4].scale;
        sample->tilt_cmd_x_rad =
            (float)(int16_t)sysid_get_u16(&cursor[10]) * sysid_fields[5].scale;
        sample->tilt_cmd_y_rad =
            (float)(int16_t)sysid_get_u16(&cursor[12]) * sysid_fields[6].scale;
        sample->thrust_n =
            (float)(int16_t)sysid_get_u16(&cursor[14]) * sysid_fields[7].scale;
        sample->angle_rad =
            (float)(int16_t)sysid_get_u16(&cursor[16]) * sysid_fields[8].scale;
        sample->erpm =
            (uint32_t)(sysid_get_u16(&cursor[18]) * (uint32_t)sysid_fields[9].scale);
        sample->torque_n_m = (float)(int16_t)sysid_get_u16(&cursor[20]) * sysid_fields[10].scale;
        sample->angle_sp_rad = (float)(int16_t)sysid_get_u16(&cursor[22]) * sysid_fields[11].scale;
        sample->offset_us = sysid_get_u16(&cursor[24]);
        sample->erpm_lower =
            (uint32_t)(sysid_get_u16(&cursor[26]) * (uint32_t)sysid_fields[13].scale);
        sample->servo_tilt_rad =
            (float)(int16_t)sysid_get_u16(&cursor[28]) * sysid_fields[14].scale;
        sample->height_m = (float)(int16_t)sysid_get_u16(&cursor[30]) * sysid_fields[15].scale;
        sample->height_raw_m = (float)(int16_t)sysid_get_u16(&cursor[32]) * sysid_fields[16].scale;
        sample->height_sp_m = (float)(int16_t)sysid_get_u16(&cursor[34]) * sysid_fields[17].scale;
        sample->vz_m_s = (float)(int16_t)sysid_get_u16(&cursor[36]) * sysid_fields[18].scale;
        sample->vz_sp_m_s = (float)(int16_t)sysid_get_u16(&cursor[38]) * sysid_fields[19].scale;
        sample->az_m_s2 = (float)(int16_t)sysid_get_u16(&cursor[40]) * sysid_fields[20].scale;
        sample->vbat_v = (float)sysid_get_u16(&cursor[42]) * sysid_fields[21].scale;
        cursor += DRV_SYSID_RECORD_BYTES;
    }

    *out_count = count;
    return DRV_SYSID_RECORD_OK;
}

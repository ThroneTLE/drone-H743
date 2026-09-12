#include "drv_dshot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void px4_reference(uint16_t first, uint16_t second, uint32_t out[34]);
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)

static int encoding(void)
{
    const uint16_t code[] = {0, 48, 49, 1000, 1047, 2047};
    const uint16_t golden[] = {0x0000, 0x0606, 0x0624, 0x7D0A, 0x82E4, 0xFFEE};
    for (size_t i = 0; i < sizeof(code) / sizeof(code[0]); ++i) {
        uint16_t frame = 0xFFFF;
        CHECK(DRV_DShot_Encode(code[i], &frame) == DRV_DSHOT_OK);
        CHECK(frame == golden[i]);
        CHECK((frame & 0x10U) == 0U);
    }
    for (uint32_t code_value = 0; code_value <= UINT16_MAX; ++code_value) {
        uint16_t frame = 0xCAFE;
        int valid = code_value == 0 || (code_value >= 48 && code_value <= 2047);
        CHECK((DRV_DShot_Encode((uint16_t)code_value, &frame) == DRV_DSHOT_OK) == valid);
        if (!valid) { CHECK(frame == 0xCAFE); }
    }
    CHECK(DRV_DShot_Encode(0, NULL) == DRV_DSHOT_INVALID);
    puts("encoding: independent golden vectors and all 65536 input values passed");
    return 0;
}

static int mapping(void)
{
    uint16_t previous = 0;
    for (uint32_t pulse = 0; pulse <= UINT16_MAX; ++pulse) {
        uint16_t value = 0xCAFE;
        int valid = pulse >= 1100 && pulse <= 1940;
        CHECK((DRV_DShot_FromPulseUs((uint16_t)pulse, &value) == DRV_DSHOT_OK) == valid);
        if (!valid) { CHECK(value == 0xCAFE); continue; }
        if (pulse == 1100) { CHECK(value == 0); }
        else { CHECK(value >= 48 && value <= 2047); CHECK(value > previous); }
        if (pulse == 1101) { CHECK(value == 48); }
        if (pulse == 1940) { CHECK(value == 2047); }
        previous = value;
    }
    CHECK(DRV_DShot_FromPulseUs(1100, NULL) == DRV_DSHOT_INVALID);
    puts("mapping: all 65536 inputs, endpoints, monotonicity and command exclusion passed");
    return 0;
}

static int upstream_compare(void)
{
    DRV_DShotTiming timing;
    CHECK(DRV_DShot_MakeTiming(120000000, &timing) == DRV_DSHOT_OK);
    /* Every valid throttle occurs in both positions; channels always differ. */
    for (uint16_t first = 0; first <= 2047; ++first) {
        if (first > 0 && first < 48) { continue; }
        uint16_t second = first == 0 ? 2047 : (uint16_t)(2095U - first);
        uint16_t code[2] = {first, second};
        uint32_t reference[34];
        uint32_t actual[DRV_DSHOT_BURST_WORDS + 1];
        actual[DRV_DSHOT_BURST_WORDS] = 0xCAFE;
        px4_reference(first, second, reference);
        CHECK(DRV_DShot_BuildBurst(code, &timing, actual, DRV_DSHOT_BURST_WORDS + 1) == DRV_DSHOT_OK);
        for (size_t word = 0; word < 32; ++word) {
            CHECK(reference[word] == 7 || reference[word] == 14);
            CHECK(actual[word] == (reference[word] == 14 ? 300U : 150U));
        }
        for (size_t word = 32; word < DRV_DSHOT_BURST_WORDS; ++word) { CHECK(actual[word] == 0); }
        CHECK(actual[DRV_DSHOT_BURST_WORDS] == 0xCAFE);
    }
    /* Check the stop word specifically: sixteen short pulses, not a held-low line. */
    uint16_t stops[2] = {0, 0};
    uint32_t actual[DRV_DSHOT_BURST_WORDS];
    CHECK(DRV_DShot_BuildBurst(stops, &timing, actual, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_OK);
    for (size_t word = 0; word < 32; ++word) { CHECK(actual[word] == 150); }
    puts("PX4 reference: 2001 dual-channel pairs, MSB order, checksum and tails passed");
    return 0;
}

static int timing_and_rejections(void)
{
    const uint32_t clocks[] = {2400000, 60000000, 120000000, 240000000};
    DRV_DShotTiming timing;
    for (size_t i = 0; i < sizeof(clocks) / sizeof(clocks[0]); ++i) {
        CHECK(DRV_DShot_MakeTiming(clocks[i], &timing) == DRV_DSHOT_OK);
        CHECK(timing.prescaler == 0);
        CHECK((timing.auto_reload + 1U) * 300000U == clocks[i]);
        CHECK(timing.zero_ticks * 8U == (timing.auto_reload + 1U) * 3U);
        CHECK(timing.one_ticks == timing.zero_ticks * 2U);
    }
    const uint32_t bad_clocks[] = {0, 1, 300000, 2100000, 61000000, 120000001, UINT32_MAX};
    DRV_DShotTiming before;
    memset(&timing, 0xA5, sizeof(timing));
    memcpy(&before, &timing, sizeof(timing));
    for (size_t i = 0; i < sizeof(bad_clocks) / sizeof(bad_clocks[0]); ++i) {
        CHECK(DRV_DShot_MakeTiming(bad_clocks[i], &timing) == DRV_DSHOT_INVALID);
        CHECK(memcmp(&timing, &before, sizeof(timing)) == 0);
    }
    CHECK(DRV_DShot_MakeTiming(120000000, NULL) == DRV_DSHOT_INVALID);
    CHECK(DRV_DShot_MakeTiming(120000000, &timing) == DRV_DSHOT_OK);
    CHECK(timing.auto_reload == 399 && timing.zero_ticks == 150 && timing.one_ticks == 300);
    uint16_t codes[2] = {48, 2047};
    uint32_t output[DRV_DSHOT_BURST_WORDS];
    uint32_t backup[DRV_DSHOT_BURST_WORDS];
    memset(output, 0xA5, sizeof(output));
    memcpy(backup, output, sizeof(output));
    for (size_t size = 0; size < DRV_DSHOT_BURST_WORDS; ++size) {
        CHECK(DRV_DShot_BuildBurst(codes, &timing, output, size) == DRV_DSHOT_INVALID);
        CHECK(memcmp(output, backup, sizeof(output)) == 0);
    }
    CHECK(DRV_DShot_BuildBurst(NULL, &timing, output, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_INVALID);
    CHECK(DRV_DShot_BuildBurst(codes, NULL, output, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_INVALID);
    CHECK(DRV_DShot_BuildBurst(codes, &timing, NULL, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_INVALID);
    for (size_t channel = 0; channel < 2; ++channel) {
        uint16_t saved = codes[channel];
        const uint16_t invalid[] = {1, 47, 2048, UINT16_MAX};
        for (size_t n = 0; n < 4; ++n) {
            codes[channel] = invalid[n];
            CHECK(DRV_DShot_BuildBurst(codes, &timing, output, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_INVALID);
            CHECK(memcmp(output, backup, sizeof(output)) == 0);
        }
        codes[channel] = saved;
    }
    DRV_DShotTiming forged[] = {timing, timing, timing, timing, timing};
    forged[0].timer_clock_hz++;
    forged[1].prescaler++;
    forged[2].auto_reload++;
    forged[3].zero_ticks++;
    forged[4].one_ticks++;
    for (size_t i = 0; i < 5; ++i) {
        CHECK(DRV_DShot_BuildBurst(codes, &forged[i], output, DRV_DSHOT_BURST_WORDS) == DRV_DSHOT_INVALID);
        CHECK(memcmp(output, backup, sizeof(output)) == 0);
    }
    puts("timing: ARR+1, exact duty, invalid clocks and transactional rejection passed");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) { return 2; }
    switch (atoi(argv[1])) {
    case 0: return encoding();
    case 1: return mapping();
    case 2: return upstream_compare();
    case 3: return timing_and_rejections();
    default: return 2;
    }
}

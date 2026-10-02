#include "svc_mag_heading.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SVC_MAG_HEADING_DEG_TO_RAD 0.01745329251994329577f

void SVC_MAG_HeadingReset(SVC_MAG_HeadingState *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

uint8_t SVC_MAG_HeadingConfigValid(const SVC_MAG_HeadingConfig *config)
{
    return ((config != NULL) &&
            isfinite(config->bias_x_mgauss) &&
            isfinite(config->bias_y_mgauss) &&
            isfinite(config->radius_xy_mgauss) &&
            (config->radius_xy_mgauss > 0.0f)) ? 1U : 0U;
}

SVC_MAG_HeadingResult SVC_MAG_HeadingUpdate(
    const SVC_MAG_HeadingConfig *config,
    SVC_MAG_HeadingState *state,
    const SVC_MAG_HeadingInput *input,
    float corrected_flu_mgauss[3])
{
    float z_sum = 0.0f;
    float xy_x;
    float xy_y;
    float rotated_x;
    float rotated_y;
    float scale;
    float c;
    float s;
    uint32_t index;

    if ((state == NULL) || (input == NULL) ||
        (corrected_flu_mgauss == NULL) ||
        (SVC_MAG_HeadingConfigValid(config) == 0U)) {
        return SVC_MAG_HEADING_INVALID;
    }
    if (config->enabled == 0U) {
        return SVC_MAG_HEADING_DISABLED;
    }
    if (state->generation != config->generation) {
        SVC_MAG_HeadingReset(state);
        state->generation = config->generation;
    }
    if ((input->timestamp_us == 0ULL) ||
        (input->timestamp_us <= state->last_timestamp_us)) {
        return SVC_MAG_HEADING_REPEATED;
    }
    state->last_timestamp_us = input->timestamp_us;
    if (!isfinite(input->raw_flu_mgauss[0]) ||
        !isfinite(input->raw_flu_mgauss[1]) ||
        !isfinite(input->raw_flu_mgauss[2]) ||
        !isfinite(input->roll_deg) || !isfinite(input->pitch_deg) ||
        !isfinite(input->yaw_deg)) {
        state->z_count = 0U;
        state->z_next = 0U;
        return SVC_MAG_HEADING_INVALID;
    }
    /* sqrt(roll^2+pitch^2) is the near-level total tilt to better than
     * 0.01 degree inside this 5-degree gate, and rejects inverted attitudes.
     * Z is not fed to the AHRS (see the output below), so the budget left is
     * the physical one: the true vertical field leaking into body XY, about
     * V/H * tilt -- roughly 6 degrees of heading at the 5-degree edge.
     */
    if ((input->roll_deg * input->roll_deg +
         input->pitch_deg * input->pitch_deg) >
        (SVC_MAG_HEADING_MAX_TILT_DEG * SVC_MAG_HEADING_MAX_TILT_DEG)) {
        state->z_count = 0U;
        state->z_next = 0U;
        ++state->tilt_count;
        return SVC_MAG_HEADING_TILT;
    }

    state->z_window_mgauss[state->z_next] = input->raw_flu_mgauss[2];
    state->z_next = (uint8_t)((state->z_next + 1U) % SVC_MAG_HEADING_Z_WINDOW);
    if (state->z_count < SVC_MAG_HEADING_Z_WINDOW) {
        ++state->z_count;
    }
    if (state->z_count < SVC_MAG_HEADING_Z_WINDOW) {
        return SVC_MAG_HEADING_WARMUP;
    }
    for (index = 0U; index < SVC_MAG_HEADING_Z_WINDOW; ++index) {
        z_sum += state->z_window_mgauss[index];
    }
    state->last_z_mean_mgauss = z_sum / (float)SVC_MAG_HEADING_Z_WINDOW;
    xy_x = input->raw_flu_mgauss[0] - config->bias_x_mgauss;
    xy_y = input->raw_flu_mgauss[1] - config->bias_y_mgauss;
    if ((xy_x * xy_x + xy_y * xy_y) <= 1.0f) {
        return SVC_MAG_HEADING_INVALID;
    }
    if (state->heading_aligned == 0U) {
        state->xy_rotation_rad =
            -input->yaw_deg * SVC_MAG_HEADING_DEG_TO_RAD - atan2f(xy_y, xy_x);
        state->heading_aligned = 1U;
    }
    c = cosf(state->xy_rotation_rad);
    s = sinf(state->xy_rotation_rad);
    /* Direction only: XY rescaled to a fixed field, Z = 0.
     *
     * The raw Z carries a large hard-iron offset (FLU mean about +567 mG,
     * while the local vertical field points down, i.e. negative in FLU).
     * The AHRS tilt-compensates with its own gravity estimate, so feeding
     * that Z projected it into heading: on the 2026-09-30 bench a 2.6-degree
     * static tilt made yaw swing about 5 degrees after every re-alignment
     * (data/analysis/mag-fit/2026-09-30). Z = 0 makes the AHRS heading the
     * level XY heading, the same basis the alignment above uses.
     *
     * The AHRS only uses the field direction; a fixed magnitude keeps its
     * 200..800 mG field gate from rejecting headings where the stale XY
     * hard-iron centre shrinks |xy|.
     */
    rotated_x = c * xy_x - s * xy_y;
    rotated_y = s * xy_x + c * xy_y;
    scale = SVC_MAG_HEADING_FUSION_FIELD_MGAUSS /
            sqrtf((xy_x * xy_x) + (xy_y * xy_y));
    corrected_flu_mgauss[0] = rotated_x * scale;
    corrected_flu_mgauss[1] = rotated_y * scale;
    corrected_flu_mgauss[2] = 0.0f;
    ++state->ready_count;
    return SVC_MAG_HEADING_READY;
}

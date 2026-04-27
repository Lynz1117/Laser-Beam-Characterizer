#include "measurement_m2.h"
#include <math.h>
#include <stddef.h>

#define M2_MAX_POSITIONS      64U
#define M2_ISO_TARGET_POINTS  10U
#define M2_ISO_POINTS_INSIDE  5U
#define M2_ISO_POINTS_OUTSIDE 5U
#define M2_MIN_FIT_POINTS     3U

static uint8_t m2_capture_both_axes_at_z(const M2MeasurementContext *ctx,
                                         uint32_t step_index)
{
    uint8_t axis0_ok = 0U;
    uint8_t axis1_ok = 0U;
    uint32_t idx = 0U;
    int32_t t0_enc = 0;
    int32_t x0_counts = 0;
    float t0_ms = 0.0f;

    if ((ctx == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->capture_axis_t0 == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    ctx->log_printf("M2_POS_BEGIN,z_mm=%.3f,step=%lu\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    if (*ctx->abort_flag)
        return 0U;

    // Capture both beam axes at the current Z stop so the host keeps the same packet flow.
    ctx->log_printf("M2_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=0\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M1, &idx, &t0_enc, &x0_counts, &t0_ms))
    {
        axis0_ok = 1U;
        ctx->log_printf("M2_AXIS_DONE,z_mm=%.3f,step=%lu,axis=0,"
                        "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index,
                        (unsigned long)idx,
                        (long)t0_enc,
                        (long)x0_counts,
                        t0_ms);
    }
    else
    {
        ctx->log_printf("M2_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=0\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
    }

    ctx->delay_ms(ctx->z_settle_ms);

    if (*ctx->abort_flag)
        return 0U;

    ctx->log_printf("M2_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=1\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M2, &idx, &t0_enc, &x0_counts, &t0_ms))
    {
        axis1_ok = 1U;
        ctx->log_printf("M2_AXIS_DONE,z_mm=%.3f,step=%lu,axis=1,"
                        "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index,
                        (unsigned long)idx,
                        (long)t0_enc,
                        (long)x0_counts,
                        t0_ms);
    }
    else
    {
        ctx->log_printf("M2_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=1\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
    }

    ctx->log_printf("M2_POS_DONE,z_mm=%.3f,step=%lu,axis0_ok=%u,axis1_ok=%u\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index,
                    (unsigned int)axis0_ok,
                    (unsigned int)axis1_ok);

    return (axis0_ok || axis1_ok) ? 1U : 0U;
}

static uint32_t __attribute__((unused)) m2_count_points_within_radius(const float *z_mm,
                                                                      uint32_t n,
                                                                      float center_mm,
                                                                      float radius_mm)
{
    uint32_t count = 0U;

    if ((z_mm == NULL) || (radius_mm < 0.0f))
        return 0U;

    for (uint32_t i = 0U; i < n; i++)
    {
        if (fabsf(z_mm[i] - center_mm) <= (radius_mm + 1.0e-3f))
            count++;
    }

    return count;
}

static uint32_t __attribute__((unused)) m2_count_points_outside_radius(const float *z_mm,
                                                                       uint32_t n,
                                                                       float center_mm,
                                                                       float radius_mm)
{
    uint32_t count = 0U;

    if ((z_mm == NULL) || (radius_mm < 0.0f))
        return 0U;

    for (uint32_t i = 0U; i < n; i++)
    {
        if (fabsf(z_mm[i] - center_mm) >= (radius_mm - 1.0e-3f))
            count++;
    }

    return count;
}

static float __attribute__((unused)) m2_compute_rayleigh_range_mm(float d0_mm, float theta_rad)
{
    if ((d0_mm <= 0.0f) || (theta_rad <= 0.0f))
        return 0.0f;

    return d0_mm / theta_rad;
}

static void __attribute__((unused)) m2_fill_linear_positions(float start_mm,
                                                             float end_mm,
                                                             uint32_t count,
                                                             float *dst,
                                                             uint32_t *idx)
{
    if ((dst == NULL) || (idx == NULL) || (count == 0U))
        return;

    if (count == 1U)
    {
        dst[*idx] = 0.5f * (start_mm + end_mm);
        (*idx)++;
        return;
    }

    for (uint32_t i = 0U; i < count; i++)
    {
        float frac = (float)i / (float)(count - 1U);
        dst[*idx] = start_mm + frac * (end_mm - start_mm);
        (*idx)++;
    }
}

static uint8_t __attribute__((unused)) m2_build_iso_positions(float z0_mm,
                                                              float z_rayleigh_mm,
                                                              float travel_max_mm,
                                                              float *target_z_mm,
                                                              uint32_t *target_count_out)
{
    float inner_lo_mm;
    float inner_hi_mm;
    float outer_left_hi_mm;
    float outer_right_lo_mm;
    float left_len_mm;
    float right_len_mm;
    uint32_t left_count = 0U;
    uint32_t right_count = 0U;
    uint32_t idx = 0U;

    if ((target_z_mm == NULL) || (target_count_out == NULL))
        return 0U;

    *target_count_out = 0U;

    if ((z_rayleigh_mm <= 0.0f) || (travel_max_mm <= 0.0f))
        return 0U;

    inner_lo_mm = z0_mm - z_rayleigh_mm;
    inner_hi_mm = z0_mm + z_rayleigh_mm;
    if (inner_lo_mm < 0.0f)
        inner_lo_mm = 0.0f;
    if (inner_hi_mm > travel_max_mm)
        inner_hi_mm = travel_max_mm;

    if (inner_hi_mm < inner_lo_mm)
        return 0U;

    outer_left_hi_mm = z0_mm - (2.0f * z_rayleigh_mm);
    outer_right_lo_mm = z0_mm + (2.0f * z_rayleigh_mm);

    if (outer_left_hi_mm < 0.0f)
        outer_left_hi_mm = 0.0f;
    if (outer_right_lo_mm > travel_max_mm)
        outer_right_lo_mm = travel_max_mm;

    left_len_mm = outer_left_hi_mm;
    right_len_mm = travel_max_mm - outer_right_lo_mm;

    if ((left_len_mm <= 0.0f) && (right_len_mm <= 0.0f))
        return 0U;

    if ((left_len_mm > 0.0f) && (right_len_mm > 0.0f))
    {
        float total_outer_len_mm = left_len_mm + right_len_mm;
        left_count = (uint32_t)((M2_ISO_POINTS_OUTSIDE * left_len_mm / total_outer_len_mm) + 0.5f);
        if (left_count == 0U)
            left_count = 1U;
        if (left_count >= M2_ISO_POINTS_OUTSIDE)
            left_count = M2_ISO_POINTS_OUTSIDE - 1U;
        right_count = M2_ISO_POINTS_OUTSIDE - left_count;
    }
    else if (left_len_mm > 0.0f)
    {
        left_count = M2_ISO_POINTS_OUTSIDE;
    }
    else
    {
        right_count = M2_ISO_POINTS_OUTSIDE;
    }

    if (left_count > 0U)
        m2_fill_linear_positions(0.0f, outer_left_hi_mm, left_count, target_z_mm, &idx);

    m2_fill_linear_positions(inner_lo_mm, inner_hi_mm, M2_ISO_POINTS_INSIDE, target_z_mm, &idx);

    if (right_count > 0U)
        m2_fill_linear_positions(outer_right_lo_mm, travel_max_mm, right_count, target_z_mm, &idx);

    *target_count_out = idx;
    return (idx == M2_ISO_TARGET_POINTS) ? 1U : 0U;
}

static uint8_t __attribute__((unused)) m2_measure_axis_at_positions(const M2MeasurementContext *ctx,
                                                                    uint8_t axis,
                                                                    const float *target_z_mm,
                                                                    uint32_t target_count,
                                                                    float *z_mm_out,
                                                                    float *d4sigma_mm_out,
                                                                    uint32_t *valid_count_out,
                                                                    const char *phase_tag)
{
    uint32_t valid_count = 0U;

    if ((ctx == NULL) ||
        (target_z_mm == NULL) ||
        (z_mm_out == NULL) ||
        (d4sigma_mm_out == NULL) ||
        (valid_count_out == NULL) ||
        (ctx->z_home_simple == NULL) ||
        (ctx->home_pressed == NULL) ||
        (ctx->z_move_to_mm == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->measure_axis_width == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    *valid_count_out = 0U;

    ctx->z_home_simple(ctx->z_home_dir);
    if (!ctx->home_pressed())
    {
        ctx->log_printf("M2_AXIS_HOME_FAIL,axis=%u,phase=%s\r\n",
                        (unsigned int)axis,
                        (phase_tag != NULL) ? phase_tag : "NA");
        return 0U;
    }

    ctx->delay_ms(ctx->z_settle_ms);

    for (uint32_t i = 0U; i < target_count; i++)
    {
        float width_mm = 0.0f;
        float target_mm = target_z_mm[i];

        ctx->z_move_to_mm(target_mm);
        ctx->delay_ms(ctx->z_settle_ms);

        ctx->log_printf("M2_TARGET_BEGIN,axis=%u,phase=%s,target_z_mm=%.3f,index=%lu\r\n",
                        (unsigned int)axis,
                        (phase_tag != NULL) ? phase_tag : "NA",
                        target_mm,
                        (unsigned long)(i + 1U));

        if (ctx->measure_axis_width(axis, &width_mm))
        {
            z_mm_out[valid_count] = *ctx->z_current_mm;
            d4sigma_mm_out[valid_count] = width_mm;
            valid_count++;

            ctx->log_printf("M2_POINT,axis=%u,phase=%s,z_mm=%.3f,d4sigma_mm=%.6f,index=%lu\r\n",
                            (unsigned int)axis,
                            (phase_tag != NULL) ? phase_tag : "NA",
                            *ctx->z_current_mm,
                            width_mm,
                            (unsigned long)valid_count);
        }
        else
        {
            ctx->log_printf("M2_POINT_FAIL,axis=%u,phase=%s,target_z_mm=%.3f,index=%lu\r\n",
                            (unsigned int)axis,
                            (phase_tag != NULL) ? phase_tag : "NA",
                            target_mm,
                            (unsigned long)(i + 1U));
        }
    }

    *valid_count_out = valid_count;
    return (valid_count >= M2_MIN_FIT_POINTS) ? 1U : 0U;
}

static uint8_t __attribute__((unused)) m2_fit_d4sigma_curve(const float *z_mm,
                                                            const float *d4sigma_mm,
                                                            uint32_t n,
                                                            float wavelength_nm,
                                                            float *z0_mm_out,
                                                            float *d0_mm_out,
                                                            float *theta_rad_out,
                                                            float *m2_out,
                                                            float *z_rayleigh_mm_out)
{
    float s0 = 0.0f;
    float s1 = 0.0f;
    float s2 = 0.0f;
    float s3 = 0.0f;
    float s4 = 0.0f;
    float t0 = 0.0f;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float a;
    float b;
    float c;
    float z0_mm;
    float d0_sq_mm2;
    float theta_rad;
    float lambda_mm;
    float aug[3][4];

    if ((z_mm == NULL) || (d4sigma_mm == NULL) ||
        (z0_mm_out == NULL) || (d0_mm_out == NULL) ||
        (theta_rad_out == NULL) || (m2_out == NULL) ||
        (z_rayleigh_mm_out == NULL))
    {
        return 0U;
    }

    if (n < M2_MIN_FIT_POINTS)
        return 0U;

    // Fit the squared D4sigma diameter with a quadratic in Z, then recover beam waist terms.
    for (uint32_t i = 0U; i < n; i++)
    {
        float z = z_mm[i];
        float y = d4sigma_mm[i] * d4sigma_mm[i];
        float z2 = z * z;

        s0 += 1.0f;
        s1 += z;
        s2 += z2;
        s3 += z2 * z;
        s4 += z2 * z2;

        t0 += y;
        t1 += z * y;
        t2 += z2 * y;
    }

    aug[0][0] = s4; aug[0][1] = s3; aug[0][2] = s2; aug[0][3] = t2;
    aug[1][0] = s3; aug[1][1] = s2; aug[1][2] = s1; aug[1][3] = t1;
    aug[2][0] = s2; aug[2][1] = s1; aug[2][2] = s0; aug[2][3] = t0;

    for (uint32_t col = 0U; col < 3U; col++)
    {
        uint32_t pivot = col;
        float max_abs = fabsf(aug[col][col]);

        for (uint32_t row = col + 1U; row < 3U; row++)
        {
            float v = fabsf(aug[row][col]);
            if (v > max_abs)
            {
                max_abs = v;
                pivot = row;
            }
        }

        if (max_abs <= 1.0e-9f)
            return 0U;

        if (pivot != col)
        {
            for (uint32_t k = col; k < 4U; k++)
            {
                float tmp = aug[col][k];
                aug[col][k] = aug[pivot][k];
                aug[pivot][k] = tmp;
            }
        }

        {
            float div = aug[col][col];
            for (uint32_t k = col; k < 4U; k++)
                aug[col][k] /= div;
        }

        for (uint32_t row = 0U; row < 3U; row++)
        {
            float factor;

            if (row == col)
                continue;

            factor = aug[row][col];
            if (factor == 0.0f)
                continue;

            for (uint32_t k = col; k < 4U; k++)
                aug[row][k] -= factor * aug[col][k];
        }
    }

    a = aug[0][3];
    b = aug[1][3];
    c = aug[2][3];

    if (a <= 0.0f)
        return 0U;

    z0_mm = -b / (2.0f * a);
    d0_sq_mm2 = c - (b * b) / (4.0f * a);
    if (d0_sq_mm2 <= 0.0f)
        return 0U;

    theta_rad = sqrtf(a);
    lambda_mm = wavelength_nm * 1.0e-6f;
    if ((theta_rad <= 0.0f) || (lambda_mm <= 0.0f))
        return 0U;

    *z0_mm_out = z0_mm;
    *d0_mm_out = sqrtf(d0_sq_mm2);
    *theta_rad_out = theta_rad;
    *m2_out = ((float)M_PI * (*d0_mm_out) * theta_rad) / (4.0f * lambda_mm);
    *z_rayleigh_mm_out = m2_compute_rayleigh_range_mm(*d0_mm_out, theta_rad);

    return 1U;
}

uint8_t Measurement_M2_Run(const M2MeasurementContext *ctx,
                           float step_mm,
                           float total_mm)
{
    uint32_t n_positions;

    if ((ctx == NULL) ||
        (ctx->busy_flag == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->z_home_simple == NULL) ||
        (ctx->home_pressed == NULL) ||
        (ctx->z_move_mm_dir == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->capture_axis_t0 == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    if ((step_mm <= 0.0f) || (total_mm <= 0.0f))
    {
        ctx->log_printf("M2_BAD_ARGS\r\n");
        return 0U;
    }

    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
    if ((n_positions == 0U) || (n_positions > M2_MAX_POSITIONS))
    {
        ctx->log_printf("M2_BAD_ARGS,positions=%lu,max=%u\r\n",
                        (unsigned long)n_positions,
                        (unsigned int)M2_MAX_POSITIONS);
        return 0U;
    }

    *ctx->busy_flag = 1U;

    ctx->log_printf("M2_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
                    step_mm,
                    total_mm,
                    (unsigned long)n_positions);

    ctx->z_home_simple(ctx->z_home_dir);
    if (!ctx->home_pressed())
    {
        ctx->log_printf("M2_HOME_FAIL\r\n");
        *ctx->busy_flag = 0U;
        return 0U;
    }

    ctx->delay_ms(ctx->z_settle_ms);

    for (uint32_t i = 0U; i < n_positions; i++)
    {
        if (*ctx->abort_flag)
        {
            ctx->log_printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                            *ctx->z_current_mm,
                            (unsigned long)(i + 1U));
            *ctx->busy_flag = 0U;
            return 0U;
        }

        // Coarse M2 scan: move, settle, then capture both axes at the stop.
        ctx->z_move_mm_dir(ctx->z_away_dir, step_mm);
        ctx->delay_ms(ctx->z_settle_ms);

        if (*ctx->abort_flag)
        {
            ctx->log_printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                            *ctx->z_current_mm,
                            (unsigned long)(i + 1U));
            *ctx->busy_flag = 0U;
            return 0U;
        }

        if (!m2_capture_both_axes_at_z(ctx, i + 1U))
        {
            ctx->log_printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                            *ctx->z_current_mm,
                            (unsigned long)(i + 1U));
            *ctx->busy_flag = 0U;
            return 0U;
        }
    }

    ctx->log_printf("M2_DONE,final_z_mm=%.3f\r\n", *ctx->z_current_mm);

    *ctx->busy_flag = 0U;
    return 1U;
}

uint8_t Measurement_M2_CaptureAtPosition(const M2MeasurementContext *ctx,
                                         float target_mm,
                                         uint32_t step_index)
{
    uint8_t ok;

    if ((ctx == NULL) ||
        (ctx->busy_flag == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->z_move_to_mm == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->capture_axis_t0 == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    if ((target_mm < 0.0f) || (target_mm > ctx->travel_max_mm))
    {
        ctx->log_printf("M2_POS_BAD_ARGS,target_z_mm=%.3f,max_z_mm=%.3f\r\n",
                        target_mm,
                        ctx->travel_max_mm);
        return 0U;
    }

    *ctx->busy_flag = 1U;

    ctx->log_printf("M2_TARGET_BEGIN,target_z_mm=%.3f,step=%lu\r\n",
                    target_mm,
                    (unsigned long)step_index);

    ctx->z_move_to_mm(target_mm);
    ctx->delay_ms(ctx->z_settle_ms);

    if (*ctx->abort_flag)
    {
        ctx->log_printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
        *ctx->busy_flag = 0U;
        return 0U;
    }

    ok = m2_capture_both_axes_at_z(ctx, step_index);

    if (*ctx->abort_flag)
    {
        ctx->log_printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
        *ctx->busy_flag = 0U;
        return 0U;
    }

    ctx->log_printf("M2_TARGET_DONE,target_z_mm=%.3f,actual_z_mm=%.3f,step=%lu,ok=%u\r\n",
                    target_mm,
                    *ctx->z_current_mm,
                    (unsigned long)step_index,
                    (unsigned int)ok);

    *ctx->busy_flag = 0U;
    return ok;
}

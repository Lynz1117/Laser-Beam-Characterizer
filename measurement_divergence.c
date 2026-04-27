#include "measurement_divergence.h"
#include <stddef.h>

static float divergence_plot_mm(const DivergenceMeasurementContext *ctx)
{
    if ((ctx == NULL) || (ctx->z_current_mm == NULL))
        return 0.0f;

    return (*ctx->z_current_mm) * ctx->z_display_scale;
}

static uint8_t divergence_run_axis_batch(const DivergenceMeasurementContext *ctx,
                                         MeasurementAxis axis,
                                         uint32_t step_index)
{
    if ((ctx == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->t0_batch_active == NULL) ||
        (ctx->t0_batch_total_axes == NULL) ||
        (ctx->t0_batch_begin == NULL) ||
        (ctx->t0_batch_update == NULL) ||
        (ctx->live_power_update_service == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    *ctx->t0_batch_total_axes = 1U;
    ctx->log_printf("DIV_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=%u\r\n",
                    divergence_plot_mm(ctx),
                    (unsigned long)step_index,
                    (unsigned int)axis);
    ctx->t0_batch_begin(axis, 10U);

    while (*ctx->t0_batch_active)
    {
        if (*ctx->abort_flag)
        {
            ctx->log_printf("DIVERGENCE_ABORT,z_mm=%.3f,step=%lu,axis=%u\r\n",
                            divergence_plot_mm(ctx),
                            (unsigned long)step_index,
                            (unsigned int)axis);
            return 0U;
        }

        ctx->t0_batch_update();
        ctx->live_power_update_service();
    }

    return 1U;
}

static uint8_t __attribute__((unused)) divergence_capture_both_axes_at_z(
    const DivergenceMeasurementContext *ctx,
    uint32_t step_index)
{
    uint8_t axis0_ok = 0U;
    uint8_t axis1_ok = 0U;
    uint32_t idx = 0U;
    int32_t t0_enc = 0;
    int32_t x0_counts = 0;
    float t0_ms = 0.0f;

    if ((ctx == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->capture_axis_t0 == NULL) ||
        (ctx->clear_active_axis_out_of_beam == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    ctx->log_printf("DIV_POS_BEGIN,z_mm=%.3f,step=%lu\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    // Capture both orthogonal profiles at the current Z position.
    ctx->log_printf("DIV_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=0\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M1, &idx, &t0_enc, &x0_counts, &t0_ms))
    {
        axis0_ok = 1U;
        ctx->log_printf("DIV_AXIS_DONE,z_mm=%.3f,step=%lu,axis=0,"
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
        ctx->log_printf("DIV_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=0\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
    }

    ctx->clear_active_axis_out_of_beam();
    ctx->delay_ms(ctx->z_settle_ms);

    ctx->log_printf("DIV_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=1\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index);

    if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M2, &idx, &t0_enc, &x0_counts, &t0_ms))
    {
        axis1_ok = 1U;
        ctx->log_printf("DIV_AXIS_DONE,z_mm=%.3f,step=%lu,axis=1,"
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
        ctx->log_printf("DIV_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=1\r\n",
                        *ctx->z_current_mm,
                        (unsigned long)step_index);
    }

    ctx->log_printf("DIV_POS_DONE,z_mm=%.3f,step=%lu,axis0_ok=%u,axis1_ok=%u\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)step_index,
                    (unsigned int)axis0_ok,
                    (unsigned int)axis1_ok);

    return (axis0_ok || axis1_ok) ? 1U : 0U;
}

uint8_t Measurement_Divergence_RunStepScan(const DivergenceMeasurementContext *ctx,
                                           float step_mm,
                                           uint32_t num_positions)
{
    if ((ctx == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->z_home_simple == NULL) ||
        (ctx->home_pressed == NULL) ||
        (ctx->z_move_mm_dir == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->capture_axis_t0 == NULL) ||
        (ctx->clear_active_axis_out_of_beam == NULL) ||
        (ctx->emit_raw_scan == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    if ((step_mm <= 0.0f) || (num_positions == 0U))
    {
        ctx->log_printf("STEP_DIV_BAD_ARGS,step_mm=%.3f,positions=%lu\r\n",
                        step_mm,
                        (unsigned long)num_positions);
        return 0U;
    }

    ctx->log_printf("STEP_DIV_BEGIN,step_mm=%.3f,positions=%lu\r\n",
                    step_mm,
                    (unsigned long)num_positions);

    ctx->z_home_simple(ctx->z_home_dir);
    if (!ctx->home_pressed())
    {
        ctx->log_printf("STEP_DIV_HOME_FAIL\r\n");
        ctx->log_printf("STEP_DIV_DONE,positions=0\r\n");
        return 0U;
    }

    ctx->delay_ms(ctx->z_settle_ms);

    for (uint32_t i = 0U; i < num_positions; i++)
    {
        uint32_t idx = 0U;
        int32_t t0_enc = 0;
        int32_t x0_counts = 0;
        float t0_ms = 0.0f;

        if (*ctx->abort_flag)
        {
            ctx->clear_active_axis_out_of_beam();
            ctx->z_stepper_disable();
            ctx->log_printf("STEP_DIV_ABORT,index=%lu,z_mm=%.3f\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm);
            ctx->log_printf("STEP_DIV_DONE,positions=%lu\r\n",
                            (unsigned long)i);
            return 0U;
        }

        // Step the Z stage, then capture and stream both profile axes.
        ctx->z_move_mm_dir(ctx->z_away_dir, step_mm);
        ctx->delay_ms(ctx->z_settle_ms);

        if (*ctx->abort_flag)
        {
            ctx->clear_active_axis_out_of_beam();
            ctx->z_stepper_disable();
            ctx->log_printf("STEP_DIV_ABORT,index=%lu,z_mm=%.3f\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm);
            ctx->log_printf("STEP_DIV_DONE,positions=%lu\r\n",
                            (unsigned long)i);
            return 0U;
        }

        ctx->log_printf("STEP_DIV_POS_BEGIN,index=%lu,z_mm=%.3f\r\n",
                        (unsigned long)(i + 1U),
                        *ctx->z_current_mm);

        ctx->log_printf("STEP_DIV_AXIS_BEGIN,index=%lu,z_mm=%.3f,axis=0\r\n",
                        (unsigned long)(i + 1U),
                        *ctx->z_current_mm);

        if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M1, &idx, &t0_enc, &x0_counts, &t0_ms))
        {
            ctx->log_printf("STEP_DIV_AXIS_RESULT,index=%lu,z_mm=%.3f,axis=0,"
                            "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm,
                            (unsigned long)idx,
                            (long)t0_enc,
                            (long)x0_counts,
                            t0_ms);
            ctx->emit_raw_scan(MEASUREMENT_AXIS_M1);
        }
        else
        {
            ctx->log_printf("STEP_DIV_AXIS_FAIL,index=%lu,z_mm=%.3f,axis=0\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm);
        }

        ctx->clear_active_axis_out_of_beam();
        ctx->delay_ms(ctx->z_settle_ms);

        if (*ctx->abort_flag)
        {
            ctx->clear_active_axis_out_of_beam();
            ctx->z_stepper_disable();
            ctx->log_printf("STEP_DIV_ABORT,index=%lu,z_mm=%.3f\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm);
            ctx->log_printf("STEP_DIV_DONE,positions=%lu\r\n",
                            (unsigned long)i);
            return 0U;
        }

        ctx->log_printf("STEP_DIV_AXIS_BEGIN,index=%lu,z_mm=%.3f,axis=1\r\n",
                        (unsigned long)(i + 1U),
                        *ctx->z_current_mm);

        if (ctx->capture_axis_t0(MEASUREMENT_AXIS_M2, &idx, &t0_enc, &x0_counts, &t0_ms))
        {
            ctx->log_printf("STEP_DIV_AXIS_RESULT,index=%lu,z_mm=%.3f,axis=1,"
                            "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm,
                            (unsigned long)idx,
                            (long)t0_enc,
                            (long)x0_counts,
                            t0_ms);
            ctx->emit_raw_scan(MEASUREMENT_AXIS_M2);
        }
        else
        {
            ctx->log_printf("STEP_DIV_AXIS_FAIL,index=%lu,z_mm=%.3f,axis=1\r\n",
                            (unsigned long)(i + 1U),
                            *ctx->z_current_mm);
        }

        ctx->clear_active_axis_out_of_beam();

        ctx->log_printf("STEP_DIV_POS_DONE,index=%lu,z_mm=%.3f\r\n",
                        (unsigned long)(i + 1U),
                        *ctx->z_current_mm);
    }

    ctx->log_printf("STEP_DIV_DONE,final_z_mm=%.3f,positions=%lu\r\n",
                    *ctx->z_current_mm,
                    (unsigned long)num_positions);

    return 1U;
}

uint8_t Measurement_Divergence_Run(const DivergenceMeasurementContext *ctx,
                                   float step_mm,
                                   float total_mm)
{
    uint32_t n_positions;
    uint32_t steps_per_position;

    if ((ctx == NULL) ||
        (ctx->busy_flag == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->t0_batch_active == NULL) ||
        (ctx->t0_batch_total_axes == NULL) ||
        (ctx->z_current_mm == NULL) ||
        (ctx->z_home_simple == NULL) ||
        (ctx->z_stepper_wake == NULL) ||
        (ctx->z_stepper_disable == NULL) ||
        (ctx->z_set_dir == NULL) ||
        (ctx->z_move_steps == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->t0_batch_begin == NULL) ||
        (ctx->t0_batch_update == NULL) ||
        (ctx->live_power_update_service == NULL) ||
        (ctx->log_printf == NULL))
    {
        return 0U;
    }

    if ((step_mm <= 0.0f) || (total_mm <= 0.0f))
    {
        ctx->log_printf("DIVERGENCE_BAD_ARGS\r\n");
        return 0U;
    }

    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
    steps_per_position = (uint32_t)(step_mm * ctx->z_steps_per_mm + 0.5f);

    *ctx->busy_flag = 1U;

    ctx->log_printf("DIVERGENCE_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
                    step_mm,
                    total_mm,
                    (unsigned long)n_positions);

    ctx->z_home_simple(ctx->z_home_dir);
    ctx->log_printf("ZHOME_DONE\r\n");

    // Keep the Z stepper awake during the full scan and re-arm it after each profile batch.
    ctx->z_stepper_wake();
    ctx->delay_ms(5U);
    ctx->z_set_dir(ctx->z_away_dir);
    ctx->delay_ms(5U);

    for (uint32_t i = 0U; i < n_positions; i++)
    {
        if (*ctx->abort_flag)
        {
            ctx->log_printf("DIVERGENCE_ABORT,z_mm=%.3f,step=%lu\r\n",
                            divergence_plot_mm(ctx),
                            (unsigned long)(i + 1U));
            ctx->z_stepper_disable();
            *ctx->busy_flag = 0U;
            return 0U;
        }

        ctx->z_move_steps(steps_per_position);
        *ctx->z_current_mm += step_mm;

        ctx->log_printf("Z_MOVE_DONE,step=%lu,z_mm=%.3f\r\n",
                        (unsigned long)(i + 1U),
                        divergence_plot_mm(ctx));

        ctx->delay_ms(ctx->z_settle_ms);

        // Run each axis as its own single-axis batch. This preserves the same
        // 10-scan workflow while avoiding the fragile in-batch axis switch.
        if (!divergence_run_axis_batch(ctx, MEASUREMENT_AXIS_M1, i + 1U))
        {
            ctx->z_stepper_disable();
            *ctx->busy_flag = 0U;
            return 0U;
        }

        ctx->delay_ms(ctx->z_settle_ms);

        if (!divergence_run_axis_batch(ctx, MEASUREMENT_AXIS_M2, i + 1U))
        {
            ctx->z_stepper_disable();
            *ctx->busy_flag = 0U;
            return 0U;
        }

        ctx->delay_ms(100U);

        ctx->log_printf("DIV_POS_DONE,z_mm=%.3f,step=%lu\r\n",
                        divergence_plot_mm(ctx),
                        (unsigned long)(i + 1U));

        ctx->z_stepper_wake();
        ctx->delay_ms(5U);
        ctx->z_set_dir(ctx->z_away_dir);
        ctx->delay_ms(5U);
    }

    ctx->log_printf("DIVERGENCE_DONE,final_z_mm=%.3f\r\n", divergence_plot_mm(ctx));

    ctx->z_stepper_disable();
    *ctx->busy_flag = 0U;
    return 1U;
}

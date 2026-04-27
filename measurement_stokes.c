#include "measurement_stokes.h"
#include <stddef.h>

void Measurement_Stokes_Run(const StokesMeasurementContext *ctx)
{
    if ((ctx == NULL) ||
        (ctx->abort_flag == NULL) ||
        (ctx->samples_raw == NULL) ||
        (ctx->samples_v == NULL) ||
        (ctx->samples_mw == NULL) ||
        (ctx->samples_stage == NULL) ||
        (ctx->stepper_wake == NULL) ||
        (ctx->set_dir == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->move_steps == NULL) ||
        (ctx->capture_autorange == NULL) ||
        (ctx->adc_counts_to_voltage == NULL) ||
        (ctx->voltage_to_power_mw == NULL) ||
        (ctx->log_printf == NULL))
    {
        return;
    }

    ctx->stepper_wake();
    ctx->set_dir(1U);
    ctx->delay_ms(2U);

    for (uint32_t k = 0U; k < ctx->num_samples; k++)
    {
        if (*ctx->abort_flag)
        {
            ctx->log_printf("STOKES_ABORT,index=%lu\r\n", (unsigned long)k);
            return;
        }

        if (k > 0U)
            ctx->move_steps(ctx->steps_per_increment);

        ctx->delay_ms(ctx->settle_ms);

        if (*ctx->abort_flag)
        {
            ctx->log_printf("STOKES_ABORT,index=%lu\r\n", (unsigned long)k);
            return;
        }

        ctx->samples_raw[k] = ctx->capture_autorange(&ctx->samples_stage[k]);
        ctx->samples_v[k] = ctx->adc_counts_to_voltage((float)ctx->samples_raw[k]);
        ctx->samples_mw[k] = ctx->voltage_to_power_mw(ctx->samples_v[k]);

        ctx->log_printf("I%lu stage=%u ADC=%u V=%.4f P=%.4f mW\r\n",
                        (unsigned long)k,
                        (unsigned int)ctx->samples_stage[k],
                        (unsigned int)ctx->samples_raw[k],
                        ctx->samples_v[k],
                        ctx->samples_mw[k]);
    }

    ctx->log_printf("STOKES_MW ");
    for (uint32_t i = 0U; i < ctx->num_samples; i++)
        ctx->log_printf("%.4f ", ctx->samples_mw[i]);

    ctx->log_printf("\r\n");
    ctx->log_printf("STOKES_DONE\r\n");
}

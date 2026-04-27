#include "measurement_wavelength.h"
#include <stddef.h>
#include <string.h>

static void wl_send_raw_packet(WavelengthMeasurementContext *ctx,
                               uint32_t window_id,
                               uint32_t chunk_id,
                               const uint16_t *samples,
                               uint32_t n)
{
    uint8_t header[16];
    uint16_t payload_len;

    if ((ctx == NULL) ||
        (samples == NULL) ||
        (n == 0U) ||
        (ctx->enqueue_blocking == NULL) ||
        (ctx->tx_kick == NULL))
    {
        return;
    }

    payload_len = (uint16_t)(n * sizeof(uint16_t));

    header[0] = 0xA5U;
    header[1] = 0x5AU;
    header[2] = 0x01U;
    header[3] = 0x00U;

    memcpy(&header[4], &window_id, sizeof(window_id));
    memcpy(&header[8], &chunk_id, sizeof(chunk_id));
    memcpy(&header[12], &n, sizeof(n));

    if (ctx->enqueue_blocking(header, (uint16_t)sizeof(header)) != sizeof(header))
        return;

    if (ctx->enqueue_blocking((const uint8_t *)samples, payload_len) != payload_len)
        return;

    ctx->tx_kick();
}

static void wl_process_dma_for_window(WavelengthMeasurementContext *ctx,
                                      uint32_t window_id,
                                      uint32_t *chunk_id,
                                      uint32_t *samples_this_window)
{
    uint32_t remaining;
    uint32_t n_send;

    if ((ctx == NULL) || (chunk_id == NULL) || (samples_this_window == NULL))
        return;

    if (*samples_this_window >= ctx->wl_window_samples)
        return;

    if (*ctx->fft_chunk_half_ready)
    {
        *ctx->fft_chunk_half_ready = 0U;

        remaining = ctx->wl_window_samples - *samples_this_window;
        n_send = (remaining >= ctx->fft_chunk_half) ? ctx->fft_chunk_half : remaining;

        wl_send_raw_packet(ctx, window_id, *chunk_id, &ctx->chunk_buf[0], n_send);

        (*chunk_id)++;
        *samples_this_window += n_send;
    }

    if (*samples_this_window >= ctx->wl_window_samples)
        return;

    if (*ctx->fft_chunk_full_ready)
    {
        *ctx->fft_chunk_full_ready = 0U;

        remaining = ctx->wl_window_samples - *samples_this_window;
        n_send = (remaining >= ctx->fft_chunk_half) ? ctx->fft_chunk_half : remaining;

        wl_send_raw_packet(ctx, window_id, *chunk_id, &ctx->chunk_buf[ctx->fft_chunk_half], n_send);

        (*chunk_id)++;
        *samples_this_window += n_send;
    }
}

void Measurement_Wavelength_Run(WavelengthMeasurementContext *ctx)
{
    uint32_t window_id = 0U;
    uint32_t step_period_us;

    if ((ctx == NULL) ||
        (ctx->hadc == NULL) ||
        (ctx->sample_timer == NULL) ||
        (ctx->fft_busy == NULL) ||
        (ctx->fft_abort_flag == NULL) ||
        (ctx->fft_chunk_half_ready == NULL) ||
        (ctx->fft_chunk_full_ready == NULL) ||
        (ctx->chunk_buf == NULL) ||
        (ctx->stepper_wake == NULL) ||
        (ctx->stepper_sleep_disable == NULL) ||
        (ctx->set_dir == NULL) ||
        (ctx->step_pulse == NULL) ||
        (ctx->delay_us == NULL) ||
        (ctx->delay_ms == NULL) ||
        (ctx->step_period_us == NULL) ||
        (ctx->get_tick == NULL) ||
        (ctx->tx_kick == NULL) ||
        (ctx->log_printf == NULL))
    {
        return;
    }

    step_period_us = ctx->step_period_us();

    *ctx->fft_busy = 1U;
    *ctx->fft_abort_flag = 0U;

    ctx->log_printf("WL_RAW_BEGIN,fs_hz=%.1f,window_samples=%lu,window_s=%.6f,"
                    "speed_mm_s=%.3f,runs=%lu\r\n",
                    ctx->fft_adc_fs_hz,
                    (unsigned long)ctx->wl_window_samples,
                    ctx->wl_window_time_s,
                    ctx->step3_speed_mm_s,
                    (unsigned long)ctx->wl_num_runs);

    ctx->stepper_wake();
    ctx->delay_ms(10U);

    for (uint32_t run = 0U; run < ctx->wl_num_runs; run++)
    {
        uint32_t chunk_id = 0U;
        uint32_t samples_this_window = 0U;
        uint32_t steps_taken = 0U;
        uint8_t dir = (run % 2U) ? 0U : 1U;
        uint32_t scan_start_ms;

        ctx->set_dir(dir);
        ctx->delay_ms(10U);

        *ctx->fft_chunk_half_ready = 0U;
        *ctx->fft_chunk_full_ready = 0U;

        ctx->log_printf("WL_WINDOW_BEGIN,window=%lu,run=%lu,dir=%u,target_samples=%lu\r\n",
                        (unsigned long)window_id,
                        (unsigned long)(run + 1U),
                        (unsigned int)dir,
                        (unsigned long)ctx->wl_window_samples);

        if (HAL_ADC_Start_DMA(ctx->hadc, (uint32_t *)ctx->chunk_buf, ctx->fft_chunk_size) != HAL_OK)
        {
            ctx->log_printf("WL_ERR,adc_start_failed\r\n");
            goto wl_done;
        }

        if (HAL_TIM_Base_Start(ctx->sample_timer) != HAL_OK)
        {
            ctx->log_printf("WL_ERR,tim6_start_failed\r\n");
            HAL_ADC_Stop_DMA(ctx->hadc);
            goto wl_done;
        }

        scan_start_ms = ctx->get_tick();

        while (samples_this_window < ctx->wl_window_samples)
        {
            if (*ctx->fft_abort_flag)
                goto wl_done;

            ctx->step_pulse();
            ctx->delay_us(step_period_us);
            steps_taken++;

            wl_process_dma_for_window(ctx, window_id, &chunk_id, &samples_this_window);

            if ((ctx->get_tick() - scan_start_ms) > 1500U)
            {
                ctx->log_printf("WL_WINDOW_TIMEOUT,window=%lu,samples=%lu,chunks=%lu,steps=%lu\r\n",
                                (unsigned long)window_id,
                                (unsigned long)samples_this_window,
                                (unsigned long)chunk_id,
                                (unsigned long)steps_taken);
                break;
            }
        }

        HAL_TIM_Base_Stop(ctx->sample_timer);
        HAL_ADC_Stop_DMA(ctx->hadc);

        wl_process_dma_for_window(ctx, window_id, &chunk_id, &samples_this_window);
        ctx->tx_kick();

        ctx->log_printf("WL_WINDOW_END,window=%lu,samples=%lu,chunks=%lu,steps=%lu,dir=%u\r\n",
                        (unsigned long)window_id,
                        (unsigned long)samples_this_window,
                        (unsigned long)chunk_id,
                        (unsigned long)steps_taken,
                        (unsigned int)dir);

        window_id++;
        ctx->delay_ms(100U);
    }

wl_done:
    HAL_TIM_Base_Stop(ctx->sample_timer);
    HAL_ADC_Stop_DMA(ctx->hadc);

    ctx->stepper_sleep_disable();
    ctx->log_printf("WL_DONE,total_windows=%lu\r\n", (unsigned long)window_id);
    ctx->tx_kick();
    *ctx->fft_busy = 0U;
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "usbd_cdc_if.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Stages
#define R1 1.455 * 1000
#define R2 13.38 * 1000
#define R3 133.64 * 1000
#define R4 1213.38 * 1000
#define GAIN_SWITCH_HIGH_V   3.0f
#define GAIN_SWITCH_LOW_V    0.3f
#define GAIN_SETTLE_MS       5
#define MAX_GAIN_ADJUST_TRIES 4

//Microstepping defines
#define Z_M0_GPIO_Port GPIOB
#define Z_M0_Pin       GPIO_PIN_0

#define Z_M1_GPIO_Port GPIOB
#define Z_M1_Pin       GPIO_PIN_1

#define Z_M2_GPIO_Port GPIOB
#define Z_M2_Pin       GPIO_PIN_2

// Stepper defines
#define DIR_GPIO_Port  GPIOA
#define DIR_Pin        GPIO_PIN_0

#define STEP_GPIO_Port GPIOA
#define STEP_Pin       GPIO_PIN_1

#define SLEEP_GPIO_Port GPIOA
#define SLEEP_Pin       GPIO_PIN_6

// Divergence Stepper (z-stage)
#define Z_DIR_GPIO_Port    GPIOA
#define Z_DIR_Pin		   GPIO_PIN_4

#define Z_STEP_GPIO_Port   GPIOA
#define Z_STEP_Pin		   GPIO_PIN_5

 //STEPPER3
#define DIR2_GPIO_Port  GPIOC
#define DIR2_Pin        GPIO_PIN_12

#define STEP2_GPIO_Port GPIOB
#define STEP2_Pin       GPIO_PIN_10

// Homing Button
#define BUTTON_GPIO_Port GPIOA
#define BUTTON_Pin		 GPIO_PIN_13 // active-high button

// Divergence defines
#define Z_STEP_DELAY_MS          200U
#define Z_STEP_DELAY_US			 700U
#define Z_SETTLE_MS              150U

#define Z_STEPS_PER_REV          200.0f
#define Z_MICROSTEP_DIV          1.0f
#define Z_LEADSCREW_PITCH_MM     1.0f      // <-- change to your real leadscrew pitch
#define Z_STEPS_PER_MM           ((Z_STEPS_PER_REV * Z_MICROSTEP_DIV) / Z_LEADSCREW_PITCH_MM)

#define Z_HOME_DIR               1U        // change if motor moves wrong way toward switch
#define Z_AWAY_DIR               0U

#define Z_HOME_MAX_STEPS         50000U    // safety limit
#define Z_HOME_BACKOFF_STEPS     1000U
#define Z_HOME_REAPPROACH_DELAY_MS 2U

// =============================
// Step-divergence scan settings
// =============================
#define STEP_DIV_STEP_MM         5.0f    // mm per Z move
#define STEP_DIV_NUM_POSITIONS   1U     // total positions after home
#define STEP_DIV_CMD_STR         "STEPDIV"

// =============================
// Divergence settings
// =============================
#define DIVERGENCE_STEP_MM        10.0f
#define DIVERGENCE_TOTAL_MM       100.0f
#define DIVERGENCE_NUM_POSITIONS  ((uint32_t)(DIVERGENCE_TOTAL_MM / DIVERGENCE_STEP_MM))

// =============================
// M2 settings
// =============================
#define M2_TOTAL_MM               300.0f
#define M2_COARSE_POINTS          10U
#define M2_STEP_MM                (M2_TOTAL_MM / (float)M2_COARSE_POINTS)
#define M2_MAX_POSITIONS          64U
#define M2_ISO_TARGET_POINTS      10U
#define M2_ISO_POINTS_INSIDE      5U
#define M2_ISO_POINTS_OUTSIDE     5U
#define M2_MIN_FIT_POINTS         3U
#define M2_WAVELENGTH_NM          532.0f   // update if your laser wavelength differs

// Stokes scan defines
#define NUM_STOKES_SAMPLES      40
#define ADC_SAMPLES_PER_POINT   32

#define MOTOR_FULL_STEPS_PER_REV  800
#define MICROSTEP_DIV             1

#define MOTOR_STEPS_PER_REV     (MOTOR_FULL_STEPS_PER_REV * MICROSTEP_DIV)
#define STEPS_FOR_180_DEG       (MOTOR_STEPS_PER_REV / 2)
#define STEPS_PER_INCREMENT     (STEPS_FOR_180_DEG / NUM_STOKES_SAMPLES)

#define STEP_DELAY_MS           5
#define SETTLE_MS               200

// ADC DMA buffer
#define ADC_BUF_LEN             1024
#define ADC_HALF_LEN            (ADC_BUF_LEN / 2)

// ADC conversion
#define ADC_VREF                2.9f
#define ADC_MAX_COUNTS          4095.0f
#define PM_VOLTAGE_OFFSET_V     (0.0f)

// DC Motor / Encoder
#define DC_PWM_MAX              99U
#define DC_SPEED_SAMPLE_MS      100U

#define ENC1_COUNTS_PER_REV     28000.0f
#define ENC2_COUNTS_PER_REV     28000.0f

// Counts-based auto-correct for motor speed
#define TARGET_COUNTS           475.0f
#define MAX_COUNTS              5000.0f
#define KP_COUNTS               (100.0f / MAX_COUNTS)

//// Beam profiling geometry
//#define APERTURE_MM             5.0f
#define ARM_LENGTH_MM           40.0f
//#define TS_SCALE                1.2f
//#define NUM_PROFILE_SCANS       10

// Beam profiling timing
#define PROFILE_TARGET_RPM      10.0f
#define PROFILE_SPINUP_MS       300U
#define PROFILE_THETA0_RAD      ((float)M_PI / 2.0f)   // initial scan angle
#define PROFILE_T90_RAD         ((float)M_PI / 2.0f)   // 90 degree reference
#define PROFILE_BLADE_CROSS_RAD 0.1667f                // blade crossing angle
#define TS_SCALE                1.2f
#define NUM_PROFILE_SCANS       10

#define PROFILE_CLEAR_CONFIRM_COUNT   50U
#define PROFILE_CLEAR_TIMEOUT_MS      5000U
#define PROFILE_CLEAR_THRESH_FRAC     0.55f
#define PROFILE_CLEAR_THRESHOLD_MW	  0.05f

#define PROFILE_RPM_TOLERANCE_RPM      0.45f
#define PROFILE_RPM_STABLE_COUNT_REQ   3U
#define PROFILE_STABILIZE_TIMEOUT_MS   4000U
#define PROFILE_MIN_VALID_RPM          1.0f
#define PROFILE_START_DUTY             50U

#define T0_SG_WIN              5U
#define T0_FIT_MAX_ITERS       6U
#define T0_FIT_X0_STEP_INIT    80.0f
#define T0_FIT_W_STEP_INIT     60.0f
#define T0_FIT_W_MIN           2.0f

#define PROFILE_INIT_COUNTS     7778
#define PROFILE_REPEAT_COUNTS    745
#define PROFILE_EVENT_COUNTS    7778
#define PROFILE_WAIT_COUNTS    (PROFILE_EVENT_COUNTS - PROFILE_REPEAT_COUNTS)   // 6255
#define T0_BATCH_MAX_ATTEMPTS  60U   // add near other defines at top
#define PROFILE_EDGE_MARGIN_DEG	5.0f // degrees to exclude at each edge

#define PROFILE_CAPTURE_MAX_SAMPLES   10000U
#define PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS     10U
#define PROFILE_REPEAT_BUFFER_MAX_SCANS            (PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS + 1U)
#define PROFILE_REPEAT_BUFFER_MAX_POINTS_PER_SCAN  224U
#define PROFILE_REPEAT_BUFFER_TOTAL_POINTS         (PROFILE_REPEAT_BUFFER_MAX_SCANS * PROFILE_REPEAT_BUFFER_MAX_POINTS_PER_SCAN)

// ADC sample rate used for profile timestamps
#define ADC_FS_HZ               10000.0f
#define ADC_DT_S                (1.0f / ADC_FS_HZ)

// Wavelength ADC
#define FFT_ADC_FS_HZ			200000.0f
#define FFT_SAMPLES				4096U 	// power of 2

/* Chunk-based streaming FFT */
#define FFT_CHUNK_SIZE      4096U
#define FFT_CHUNK_HALF      (FFT_CHUNK_SIZE / 2U)

#define WL_WINDOW_SAMPLES      60000U
#define WL_WINDOW_HALF_CHUNKS  ((WL_WINDOW_SAMPLES + FFT_CHUNK_HALF - 1U) / FFT_CHUNK_HALF)

// NEW WAVELENGTH DEFINES
#define WL_RAW_MAGIC0              0xAA
#define WL_RAW_MAGIC1              0x55
#define WL_RAW_PACKET_TYPE_ADC     0x01

#define WL_WINDOW_SAMPLES          60000U
#define WL_WINDOW_TIME_S           ((float)WL_WINDOW_SAMPLES / FFT_ADC_FS_HZ)
#define WL_WALL_GUARD_S            WL_WINDOW_TIME_S

#define WL_USB_FLUSH_DELAY_MS      500U
/* Optical conversion
   Whiteboard shows lambda = 2*v/favg.
   If your final setup is one-pass, change to 1.0f.
*/
#define WL_PATH_FACTOR              2.0f

/* Stepper 3 motion model */
#define STEP3_FULL_STEPS_PER_REV    200.0f
#define STEP3_MICROSTEP_DIV         1.0f     // set to your actual microstep setting
#define STEP3_LEADSCREW_PITCH_MM    1.0f
#define STEP3_SPEED_MM_S            7.5f

#define STEP3_STEPS_PER_MM \
    ((STEP3_FULL_STEPS_PER_REV * STEP3_MICROSTEP_DIV) / STEP3_LEADSCREW_PITCH_MM)

#define STEP3_STEP_RATE_HZ          (STEP3_SPEED_MM_S * STEP3_STEPS_PER_MM)
#define STEP3_STEP_PERIOD_US        ((uint32_t)((1000000.0f / STEP3_STEP_RATE_HZ) + 0.5f))

#define WL_LIMIT_MM					20.0f // turnaround distance
#define WL_NUM_RUNS					10U

/* Safety timeout for one FFT capture */
#define FFT_CAPTURE_TIMEOUT_MS      7000U

/* Optional: discard very low FFT bins near DC */
#define FFT_MIN_BIN                 2U


// Synthetic raw-profile mode for host-driven beam-profile testing.
// Motors and encoders remain real; RAW_SCAN power is synthesized from x_mm.
#define PROFILE_SYNTHETIC_RAW_ENABLE      0U
#define PROFILE_SYNTH_REPLAY_ENABLE       0U
#define PROFILE_SYNTH_BASELINE_MW         20.0f
#define PROFILE_SYNTH_AMPLITUDE_MW        520.0f
#define PROFILE_SYNTH_INIT_SIGMA_MM       1.40f
#define PROFILE_SYNTH_REPEAT_SIGMA_MM     1.00f
#define PROFILE_SYNTH_INIT_CENTER_FRAC    0.32f
#define PROFILE_SYNTH_REPEAT_CENTER_FRAC  0.50f
#define PROFILE_SYNTH_INIT_NOISE_FRAC     0.000f
#define PROFILE_SYNTH_REPEAT_NOISE_FRAC   0.000f
#define PROFILE_SYNTH_POWER_FULL_SCALE_MW 600.0f
#define PROFILE_SYNTH_REPLAY_LUT_LEN      64U
#if PROFILE_SYNTHETIC_RAW_ENABLE
#define PROFILE_CAPTURE_STORE_STRIDE      3U
#define PROFILE_HOST_EMIT_STRIDE          2U
#else
#define PROFILE_CAPTURE_STORE_STRIDE      1U
#define PROFILE_HOST_EMIT_STRIDE          1U
#endif

// Beam-profile output decimation
#define PROFILE_MAX_POINTS      1280
#define PROFILE_DECIM           8

#define STEP_PROFILE_NUM_STEPS       200U      // steps per Z move  <-- change here
#define STEP_PROFILE_NUM_POSITIONS   10U       // total positions   <-- change here
#define STEP_PROFILE_AXIS            PROFILE_AXIS_M1  // which axis

#define PROFILE_WORK_MAX_SAMPLES 	6000U
#define T0_DUMP_HALF_WINDOW			1250U

// Commands
#define PROFILE_CMD_STR         "PROFILE"
#define STOKES_CMD_STR          "STOKES"
#define ABORT_CMD_STR           "ABORT"
#define T0_M1_CMD_STR           "T0M1"
#define T0_M2_CMD_STR           "T0M2"
#define T0RAW_M1_CMD_STR		"T0RAWM1"
#define T0RAW_M2_CMD_STR		"T0RAWM2"
#define HOME_STEPPER			"HOME"
#define DIVERGENCE_CMD_STR		"DIVERGENCE"
#define M2_CMD_STR               "M2"
#define M2_POS_CMD_PREFIX        "M2POS,"
#define WAVELENGTH_CMD_STR		"Wavelength"
#define STEP_PROFILE_CMD_STR	"STEPPROFILE"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
typedef enum
{
    T0_BATCH_IDLE = 0,
    T0_BATCH_WAIT_STABLE,
    T0_BATCH_CAPTURE,
    T0_BATCH_COMPUTE,
    T0_BATCH_DUMP,
    T0_BATCH_NEXT,
	T0_BATCH_CLEAR_BEAM,
	T0_BATCH_SWITCH_AXIS,
    T0_BATCH_DONE
} T0BatchState_t;

typedef enum
{
    HOST_IDLE = 0,
    HOST_WAIT_STABLE,
    HOST_CAPTURE_INIT,
    HOST_CAPTURE_REPEAT,
    HOST_CLEAR_AXIS1,
    HOST_CLEAR_AXIS2,
    HOST_DUMP_REPEAT,
    HOST_DONE
} HostProfileState_t;

typedef enum
{
    PROFILE_AXIS_M1 = 0,
    PROFILE_AXIS_M2 = 1
} ProfileAxis_t;

typedef enum
{
    PROFILE_IDLE = 0,
    PROFILE_INITIAL_SCAN,
    PROFILE_FIND_T0,
    PROFILE_WAIT_END_90,
    PROFILE_WAIT_NEXT_SCAN,
    PROFILE_CAPTURE_SCAN,
	PROFILE_CLEAR_BEAM_POWER,
    PROFILE_DONE
} ProfileState_t;

typedef struct
{
    ProfileState_t state;
    uint8_t active;
    uint8_t scan_count;

    uint32_t state_start_ms;
    uint32_t wait_ms;
    uint32_t scan_time_ms;


    // NEW: fully position-based profiling
    int32_t scan_start_enc_count;
    int32_t scan_end_enc_count;
    int32_t scan_stop_enc_target;

    int32_t t0_enc_count;
    int32_t x0_counts;
    int32_t xdelay_counts;
    int32_t next_scan_enc_target;

    float omega_rad_s;
    float theta_scan_rad;
    float t_cross_s;
    float ts_init_s; // long initial T0-finding scan
    float ts_repeat_s; // short repeated scans
    float t90_s;
    float t_wait_s;
    float t_wait_first_s;
    uint32_t active_window_ms;
    uint32_t window_start_idx; // DMA write index when this scan window starts



    uint32_t t0_idx;
    float t0_s;

    ProfileAxis_t axis;			// current profiling axis
    uint8_t axis_sequence_done; // 0 = still need second axis, 1 = all done
    float p_min_init_mw;
    float p_max_init_mw;
    float p_clear_thresh_mw;
    uint8_t clear_confirm_count;
} BeamProfile_t;

typedef struct
{
    TIM_HandleTypeDef *enc_tim;
    uint32_t pwm_channel;
    int32_t prev_count;
    int32_t delta_count;
    float rpm;
    float omega;   // rad/s
    uint16_t duty;
} DCMotor_t;

typedef enum
{
    GAIN_STAGE_1 = 1,
    GAIN_STAGE_2 = 2,
    GAIN_STAGE_3 = 3,
    GAIN_STAGE_4 = 4
} GainStage_t;

COM_InitTypeDef BspCOMInit;

BeamProfile_t profiler = {0};

DCMotor_t dc1;
DCMotor_t dc2;
uint32_t last_dc_speed_ms = 0;

// Gain stage
volatile GainStage_t current_gain = GAIN_STAGE_1;
float latest_v_meas = 0.0f;
float latest_p_mw = 0.0f;
uint16_t latest_raw_meas = 0U;
volatile uint8_t gain_settle_skip_live = 0;

// Continuous ADC DMA buffer
uint16_t adc_buf[ADC_BUF_LEN];

// Beam-profile processed window buffers
float    profile_power_mw[PROFILE_MAX_POINTS];
float    profile_v_avg[PROFILE_MAX_POINTS];
uint16_t profile_raw_avg[PROFILE_MAX_POINTS];
uint8_t  profile_stage_avg[PROFILE_MAX_POINTS];
float    profile_t_ms[PROFILE_MAX_POINTS];
float    profile_deg[PROFILE_MAX_POINTS];
uint32_t profile_point_count = 0;
volatile uint8_t gain_settle_skip = 0;

// NEW
static volatile T0BatchState_t t0_batch_state = T0_BATCH_IDLE;

static uint8_t  t0_batch_active = 0U;
static uint8_t  t0_batch_axis = 0U;
static uint8_t  t0_batch_num_scans = 10U;
static uint8_t  t0_batch_scan_index = 0U;

static uint32_t t0_dump_emit_stride = 8U;
static uint32_t t0_dump_idx = 0U;
static uint8_t  t0_dump_started = 0U;
static uint8_t  t0_batch_valid_scans = 0U;

//static uint8_t  t0_batch_target_valid_scans = 10U;
//static uint8_t  t0_batch_valid_scans = 0U;
//static uint8_t  t0_batch_attempt_count = 0U;
//static uint8_t  t0_batch_emit_scan_no = 0U;   // valid scan number sent to host

static uint32_t t0_result_idx[10];
static int32_t  t0_result_enc[10];
static int32_t  t0_result_x0_counts[10];
static float    t0_result_ms[10];

//NEW END

/* =========================
   Wavelength / FFT globals
   ========================= */
volatile uint8_t start_fft_flag = 0U;
volatile uint8_t fft_busy = 0U;
volatile uint8_t fft_abort_flag = 0U;
volatile uint8_t fft_dma_done = 0U;
static uint32_t chunks_this_run = 0U;

static uint16_t fft_adc_buf[FFT_SAMPLES];
static float    fft_time_buf[FFT_SAMPLES];
static float    fft_out_buf[FFT_SAMPLES];
static float    fft_mag_accum[FFT_SAMPLES / 2U];

static uint16_t          fft_chunk_buf[FFT_CHUNK_SIZE];
static uint16_t          fft_last_chunk_snap[FFT_CHUNK_HALF];
volatile uint8_t         fft_chunk_half_ready  = 0U;
volatile uint8_t         fft_chunk_full_ready  = 0U;
volatile uint32_t        fft_chunks_processed  = 0U;

// NEW
static void wl_send_raw_packet(uint32_t window_id,
                               uint32_t chunk_id,
                               const uint16_t *samples,
                               uint32_t n);

static void wl_process_dma_for_window(uint32_t window_id,
                                      uint32_t *chunk_id,
                                      uint32_t *samples_this_window);
// END

static arm_rfft_fast_instance_f32 fft_inst;


volatile uint8_t profile_rpm_stable_count = 0U;
volatile float   profile_locked_omega_rad_s = 0.0f;
volatile float   profile_locked_rpm = 0.0f;

static volatile HostProfileState_t host_profile_state = HOST_IDLE;

static volatile uint8_t host_cmd_start_axis0 = 0U;
static volatile uint8_t host_cmd_start_axis1 = 0U;
static volatile uint8_t host_cmd_run_repeat_seq = 0U;
static volatile uint8_t host_cmd_abort_profile = 0U;
static volatile uint8_t host_local_t0_ready = 0U;

static int32_t host_first_wait_counts = 0;
static uint32_t host_repeat_num_scans = 10U;
static uint32_t host_repeat_scan_width_counts = 745U;
static int32_t host_repeat_wait_counts = 6255;

static uint32_t profile_last_adc_idx = 0U;
static uint32_t host_repeat_scan_index = 0U;

static int32_t host_next_scan_target = 0;

static uint16_t profile_raw_power_buf[PROFILE_CAPTURE_MAX_SAMPLES];
static int32_t  profile_raw_enc_buf[PROFILE_CAPTURE_MAX_SAMPLES];
static uint32_t profile_raw_count = 0U;
static uint8_t  profile_raw_stage_buf[PROFILE_CAPTURE_MAX_SAMPLES];
static uint32_t profile_capture_stride_counter = 0U;
static uint16_t host_repeat_buf_start[PROFILE_REPEAT_BUFFER_MAX_SCANS];
static uint16_t host_repeat_buf_count[PROFILE_REPEAT_BUFFER_MAX_SCANS];
static uint16_t host_repeat_buf_emit_stride[PROFILE_REPEAT_BUFFER_MAX_SCANS];
static int32_t  host_repeat_buf_scan_start_enc[PROFILE_REPEAT_BUFFER_MAX_SCANS];
static int32_t  host_repeat_buf_scan_end_enc[PROFILE_REPEAT_BUFFER_MAX_SCANS];
static int32_t  host_repeat_buf_enc[PROFILE_REPEAT_BUFFER_TOTAL_POINTS];
static uint16_t host_repeat_buf_raw[PROFILE_REPEAT_BUFFER_TOTAL_POINTS];
static uint8_t  host_repeat_buf_stage[PROFILE_REPEAT_BUFFER_TOTAL_POINTS];
static uint16_t host_repeat_buf_used = 0U;
static uint8_t  host_repeat_buf_valid_scans = 0U;

static void beam_profile_sample_position_window(void);
static uint8_t beam_profile_window_done_counts(void);
static void beam_profile_start_window_counts(int32_t span_counts);

// One shared float scratch buffer for host-side optional local preprocessing if needed
static float profile_work_buf[PROFILE_WORK_MAX_SAMPLES];
static void t0_emit_raw_scan(ProfileAxis_t axis);
static void host_profile_learn_clear_threshold_from_initial_scan(void);
static void clear_active_axis_out_of_beam(void);

volatile uint8_t start_step_profile_flag = 0U;

uint32_t t0_raw_count = 0U;
static uint32_t t0_dump_win_end = 0U;

// Stokes buffers
uint16_t stokes_I[NUM_STOKES_SAMPLES];
float    stokes_V[NUM_STOKES_SAMPLES];
float    stokes_mW[NUM_STOKES_SAMPLES];
uint8_t  stokes_stage[NUM_STOKES_SAMPLES];

volatile uint8_t adc_dma_done = 0;
volatile uint8_t burst_capture_active = 0;

volatile uint8_t adc_half_ready = 0;
volatile uint8_t adc_full_ready = 0;

// Divergence z-stage helpers
static inline void z_set_dir(uint8_t dir);
static uint8_t home_pressed(void);
static void z_move_steps_dir(uint8_t dir, uint32_t n);
static void z_move_mm(float mm);
static float z_current_mm = 0.0f;
volatile uint8_t start_divergence_flag = 0;
volatile uint8_t start_m2_flag = 0;
volatile uint8_t start_m2_position_flag = 0U;
volatile uint8_t divergence_busy = 0U;
static void z_move_mm_dir(uint8_t dir, float mm);
static uint8_t divergence_capture_both_axes_at_z(uint32_t step_index);
static uint8_t m2_capture_both_axes_at_z(uint32_t step_index);
static uint8_t run_divergence_measurement(float step_mm, float total_mm);
static uint8_t run_m2_measurement(float step_mm, float total_mm);
static uint8_t run_m2_capture_at_position(float target_mm, uint32_t step_index);
static uint8_t run_step_and_profile(uint32_t num_steps, uint32_t num_positions, ProfileAxis_t axis);
volatile uint8_t start_step_div_flag = 0U;
static float m2_target_z_mm = 0.0f;
static uint32_t m2_target_step_index = 0U;


static float compute_d4sigma_mm_from_raw_scan(
    const uint16_t *raw_buf,
    const uint8_t *stage_buf,
    const int32_t *enc_buf,
    uint32_t n
);

static uint8_t measure_beam_width_d4sigma(float *w_mm_out);
static void z_move_to_mm(float target_mm);
static uint8_t m2_measure_axis_at_positions(ProfileAxis_t axis,
                                            const float *target_z_mm,
                                            uint32_t target_count,
                                            float *z_mm_out,
                                            float *d4sigma_mm_out,
                                            uint32_t *valid_count_out,
                                            const char *phase_tag);
static void m2_fill_linear_positions(float start_mm,
                                     float end_mm,
                                     uint32_t count,
                                     float *dst,
                                     uint32_t *idx);
static uint8_t m2_build_iso_positions(float z0_mm,
                                      float z_rayleigh_mm,
                                      float travel_max_mm,
                                      float *target_z_mm,
                                      uint32_t *target_count_out);
static uint32_t m2_count_points_within_radius(const float *z_mm,
                                              uint32_t n,
                                              float center_mm,
                                              float radius_mm);
static uint32_t m2_count_points_outside_radius(const float *z_mm,
                                               uint32_t n,
                                               float center_mm,
                                               float radius_mm);
static float m2_compute_rayleigh_range_mm(float d0_mm, float theta_rad);
static uint8_t m2_fit_d4sigma_curve(const float *z_mm,
                                    const float *d4sigma_mm,
                                    uint32_t n,
                                    float wavelength_nm,
                                    float *z0_mm_out,
                                    float *d0_mm_out,
                                    float *theta_rad_out,
                                    float *m2_out,
                                    float *z_rayleigh_mm_out);
static float host_profile_sample_power_mw(uint8_t axis,
                                          uint8_t scan_no,
                                          uint32_t sample_idx,
                                          uint16_t raw,
                                          uint8_t saved_stage,
                                          int32_t enc_now,
                                          int32_t scan_start_enc,
                                          int32_t scan_end_enc);


// Command parser / mode flags
volatile uint8_t start_stokes_flag = 0;
volatile uint8_t stokes_busy = 0;
char cmd_buf[32];
uint8_t cmd_idx = 0;
volatile uint8_t start_profile_flag = 0;
volatile uint8_t start_t0_m1_flag = 0;
volatile uint8_t start_t0_m2_flag = 0;
uint32_t profile_start_request_ms = 0;
uint8_t profile_spinup_pending = 0;

volatile uint8_t start_t0raw_m1_flag = 0;
volatile uint8_t start_t0raw_m2_flag = 0;
volatile uint8_t live_sample_ready = 0;

volatile uint8_t z_home_start_flag = 0;

// wrap for negative
volatile uint8_t MaxTCts = 0;
volatile uint8_t t_diff = 0;

volatile uint8_t start_stepper1_move_flag = 0U;
float stepper1_move_mm_cmd = 0.0f;
volatile uint8_t stepper1_busy = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static inline void drv_set_dir(uint8_t dir);
static inline void stepper_enable(void);
static inline void stepper_disable(void);

static void delay_us(uint32_t us);
static void step_pulse(void);
static void move_steps(uint32_t n);

static float mean_u16(const uint16_t *x, uint32_t n);
static float adc_counts_to_voltage(float adc_counts);
static float get_feedback_resistor(void);
static float voltage_to_power_mw(float voltage);
static float gain_stage_feedback_resistor(uint8_t stage);
static float voltage_to_power_mw_for_stage(float voltage, uint8_t stage);

static void gain_init_stage1(void);
static void gain_step_up(void);
static void gain_step_down(void);
static uint8_t update_gain_stage(float v_meas);

static uint16_t capture_adc_average(void);
static uint16_t capture_adc_autorange(uint8_t *stage_used);
static void do_stokes_scan(void);

static void dc_motors_init(void);
static void dc_start_continuous(void);
static void dc_motor_set_duty(DCMotor_t *m, uint16_t duty);
static void dc_motors_stop_all(void);
static void dc_update_speed(DCMotor_t *m, float counts_per_rev, float dt_s);
static void dc_motor_auto_correct_counts(DCMotor_t *m);
static void dc_motors_auto_correct_counts(void);
static void dc_print_status(void);

static void dc_motor_auto_correct_active_axis(void);
static void dc_motor_start_profile_axis(ProfileAxis_t axis);
static float beam_profile_get_active_omega(void);

static float dc_get_active_rpm(void);
static uint8_t dc_active_axis_rpm_is_stable(void);
static uint8_t dc_wait_for_active_axis_stable_rpm(uint32_t timeout_ms);

static uint32_t adc_dma_write_index(void);
static void profile_extract_latest_window(uint16_t *dst, uint32_t nsamp);
static void profile_update_power_buffer_from_adc_window(const uint16_t *src, uint32_t nsamp);
static uint32_t find_t0_index_from_profile(void);
static void profile_build_emit_window(uint8_t scan_no, float ts_s, float omega_rad_s);
static void profile_extract_window_from_index(uint16_t *dst, uint32_t start_idx, uint32_t nsamp);

static float profile_initial_scan_time_s(float omega_rad_s);
static void profile_find_t0_only(ProfileAxis_t axis);
static uint8_t profile_find_t0_for_scheduler(ProfileAxis_t axis);

static void beam_profile_compute_timing(float omega_rad_s);
static void beam_profile_start_window(uint32_t duration_ms);
static uint8_t beam_profile_window_done(void);
static void beam_profile_begin(void);
static void beam_profile_update(void);
static uint32_t profile_find_t0_from_profile_buffer(void);
static float profile_counts_to_deg(int32_t enc_now, int32_t enc_start, float counts_per_rev);
static float profile_counts_to_x_mm(int32_t enc_now, int32_t enc_start, float counts_per_rev);
static int32_t profile_get_active_count_direction(void);
static int32_t interp_encoder_count_at_index(const int32_t *enc_buf, uint32_t n, float idx_f);
static void host_repeat_buffer_reset(void);
static uint8_t host_buffer_current_repeat_scan(uint8_t scan_no);
static void host_emit_buffered_repeat_scans(uint8_t axis);

//NEW T0 FINDING
//static void t0_batch_begin(ProfileAxis_t axis, uint8_t num_scans);
static void t0_batch_begin(ProfileAxis_t axis, uint8_t num_valid_scans);
static void t0_batch_update(void);
static uint8_t t0_dump_scan_step(ProfileAxis_t axis, uint8_t scan_no);
static uint8_t t0_batch_total_axes = 1U;
//NEW END

static void stepper1_move_mm(float mm);

// WAVELENGTH prototypes
static void do_wavelength_fft_measurement(void);
static void fft_process_chunk(const uint16_t *src, uint32_t n);
static float fft_find_peak_hz_and_accumulate(const uint16_t *src, uint32_t n, uint8_t clear_accum);
static float fft_find_avg_peak_hz_from_accum(uint32_t n_avg, uint32_t n_fft);
static float wavelength_nm_from_peak_hz(float f_hz);
static inline uint32_t step3_step_period_us(void);

// Added for finding t0
static void sg_smooth_5point(const float *src, float *dst, uint32_t n);
static void erf_fit_curve(
    const float *y, uint32_t n, float dt_s,
    float *x0_idx, float *w_idx, float *y_lo, float *y_hi
);
static void build_erf_fit_and_derivative(
    float *fit_y, float *fit_d, uint32_t n, float dt_s,
    float x0_idx, float w_idx, float y_lo, float y_hi
);
static float profile_find_t0_fit_index_from_raw_power(
    const uint16_t *raw_buf,
    const uint8_t *stage_buf,
    uint32_t n
);
static void host_profile_compute_local_t0(void);
static void host_arm_repeat_sequence(uint8_t reset_buffer);

void process_cmd_byte(uint8_t b);

static void live_power_update_service(void);
static void profile_stream_raw_scan(ProfileAxis_t axis);

static uint8_t startup_clear_done = 0U;

static float z_plot_mm(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <math.h>
#include <stdlib.h>

// Binary Packet Sender
static void wl_send_raw_packet(uint32_t window_id,
                               uint32_t chunk_id,
                               const uint16_t *samples,
                               uint32_t n)
{
    uint8_t header[16];

    if (samples == NULL || n == 0U)
        return;

    header[0] = WL_RAW_MAGIC0;
    header[1] = WL_RAW_MAGIC1;
    header[2] = WL_RAW_PACKET_TYPE_ADC;
    header[3] = 0x00;

    memcpy(&header[4],  &window_id, 4);
    memcpy(&header[8],  &chunk_id,  4);
    memcpy(&header[12], &n,         4);

    CDC_Enqueue_TX(header, sizeof(header));
    CDC_Enqueue_TX((const uint8_t *)samples, (uint16_t)(n * sizeof(uint16_t)));
    CDC_TX_Kick();
}

// -----------------------------
// Stepper helpers
// -----------------------------
static void z_set_microstep(uint8_t mode)
{
    switch (mode)
    {
        case 1: // full step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_RESET);
            break;

        case 2: // half step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_RESET);
            break;

        case 4: // quarter step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_RESET);
            break;

        case 8: // eighth step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_RESET);
            break;

        case 16: // sixteenth step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_SET);
            break;

        case 32: // thirty-second step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_SET);
            break;

        default: // default to full step
            HAL_GPIO_WritePin(Z_M0_GPIO_Port, Z_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M1_GPIO_Port, Z_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Z_M2_GPIO_Port, Z_M2_Pin, GPIO_PIN_RESET);
            break;
    }
}

static inline void drv_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void stepper_sleep_wake(void)
{
    HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
}

static inline void stepper_sleep_disable(void)
{
    HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
}

static inline void drv2_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_ramp_to_speed(uint32_t target_period_us)
{
    for (uint32_t d = 2000U; d > target_period_us; d -= 50U)
    {
        step_pulse();
        delay_us(d);
    }
}

// -----------------------------
// Z-stage stepper helpers (M2)
// -----------------------------
static inline void z_stepper_wake(void)
{
    HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
}

static inline void z_stepper_disable(void)
{
    HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
}

static void z_step_pulse(void)
{
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
}

static void step2_pulse(void)
{
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
}


static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

static void step_pulse(void)
{
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
}

static void move_steps(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        step_pulse();
        HAL_Delay(1);
    }
}

// M2 stepper move
static void z_move_steps(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        z_step_pulse();
        HAL_Delay(1);
    }
}

static void move2_steps(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        step2_pulse();
        HAL_Delay(1);
    }
}

static void step2_ramp_to_speed(uint32_t target_period_us)
{
    uint32_t d;

    for (d = 5000U; d > target_period_us; d -= 10U)
    {
        step2_pulse();
        delay_us(d);
    }
}

static inline void z_step(void)
{
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
}

static inline void z_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void z_home_simple(uint8_t dir)
{
    uint32_t steps_taken = 0U;

    printf("HOMING START\r\n");
    printf("HOME_PIN_STATE=%u\r\n",
           (unsigned int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13));

    z_stepper_wake();
    HAL_Delay(10);

    if (home_pressed())
    {
        printf("ALREADY AT HOME\r\n");
        z_current_mm = 0.0f;
        printf("Z_HOME_DONE, z_mm=%.3f\r\n", z_current_mm);
        z_stepper_disable();
        HAL_Delay(50);
        return;
    }

    z_set_dir(dir);
    HAL_Delay(10);

    while (!home_pressed())
    {
        z_step();
        HAL_Delay(1);
//        delay_us(Z_STEP_DELAY_US);
        steps_taken++;

        if (steps_taken >= Z_HOME_MAX_STEPS)
        {
            printf("HOME_FAIL_TIMEOUT\r\n");
            z_stepper_disable();
            return;
        }
    }

    z_current_mm = 0.0f;
    printf("HOMED\r\n");
    printf("Z_HOME_DONE, z_mm=%.3f\r\n", z_current_mm);
    z_stepper_disable();
    HAL_Delay(5);
}

//void z_home_simple(uint8_t dir)
//{
//    uint8_t homed = 0U;
//    uint32_t steps_taken = 0U;
//
//    printf("HOMING START\r\n");
//
//    z_stepper_wake();
//    HAL_Delay(2);
//
//    if (!home_pressed())
//    {
//        z_set_dir(dir);
//        HAL_Delay(1);
//
//        while (!home_pressed())
//        {
//        	while(!home_pressed()){
//                z_step();
//                HAL_Delay(1);
//                steps_taken++;
//        	}
//        	HAL_Delay(1);
//        }
//
//        if (home_pressed())
//        {
//            homed = 1U;
//            printf("HOMED\r\n");
//
//            z_current_mm = 0.0f;
//            printf("Z_HOME_DONE, z_mm=%.3f\r\n", z_current_mm);
//        }
//        else
//        {
//            printf("HOME_FAIL_TIMEOUT\r\n");
//        }
//    }
//    else
//    {
//        printf("ALREADY AT HOME\r\n");
//        homed = 1U;
//    }
//    z_stepper_disable();
//}


static uint8_t home_pressed(void)
{
    // Require 3 consecutive HIGH reads ~1ms apart to confirm button press
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) != GPIO_PIN_SET)
            return 0U;
        HAL_Delay(1);
    }
    return 1U;
}

static void z_move_steps_dir(uint8_t dir, uint32_t n)
{
    z_set_dir(dir);
    HAL_Delay(10);   // give driver enough time to latch new direction

    for (uint32_t i = 0; i < n; i++)
    {
        z_step_pulse();
        HAL_Delay(1);
    }
}

//static void z_move_mm(float mm)
//{
//    if (mm <= 0.0f)
//        return;
//
//    uint32_t steps = (uint32_t)(mm * Z_STEPS_PER_MM + 0.5f);
//    z_move_steps_dir(Z_AWAY_DIR, steps);
//
//    printf("Z_MOVE_DONE,x_mm=%.4f,steps=%lu\r\n",
//           mm, (unsigned long)steps);
//}

static void z_move_mm_dir(uint8_t dir, float mm)
{
    if (mm <= 0.0f)
        return;

    uint32_t steps = (uint32_t)(mm * Z_STEPS_PER_MM + 0.5f);

    z_stepper_wake();
    HAL_Delay(50);      // long wake delay

    z_set_dir(dir);
    HAL_Delay(50);      // long direction latch delay

    for (uint32_t i = 0; i < steps; i++)
    {
        z_step_pulse();
        delay_us(Z_STEP_DELAY_US);
    }

    if (dir == Z_AWAY_DIR)
        z_current_mm += mm;
    else
        z_current_mm -= mm;

    if (z_current_mm < 0.0f)
        z_current_mm = 0.0f;

    printf("Z_MOVE_DONE,dir=%u,move_mm=%.3f,z_mm=%.3f,steps=%lu\r\n",
           (unsigned int)dir, mm, z_plot_mm(), (unsigned long)steps);

    z_stepper_disable();
}

static void z_move_mm(float mm)
{
    z_move_mm_dir(Z_AWAY_DIR, mm);
}

static void z_move_to_mm(float target_mm)
{
    float delta_mm;

    if (target_mm < 0.0f)
        target_mm = 0.0f;
    if (target_mm > M2_TOTAL_MM)
        target_mm = M2_TOTAL_MM;

    delta_mm = target_mm - z_current_mm;
    if (delta_mm > 0.01f)
    {
        z_move_mm_dir(Z_AWAY_DIR, delta_mm);
    }
    else if (delta_mm < -0.01f)
    {
        z_move_mm_dir(Z_HOME_DIR, -delta_mm);
    }
}

// -----------------------------
// ADC / power helpers
// -----------------------------
static float mean_u16(const uint16_t *x, uint32_t n)
{
    uint64_t sum = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        sum += x[i];
    }
    return (n > 0U) ? ((float)sum / (float)n) : 0.0f;
}

static float adc_counts_to_voltage(float adc_counts)
{
    return (adc_counts / ADC_MAX_COUNTS) * ADC_VREF + PM_VOLTAGE_OFFSET_V;
}

static float get_feedback_resistor(void)
{
    switch (current_gain)
    {
        case GAIN_STAGE_4: return R4;
        case GAIN_STAGE_3: return R3;
        case GAIN_STAGE_2: return R2;
        case GAIN_STAGE_1:
        default:           return R1;
    }
}

static float gain_stage_feedback_resistor(uint8_t stage)
{
    switch ((GainStage_t)stage)
    {
        case GAIN_STAGE_4: return R4;
        case GAIN_STAGE_3: return R3;
        case GAIN_STAGE_2: return R2;
        case GAIN_STAGE_1:
        default:           return R1;
    }
}

static float voltage_to_power_mw(float voltage)
{
    float rf = get_feedback_resistor();
    return ((voltage / rf) / 0.00213f) * 1000.0f;
}

static float voltage_to_power_mw_for_stage(float voltage, uint8_t stage)
{
    float rf = gain_stage_feedback_resistor(stage);
    return ((voltage / rf) / 0.00213f) * 1000.0f;
}

// -----------------------------
// Gain stage helpers
// -----------------------------

// -----------------------------
// Initialize gain stage to lowest gain (Stage 1)
// This prevents saturation at startup
//
// Stage meaning (based on feedback resistors):
//   Stage 1 → lowest gain (R1)
//   Stage 4 → highest gain (R4)
//
// Only ONE stage should be active at a time
// -----------------------------
static void gain_init_stage1(void)
{
    // Activate Stage 1 (lowest gain)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    // Deactivate all other stages
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    current_gain = GAIN_STAGE_1;
}

// -----------------------------
// Decrease gain (reduce amplification)
// Used when signal is too large (risk of saturation)
//
// Example:
//   Stage 4 → Stage 3 → Stage 2 → Stage 1
// -----------------------------
static void gain_step_down(void)
{
    switch (current_gain)
    {
        case GAIN_STAGE_4:
            // Move to Stage 3
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            current_gain = GAIN_STAGE_3;
            printf("SWITCH -> stage 3\r\n");
            break;

        case GAIN_STAGE_3:
            // Move to Stage 2
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            current_gain = GAIN_STAGE_2;
            printf("SWITCH -> stage 2\r\n");
            break;

        case GAIN_STAGE_2:
            // Move to Stage 1 (lowest gain)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            current_gain = GAIN_STAGE_1;
            printf("SWITCH -> stage 1\r\n");
            break;

        case GAIN_STAGE_1:
        default:
            // Already at lowest gain
            break;
    }
}

// -----------------------------
// Increase gain (move to higher sensitivity stage)
// Used when signal is too small (low voltage)
//
// Example:
//   Stage 1 → Stage 2 → Stage 3 → Stage 4
//
// IMPORTANT:
// Only one stage is active at a time
// -----------------------------
static void gain_step_up(void)
{
    switch (current_gain)
    {
        case GAIN_STAGE_1:
            // Move to Stage 2
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // activate
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // deactivate
            current_gain = GAIN_STAGE_2;
            printf("SWITCH -> stage 2\r\n");
            break;

        case GAIN_STAGE_2:
            // Move to Stage 3
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            current_gain = GAIN_STAGE_3;
            printf("SWITCH -> stage 3\r\n");
            break;

        case GAIN_STAGE_3:
            // Move to Stage 4 (maximum gain)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            current_gain = GAIN_STAGE_4;
            printf("SWITCH -> stage 4\r\n");
            break;

        case GAIN_STAGE_4:
        default:
            // Already at highest gain
            break;
    }
}

// -----------------------------
// Real-time gain control based on measured voltage
//
// If voltage is:
//   Too LOW  → increase gain (better sensitivity)
//   Too HIGH → decrease gain (avoid saturation)
//
// Thresholds:
//   LOW  threshold (~0.3V) → below this = increase gain
//   HIGH threshold (~3.0V) → above this = decrease gain
//
// This function is used during continuous ADC streaming
// -----------------------------
static uint8_t update_gain_stage(float v_meas)
{
	// free gain switching during t0 burst capture / locked measurement periods
	if(gain_settle_skip || gain_settle_skip_live)
		return 0U;

    // Signal too small → increase gain
    if (v_meas <= GAIN_SWITCH_LOW_V && current_gain != GAIN_STAGE_4)
    {
        gain_step_up();
        return 1U;
    }

    // Signal too large → decrease gain
    if (v_meas >= GAIN_SWITCH_HIGH_V && current_gain != GAIN_STAGE_1)
    {
        gain_step_down();
        return 1U;
    }
    return 0U;

    // Otherwise: gain is already optimal → do nothing
}

// -----------------------------
// USB printf
// -----------------------------
extern void CDC_Enqueue_TX(const uint8_t *buf, uint16_t len);

int _write(int file, char *ptr, int len)
{
    (void)file;

    if ((ptr == NULL) || (len <= 0))
        return 0;

    CDC_Enqueue_TX((const uint8_t *)ptr, (uint16_t)len);
    return len;
}

static void cdc_write_blocking(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
        return;

    CDC_Enqueue_TX(buf, len);
}

static void cdc_printf_blocking(const char *fmt, ...)
{
    char tx[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tx, sizeof(tx), fmt, ap);
    va_end(ap);

    if (n <= 0)
        return;

    if (n > (int)sizeof(tx))
        n = sizeof(tx);

    CDC_Enqueue_TX((const uint8_t *)tx, (uint16_t)n);
}
// -----------------------------
// Encoder Counts Helpers
// -----------------------------
static void reset_axis_encoder(ProfileAxis_t axis)
{
    if (axis == PROFILE_AXIS_M1)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        dc1.prev_count = 0;
        dc1.delta_count = 0;
    }
    else
    {
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        dc2.prev_count = 0;
        dc2.delta_count = 0;
    }
}

static int32_t enc_read_axis_count(ProfileAxis_t axis)
{
    if (axis == PROFILE_AXIS_M1)
        return -(int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    else
        return -(int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

static int32_t enc_delta_counts(int32_t now, int32_t then)
{
    return (int32_t)(int16_t)(now - then);
}


static float profile_get_active_counts_per_rev(void)
{
    return (profiler.axis == PROFILE_AXIS_M1) ? ENC1_COUNTS_PER_REV : ENC2_COUNTS_PER_REV;
}

static void profile_stream_raw_scan(ProfileAxis_t axis)
{
    uint32_t t_start;
    uint32_t last_speed_update_ms;
    uint32_t last_print_ms;
    uint32_t sample_idx = 0U;
    float dt_s;
    float omega_rad_s;
    float scan_time_s;

    profiler.active = 0U;
    profiler.state = PROFILE_IDLE;
    profile_spinup_pending = 0U;
    start_profile_flag = 0U;

    profiler.axis = axis;

//    // Make sure motors are already running continuously
//    dc_start_continuous();

    // Let the live correction loop establish a stable speed naturally
    HAL_Delay(300);

    // Use the active axis omega directly instead of lock/wait logic
    omega_rad_s = beam_profile_get_active_omega();

    if (omega_rad_s < 0.2f)
    {
        printf("T0RAW_BAD_OMEGA,axis=%u,rpm=%.4f,omega=%.6f\r\n",
               (unsigned int)axis,
               dc_get_active_rpm(),
               omega_rad_s);
        return;
    }

    scan_time_s = profile_initial_scan_time_s(omega_rad_s);

    printf("T0RAW_BEGIN,axis=%u,rpm=%.4f,omega=%.6f,scan_time_s=%.6f\r\n",
           (unsigned int)axis,
           dc_get_active_rpm(),
           omega_rad_s,
           scan_time_s);

    t_start = HAL_GetTick();
    last_speed_update_ms = t_start;
    last_print_ms = t_start;
    live_sample_ready = 0U;

    while ((HAL_GetTick() - t_start) < (uint32_t)(scan_time_s * 1000.0f + 0.5f))
    {
        live_power_update_service();

        uint32_t now = HAL_GetTick();

        // Keep continuous speed correction running during scan
        if ((now - last_speed_update_ms) >= DC_SPEED_SAMPLE_MS)
        {
            dt_s = (float)(now - last_speed_update_ms) / 1000.0f;

            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

            // Correct both motors continuously
            dc_motors_auto_correct_counts();

            last_speed_update_ms = now;

            // Re-read omega after correction
            omega_rad_s = beam_profile_get_active_omega();
        }

        if (live_sample_ready)
        {
            live_sample_ready = 0U;

            float t_s = (float)(HAL_GetTick() - t_start) / 1000.0f;
            float deg = omega_rad_s * t_s * (180.0f / (float)M_PI);

            printf("RAW,%.6f,%.6f,%.6f\r\n",
                   t_s * 1000.0f,
                   deg,
                   latest_p_mw);

            sample_idx++;
        }

        if ((now - last_print_ms) >= 500U)
        {
            dc_print_status();
            last_print_ms = now;
        }
    }

    printf("T0RAW_DONE,axis=%u\r\n", (unsigned int)axis);

    // DO NOT stop motors here if you want continuous running
    // dc_motors_stop_all();
}

//static void profile_stream_raw_scan(ProfileAxis_t axis)
//{
//    uint32_t t_start;
//    uint32_t last_speed_update_ms;
//    uint32_t last_print_ms;
//    uint32_t sample_idx = 0U;
//    float dt_s;
//    float omega_rad_s;
//    float scan_time_s;
//    uint8_t locked;
//
//    profiler.active = 0U;
//    profiler.state = PROFILE_IDLE;
//    profile_spinup_pending = 0U;
//    start_profile_flag = 0U;
//
//    profiler.axis = axis;
//
//    profile_locked_rpm = 0.0f;
//    profile_locked_omega_rad_s = 0.0f;
//    profile_rpm_stable_count = 0U;
//
//    dc_motor_start_profile_axis(axis);
//
//    // initial open-loop spin-up
//    HAL_Delay(PROFILE_SPINUP_MS);
//
//    // wait until active-axis RPM is stable and lock omega
//    locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);
//
//    omega_rad_s = profile_locked_omega_rad_s;
//    if (omega_rad_s <= 1e-6f)
//        omega_rad_s = beam_profile_get_active_omega();
//
//    if (omega_rad_s < 0.2f)
//    {
//        printf("T0RAW_BAD_OMEGA,axis=%u,locked=%u,rpm=%.4f,omega=%.6f\r\n",
//               (unsigned int)axis,
//               (unsigned int)locked,
//               profile_locked_rpm,
//               omega_rad_s);
//        dc_motors_stop_all();
//        return;
//    }
//
//    scan_time_s = profile_initial_scan_time_s(omega_rad_s);
//
//    printf("T0RAW_BEGIN,axis=%u,locked=%u,rpm=%.4f,omega=%.6f,scan_time_s=%.6f\r\n",
//           (unsigned int)axis,
//           (unsigned int)locked,
//           profile_locked_rpm,
//           omega_rad_s,
//           scan_time_s);
//
//    t_start = HAL_GetTick();
//    last_speed_update_ms = t_start;
//    last_print_ms = t_start;
//    live_sample_ready = 0U;
//
//    while ((HAL_GetTick() - t_start) < (uint32_t)(scan_time_s * 1000.0f + 0.5f))
//    {
//        live_power_update_service();
//
//        uint32_t now = HAL_GetTick();
//
//        // keep live RPM control active during the scan
//        if ((now - last_speed_update_ms) >= DC_SPEED_SAMPLE_MS)
//        {
//            dt_s = (float)(now - last_speed_update_ms) / 1000.0f;
//
//            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//            dc_motor_auto_correct_active_axis();
//
//            last_speed_update_ms = now;
//        }
//
//        if (live_sample_ready)
//        {
//            live_sample_ready = 0U;
//
//            // throttle USB output: emit every 10th live sample
//            if ((sample_idx % 1U) == 0U)
//            {
//                float t_s = (float)(HAL_GetTick() - t_start) / 1000.0f;
//                float deg = omega_rad_s * t_s * (180.0f / (float)M_PI);
//
//                printf("RAW,%.6f,%.6f,%.6f\r\n",
//                       t_s * 1000.0f,
//                       deg,
//                       latest_p_mw);
//            }
//
//            sample_idx++;
//        }
//
//        if ((now - last_print_ms) >= 500U)
//        {
//            dc_print_status();
//            last_print_ms = now;
//        }
//    }
//
//    printf("T0RAW_DONE,axis=%u\r\n", (unsigned int)axis);
//    dc_motors_stop_all();
//}

// -----------------------------
// Capture averaged ADC sample using DMA burst
// Used primarily for Stokes scan measurements
// -----------------------------
static uint16_t capture_adc_average(void)
{
    uint32_t sum = 0;

    // Reset DMA flags
    adc_dma_done = 0;
    burst_capture_active = 1;

    // Stop continuous ADC + trigger timer
    HAL_ADC_Stop_DMA(&hadc4);
    HAL_TIM_Base_Stop(&htim2);

    // Reset timer
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    // Clear buffer
    for (uint32_t i = 0; i < ADC_SAMPLES_PER_POINT; i++)
    {
        adc_buf[i] = 0;
    }

    // Start DMA for finite burst
    if (HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_buf, ADC_SAMPLES_PER_POINT) != HAL_OK)
    {
        Error_Handler();
    }

    // Start sampling timer
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    // Wait for DMA completion (blocking)
    uint32_t t0 = HAL_GetTick();
    while (!adc_dma_done)
    {
        // Timeout safety
        if ((HAL_GetTick() - t0) > 1000U)
        {
            HAL_TIM_Base_Stop(&htim2);
            HAL_ADC_Stop_DMA(&hadc4);
            burst_capture_active = 0;
            return 0;
        }
    }

    // Stop sampling
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc4);

    // Compute average of samples
    for (uint32_t i = 0; i < ADC_SAMPLES_PER_POINT; i++)
    {
        sum += adc_buf[i];
    }

    burst_capture_active = 0;

    return (uint16_t)(sum / ADC_SAMPLES_PER_POINT);
}

// -----------------------------
// Capture ADC sample with automatic gain selection
// Ensures measurement stays within optimal voltage range
// -----------------------------
static uint16_t capture_adc_autorange(uint8_t *stage_used)
{
    uint16_t raw = 0;
    float v = 0.0f;

    // Try multiple times to settle into correct gain range
    for (uint8_t tries = 0; tries < MAX_GAIN_ADJUST_TRIES; tries++)
    {
        raw = capture_adc_average();
        v = adc_counts_to_voltage((float)raw);

        // If signal too small → increase gain
        if ((v <= GAIN_SWITCH_LOW_V) && (current_gain != GAIN_STAGE_4))
        {
            gain_step_up();
            HAL_Delay(GAIN_SETTLE_MS);
            continue;
        }

        // If signal too large → decrease gain
        if ((v >= GAIN_SWITCH_HIGH_V) && (current_gain != GAIN_STAGE_1))
        {
            gain_step_down();
            HAL_Delay(GAIN_SETTLE_MS);
            continue;
        }

        // Gain is acceptable → stop adjusting
        break;
    }

    // Return final gain stage used
    if (stage_used != NULL)
    {
        *stage_used = (uint8_t)current_gain;
    }

    return raw;
}

// -----------------------------
// Stokes scan (polarization measurement)
// -----------------------------
static void do_stokes_scan(void)
{
	stepper_sleep_wake();     // sleep set high

    // Set rotation direction (1 = forward)
    drv_set_dir(1);

    HAL_Delay(2);   // let sleep wake up driver

    // Loop through all polarization sample angles
    for (uint32_t k = 0; k < NUM_STOKES_SAMPLES; k++)
    {
        // Move to next angular position (skip for first point)
        if (k > 0)
        {
            move_steps(STEPS_PER_INCREMENT);
        }

        // Allow mechanical + optical system to settle
        HAL_Delay(SETTLE_MS);

        // Capture ADC with auto-ranging (gain switching)
        // This ensures signal stays within optimal ADC range
        stokes_I[k] = capture_adc_autorange(&stokes_stage[k]);

        // Convert ADC counts → voltage
        stokes_V[k] = adc_counts_to_voltage((float)stokes_I[k]);

        // Convert voltage → optical power (mW)
        stokes_mW[k] = voltage_to_power_mw(stokes_V[k]);

        // Stream individual measurement point to PC
        printf("I%lu stage=%u ADC=%u V=%.4f P=%.4f mW\r\n",
               (unsigned long)k,
               stokes_stage[k],
               stokes_I[k],
               stokes_V[k],
               stokes_mW[k]);
    }

    // Move one final step to complete rotation symmetry
    move_steps(STEPS_PER_INCREMENT);
    HAL_Delay(SETTLE_MS);

    // Send full dataset as a single line (easy parsing in Python)
    printf("STOKES_MW ");
    for (uint32_t i = 0; i < NUM_STOKES_SAMPLES; i++)
    {
        printf("%.4f ", stokes_mW[i]);
    }
    printf("\r\n");

    // Signal end of scan
    printf("STOKES_DONE\r\n");

    // Disable stepper motor to reduce heating
    stepper_sleep_disable();
}

// -----------------------------
// DC motor helpers
// -----------------------------
static void dc_start_continuous(void)
{
    // Start both motors at a base duty.
    // Auto-correct will then adjust duty every DC_SPEED_SAMPLE_MS.
    dc_motor_set_duty(&dc1, 50);
    dc_motor_set_duty(&dc2, 50);

    // Reset previous encoder counts so first delta is clean
    dc1.prev_count = enc_read_axis_count(PROFILE_AXIS_M1);
    dc2.prev_count = enc_read_axis_count(PROFILE_AXIS_M2);
}

static void dc_motors_init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    dc1.enc_tim     = &htim3;
    dc1.pwm_channel = TIM_CHANNEL_3;
    dc1.prev_count  = enc_read_axis_count(PROFILE_AXIS_M1);
    dc1.rpm         = 0.0f;
    dc1.omega       = 0.0f;
    dc1.duty        = 0;

    dc2.enc_tim     = &htim1;
    dc2.pwm_channel = TIM_CHANNEL_1;
    dc2.prev_count  = enc_read_axis_count(PROFILE_AXIS_M2);
    dc2.rpm         = 0.0f;
    dc2.omega       = 0.0f;
    dc2.duty        = 0;

    dc_motors_stop_all();
}

static void dc_motor_set_duty(DCMotor_t *m, uint16_t duty)
{
    if (duty > DC_PWM_MAX)
        duty = DC_PWM_MAX;

    m->duty = duty;
    __HAL_TIM_SET_COMPARE(&htim8, m->pwm_channel, duty);
}

static void dc_motors_stop_all(void)
{
    dc_motor_set_duty(&dc1, 0);
    dc_motor_set_duty(&dc2, 0);
}

static void dc_update_speed(DCMotor_t *m, float counts_per_rev, float dt_s)
{
    int32_t now;
    int32_t delta;

    if (m == &dc1)
        now = enc_read_axis_count(PROFILE_AXIS_M1);
    else
        now = enc_read_axis_count(PROFILE_AXIS_M2);

    delta = enc_delta_counts(now, m->prev_count);
    m->prev_count  = now;
    m->delta_count = delta;

    if (counts_per_rev > 0.0f && dt_s > 0.0f)
    {
        m->rpm = ((float)delta * 60.0f) / (counts_per_rev * dt_s);
        m->omega = m->rpm * 0.10472f;
    }
    else
    {
        m->rpm = 0.0f;
        m->omega = 0.0f;
    }
}

static void dc_motor_auto_correct_counts(DCMotor_t *m)
{
    float measured = (float)abs(m->delta_count);
    float error = TARGET_COUNTS - measured;
    float newDuty = (float)m->duty + KP_COUNTS * error;

    if (newDuty < 0.0f) newDuty = 0.0f;
    if (newDuty > DC_PWM_MAX) newDuty = DC_PWM_MAX;

    dc_motor_set_duty(m, (uint16_t)(newDuty + 0.5f));
}

static void dc_motors_auto_correct_counts(void)
{
    dc_motor_auto_correct_counts(&dc1);
    dc_motor_auto_correct_counts(&dc2);
}

static void dc_motor_auto_correct_active_axis(void)
{
    if (profiler.axis == PROFILE_AXIS_M1)
    {
        dc_motor_auto_correct_counts(&dc1);
    }
    else
    {
        dc_motor_auto_correct_counts(&dc2);
    }
}

static float dc_get_active_rpm(void)
{
    if (profiler.axis == PROFILE_AXIS_M1)
        return fabsf(dc1.rpm);
    else
        return fabsf(dc2.rpm);
}

static uint8_t dc_active_axis_rpm_is_stable(void)
{
    float rpm = dc_get_active_rpm();
    float err = fabsf(rpm - PROFILE_TARGET_RPM);

    if (err <= PROFILE_RPM_TOLERANCE_RPM)
    {
        if (profile_rpm_stable_count < 255U)
            profile_rpm_stable_count++;
    }
    else
    {
        profile_rpm_stable_count = 0U;
    }

    return (profile_rpm_stable_count >= PROFILE_RPM_STABLE_COUNT_REQ) ? 1U : 0U;
}

static uint8_t dc_wait_for_active_axis_stable_rpm(uint32_t timeout_ms)
{
    uint32_t t0_ms = HAL_GetTick();
    uint32_t last_update_ms = HAL_GetTick();
    uint32_t last_print_ms = HAL_GetTick();

    profile_rpm_stable_count = 0U;

    while ((HAL_GetTick() - t0_ms) < timeout_ms)
    {
        // keep live ADC/gain updating while waiting
        live_power_update_service();

        uint32_t now = HAL_GetTick();

        if ((now - last_update_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_update_ms) / 1000.0f;

            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

            // actively keep correcting the selected motor
            dc_motor_auto_correct_active_axis();

            last_update_ms = now;

            if (dc_active_axis_rpm_is_stable())
            {
                profile_locked_rpm = dc_get_active_rpm();
                profile_locked_omega_rad_s = beam_profile_get_active_omega();
                return 1U;
            }
        }

        if ((now - last_print_ms) >= 500U)
        {
            printf("RPM_WAIT,axis=%u,rpm=%.3f,target=%.3f,stable=%u/%u,duty=%u\r\n",
                   (unsigned int)profiler.axis,
                   dc_get_active_rpm(),
                   PROFILE_TARGET_RPM,
                   (unsigned int)profile_rpm_stable_count,
                   (unsigned int)PROFILE_RPM_STABLE_COUNT_REQ,
                   (profiler.axis == PROFILE_AXIS_M1) ? dc1.duty : dc2.duty);
            last_print_ms = now;
        }
    }

    // timeout case: use best available value
    profile_locked_rpm = dc_get_active_rpm();
    profile_locked_omega_rad_s = beam_profile_get_active_omega();
    return 0U;
}

static void dc_print_status(void)
{
    int32_t c1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t c2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);

    printf("DC,"
           "M1_D=%u,M1_DELTA=%ld,M1_C=%ld,M1_RPM=%.2f,M1_W=%.3f,"
           "M2_D=%u,M2_DELTA=%ld,M2_C=%ld,M2_RPM=%.2f,M2_W=%.3f\r\n",
           dc1.duty, dc1.delta_count, c1, dc1.rpm, dc1.omega,
           dc2.duty, dc2.delta_count, c2, dc2.rpm, dc2.omega);
}

static void dc_motor_start_profile_axis(ProfileAxis_t axis)
{
    // stop both first
    dc_motors_stop_all();

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    dc1.prev_count = 0;
    dc2.prev_count = 0;
    dc1.delta_count = 0;
    dc2.delta_count = 0;
    dc1.rpm = 0.0f;
    dc2.rpm = 0.0f;
    dc1.omega = 0.0f;
    dc2.omega = 0.0f;

    if (axis == PROFILE_AXIS_M1)
    {
        dc_motor_set_duty(&dc1, PROFILE_START_DUTY);
        dc_motor_set_duty(&dc2, 0);
    }
    else
    {
        dc_motor_set_duty(&dc1, 0);
        dc_motor_set_duty(&dc2, PROFILE_START_DUTY);
    }
}

static int32_t dc_get_active_encoder_count(void)
{
    return enc_read_axis_count(profiler.axis);
}
// -----------------------------
// Get current write index of circular ADC DMA buffer
// Used to extract most recent samples for profiling
// -----------------------------
static uint32_t adc_dma_write_index(void)
{
    // Remaining transfers in DMA
    uint32_t rem = __HAL_DMA_GET_COUNTER(&hdma_adc4);

    // Convert to write index
    uint32_t wr = ADC_BUF_LEN - rem;

    if (wr >= ADC_BUF_LEN) wr = 0;

    return wr;
}

// ADDED
static void live_power_update_service(void)
{
	CDC_TX_Kick();

    if (burst_capture_active)
        return;

    if (adc_half_ready)
    {
        adc_half_ready = 0U;

        float avg_counts = mean_u16(&adc_buf[0], ADC_HALF_LEN);
        float v_meas = adc_counts_to_voltage(avg_counts);

        if (gain_settle_skip_live > 0U)
        {
            gain_settle_skip_live--;
            return;
        }

        if (update_gain_stage(v_meas))
        {
            latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
            latest_v_meas   = v_meas;
            latest_p_mw     = voltage_to_power_mw(v_meas);
            live_sample_ready = 1U;

            gain_settle_skip_live = 2U;
            return;
        }

        latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
        latest_v_meas   = v_meas;
        latest_p_mw     = voltage_to_power_mw(v_meas);
        live_sample_ready = 1U;
    }

    if (adc_full_ready)
    {
        adc_full_ready = 0U;

        float avg_counts = mean_u16(&adc_buf[ADC_HALF_LEN], ADC_HALF_LEN);
        float v_meas = adc_counts_to_voltage(avg_counts);

        if (gain_settle_skip_live > 0U)
        {
            gain_settle_skip_live--;
            return;
        }

        if (update_gain_stage(v_meas))
        {
            latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
            latest_v_meas   = v_meas;
            latest_p_mw     = voltage_to_power_mw(v_meas);
            live_sample_ready = 1U;

            gain_settle_skip_live = 2U;
            return;
        }

        latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
        latest_v_meas   = v_meas;
        latest_p_mw     = voltage_to_power_mw(v_meas);
        live_sample_ready = 1U;
    }
}

static float beam_profile_get_active_omega(void)
{
    if (profiler.axis == PROFILE_AXIS_M1)
        return fabsf(dc1.omega);
    else
        return fabsf(dc2.omega);
}

static float profile_initial_scan_time_s(float omega_rad_s)
{
    if (omega_rad_s <= 1e-6f)
        return 0.5f;

    return TS_SCALE * PROFILE_THETA0_RAD / omega_rad_s;
}

static int32_t profile_t0_target_counts_from_scan_end(float scan_time_s, float t0_s, float counts_per_rev)
{
    float omega = beam_profile_get_active_omega();
    float trev_s;
    float dt_s;
    float frac_rev;
    float counts_f;

    if (omega <= 1e-6f || counts_per_rev <= 0.0f)
        return 0;

    trev_s = (2.0f * (float)M_PI) / omega;

    // From end of scan to the next time the blade reaches t0
    dt_s = trev_s - (scan_time_s - t0_s);

    while (dt_s < 0.0f)
        dt_s += trev_s;
    while (dt_s >= trev_s)
        dt_s -= trev_s;

    frac_rev = dt_s / trev_s;
    counts_f = frac_rev * counts_per_rev;

    return (int32_t)(counts_f + 0.5f);
}

//ADDED
static void dc_wait_active_axis_counts(int32_t target_counts)
{
    int32_t start_count = dc_get_active_encoder_count();
    uint32_t t0_ms = HAL_GetTick();
    uint32_t last_print_ms = HAL_GetTick();

    while (1)
    {
        live_power_update_service();

        int32_t now_count = dc_get_active_encoder_count();
        int32_t moved = now_count - start_count;

        if (moved < 0)
            moved = -moved;

        if (moved >= target_counts)
            break;

        uint32_t now = HAL_GetTick();

        // 🔥 Print RPM while moving
        if ((now - last_print_ms) >= 500U)
        {
            dc_print_status();
            last_print_ms = now;
        }

        if ((now - t0_ms) > 5000U)
            break;
    }
}
static uint32_t profile_time_from_scan_end_to_t0_ms(float scan_time_s, float t0_s)
{
    // We capture one initial scan from t = 0 to t = scan_time_s.
    // If t0 occurs inside that window, then at the end of the scan the blade
    // has already passed t0. So we wait until the same physical blade position
    // comes back around on the next revolution.

    float omega = beam_profile_get_active_omega();
    float trev_s;
    float dt_s;

    if (omega <= 1e-6f)
        return 0U;

    trev_s = (2.0f * (float)M_PI) / omega;

    // move from end-of-scan to next occurrence of t0
    dt_s = trev_s - (scan_time_s - t0_s);

    while (dt_s < 0.0f)
        dt_s += trev_s;

    return (uint32_t)(dt_s * 1000.0f + 0.5f);
}

// 1st method - noise filtering (moving average)
static void moving_average_same(const float *src, float *dst, uint32_t n, uint32_t win)
{
    if (n == 0U)
        return;

    if (win < 1U)
        win = 1U;

    if ((win % 2U) == 0U)
        win += 1U;

    uint32_t half = win / 2U;

    for (uint32_t i = 0U; i < n; i++)
    {
        uint32_t start = (i > half) ? (i - half) : 0U;
        uint32_t end = i + half;
        if (end >= n)
            end = n - 1U;

        float sum = 0.0f;
        uint32_t count = 0U;

        for (uint32_t k = start; k <= end; k++)
        {
            sum += src[k];
            count++;
        }

        dst[i] = (count > 0U) ? (sum / (float)count) : src[i];
    }
}

// 2nd method - noise filtering (median filter first, then exponential low-pass
static void sort_small_float_array(float *a, uint32_t n)
{
    for (uint32_t i = 0U; i < n; i++)
    {
        for (uint32_t j = i + 1U; j < n; j++)
        {
            if (a[j] < a[i])
            {
                float tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
        }
    }
}

static void median_filter_same(const float *src, float *dst, uint32_t n, uint32_t win)
{
    if (n == 0U)
        return;

    if (win < 1U)
        win = 1U;

    if ((win % 2U) == 0U)
        win += 1U;

    uint32_t half = win / 2U;

    for (uint32_t i = 0U; i < n; i++)
    {
        float tmp[11];
        uint32_t count = 0U;

        uint32_t start = (i > half) ? (i - half) : 0U;
        uint32_t end = i + half;
        if (end >= n)
            end = n - 1U;

        for (uint32_t k = start; k <= end; k++)
        {
            if (count < 11U)
                tmp[count++] = src[k];
        }

        sort_small_float_array(tmp, count);
        dst[i] = tmp[count / 2U];
    }
}

static void exp_lowpass_filter(const float *src, float *dst, uint32_t n, float alpha)
{
    if (n == 0U)
        return;

    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    dst[0] = src[0];

    for (uint32_t i = 1U; i < n; i++)
    {
        dst[i] = alpha * src[i] + (1.0f - alpha) * dst[i - 1U];
    }
}

// 3rd method - noise filtering Savitzky-Golay style derivative
static void sg_derivative_5point(const float *src, float *dst, uint32_t n)
{
    if (n == 0U)
        return;

    if (n < 5U)
    {
        dst[0] = 0.0f;
        for (uint32_t i = 1U; i < n; i++)
            dst[i] = src[i] - src[i - 1U];
        return;
    }

    dst[0] = 0.0f;
    dst[1] = src[2] - src[0];

    for (uint32_t i = 2U; i < (n - 2U); i++)
    {
        dst[i] = (-2.0f * src[i - 2U]
                 - 1.0f * src[i - 1U]
                 + 1.0f * src[i + 1U]
                 + 2.0f * src[i + 2U]) / 10.0f;
    }

    dst[n - 2U] = src[n - 1U] - src[n - 3U];
    dst[n - 1U] = 0.0f;
}

static void sg_smooth_5point(const float *src, float *dst, uint32_t n)
{
    if (n == 0U)
        return;

    if (n < 5U)
    {
        for (uint32_t i = 0; i < n; i++)
            dst[i] = src[i];
        return;
    }

    dst[0] = src[0];
    dst[1] = src[1];

    for (uint32_t i = 2U; i < (n - 2U); i++)
    {
        // 5-point quadratic SG smoothing:
        // y[i] = (-3y[i-2] + 12y[i-1] + 17y[i] + 12y[i+1] - 3y[i+2]) / 35
        dst[i] = (-3.0f * src[i - 2U]
                 +12.0f * src[i - 1U]
                 +17.0f * src[i]
                 +12.0f * src[i + 1U]
                 -3.0f * src[i + 2U]) / 35.0f;
    }

    dst[n - 2U] = src[n - 2U];
    dst[n - 1U] = src[n - 1U];
}

static float mean_range(const float *x, uint32_t i0, uint32_t i1)
{
    if (i1 < i0)
        return 0.0f;

    float sum = 0.0f;
    uint32_t count = 0U;
    for (uint32_t i = i0; i <= i1; i++)
    {
        sum += x[i];
        count++;
    }
    return (count > 0U) ? (sum / (float)count) : 0.0f;
}

static float erf_model_point(float idx, float x0_idx, float w_idx, float y_lo, float y_hi)
{
    if (w_idx < T0_FIT_W_MIN)
        w_idx = T0_FIT_W_MIN;

    float a = 0.5f * (y_hi - y_lo);
    float b = 0.5f * (y_hi + y_lo);
    float z = (idx - x0_idx) / w_idx;

    return b + a * erff(z);
}

static float erf_fit_sse(
    const float *y, uint32_t n,
    float x0_idx, float w_idx, float y_lo, float y_hi
)
{
    float sse = 0.0f;

    for (uint32_t i = 0U; i < n; i++)
    {
        float e = y[i] - erf_model_point((float)i, x0_idx, w_idx, y_lo, y_hi);
        sse += e * e;
    }
    return sse;
}

static void erf_fit_curve(
    const float *y, uint32_t n, float dt_s,
    float *x0_idx, float *w_idx, float *y_lo, float *y_hi
)
{
    (void)dt_s; // kept in signature in case you want time-domain tuning later

    if (n < 10U)
    {
        *x0_idx = (float)(n / 2U);
        *w_idx  = 10.0f;
        *y_lo   = y[0];
        *y_hi   = y[n - 1U];
        return;
    }

    uint32_t edge_n = n / 10U;
    if (edge_n < 5U) edge_n = 5U;
    if (edge_n > (n / 3U)) edge_n = n / 3U;

    *y_lo = mean_range(y, 0U, edge_n - 1U);
    *y_hi = mean_range(y, n - edge_n, n - 1U);

    // initial x0 guess from midpoint crossing
    float mid = 0.5f * ((*y_lo) + (*y_hi));
    uint32_t best_mid_idx = 0U;
    float best_mid_err = fabsf(y[0] - mid);

    for (uint32_t i = 1U; i < n; i++)
    {
        float err = fabsf(y[i] - mid);
        if (err < best_mid_err)
        {
            best_mid_err = err;
            best_mid_idx = i;
        }
    }

    *x0_idx = (float)best_mid_idx;
    *w_idx  = 25.0f;

    float x_step = T0_FIT_X0_STEP_INIT;
    float w_step = T0_FIT_W_STEP_INIT;

    for (uint32_t iter = 0U; iter < T0_FIT_MAX_ITERS; iter++)
    {
        float best_x0 = *x0_idx;
        float best_w  = *w_idx;
        float best_sse = 1.0e30f;

        for (int xi = -3; xi <= 3; xi++)
        {
            float test_x0 = *x0_idx + ((float)xi * x_step);

            if (test_x0 < 0.0f)
                test_x0 = 0.0f;
            if (test_x0 > (float)(n - 1U))
                test_x0 = (float)(n - 1U);

            for (int wi = -3; wi <= 3; wi++)
            {
                float test_w = *w_idx + ((float)wi * w_step);
                if (test_w < T0_FIT_W_MIN)
                    test_w = T0_FIT_W_MIN;

                float sse = erf_fit_sse(y, n, test_x0, test_w, *y_lo, *y_hi);
                if (sse < best_sse)
                {
                    best_sse = sse;
                    best_x0 = test_x0;
                    best_w  = test_w;
                }
            }
        }

        *x0_idx = best_x0;
        *w_idx  = best_w;

        x_step *= 0.45f;
        w_step *= 0.45f;

        if (x_step < 0.25f) x_step = 0.25f;
        if (w_step < 0.25f) w_step = 0.25f;
    }
}

static void build_erf_fit_and_derivative(
    float *fit_y, float *fit_d, uint32_t n, float dt_s,
    float x0_idx, float w_idx, float y_lo, float y_hi
)
{
    if (w_idx < T0_FIT_W_MIN)
        w_idx = T0_FIT_W_MIN;

    float a = 0.5f * (y_hi - y_lo);
    const float inv_sqrt_pi = 0.5641895835f; // 1/sqrt(pi)

    for (uint32_t i = 0U; i < n; i++)
    {
        float x = (float)i;
        float z = (x - x0_idx) / w_idx;

        fit_y[i] = 0.5f * (y_hi + y_lo) + a * erff(z);

        // derivative wrt time
        // d/dx erf((x-x0)/w) = (2/sqrt(pi)) * exp(-z^2) / w
        // dx/dt = 1/dt_s, since x is sample index
        fit_d[i] = a * (2.0f * inv_sqrt_pi) * expf(-(z * z)) / (w_idx * dt_s);
    }
}

// Finding t0 from the raw initial scan buffer.
// This keeps the known-good SG -> erf fit -> derivative flow, but bins long
// raw scans down to PROFILE_MAX_POINTS so it fits the existing scratch space.
static float profile_find_t0_fit_index_from_raw_power(
    const uint16_t *raw_buf,
    const uint8_t *stage_buf,
    uint32_t n
)
{
    uint32_t i;
    uint32_t best_idx = 0U;
    uint32_t fit_n = n;
    float fit_x0_idx = 0.0f;
    float fit_w_idx  = 0.0f;
    float fit_y_lo   = 0.0f;
    float fit_y_hi   = 0.0f;
    float max_abs_d  = -1.0f;
    float idx_scale = 1.0f;
    float dt_s = ADC_DT_S;

    if (n < 5U)
        return 0.0f;

    if (fit_n > PROFILE_MAX_POINTS)
    {
        fit_n = PROFILE_MAX_POINTS;
        idx_scale = (float)n / (float)fit_n;
        dt_s *= idx_scale;
    }

    /* -------------------------------------------------------------
     * 1) Convert raw ADC samples -> power in binned sample-index space
     *    profile_power_mw = raw/bin-averaged power
     *    profile_v_avg    = scratch / SG output / fitted erf
     * ------------------------------------------------------------- */
    for (i = 0U; i < fit_n; i++)
    {
        uint32_t start = (uint32_t)((float)i * idx_scale);
        uint32_t end = (i == (fit_n - 1U))
                     ? n
                     : (uint32_t)((float)(i + 1U) * idx_scale);
        float sum_p = 0.0f;
        uint32_t count = 0U;

        if (start >= n)
            start = n - 1U;
        if (end <= start)
            end = start + 1U;
        if (end > n)
            end = n;

        for (uint32_t j = start; j < end; j++)
        {
            float v = adc_counts_to_voltage((float)raw_buf[j]);
            sum_p += voltage_to_power_mw_for_stage(v, stage_buf[j]);
            count++;
        }

        profile_power_mw[i] = (count > 0U) ? (sum_p / (float)count) : 0.0f;
        profile_v_avg[i] = 0.0f;
    }

    /* -------------------------------------------------------------
     * 2) SG smooth raw power
     * ------------------------------------------------------------- */
    sg_smooth_5point(profile_power_mw, profile_v_avg, fit_n);

    /* -------------------------------------------------------------
     * 3) Fit erf to SG-smoothed power
     * ------------------------------------------------------------- */
    erf_fit_curve(
        profile_v_avg,
        fit_n,
        dt_s,
        &fit_x0_idx,
        &fit_w_idx,
        &fit_y_lo,
        &fit_y_hi
    );

    /* -------------------------------------------------------------
     * 4) Build fitted erf and derivative of fitted erf
     * ------------------------------------------------------------- */
    build_erf_fit_and_derivative(
        profile_v_avg,
        profile_power_mw,
        fit_n,
        dt_s,
        fit_x0_idx,
        fit_w_idx,
        fit_y_lo,
        fit_y_hi
    );

    /* -------------------------------------------------------------
     * 5) Find T0 from peak absolute derivative
     * ------------------------------------------------------------- */
    for (i = 0U; i < fit_n; i++)
    {
        float ad = fabsf(profile_power_mw[i]);
        if (ad > max_abs_d)
        {
            max_abs_d = ad;
            best_idx = i;
        }
    }


    return (float)best_idx * idx_scale;
}

static void host_profile_compute_local_t0(void)
{
    float fit_idx;
    int32_t dir = profile_get_active_count_direction();
    int32_t init_counts_measured;
    int32_t counts_after_t0;
    int32_t first_repeat_target_counts;

    host_local_t0_ready = 0U;

    if (profile_raw_count < 5U)
    {
        printf("PROFILE_T0_FAIL,axis=%u,reason=too_few_samples,n=%lu\r\n",
               (unsigned int)profiler.axis,
               (unsigned long)profile_raw_count);
        return;
    }

    fit_idx = profile_find_t0_fit_index_from_raw_power(
        profile_raw_power_buf,
        profile_raw_stage_buf,
        profile_raw_count
    );

    profiler.t0_idx = (uint32_t)(fit_idx + 0.5f);
    if (profiler.t0_idx >= profile_raw_count)
        profiler.t0_idx = profile_raw_count - 1U;

    profiler.t0_enc_count =
        interp_encoder_count_at_index(profile_raw_enc_buf, profile_raw_count, fit_idx);

    profiler.x0_counts =
        enc_delta_counts(profiler.t0_enc_count, profiler.scan_start_enc_count);

    if (dir < 0)
        profiler.x0_counts = -profiler.x0_counts;

    profiler.t0_s = fit_idx * ADC_DT_S * (float)PROFILE_CAPTURE_STORE_STRIDE;

    init_counts_measured =
        enc_delta_counts(profiler.scan_end_enc_count, profiler.scan_start_enc_count);

    if (dir < 0)
        init_counts_measured = -init_counts_measured;

    if (init_counts_measured < 0)
        init_counts_measured = 0;

    counts_after_t0 =
        enc_delta_counts(profiler.scan_end_enc_count, profiler.t0_enc_count);

    if (dir < 0)
        counts_after_t0 = -counts_after_t0;

    if (counts_after_t0 < 0)
        counts_after_t0 = 0;

    // Reject obviously bad T0 values near the scan edges
    if ((profiler.x0_counts < 200) ||
        (profiler.x0_counts > (PROFILE_INIT_COUNTS - 200)))
    {
        printf("PROFILE_T0_FAIL,axis=%u,reason=edge_t0,fit_idx=%.3f,x0=%ld,init=%ld,after_t0=%ld\r\n",
               (unsigned int)profiler.axis,
               fit_idx,
               (long)profiler.x0_counts,
               (long)init_counts_measured,
               (long)counts_after_t0);
        return;
    }

    // First repeat start target from the initial scan start:
    // xs1 = xwait + xs/2 + x0
    first_repeat_target_counts =
        (int32_t)(PROFILE_WAIT_COUNTS
                + (PROFILE_REPEAT_COUNTS / 2)
                + profiler.x0_counts);

    while (first_repeat_target_counts < init_counts_measured)
    {
        first_repeat_target_counts += PROFILE_EVENT_COUNTS;
    }

    profiler.xdelay_counts = first_repeat_target_counts - init_counts_measured;

    while (profiler.xdelay_counts < 0)
    {
        profiler.xdelay_counts += PROFILE_EVENT_COUNTS;
    }

    host_first_wait_counts = profiler.xdelay_counts;

    // IMPORTANT: xdelay is from initial scan END, not start
    profiler.next_scan_enc_target =
        profiler.scan_end_enc_count + (dir * profiler.xdelay_counts);

    printf("PROFILE_T0_ENC,axis=%u,fit_idx=%.3f,idx=%lu,t0_enc=%ld,x0=%ld,init=%ld,after_t0=%ld,xs1=%ld,xdelay=%ld,next=%ld\r\n",
           (unsigned int)profiler.axis,
           fit_idx,
           (unsigned long)profiler.t0_idx,
           (long)profiler.t0_enc_count,
           (long)profiler.x0_counts,
           (long)init_counts_measured,
           (long)counts_after_t0,
           (long)first_repeat_target_counts,
           (long)profiler.xdelay_counts,
           (long)profiler.next_scan_enc_target);

    host_local_t0_ready = 1U;
}

// Converts fractional indes into precise encoder count
static int32_t interp_encoder_count_at_index(const int32_t *enc_buf, uint32_t n, float idx_f)
{
    if (n == 0U)
        return 0;

    if (idx_f <= 0.0f)
        return enc_buf[0];
    if (idx_f >= (float)(n - 1U))
        return enc_buf[n - 1U];

    uint32_t i0 = (uint32_t)idx_f;
    uint32_t i1 = i0 + 1U;
    float a = idx_f - (float)i0;

    float enc_f = (1.0f - a) * (float)enc_buf[i0] + a * (float)enc_buf[i1];
    return (int32_t)(enc_f + 0.5f);
}

//static uint32_t profile_find_t0_from_profile_buffer(void)
//{
//    uint32_t n = profile_point_count;
//    uint32_t best_idx = 0U;
//    uint32_t i;
//    float fit_x0_idx = 0.0f;
//    float fit_w_idx  = 0.0f;
//    float fit_y_lo   = 0.0f;
//    float fit_y_hi   = 0.0f;
//    float max_abs_d  = -1.0f;
//    float dt_s;
//
//    if (n < 5U)
//        return 0U;
//
//    if (n > T0_RAW_MAX_SAMPLES)
//        n = T0_RAW_MAX_SAMPLES;
//
//    dt_s = ((float)PROFILE_DECIM) * ADC_DT_S;
//
//    /* -----------------------------------------------------------------
//     * 1) Copy raw power from the profile buffer
//     * ----------------------------------------------------------------- */
//    for (i = 0U; i < n; i++)
//    {
//        t0_power_buf[i] = profile_power_mw[i];
//        t0_work_buf[i]  = 0.0f;
//    }
//
//    /* -----------------------------------------------------------------
//     * 2) SG smooth the raw power
//     *    input  = t0_power_buf  (raw power)
//     *    output = t0_work_buf   (SG-smoothed power)
//     * ----------------------------------------------------------------- */
//    sg_smooth_5point(t0_power_buf, t0_work_buf, n);
//
//    /* Optional debug: print a few points if needed
//    printf("PROFILE_T0_DBG,axis=%u,n=%lu,dt_s=%.8f\r\n",
//           (unsigned int)profiler.axis,
//           (unsigned long)n,
//           dt_s);
//    */
//
//    /* -----------------------------------------------------------------
//     * 3) Fit erf to the SG-smoothed power
//     *    fit_x0_idx is in sample-index units
//     * ----------------------------------------------------------------- */
//    erf_fit_curve(
//        t0_work_buf,
//        n,
//        dt_s,
//        &fit_x0_idx,
//        &fit_w_idx,
//        &fit_y_lo,
//        &fit_y_hi
//    );
//
//    /* -----------------------------------------------------------------
//     * 4) Build fitted erf and derivative of fitted erf
//     *
//     * After this call:
//     *   t0_work_buf  = fitted erf curve
//     *   t0_power_buf = derivative of fitted erf
//     * ----------------------------------------------------------------- */
//    build_erf_fit_and_derivative(
//        t0_work_buf,   /* fitted erf curve */
//        t0_power_buf,  /* derivative of fitted erf */
//        n,
//        dt_s,
//        fit_x0_idx,
//        fit_w_idx,
//        fit_y_lo,
//        fit_y_hi
//    );
//
//    /* -----------------------------------------------------------------
//     * 5) Pick T0 from fitted erf center, not raw derivative peak
//     * ----------------------------------------------------------------- */
//    {
//        int32_t x0_rounded = (int32_t)(fit_x0_idx + 0.5f);
//
//        if (x0_rounded < 0)
//            x0_rounded = 0;
//        if ((uint32_t)x0_rounded >= n)
//            x0_rounded = (int32_t)(n - 1U);
//
//        best_idx = (uint32_t)x0_rounded;
//    }
//}

//static uint8_t profile_find_t0_for_scheduler(ProfileAxis_t axis)
//{
//    uint8_t locked;
//    float omega_rad_s;
//    float scan_time_s;
//    uint32_t nsamp;
//    uint32_t t0_ms_start;
//    uint32_t i;
//    uint32_t best_idx = 0U;
//    uint32_t last_cap_speed_update_ms;
//    float t0_v;
//    float t0_p_mw;
//
//    float fit_x0_idx = 0.0f;
//    float fit_w_idx  = 0.0f;
//    float fit_y_lo   = 0.0f;
//    float fit_y_hi   = 0.0f;
//    float max_abs_d_fit = -1.0f;
//
//    profiler.axis = axis;
//
//    profile_locked_rpm = 0.0f;
//    profile_locked_omega_rad_s = 0.0f;
//    profile_rpm_stable_count = 0U;
//
//    dc_motor_start_profile_axis(axis);
//
//    // initial spin-up
//    HAL_Delay(PROFILE_SPINUP_MS);
//
//    // wait until active axis RPM is stable and lock omega
//    locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);
//
//    omega_rad_s = profile_locked_omega_rad_s;
//    if (omega_rad_s <= 1e-6f)
//        omega_rad_s = beam_profile_get_active_omega();
//
//    if (omega_rad_s < 0.2f)
//    {
//        cdc_printf_blocking("T0_BAD_OMEGA,axis=%u,locked=%u,rpm=%.4f,omega=%.6f\r\n",
//                            (unsigned int)axis,
//                            (unsigned int)locked,
//                            profile_locked_rpm,
//                            omega_rad_s);
//        dc_motors_stop_all();
//        return 0U;
//    }
//
//    // Use same initial scan duration logic
//    scan_time_s = profile_initial_scan_time_s(omega_rad_s);
//
//    nsamp = (uint32_t)(scan_time_s * ADC_FS_HZ + 0.5f);
//    if (nsamp < 16U)
//        nsamp = 16U;
//    if (nsamp > T0_RAW_MAX_SAMPLES)
//        nsamp = T0_RAW_MAX_SAMPLES;
//
//    t0_raw_count = nsamp;
//
//    cdc_printf_blocking("T0_BEGIN,axis=%u,locked=%u,rpm=%.4f,omega=%.6f,scan_time_s=%.6f,nsamp=%lu\r\n",
//                        (unsigned int)axis,
//                        (unsigned int)locked,
//                        profile_locked_rpm,
//                        omega_rad_s,
//                        scan_time_s,
//                        (unsigned long)nsamp);
//
//    // freeze gain switching during capture
//    gain_settle_skip = 1U;
//    gain_settle_skip_live = 1U;
//
//    // stop background circular DMA before finite burst capture
//    HAL_ADC_Stop_DMA(&hadc4);
//    HAL_TIM_Base_Stop(&htim2);
//
//    adc_dma_done = 0U;
//    burst_capture_active = 1U;
//    adc_half_ready = 0U;
//    adc_full_ready = 0U;
//    live_sample_ready = 0U;
//
//    __HAL_TIM_SET_COUNTER(&htim2, 0);
//    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
//
//    memset(t0_raw_buf, 0, sizeof(t0_raw_buf));
//
//    if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)t0_raw_buf, nsamp) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    t0_ms_start = HAL_GetTick();
//    last_cap_speed_update_ms = t0_ms_start;
//
//    while (!adc_dma_done)
//    {
//        uint32_t now = HAL_GetTick();
//
//        // keep motor regulation alive during capture
//        if ((now - last_cap_speed_update_ms) >= DC_SPEED_SAMPLE_MS)
//        {
//            float dt_s = (float)(now - last_cap_speed_update_ms) / 1000.0f;
//
//            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//            dc_motor_auto_correct_active_axis();
//
//            last_cap_speed_update_ms = now;
//        }
//
//        if ((now - t0_ms_start) > 3000U)
//        {
//            HAL_TIM_Base_Stop(&htim2);
//            HAL_ADC_Stop_DMA(&hadc4);
//            burst_capture_active = 0U;
//
//            gain_settle_skip = 0U;
//            gain_settle_skip_live = 0U;
//
//            dc_motors_stop_all();
//            cdc_printf_blocking("T0_TIMEOUT,axis=%u\r\n", (unsigned int)axis);
//            return 0U;
//        }
//    }
//
//    HAL_TIM_Base_Stop(&htim2);
//    HAL_ADC_Stop_DMA(&hadc4);
//    burst_capture_active = 0U;
//
//    // restart background circular stream
//    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    // raw ADC -> power
//    for (i = 0U; i < nsamp; i++)
//    {
//        float v_meas = adc_counts_to_voltage((float)t0_raw_buf[i]);
//        t0_power_buf[i] = voltage_to_power_mw(v_meas);
//    }
//
//    // SG smooth -> erf fit -> derivative of fit
//    sg_smooth_5point(t0_power_buf, t0_work_buf, nsamp);
//
//    erf_fit_curve(
//        t0_work_buf,
//        nsamp,
//        ADC_DT_S,
//        &fit_x0_idx,
//        &fit_w_idx,
//        &fit_y_lo,
//        &fit_y_hi
//    );
//
//    build_erf_fit_and_derivative(
//        t0_work_buf,   // fitted curve
//        t0_power_buf,  // derivative of fitted erf
//        nsamp,
//        ADC_DT_S,
//        fit_x0_idx,
//        fit_w_idx,
//        fit_y_lo,
//        fit_y_hi
//    );
//
//    for (i = 0U; i < nsamp; i++)
//    {
//        float ad = fabsf(t0_power_buf[i]);
//        if (ad > max_abs_d_fit)
//        {
//            max_abs_d_fit = ad;
//            best_idx = i;
//        }
//    }
//
//    profiler.t0_idx = best_idx;
//    profiler.t0_s   = ((float)best_idx) / ADC_FS_HZ;
//    profiler.omega_rad_s = omega_rad_s;
//
//    t0_v    = adc_counts_to_voltage((float)t0_raw_buf[profiler.t0_idx]);
//    t0_p_mw = t0_work_buf[profiler.t0_idx];
//
//    cdc_printf_blocking("PROFILE_T0,axis=%u,idx=%lu,t0_s=%.6f,t0_ms=%.3f,p_mw=%.6f\r\n",
//                        (unsigned int)axis,
//                        (unsigned long)profiler.t0_idx,
//                        profiler.t0_s,
//                        profiler.t0_s * 1000.0f,
//                        t0_p_mw);
//
//    cdc_printf_blocking("PROFILE_T0_FIT,axis=%u,x0_idx=%.3f,w_idx=%.3f,ylo=%.6f,yhi=%.6f,best_idx=%lu,V=%.6f\r\n",
//                        (unsigned int)axis,
//                        fit_x0_idx,
//                        fit_w_idx,
//                        fit_y_lo,
//                        fit_y_hi,
//                        (unsigned long)best_idx,
//                        t0_v);
//
//    // optional raw stream for debugging
//    cdc_printf_blocking("T0_RAW_BEGIN,axis=%u,nsamp=%lu,fs=%.1f,y=raw,fit,dfit\r\n",
//                        (unsigned int)axis,
//                        (unsigned long)nsamp,
//                        ADC_FS_HZ);
//
//    for (i = 0U; i < nsamp; i++)
//    {
//        float t_ms_sample = ((float)i * 1000.0f) / ADC_FS_HZ;
//        float raw_p = voltage_to_power_mw(adc_counts_to_voltage((float)t0_raw_buf[i]));
//
//        cdc_printf_blocking("RAW,%u,%lu,%.6f,%.6f,%.6f,%.6f\r\n",
//                            (unsigned int)axis,
//                            (unsigned long)i,
//                            t_ms_sample,
//                            raw_p,
//                            t0_work_buf[i],
//                            t0_power_buf[i]);
//    }
//
//    cdc_printf_blocking("T0_RAW_DONE,axis=%u\r\n", (unsigned int)axis);
//
//    // IMPORTANT for scheduler use:
//    // do NOT wait for recorded T0 position
//    // do NOT stop the motors
//    // do re-enable normal live gain behavior
//    gain_settle_skip = 0U;
//    gain_settle_skip_live = 0U;
//
//    return 1U;
//}

// -----------------------------
// FINDING T0 Helpers
// -----------------------------
static void t0_batch_begin(ProfileAxis_t axis, uint8_t num_valid_scans)
{
    t0_batch_active = 1U;
    t0_batch_state = T0_BATCH_WAIT_STABLE;

    profiler.axis = axis;
    t0_batch_axis = (uint8_t)axis;
    t0_batch_num_scans = num_valid_scans;
    t0_batch_scan_index = 0U;
    t0_batch_valid_scans = 0U;

    host_local_t0_ready = 0U;
    profiler.t0_idx = 0U;
    profiler.t0_enc_count = 0;
    profiler.x0_counts = 0;
    profiler.xdelay_counts = 0;
    profiler.next_scan_enc_target = 0;

    profile_locked_rpm = 0.0f;
    profile_locked_omega_rad_s = 0.0f;
    profile_rpm_stable_count = 0U;

    host_repeat_buffer_reset();

    dc_motor_start_profile_axis(axis);   // start motor ONCE
    HAL_Delay(PROFILE_SPINUP_MS);        // spin up ONCE

    reset_axis_encoder(axis);
    if (axis == PROFILE_AXIS_M1)
    {
        dc1.prev_count = enc_read_axis_count(PROFILE_AXIS_M1);
        dc1.delta_count = 0;
        dc1.rpm = 0.0f;
        dc1.omega = 0.0f;
    }
    else
    {
        dc2.prev_count = enc_read_axis_count(PROFILE_AXIS_M2);
        dc2.delta_count = 0;
        dc2.rpm = 0.0f;
        dc2.omega = 0.0f;
    }

    last_dc_speed_ms = HAL_GetTick();

    printf("T0_BATCH_BEGIN,axis=%u,num_scans=%u\r\n",
           (unsigned int)t0_batch_axis,
           (unsigned int)t0_batch_num_scans);
}

//static void t0_batch_begin(ProfileAxis_t axis, uint8_t num_scans)
//{
//    t0_batch_active      = 1U;
//    t0_batch_axis        = (uint8_t)axis;
//    t0_batch_num_scans   = num_scans;
//    t0_batch_scan_index  = 0U;
//    t0_batch_valid_scans = 0U;
//    t0_batch_total_axes  = 1U;   // default: single axis; set to 2 externally if needed
//
//    profiler.axis        = axis;
//    host_local_t0_ready  = 0U;
//
//    if (t0_batch_num_scans > 10U) t0_batch_num_scans = 10U;
//    if (t0_batch_num_scans == 0U) t0_batch_num_scans = 1U;
//
//    profile_locked_rpm           = 0.0f;
//    profile_locked_omega_rad_s   = 0.0f;
//    profile_rpm_stable_count     = 0U;
//
//    dc_motor_start_profile_axis(axis);
//    HAL_Delay(PROFILE_SPINUP_MS);
//    reset_axis_encoder(axis);
//
//    t0_batch_state = T0_BATCH_WAIT_STABLE;
//
//    printf("T0_BATCH_BEGIN,axis=%u,num_scans=%u\r\n",
//           (unsigned int)axis,
//           (unsigned int)t0_batch_num_scans);
//}

//static void t0_batch_begin(ProfileAxis_t axis, uint8_t num_valid_scans)
//{
//    profiler.axis = axis;
//
//    t0_batch_axis = (uint8_t)axis;
//    t0_batch_target_valid_scans = num_valid_scans;
//    t0_batch_valid_scans = 0U;
//    t0_batch_attempt_count = 0U;
//    t0_batch_emit_scan_no = 0U;
//
//    t0_batch_active = 1U;
//    t0_batch_state = T0_BATCH_WAIT_STABLE;
//
//    t0_dump_idx = 0U;
//    t0_dump_started = 0U;
//    t0_dump_emit_stride = PROFILE_HOST_EMIT_STRIDE;
//
//    host_repeat_buffer_reset();
//
//    printf("T0_BATCH_BEGIN,axis=%u,target_valid=%u,max_attempts=%u\r\n",
//           (unsigned int)axis,
//           (unsigned int)t0_batch_target_valid_scans,
//           (unsigned int)T0_BATCH_MAX_ATTEMPTS);
//}

static uint8_t t0_batch_scan_is_valid(void)
{
    if (profile_raw_count < 50U)
        return 0U;

    if (profiler.t0_idx >= profile_raw_count)
        return 0U;

    if (beam_profile_get_active_omega() < 0.2f)
        return 0U;

    int32_t enc_span = profile_raw_enc_buf[profile_raw_count - 1U] - profile_raw_enc_buf[0];
    if (enc_span < (PROFILE_INIT_COUNTS / 2))
        return 0U;

    return 1U;
}

//static uint8_t t0_dump_scan_step(ProfileAxis_t axis, uint8_t scan_no)
//{
//    float counts_per_rev = profile_get_active_counts_per_rev();
//    uint32_t emitted_n = 0U;
//    uint32_t lines_this_call = 0U;
//    const uint32_t max_lines_per_call = 8U;
//
//    if (!t0_dump_started)
//    {
//        t0_dump_emit_stride = 1U;
//        while ((profile_raw_count / t0_dump_emit_stride) > 300U)
//            t0_dump_emit_stride++;
//
//        t0_dump_idx = 0U;
//        t0_dump_started = 1U;
//
//        printf("RAW_SCAN_BEGIN,axis=%u,scan=%u,stride=1,emit_stride=%lu\r\n",
//               (unsigned int)axis,
//               (unsigned int)scan_no,
//               (unsigned long)t0_dump_emit_stride);
//    }
//
//    while ((t0_dump_idx < profile_raw_count) && (lines_this_call < max_lines_per_call))
//    {
//        uint32_t i = t0_dump_idx;
//        int32_t enc = profile_raw_enc_buf[i];
//
//        float deg = profile_counts_to_deg(
//            enc,
//            profiler.scan_start_enc_count,
//            counts_per_rev
//        );
//
//        float x_mm = profile_counts_to_x_mm(
//            enc,
//            profiler.scan_start_enc_count,
//            counts_per_rev
//        );
//
//        float v = adc_counts_to_voltage((float)profile_raw_power_buf[i]);
//        float p = voltage_to_power_mw_for_stage(v, profile_raw_stage_buf[i]);
//
//        printf("RAW_SCAN,%u,%u,%lu,%ld,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
//               (unsigned int)axis,
//               (unsigned int)scan_no,
//               (unsigned long)i,
//               (long)enc,
//               deg,
//               x_mm,
//               (unsigned int)profile_raw_stage_buf[i],
//               (unsigned int)profile_raw_power_buf[i],
//               v,
//               p);
//
//        t0_dump_idx += t0_dump_emit_stride;
//        lines_this_call++;
//
//        if ((lines_this_call % 4) == 0U)
//			CDC_TX_Kick();
//    }
//
//    if (t0_dump_idx >= profile_raw_count)
//    {
//        emitted_n = (profile_raw_count + t0_dump_emit_stride - 1U) / t0_dump_emit_stride;
//        CDC_TX_Kick();
//
//        printf("RAW_SCAN_DONE,axis=%u,scan=%u,n=%lu\r\n",
//               (unsigned int)axis,
//               (unsigned int)scan_no,
//               (unsigned long)emitted_n);
//        CDC_TX_Kick();
//
//        t0_dump_started = 0U;
//        t0_dump_idx = 0U;
//        return 1U;
//    }
//
//    return 0U;
//}

static uint8_t t0_dump_scan_step(ProfileAxis_t axis, uint8_t scan_no)
{
    float    counts_per_rev = profile_get_active_counts_per_rev();
    uint32_t lines_this_call = 0U;
    const uint32_t max_lines_per_call = 8U;

    if (!t0_dump_started)
    {
        uint32_t n = profile_raw_count;

        t0_dump_idx         = 0U;
        t0_dump_win_end     = n;
        t0_dump_emit_stride = 1U;
        t0_dump_started     = 1U;

        CDC_TX_Kick();
        HAL_Delay(2);

        printf("RAW_SCAN_BEGIN,axis=%u,scan=%u,stride=%lu,emit_stride=1,"
               "t0_idx=%lu,win_start=0,win_end=%lu,n=%lu,full_window=1\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned long)PROFILE_CAPTURE_STORE_STRIDE,
               (unsigned long)profiler.t0_idx,
               (unsigned long)n,
               (unsigned long)n);

        CDC_TX_Kick();
        return 0U;
    }

    while ((t0_dump_idx < t0_dump_win_end) && (lines_this_call < max_lines_per_call))
    {
        uint32_t i   = t0_dump_idx;
        int32_t  enc = profile_raw_enc_buf[i];

        float deg  = profile_counts_to_deg(enc,
                         profiler.scan_start_enc_count, counts_per_rev);
        float x_mm = profile_counts_to_x_mm(enc,
                         profiler.scan_start_enc_count, counts_per_rev);
        float v    = adc_counts_to_voltage((float)profile_raw_power_buf[i]);
        float p    = voltage_to_power_mw_for_stage(v, profile_raw_stage_buf[i]);

        printf("RAW_SCAN,%u,%u,%lu,%ld,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned long)i,
               (long)enc,
               deg, x_mm,
               (unsigned int)profile_raw_stage_buf[i],
               (unsigned int)profile_raw_power_buf[i],
               v, p);

        t0_dump_idx++;
        lines_this_call++;

        if ((lines_this_call % 4U) == 0U)
            CDC_TX_Kick();
    }

    if (t0_dump_idx >= t0_dump_win_end)
    {
        uint32_t emitted_n = t0_dump_win_end;

        CDC_TX_Kick();

        printf("RAW_SCAN_DONE,axis=%u,scan=%u,n=%lu,full_window=1\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned long)emitted_n);

        CDC_TX_Kick();

        t0_dump_started = 0U;
        t0_dump_idx     = 0U;
        t0_dump_win_end = 0U;

        return 1U;
    }

    return 0U;
}

static void t0_batch_update(void)
{
    if (!t0_batch_active)
        return;

    // Motor regulation on every call regardless of sub-state.
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
            dc_motor_auto_correct_active_axis();
            last_dc_speed_ms = now;
        }
    }

    switch (t0_batch_state)
    {
        case T0_BATCH_IDLE:
        default:
            break;

        // --------------------------------------------------
        // Block until the active axis reaches stable RPM.
        // live_power_update_service() is called internally.
        // --------------------------------------------------
        case T0_BATCH_WAIT_STABLE:
        {
            uint8_t locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);

            if (beam_profile_get_active_omega() < 0.2f)
            {
                printf("T0_BATCH_BAD_OMEGA,axis=%u,scan=%u,locked=%u,rpm=%.4f,omega=%.6f\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (unsigned int)locked,
                       profile_locked_rpm,
                       profile_locked_omega_rad_s);

                t0_batch_state = T0_BATCH_DONE;
                break;
            }

            last_dc_speed_ms = HAL_GetTick();

            beam_profile_start_window_counts(PROFILE_INIT_COUNTS);

            printf("T0_SCAN_BEGIN,axis=%u,scan=%u,counts=%d\r\n",
                   (unsigned int)t0_batch_axis,
                   (unsigned int)t0_batch_scan_index,
                   (int)PROFILE_INIT_COUNTS);

            t0_batch_state = T0_BATCH_CAPTURE;
        }
        break;

        // --------------------------------------------------
        // Sample ADC + encoder until encoder window complete.
        // Motor regulation handled by top-of-function block.
        // --------------------------------------------------
        case T0_BATCH_CAPTURE:
            live_power_update_service();
            beam_profile_sample_position_window();

            if (profile_raw_count > PROFILE_CAPTURE_MAX_SAMPLES)
            {
                printf("T0_SCAN_OVERFLOW,axis=%u,scan=%u,n=%lu\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (unsigned long)profile_raw_count);
                t0_batch_state = T0_BATCH_NEXT;
                break;
            }

            if (beam_profile_window_done_counts())
            {
                printf("T0_SCAN_CAPTURE_DONE,axis=%u,scan=%u,n=%lu\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (unsigned long)profile_raw_count);

                t0_batch_state = T0_BATCH_COMPUTE;
            }
            break;

        // --------------------------------------------------
        // Run erf-fit T0 finder on the captured data.
        // --------------------------------------------------
//        case T0_BATCH_COMPUTE:
//        {
//            int32_t counts_travelled = enc_delta_counts(
//                profiler.scan_end_enc_count,
//                profiler.scan_start_enc_count
//            );
//            if (counts_travelled < 0)
//                counts_travelled = -counts_travelled;
//
//            if (counts_travelled < (PROFILE_INIT_COUNTS / 2))
//            {
//                printf("T0_SCAN_INVALID...\n");
//
//                // IMPORTANT: do NOT count this as a full attempt
//                if (t0_batch_scan_index > 0)
//                    t0_batch_scan_index--;   // undo attempt increment
//
//                t0_batch_state = T0_BATCH_NEXT;
//                break;
//            }
//
//            // Valid scan — run the fit and count it
//            host_profile_compute_local_t0();
//
//            t0_result_idx[t0_batch_valid_scans]       = profiler.t0_idx;
//            t0_result_enc[t0_batch_valid_scans]        = profiler.t0_enc_count;
//            t0_result_x0_counts[t0_batch_valid_scans] = profiler.x0_counts;
//            t0_result_ms[t0_batch_valid_scans]         = profiler.t0_s * 1000.0f;
//
//            t0_batch_state = T0_BATCH_DUMP;
//            break;
//        }

//        case T0_BATCH_COMPUTE:
//        {
//            uint8_t scan_valid;
//
//            t0_batch_attempt_count++;
//
//            profiler.t0_idx = profile_find_t0_from_profile_buffer();
//            profiler.t0_enc_count = interp_encoder_count_at_index(
//                profile_raw_enc_buf,
//                profile_raw_count,
//                (float)profiler.t0_idx
//            );
//            profiler.x0_counts = profiler.t0_enc_count - profile_raw_enc_buf[0];
//            profiler.t0_s = ((float)profiler.t0_idx) * ADC_DT_S * PROFILE_CAPTURE_STORE_STRIDE;
//
//            scan_valid = t0_batch_scan_is_valid();
//
//            if (scan_valid)
//            {
//                uint8_t out_scan_no = t0_batch_emit_scan_no;
//
//                t0_result_idx[out_scan_no] = profiler.t0_idx;
//                t0_result_enc[out_scan_no] = profiler.t0_enc_count;
//                t0_result_x0_counts[out_scan_no] = profiler.x0_counts;
//                t0_result_ms[out_scan_no] = profiler.t0_s * 1000.0f;
//
//                printf("T0_RESULT,axis=%u,scan=%u,idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
//                       (unsigned int)profiler.axis,
//                       (unsigned int)out_scan_no,
//                       (unsigned long)profiler.t0_idx,
//                       (long)profiler.t0_enc_count,
//                       (long)profiler.x0_counts,
//                       profiler.t0_s * 1000.0f);
//
//                t0_batch_valid_scans++;
//                t0_batch_emit_scan_no++;
//                t0_batch_state = T0_BATCH_DUMP;
//            }
//            else
//            {
//                printf("T0_FAIL,axis=%u,attempt=%u,valid=%u\r\n",
//                       (unsigned int)profiler.axis,
//                       (unsigned int)t0_batch_attempt_count,
//                       (unsigned int)t0_batch_valid_scans);
//
//                if (t0_batch_attempt_count >= T0_BATCH_MAX_ATTEMPTS)
//                {
//                    t0_batch_state = T0_BATCH_DONE;
//                }
//                else
//                {
//                    t0_batch_state = T0_BATCH_NEXT;
//                }
//            }
//        }
//        break;

        case T0_BATCH_COMPUTE:
        {
            float counts_per_rev       = profile_get_active_counts_per_rev();
            float edge_margin_counts   = (PROFILE_EDGE_MARGIN_DEG / 360.0f) * counts_per_rev;

            int32_t counts_travelled = enc_delta_counts(
                profiler.scan_end_enc_count,
                profiler.scan_start_enc_count
            );
            if (counts_travelled < 0)
                counts_travelled = -counts_travelled;

            // check scan travelled enough counts
            if (counts_travelled < (PROFILE_INIT_COUNTS / 2))
            {
                printf("T0_SCAN_INVALID,axis=%u,scan=%u,reason=short_scan,counts=%ld\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (long)counts_travelled);

                if (t0_batch_scan_index > 0)
                    t0_batch_scan_index--;

                t0_batch_state = T0_BATCH_NEXT;
                break;
            }

            // run erf fit and find T0
            host_profile_compute_local_t0();

            // check T0 not within edge margin
            int32_t counts_from_start = enc_delta_counts(
                profiler.t0_enc_count,
                profiler.scan_start_enc_count
            );
            if (counts_from_start < 0)
                counts_from_start = -counts_from_start;

            int32_t counts_from_end = enc_delta_counts(
                profiler.scan_end_enc_count,
                profiler.t0_enc_count
            );
            if (counts_from_end < 0)
                counts_from_end = -counts_from_end;

            if ((float)counts_from_start < edge_margin_counts ||
                (float)counts_from_end   < edge_margin_counts)
            {
                printf("T0_SCAN_INVALID,axis=%u,scan=%u,"
                       "reason=t0_near_edge,"
                       "from_start=%ld,from_end=%ld,margin=%.0f\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (long)counts_from_start,
                       (long)counts_from_end,
                       edge_margin_counts);

                if (t0_batch_scan_index > 0)
                    t0_batch_scan_index--;

                t0_batch_state = T0_BATCH_NEXT;
                break;
            }

            // valid — store results
            t0_result_idx[t0_batch_valid_scans]       = profiler.t0_idx;
            t0_result_enc[t0_batch_valid_scans]        = profiler.t0_enc_count;
            t0_result_x0_counts[t0_batch_valid_scans] = profiler.x0_counts;
            t0_result_ms[t0_batch_valid_scans]         = profiler.t0_s * 1000.0f;

            t0_batch_state = T0_BATCH_DUMP;
            break;
        }

        // --------------------------------------------------
        // Stream raw scan to host in small non-blocking bursts.
        // --------------------------------------------------
        case T0_BATCH_DUMP:
        	CDC_TX_Kick();
            if (t0_dump_scan_step((ProfileAxis_t)t0_batch_axis, t0_batch_valid_scans))
            {
            	CDC_TX_Kick();
                printf("T0_RESULT,axis=%u,scan=%u,idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_valid_scans,   // <-- valid index not raw index
                       (unsigned long)t0_result_idx[t0_batch_valid_scans],
                       (long)t0_result_enc[t0_batch_valid_scans],
                       (long)t0_result_x0_counts[t0_batch_valid_scans],
                       t0_result_ms[t0_batch_valid_scans]);

                CDC_TX_Kick();

                t0_batch_valid_scans++;   // only increment after confirmed valid + dumped
                t0_batch_state = T0_BATCH_NEXT;
            }
            break;
        // --------------------------------------------------
        // Advance scan counter. Either start the next scan
        // on this axis, or move to clear-beam when all scans
        // for this axis are done.
        // --------------------------------------------------
//        case T0_BATCH_NEXT:
//        {
//            t0_batch_scan_index++;   // total attempts
//
//            if (t0_batch_scan_index >= T0_BATCH_MAX_ATTEMPTS)
//            {
//                printf("T0_BATCH_MAX_ATTEMPTS_REACHED,axis=%u,valid=%u,attempts=%u\r\n",
//                       (unsigned int)t0_batch_axis,
//                       (unsigned int)t0_batch_valid_scans,
//                       (unsigned int)t0_batch_scan_index);
//
//                host_profile_learn_clear_threshold_from_initial_scan();
//                t0_batch_state = T0_BATCH_CLEAR_BEAM;
//                break;
//            }
//
//            if (t0_batch_valid_scans >= t0_batch_num_scans)
//            {
//                printf("T0_BATCH_SCANS_DONE,axis=%u,valid=%u,attempts=%u\r\n",
//                       (unsigned int)t0_batch_axis,
//                       (unsigned int)t0_batch_valid_scans,
//                       (unsigned int)t0_batch_scan_index);
//
//                host_profile_learn_clear_threshold_from_initial_scan();
//                t0_batch_state = T0_BATCH_CLEAR_BEAM;
//            }
//            else
//            {
//                // Need another valid scan, but keep the motor continuously running.
//                printf("T0_SCAN_RETRY,axis=%u,attempt=%u,valid_so_far=%u,need=%u\r\n",
//                       (unsigned int)t0_batch_axis,
//                       (unsigned int)t0_batch_scan_index,
//                       (unsigned int)t0_batch_valid_scans,
//                       (unsigned int)t0_batch_num_scans);
//
//                // Re-zero this axis so the next capture is a fresh 7000-count window
//                // relative to "now", without stopping or re-spinning the motor.
//                reset_axis_encoder((ProfileAxis_t)t0_batch_axis);
//
//                if ((ProfileAxis_t)t0_batch_axis == PROFILE_AXIS_M1)
//                {
//                    dc1.prev_count  = enc_read_axis_count(PROFILE_AXIS_M1);
//                    dc1.delta_count = 0;
//                }
//                else
//                {
//                    dc2.prev_count  = enc_read_axis_count(PROFILE_AXIS_M2);
//                    dc2.delta_count = 0;
//                }
//
//                last_dc_speed_ms = HAL_GetTick();
//
//                beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
//
//                printf("T0_SCAN_BEGIN,axis=%u,scan=%u,counts=%d\r\n",
//                       (unsigned int)t0_batch_axis,
//                       (unsigned int)t0_batch_scan_index,
//                       (int)PROFILE_INIT_COUNTS);
//
//                t0_batch_state = T0_BATCH_CAPTURE;
//            }
//        }
//        break;

        case T0_BATCH_NEXT:
        {
            t0_batch_scan_index++;   // always tracks total attempts

            if (t0_batch_scan_index >= T0_BATCH_MAX_ATTEMPTS)
            {
                printf("T0_BATCH_MAX_ATTEMPTS_REACHED,axis=%u,valid=%u,attempts=%u\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_valid_scans,
                       (unsigned int)t0_batch_scan_index);

                host_profile_learn_clear_threshold_from_initial_scan();
                t0_batch_state = T0_BATCH_CLEAR_BEAM;
                break;
            }

            if (t0_batch_valid_scans >= t0_batch_num_scans)
            {
                // Got enough valid scans
                printf("T0_BATCH_SCANS_DONE,axis=%u,valid=%u,attempts=%u\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_valid_scans,
                       (unsigned int)t0_batch_scan_index);

                host_profile_learn_clear_threshold_from_initial_scan();
                t0_batch_state = T0_BATCH_CLEAR_BEAM;
            }
            else
            {
                // Need more valid scans — try again
                printf("T0_SCAN_RETRY,axis=%u,attempt=%u,valid_so_far=%u,need=%u\r\n",
                       (unsigned int)t0_batch_axis,
                       (unsigned int)t0_batch_scan_index,
                       (unsigned int)t0_batch_valid_scans,
                       (unsigned int)t0_batch_num_scans);

                reset_axis_encoder((ProfileAxis_t)t0_batch_axis);

                if ((ProfileAxis_t)t0_batch_axis == PROFILE_AXIS_M1)
                {
                    dc1.prev_count  = enc_read_axis_count(PROFILE_AXIS_M1);
                    dc1.delta_count = 0;
                }
                else
                {
                    dc2.prev_count  = enc_read_axis_count(PROFILE_AXIS_M2);
                    dc2.delta_count = 0;
                }

                {
                    uint32_t wait_start = HAL_GetTick();
                    while ((HAL_GetTick() - wait_start) < (DC_SPEED_SAMPLE_MS + 5U))
                    {
                        live_power_update_service();
                        uint32_t now = HAL_GetTick();
                        if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
                        {
                            float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
                            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
                            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
                            dc_motor_auto_correct_active_axis();
                            last_dc_speed_ms = now;
                        }
                    }
                }

                beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
                t0_batch_state = T0_BATCH_CAPTURE;
            }
        }
        break;

        // --------------------------------------------------
        // Spin blade out of aperture using live power
        // feedback. Blocking but internally calls
        // live_power_update_service() + motor correction.
        // --------------------------------------------------
        case T0_BATCH_CLEAR_BEAM:
            printf("T0_BATCH_CLEARING_BEAM,axis=%u\r\n",
                   (unsigned int)t0_batch_axis);

            clear_active_axis_out_of_beam();

            // Decide whether to switch to the second axis or finish.
            if (t0_batch_total_axes == 2U && (ProfileAxis_t)t0_batch_axis == PROFILE_AXIS_M1)
            {
                t0_batch_state = T0_BATCH_SWITCH_AXIS;
            }
            else
            {
                t0_batch_state = T0_BATCH_DONE;
            }
            break;

        // --------------------------------------------------
        // NEW: M1 is done. Stop M1, spin up M2 and restart
        // the scan sequence from scan index 0.
        // --------------------------------------------------
        case T0_BATCH_SWITCH_AXIS:
        {
            printf("T0_BATCH_AXIS_DONE,axis=%u\r\n", (unsigned int)t0_batch_axis);

            dc_motor_set_duty(&dc1, 0U);

            // Reset all per-axis scan counters
            t0_batch_axis        = (uint8_t)PROFILE_AXIS_M2;
            t0_batch_scan_index  = 0U;
            t0_batch_valid_scans = 0U;   // fix bug 1

            profiler.axis             = PROFILE_AXIS_M2;
            host_local_t0_ready       = 0U;
            profiler.t0_idx           = 0U;
            profiler.t0_enc_count     = 0;
            profiler.x0_counts        = 0;
            profiler.xdelay_counts    = 0;
            profiler.next_scan_enc_target = 0;

            profile_locked_rpm         = 0.0f;
            profile_locked_omega_rad_s = 0.0f;
            profile_rpm_stable_count   = 0U;

            // Spin up M2 first, THEN reset encoder so prev_count
            // is synced to post-spinup position not pre-spinup   -- fix bug 5
            dc_motor_start_profile_axis(PROFILE_AXIS_M2);
            HAL_Delay(PROFILE_SPINUP_MS);

            reset_axis_encoder(PROFILE_AXIS_M2);   // reset AFTER spinup delay

            // Now sync prev_count to the freshly zeroed encoder  -- fix bug 5
            dc2.prev_count  = enc_read_axis_count(PROFILE_AXIS_M2);  // reads ~0
            dc2.delta_count = 0;
            dc2.rpm         = 0.0f;
            dc2.omega       = 0.0f;

            last_dc_speed_ms = HAL_GetTick();

            printf("T0_BATCH_BEGIN,axis=%u,num_scans=%u\r\n",
                   (unsigned int)t0_batch_axis,
                   (unsigned int)t0_batch_num_scans);

            t0_batch_state = T0_BATCH_WAIT_STABLE;
        }
        break;

//        case T0_BATCH_SWITCH_AXIS:
//        {
//            profiler.axis = PROFILE_AXIS_M2;
//            t0_batch_axis = (uint8_t)PROFILE_AXIS_M2;
//
//            t0_batch_valid_scans = 0U;
//            t0_batch_attempt_count = 0U;
//            t0_batch_emit_scan_no = 0U;
//
//            host_repeat_buffer_reset();
//
//            printf("T0_BATCH_SWITCH_AXIS,axis=%u,target_valid=%u,max_attempts=%u\r\n",
//                   (unsigned int)profiler.axis,
//                   (unsigned int)t0_batch_target_valid_scans,
//                   (unsigned int)T0_BATCH_MAX_ATTEMPTS);
//
//            t0_batch_state = T0_BATCH_WAIT_STABLE;
//        }
//        break;

        // --------------------------------------------------
        // All axes complete. Stop everything.
        // --------------------------------------------------
        case T0_BATCH_DONE:
            printf("T0_BATCH_DONE,axis=%u,num_scans=%u\r\n",
                   (unsigned int)t0_batch_axis,
                   (unsigned int)t0_batch_num_scans);
            // flush USB TX before signnalling done to Python
            for(uint32_t flush_i = 0U; flush_i < 10U; flush_i++)
            {
            	CDC_TX_Kick();
            	HAL_Delay(5);
            }

            printf("T0_BATCH_ALL_DONE\r\n");
            CDC_TX_Kick();

            dc_motors_stop_all();
            t0_batch_active = 0U;
            t0_batch_state  = T0_BATCH_IDLE;
            break;
    }
}

static void clear_beam_after_t0(void)
{
    int32_t start = enc_read_axis_count(profiler.axis);

    // Move another 90 degrees worth of counts
    int32_t target_counts = PROFILE_INIT_COUNTS;  // 7000 if using 90°

    while (1)
    {
        live_power_update_service();

        int32_t now = enc_read_axis_count(profiler.axis);
        int32_t moved = now - start;

        if (moved < 0)
            moved = -moved;

        if (moved >= target_counts)
            break;
    }

    printf("BEAM_CLEARED\r\n");
}



static void profile_find_t0_only(ProfileAxis_t axis)
{
    uint8_t locked;
    uint32_t last_speed_update_ms;
    uint32_t last_print_ms;

    profiler.axis = axis;
    host_local_t0_ready = 0U;

    profiler.t0_idx = 0U;
    profiler.t0_enc_count = 0;
    profiler.x0_counts = 0;
    profiler.t0_s = 0.0f;

    profile_raw_count = 0U;
    profile_capture_stride_counter = 0U;

    dc_motor_start_profile_axis(axis);

    HAL_Delay(PROFILE_SPINUP_MS);

    locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);

    if (beam_profile_get_active_omega() < 0.2f)
    {
        printf("T0_BAD_OMEGA,axis=%u,locked=%u,rpm=%.4f,omega=%.6f\r\n",
               (unsigned int)axis,
               (unsigned int)locked,
               profile_locked_rpm,
               beam_profile_get_active_omega());
        dc_motors_stop_all();
        return;
    }

    printf("T0_START,axis=%u,scan=0,counts=%d\r\n",
           (unsigned int)axis,
           PROFILE_INIT_COUNTS);

    beam_profile_start_window_counts(PROFILE_INIT_COUNTS);

    last_speed_update_ms = HAL_GetTick();
    last_print_ms = HAL_GetTick();

    while (1)
    {
        live_power_update_service();
        beam_profile_sample_position_window();

        if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
        {
            printf("T0_SCAN_OVERFLOW,axis=%u,scan=0,n=%lu\r\n",
                   (unsigned int)axis,
                   (unsigned long)profile_raw_count);
            dc_motors_stop_all();
            printf("T0_FAIL,axis=%u,scan=0\r\n", (unsigned int)axis);
            return;
        }

        if (beam_profile_window_done_counts())
        {
            printf("T0_SCAN_CAPTURE_DONE,axis=%u,scan=0,n=%lu\r\n",
                   (unsigned int)axis,
                   (unsigned long)profile_raw_count);
            break;
        }

        uint32_t now = HAL_GetTick();

        if ((now - last_speed_update_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_speed_update_ms) / 1000.0f;

            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
            dc_motor_auto_correct_active_axis();

            last_speed_update_ms = now;
        }

        if ((now - last_print_ms) >= 500U)
        {
            printf("T0_PROGRESS,axis=%u,scan=0,start=%ld,now=%ld,target=%ld,n=%lu\r\n",
                   (unsigned int)axis,
                   (long)profiler.scan_start_enc_count,
                   (long)enc_read_axis_count(axis),
                   (long)profiler.scan_stop_enc_target,
                   (unsigned long)profile_raw_count);
            last_print_ms = now;
        }
    }

    // Same validity gate as T0_BATCH_COMPUTE
    {
        int32_t counts_travelled = enc_delta_counts(
            profiler.scan_end_enc_count,
            profiler.scan_start_enc_count
        );

        if (counts_travelled < 0)
            counts_travelled = -counts_travelled;

        if (counts_travelled < (PROFILE_INIT_COUNTS / 2))
        {
            printf("T0_SCAN_INVALID,axis=%u,scan=0,counts=%ld\r\n",
                   (unsigned int)axis,
                   (long)counts_travelled);
            dc_motors_stop_all();
            printf("T0_FAIL,axis=%u,scan=0\r\n", (unsigned int)axis);
            return;
        }
    }

    host_profile_compute_local_t0();

    if (!host_local_t0_ready)
    {
        printf("T0_FAIL,axis=%u,scan=0\r\n", (unsigned int)axis);
        dc_motors_stop_all();
        return;
    }

    host_profile_learn_clear_threshold_from_initial_scan();

    t0_emit_raw_scan(axis);

    printf("T0_DONE,axis=%u,scan=0,start=%ld,end=%ld,n=%lu\r\n",
           (unsigned int)axis,
           (long)profiler.scan_start_enc_count,
           (long)profiler.scan_end_enc_count,
           (unsigned long)profile_raw_count);

    // Match batch output format
    printf("T0_RESULT,axis=%u,scan=0,idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
           (unsigned int)axis,
           (unsigned long)profiler.t0_idx,
           (long)profiler.t0_enc_count,
           (long)profiler.x0_counts,
           profiler.t0_s * 1000.0f);

    dc_motors_stop_all();
}

// DIVERGENCE MEASUREMENT
static uint8_t divergence_capture_both_axes_at_z(uint32_t step_index)
{
    uint8_t axis0_ok = 0U;
    uint8_t axis1_ok = 0U;

    printf("DIV_POS_BEGIN,z_mm=%.3f,step=%lu\r\n",
           z_current_mm,
           (unsigned long)step_index);

    // -------- Axis 0 --------
    printf("DIV_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=0\r\n",
           z_current_mm,
           (unsigned long)step_index);

    profile_find_t0_only(PROFILE_AXIS_M1);
    clear_active_axis_out_of_beam();

    if (!host_local_t0_ready)
    {
        printf("DIV_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=0\r\n",
               z_current_mm,
               (unsigned long)step_index);
    }
    else
    {
        axis0_ok = 1U;
        printf("DIV_AXIS_DONE,z_mm=%.3f,step=%lu,axis=0,"
               "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
               z_current_mm,
               (unsigned long)step_index,
               (unsigned long)profiler.t0_idx,
               (long)profiler.t0_enc_count,
               (long)profiler.x0_counts,
               profiler.t0_s * 1000.0f);
    }

    HAL_Delay(Z_SETTLE_MS);

    // -------- Axis 1 --------
    printf("DIV_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=1\r\n",
           z_current_mm,
           (unsigned long)step_index);

    profile_find_t0_only(PROFILE_AXIS_M2);
    clear_active_axis_out_of_beam();

    if (!host_local_t0_ready)
    {
        printf("DIV_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=1\r\n",
               z_current_mm,
               (unsigned long)step_index);
    }
    else
    {
        axis1_ok = 1U;
        printf("DIV_AXIS_DONE,z_mm=%.3f,step=%lu,axis=1,"
               "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
               z_current_mm,
               (unsigned long)step_index,
               (unsigned long)profiler.t0_idx,
               (long)profiler.t0_enc_count,
               (long)profiler.x0_counts,
               profiler.t0_s * 1000.0f);
    }

    printf("DIV_POS_DONE,z_mm=%.3f,step=%lu,axis0_ok=%u,axis1_ok=%u\r\n",
           z_current_mm,
           (unsigned long)step_index,
           (unsigned int)axis0_ok,
           (unsigned int)axis1_ok);

    // Only abort the run if BOTH axes failed at this position.
    return (axis0_ok || axis1_ok) ? 1U : 0U;
}

static uint8_t m2_capture_both_axes_at_z(uint32_t step_index)
{
    uint8_t axis0_ok = 0U;
    uint8_t axis1_ok = 0U;

    printf("M2_POS_BEGIN,z_mm=%.3f,step=%lu\r\n",
           z_current_mm,
           (unsigned long)step_index);

    printf("M2_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=0\r\n",
           z_current_mm,
           (unsigned long)step_index);

    profile_find_t0_only(PROFILE_AXIS_M1);

    if (!host_local_t0_ready)
    {
        printf("M2_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=0\r\n",
               z_current_mm,
               (unsigned long)step_index);
    }
    else
    {
        axis0_ok = 1U;
        printf("M2_AXIS_DONE,z_mm=%.3f,step=%lu,axis=0,"
               "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
               z_current_mm,
               (unsigned long)step_index,
               (unsigned long)profiler.t0_idx,
               (long)profiler.t0_enc_count,
               (long)profiler.x0_counts,
               profiler.t0_s * 1000.0f);
    }

    HAL_Delay(Z_SETTLE_MS);

    printf("M2_AXIS_BEGIN,z_mm=%.3f,step=%lu,axis=1\r\n",
           z_current_mm,
           (unsigned long)step_index);

    profile_find_t0_only(PROFILE_AXIS_M2);

    if (!host_local_t0_ready)
    {
        printf("M2_AXIS_FAIL,z_mm=%.3f,step=%lu,axis=1\r\n",
               z_current_mm,
               (unsigned long)step_index);
    }
    else
    {
        axis1_ok = 1U;
        printf("M2_AXIS_DONE,z_mm=%.3f,step=%lu,axis=1,"
               "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
               z_current_mm,
               (unsigned long)step_index,
               (unsigned long)profiler.t0_idx,
               (long)profiler.t0_enc_count,
               (long)profiler.x0_counts,
               profiler.t0_s * 1000.0f);
    }

    printf("M2_POS_DONE,z_mm=%.3f,step=%lu,axis0_ok=%u,axis1_ok=%u\r\n",
           z_current_mm,
           (unsigned long)step_index,
           (unsigned int)axis0_ok,
           (unsigned int)axis1_ok);

    return (axis0_ok || axis1_ok) ? 1U : 0U;
}

static void t0_emit_raw_scan(ProfileAxis_t axis)
{
    t0_dump_started = 0U;
    t0_dump_idx     = 0U;

    while (!t0_dump_scan_step(axis, 0U))
    {
        live_power_update_service();

        // Block here until ring has at least 1KB free
        // before attempting the next 8-line burst
        uint32_t wait_start = HAL_GetTick();
        while (CDC_TX_Count() > (65536U - 1024U))
        {
            CDC_TX_Kick();
            live_power_update_service();
            if ((HAL_GetTick() - wait_start) > 500U)
                break;
        }

        uint32_t now = HAL_GetTick();
        if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
            dc_motor_auto_correct_active_axis();
            last_dc_speed_ms = now;
        }
    }

    live_power_update_service();
}

static float compute_d4sigma_mm_from_raw_scan(
    const uint16_t *raw_buf,
    const uint8_t *stage_buf,
    const int32_t *enc_buf,
    uint32_t n
)
{
    float p_first_avg = 0.0f;
    float p_last_avg = 0.0f;
    float sum_w = 0.0f;
    float sum_x = 0.0f;
    float sum_x2 = 0.0f;
    float mean_x;
    float sigma_sq;
    float edge_sign;
    float counts_per_rev;
    uint32_t avg_n;

    if ((raw_buf == NULL) || (stage_buf == NULL) || (enc_buf == NULL))
        return 0.0f;

    if (n < 4U)
        return 0.0f;

    counts_per_rev = profile_get_active_counts_per_rev();
    if (counts_per_rev <= 0.0f)
        return 0.0f;

    avg_n = (n < 8U) ? n : 8U;
    for (uint32_t i = 0U; i < avg_n; i++)
    {
        p_first_avg += host_profile_sample_power_mw((uint8_t)profiler.axis,
                                                    0U,
                                                    i,
                                                    raw_buf[i],
                                                    stage_buf[i],
                                                    enc_buf[i],
                                                    profiler.scan_start_enc_count,
                                                    profiler.scan_end_enc_count);

        p_last_avg += host_profile_sample_power_mw((uint8_t)profiler.axis,
                                                   0U,
                                                   n - avg_n + i,
                                                   raw_buf[n - avg_n + i],
                                                   stage_buf[n - avg_n + i],
                                                   enc_buf[n - avg_n + i],
                                                   profiler.scan_start_enc_count,
                                                   profiler.scan_end_enc_count);
    }

    p_first_avg /= (float)avg_n;
    p_last_avg  /= (float)avg_n;
    edge_sign = (p_last_avg >= p_first_avg) ? 1.0f : -1.0f;

    for (uint32_t i = 1U; i < n; i++)
    {
        float x_prev;
        float x_now;
        float x_mid;
        float p_prev;
        float p_now;
        float d_edge_power;

        x_prev = profile_counts_to_x_mm(enc_buf[i - 1U],
                                        profiler.scan_start_enc_count,
                                        counts_per_rev);
        x_now = profile_counts_to_x_mm(enc_buf[i],
                                       profiler.scan_start_enc_count,
                                       counts_per_rev);
        x_mid = 0.5f * (x_prev + x_now);

        p_prev = host_profile_sample_power_mw((uint8_t)profiler.axis,
                                              0U,
                                              i - 1U,
                                              raw_buf[i - 1U],
                                              stage_buf[i - 1U],
                                              enc_buf[i - 1U],
                                              profiler.scan_start_enc_count,
                                              profiler.scan_end_enc_count);
        p_now = host_profile_sample_power_mw((uint8_t)profiler.axis,
                                             0U,
                                             i,
                                             raw_buf[i],
                                             stage_buf[i],
                                             enc_buf[i],
                                             profiler.scan_start_enc_count,
                                             profiler.scan_end_enc_count);

        d_edge_power = edge_sign * (p_now - p_prev);
        if (d_edge_power <= 0.0f)
            continue;

        sum_w += d_edge_power;
        sum_x += d_edge_power * x_mid;
        sum_x2 += d_edge_power * x_mid * x_mid;
    }

    if (sum_w <= 1.0e-6f)
        return 0.0f;

    mean_x = sum_x / sum_w;
    sigma_sq = (sum_x2 / sum_w) - (mean_x * mean_x);

    if (sigma_sq <= 0.0f)
        return 0.0f;

    return 4.0f * sqrtf(sigma_sq);
}

static uint8_t measure_beam_width_d4sigma(float *w_mm_out)
{
    uint8_t axis;
    uint8_t locked;
    uint32_t last_speed_update_ms;
    uint32_t last_print_ms;
    float width_mm;

    if (w_mm_out == NULL)
        return 0U;

    axis = (uint8_t)profiler.axis;
    *w_mm_out = 0.0f;

    profile_raw_count = 0U;
    profile_capture_stride_counter = 0U;
    host_local_t0_ready = 0U;

    dc_motor_start_profile_axis(profiler.axis);
    HAL_Delay(PROFILE_SPINUP_MS);

    locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);
    if (beam_profile_get_active_omega() < 0.2f)
    {
        printf("M2_SCAN_FAIL,axis=%u,z_mm=%.3f,reason=bad_omega,locked=%u,rpm=%.4f,omega=%.6f\r\n",
               (unsigned int)axis,
               z_current_mm,
               (unsigned int)locked,
               profile_locked_rpm,
               beam_profile_get_active_omega());
        dc_motors_stop_all();
        return 0U;
    }

    printf("M2_SCAN_BEGIN,axis=%u,z_mm=%.3f,counts=%d,rpm=%.4f,omega=%.6f\r\n",
           (unsigned int)axis,
           z_current_mm,
           PROFILE_INIT_COUNTS,
           profile_locked_rpm,
           beam_profile_get_active_omega());

    beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
    last_speed_update_ms = HAL_GetTick();
    last_print_ms = last_speed_update_ms;

    while (1)
    {
        live_power_update_service();
        beam_profile_sample_position_window();

        if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
        {
            printf("M2_SCAN_FAIL,axis=%u,z_mm=%.3f,reason=overflow,n=%lu\r\n",
                   (unsigned int)axis,
                   z_current_mm,
                   (unsigned long)profile_raw_count);
            dc_motors_stop_all();
            return 0U;
        }

        if (beam_profile_window_done_counts())
            break;

        {
            uint32_t now = HAL_GetTick();

            if ((now - last_speed_update_ms) >= DC_SPEED_SAMPLE_MS)
            {
                float dt_s = (float)(now - last_speed_update_ms) / 1000.0f;

                dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
                dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
                dc_motor_auto_correct_active_axis();
                last_speed_update_ms = now;
            }

            if ((now - last_print_ms) >= 500U)
            {
                printf("M2_SCAN_PROGRESS,axis=%u,z_mm=%.3f,start=%ld,now=%ld,target=%ld,n=%lu\r\n",
                       (unsigned int)axis,
                       z_current_mm,
                       (long)profiler.scan_start_enc_count,
                       (long)enc_read_axis_count(profiler.axis),
                       (long)profiler.scan_stop_enc_target,
                       (unsigned long)profile_raw_count);
                last_print_ms = now;
            }
        }
    }

    {
        int32_t counts_travelled = enc_delta_counts(profiler.scan_end_enc_count,
                                                    profiler.scan_start_enc_count);

        if (counts_travelled < 0)
            counts_travelled = -counts_travelled;

        if (counts_travelled < (PROFILE_INIT_COUNTS / 2))
        {
            printf("M2_SCAN_FAIL,axis=%u,z_mm=%.3f,reason=short_scan,counts=%ld\r\n",
                   (unsigned int)axis,
                   z_current_mm,
                   (long)counts_travelled);
            dc_motors_stop_all();
            return 0U;
        }
    }

    host_profile_learn_clear_threshold_from_initial_scan();

    width_mm = compute_d4sigma_mm_from_raw_scan(profile_raw_power_buf,
                                                profile_raw_stage_buf,
                                                profile_raw_enc_buf,
                                                profile_raw_count);

    clear_active_axis_out_of_beam();

    if (width_mm <= 0.0f)
    {
        printf("M2_SCAN_FAIL,axis=%u,z_mm=%.3f,reason=bad_width,n=%lu\r\n",
               (unsigned int)axis,
               z_current_mm,
               (unsigned long)profile_raw_count);
        dc_motors_stop_all();
        return 0U;
    }

    *w_mm_out = width_mm;

    printf("M2_WIDTH,axis=%u,z_mm=%.3f,d4sigma_mm=%.6f,n=%lu\r\n",
           (unsigned int)axis,
           z_current_mm,
           width_mm,
           (unsigned long)profile_raw_count);

    return 1U;
}

static uint32_t m2_count_points_within_radius(const float *z_mm,
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

static uint32_t m2_count_points_outside_radius(const float *z_mm,
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

static float m2_compute_rayleigh_range_mm(float d0_mm, float theta_rad)
{
    if ((d0_mm <= 0.0f) || (theta_rad <= 0.0f))
        return 0.0f;

    return d0_mm / theta_rad;
}

static void m2_fill_linear_positions(float start_mm,
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

static uint8_t m2_build_iso_positions(float z0_mm,
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

static uint8_t m2_measure_axis_at_positions(ProfileAxis_t axis,
                                            const float *target_z_mm,
                                            uint32_t target_count,
                                            float *z_mm_out,
                                            float *d4sigma_mm_out,
                                            uint32_t *valid_count_out,
                                            const char *phase_tag)
{
    uint32_t valid_count = 0U;

    if ((target_z_mm == NULL) || (z_mm_out == NULL) ||
        (d4sigma_mm_out == NULL) || (valid_count_out == NULL))
        return 0U;

    *valid_count_out = 0U;

    z_home_simple(Z_HOME_DIR);
    if (!home_pressed())
    {
        printf("M2_AXIS_HOME_FAIL,axis=%u,phase=%s\r\n",
               (unsigned int)axis,
               (phase_tag != NULL) ? phase_tag : "NA");
        return 0U;
    }

    HAL_Delay(Z_SETTLE_MS);

    for (uint32_t i = 0U; i < target_count; i++)
    {
        float width_mm = 0.0f;
        float target_mm = target_z_mm[i];

        z_move_to_mm(target_mm);
        HAL_Delay(Z_SETTLE_MS);

        profiler.axis = axis;
        printf("M2_TARGET_BEGIN,axis=%u,phase=%s,target_z_mm=%.3f,index=%lu\r\n",
               (unsigned int)axis,
               (phase_tag != NULL) ? phase_tag : "NA",
               target_mm,
               (unsigned long)(i + 1U));

        if (measure_beam_width_d4sigma(&width_mm))
        {
            z_mm_out[valid_count] = z_current_mm;
            d4sigma_mm_out[valid_count] = width_mm;
            valid_count++;

            printf("M2_POINT,axis=%u,phase=%s,z_mm=%.3f,d4sigma_mm=%.6f,index=%lu\r\n",
                   (unsigned int)axis,
                   (phase_tag != NULL) ? phase_tag : "NA",
                   z_current_mm,
                   width_mm,
                   (unsigned long)valid_count);
        }
        else
        {
            printf("M2_POINT_FAIL,axis=%u,phase=%s,target_z_mm=%.3f,index=%lu\r\n",
                   (unsigned int)axis,
                   (phase_tag != NULL) ? phase_tag : "NA",
                   target_mm,
                   (unsigned long)(i + 1U));
        }
    }

    *valid_count_out = valid_count;
    return (valid_count >= M2_MIN_FIT_POINTS) ? 1U : 0U;
}

static uint8_t m2_fit_d4sigma_curve(const float *z_mm,
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
        return 0U;

    if (n < M2_MIN_FIT_POINTS)
        return 0U;

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

static uint8_t run_m2_measurement(float step_mm, float total_mm)
{
    uint32_t n_positions;

    if (step_mm <= 0.0f || total_mm <= 0.0f)
    {
        printf("M2_BAD_ARGS\r\n");
        return 0U;
    }

    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
    if ((n_positions == 0U) || (n_positions > M2_MAX_POSITIONS))
    {
        printf("M2_BAD_ARGS,positions=%lu,max=%u\r\n",
               (unsigned long)n_positions,
               (unsigned int)M2_MAX_POSITIONS);
        return 0U;
    }

    divergence_busy = 1U;

    printf("M2_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
           step_mm,
           total_mm,
           (unsigned long)n_positions);

    z_home_simple(Z_HOME_DIR);
    if (!home_pressed())
    {
        printf("M2_HOME_FAIL\r\n");
        divergence_busy = 0U;
        return 0U;
    }

    HAL_Delay(Z_SETTLE_MS);

    for (uint32_t i = 0U; i < n_positions; i++)
    {
        z_move_mm_dir(Z_AWAY_DIR, step_mm);
        HAL_Delay(Z_SETTLE_MS);

        if (!m2_capture_both_axes_at_z(i + 1U))
        {
            printf("M2_ABORT,z_mm=%.3f,step=%lu\r\n",
                   z_current_mm,
                   (unsigned long)(i + 1U));
            divergence_busy = 0U;
            return 0U;
        }
    }

    printf("M2_DONE,final_z_mm=%.3f\r\n", z_current_mm);

    divergence_busy = 0U;
    return 1U;
}

static uint8_t run_m2_capture_at_position(float target_mm, uint32_t step_index)
{
    uint8_t ok;

    if ((target_mm < 0.0f) || (target_mm > M2_TOTAL_MM))
    {
        printf("M2_POS_BAD_ARGS,target_z_mm=%.3f,max_z_mm=%.3f\r\n",
               target_mm,
               M2_TOTAL_MM);
        return 0U;
    }

    divergence_busy = 1U;

    printf("M2_TARGET_BEGIN,target_z_mm=%.3f,step=%lu\r\n",
           target_mm,
           (unsigned long)step_index);

    z_move_to_mm(target_mm);
    HAL_Delay(Z_SETTLE_MS);

    ok = m2_capture_both_axes_at_z(step_index);

    printf("M2_TARGET_DONE,target_z_mm=%.3f,actual_z_mm=%.3f,step=%lu,ok=%u\r\n",
           target_mm,
           z_current_mm,
           (unsigned long)step_index,
           (unsigned int)ok);

    divergence_busy = 0U;
    return ok;
}

static float z_plot_mm(void)
{
	return 2.0f * z_current_mm;
}

// =============================
// Step-and-profile scan
// Moves Z stage by fixed step count, captures beam width at each stop.
// num_steps    : number of motor steps per move
// num_positions: total number of positions to measure
// axis         : which beam axis to profile (PROFILE_AXIS_M1 or PROFILE_AXIS_M2)
// =============================
static uint8_t run_step_and_profile(uint32_t num_steps,
                                    uint32_t num_positions,
                                    ProfileAxis_t axis)
{
    if (num_steps == 0U || num_positions == 0U)
    {
        printf("STEP_PROFILE_BAD_ARGS,steps=%lu,positions=%lu\r\n",
               (unsigned long)num_steps,
               (unsigned long)num_positions);
        return 0U;
    }

    float step_mm = (float)num_steps / Z_STEPS_PER_MM;

    printf("STEP_PROFILE_BEGIN,steps=%lu,step_mm=%.4f,positions=%lu,axis=%u\r\n",
           (unsigned long)num_steps,
           step_mm,
           (unsigned long)num_positions,
           (unsigned int)axis);

    z_home_simple(Z_HOME_DIR);
    if (!home_pressed())
    {
        printf("STEP_PROFILE_HOME_FAIL\r\n");
        printf("STEP_PROFILE_DONE,positions=0\r\n");
        return 0U;
    }

    HAL_Delay(Z_SETTLE_MS);

    profiler.axis = axis;

    for (uint32_t i = 0U; i < num_positions; i++)
    {
        float width_mm = 0.0f;

        z_move_steps_dir(Z_AWAY_DIR, num_steps);
        z_current_mm += step_mm;
        if (z_current_mm < 0.0f)
            z_current_mm = 0.0f;

        HAL_Delay(Z_SETTLE_MS);

        printf("STEP_PROFILE_POS,index=%lu,total_steps=%lu,z_mm=%.4f\r\n",
               (unsigned long)(i + 1U),
               (unsigned long)((i + 1U) * num_steps),
               z_current_mm);

        if (measure_beam_width_d4sigma(&width_mm))
        {
            printf("STEP_PROFILE_POINT,index=%lu,z_mm=%.4f,d4sigma_mm=%.6f\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm,
                   width_mm);

            // Stream the raw scan so Python can plot beam profile per position.
            // profile_raw_power_buf, profile_raw_stage_buf, profile_raw_enc_buf
            // and profile_raw_count are all still valid here because
            // measure_beam_width_d4sigma fills them and does not clear them.
            // profiler.scan_start_enc_count and scan_end_enc_count are also
            // still set correctly from that call.
            t0_emit_raw_scan(axis);
        }
        else
        {
            printf("STEP_PROFILE_FAIL,index=%lu,z_mm=%.4f\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm);
        }
    }

    printf("STEP_PROFILE_DONE,final_z_mm=%.4f,positions=%lu,axis=%u\r\n",
           z_current_mm,
           (unsigned long)num_positions,
           (unsigned int)axis);

    return 1U;
}

// =============================
// Step-divergence scan
// Homes Z stage, then steps STEP_DIV_STEP_MM for STEP_DIV_NUM_POSITIONS stops.
// At each stop, profiles both axes using profile_find_t0_only and streams
// raw scan data via t0_emit_raw_scan.
// =============================
static uint8_t run_step_divergence(float step_mm, uint32_t num_positions)
{
    if ((step_mm <= 0.0f) || (num_positions == 0U))
    {
        printf("STEP_DIV_BAD_ARGS,step_mm=%.3f,positions=%lu\r\n",
               step_mm,
               (unsigned long)num_positions);
        return 0U;
    }

    printf("STEP_DIV_BEGIN,step_mm=%.3f,positions=%lu\r\n",
           step_mm,
           (unsigned long)num_positions);

    // Home Z stage first
    z_home_simple(Z_HOME_DIR);
    if (!home_pressed())
    {
        printf("STEP_DIV_HOME_FAIL\r\n");
        printf("STEP_DIV_DONE,positions=0\r\n");
        return 0U;
    }

    HAL_Delay(Z_SETTLE_MS);

    for (uint32_t i = 0U; i < num_positions; i++)
    {
        // Move one step forward
        z_move_mm_dir(Z_AWAY_DIR, step_mm);
        HAL_Delay(Z_SETTLE_MS);

        printf("STEP_DIV_POS_BEGIN,index=%lu,z_mm=%.3f\r\n",
               (unsigned long)(i + 1U),
               z_current_mm);

        // -------- Axis 0 --------
        printf("STEP_DIV_AXIS_BEGIN,index=%lu,z_mm=%.3f,axis=0\r\n",
               (unsigned long)(i + 1U),
               z_current_mm);

        profiler.axis = PROFILE_AXIS_M1;
        profile_find_t0_only(PROFILE_AXIS_M1);

        if (host_local_t0_ready)
        {
            printf("STEP_DIV_AXIS_RESULT,index=%lu,z_mm=%.3f,axis=0,"
                   "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm,
                   (unsigned long)profiler.t0_idx,
                   (long)profiler.t0_enc_count,
                   (long)profiler.x0_counts,
                   profiler.t0_s * 1000.0f);

            // Stream raw scan to host
            t0_emit_raw_scan(PROFILE_AXIS_M1);
        }
        else
        {
            printf("STEP_DIV_AXIS_FAIL,index=%lu,z_mm=%.3f,axis=0\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm);
        }

        // Move blade out of beam before switching axes
        clear_active_axis_out_of_beam();
        HAL_Delay(Z_SETTLE_MS);

        // -------- Axis 1 --------
        printf("STEP_DIV_AXIS_BEGIN,index=%lu,z_mm=%.3f,axis=1\r\n",
               (unsigned long)(i + 1U),
               z_current_mm);

        profiler.axis = PROFILE_AXIS_M2;
        profile_find_t0_only(PROFILE_AXIS_M2);

        if (host_local_t0_ready)
        {
            printf("STEP_DIV_AXIS_RESULT,index=%lu,z_mm=%.3f,axis=1,"
                   "idx=%lu,t0_enc=%ld,x0_counts=%ld,t0_ms=%.6f\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm,
                   (unsigned long)profiler.t0_idx,
                   (long)profiler.t0_enc_count,
                   (long)profiler.x0_counts,
                   profiler.t0_s * 1000.0f);

            // Stream raw scan to host
            t0_emit_raw_scan(PROFILE_AXIS_M2);
        }
        else
        {
            printf("STEP_DIV_AXIS_FAIL,index=%lu,z_mm=%.3f,axis=1\r\n",
                   (unsigned long)(i + 1U),
                   z_current_mm);
        }

        // Move blade out of beam before next Z step
        clear_active_axis_out_of_beam();

        printf("STEP_DIV_POS_DONE,index=%lu,z_mm=%.3f\r\n",
               (unsigned long)(i + 1U),
               z_current_mm);
    }

    printf("STEP_DIV_DONE,final_z_mm=%.3f,positions=%lu\r\n",
           z_current_mm,
           (unsigned long)num_positions);

    return 1U;
}

//static uint8_t run_divergence_measurement(float step_mm, float total_mm)
//{
//    uint32_t n_positions;
//
//    if (step_mm <= 0.0f || total_mm <= 0.0f)
//    {
//        printf("DIVERGENCE_BAD_ARGS\r\n");
//        return 0U;
//    }
//
//    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
//
//    divergence_busy = 1U;
//
//    printf("DIVERGENCE_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
//           step_mm, total_mm, (unsigned long)n_positions);
//
//    // Home first
//    z_home_simple(Z_HOME_DIR);
//    printf("ZHOME_DONE\r\n");
//
//    // Wake and set direction away, keep awake for entire run
//    z_stepper_wake();
//    HAL_Delay(5);
//    z_set_dir(Z_AWAY_DIR);
//    HAL_Delay(5);
//
//    uint32_t steps_per_position = (uint32_t)(step_mm * Z_STEPS_PER_MM + 0.5f);
//
//    for (uint32_t i = 0U; i < n_positions; i++)
//    {
//        // Move one step_mm forward
//        z_move_steps(steps_per_position);
//        z_current_mm += step_mm;
//
//        printf("Z_MOVE_DONE,step=%lu,z_mm=%.3f\r\n",
//               (unsigned long)(i + 1U),
//               z_current_mm);
//
//        HAL_Delay(Z_SETTLE_MS);
//
//        // Profile both axes at this position
//        if (!divergence_capture_both_axes_at_z(i + 1U))
//        {
//            printf("DIVERGENCE_ABORT,z_mm=%.3f,step=%lu\r\n",
//                   z_current_mm, (unsigned long)(i + 1U));
//            z_stepper_disable();
//            divergence_busy = 0U;
//            return 0U;
//        }
//
//        // Re-arm direction after each capture since profile_find_t0_only
//        // uses dc motors which may disturb sleep pin state
//        z_stepper_wake();
//        HAL_Delay(5);
//        z_set_dir(Z_AWAY_DIR);
//        HAL_Delay(5);
//    }
//
//    printf("DIVERGENCE_DONE,final_z_mm=%.3f\r\n", z_current_mm);
//
//    z_stepper_disable();
//    divergence_busy = 0U;
//    return 1U;
//}


// WORKING DIVERGENCE
static uint8_t run_divergence_measurement(float step_mm, float total_mm)
{
    uint32_t n_positions;
    uint32_t steps_per_position;

    if (step_mm <= 0.0f || total_mm <= 0.0f)
    {
        printf("DIVERGENCE_BAD_ARGS\r\n");
        return 0U;
    }

    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
    steps_per_position = (uint32_t)(step_mm * Z_STEPS_PER_MM + 0.5f);

    divergence_busy = 1U;

    printf("DIVERGENCE_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
           step_mm, total_mm, (unsigned long)n_positions);

    // Home first
    z_home_simple(Z_HOME_DIR);
    printf("ZHOME_DONE\r\n");

    // Wake stepper and set away direction for entire run
    z_stepper_wake();
    HAL_Delay(5);
    z_set_dir(Z_AWAY_DIR);
    HAL_Delay(5);

    for (uint32_t i = 0U; i < n_positions; i++)
    {
        // Move away from home by one step_mm position
        z_move_steps(steps_per_position);
        z_current_mm += step_mm;

        printf("Z_MOVE_DONE,step=%lu,z_mm=%.3f\r\n",
               (unsigned long)(i + 1U), z_plot_mm());

        HAL_Delay(Z_SETTLE_MS);

        // Start t0_batch for both axes
        t0_batch_total_axes = 2U;   // scan M1 then auto-switch to M2
        t0_batch_begin(PROFILE_AXIS_M1, 10U);

        // Run t0_batch state machine until complete
        while (t0_batch_active)
        {
            t0_batch_update();
            live_power_update_service();
        }

        HAL_Delay(100);

        printf("DIV_POS_DONE,z_mm=%.3f,step=%lu\r\n",
               z_plot_mm(), (unsigned long)(i + 1U));

        // Re-arm Z stepper for next move
        z_stepper_wake();
        HAL_Delay(5);
        z_set_dir(Z_AWAY_DIR);
        HAL_Delay(5);
    }

    printf("DIVERGENCE_DONE,final_z_mm=%.3f\r\n", z_plot_mm());

    z_stepper_disable();
    divergence_busy = 0U;
    return 1U;
}

//static uint8_t run_divergence_measurement(float step_mm, float total_mm)
//{
//    uint32_t n_positions;
//    uint32_t steps_per_position;
//    uint32_t initial_steps;
//    float actual_step_mm;
//    float actual_total_mm;
//
//    if (step_mm <= 0.0f || total_mm <= 0.0f)
//    {
//        printf("DIVERGENCE_BAD_ARGS\r\n");
//        return 0U;
//    }
//
//    steps_per_position = (uint32_t)(step_mm * Z_STEPS_PER_MM + 0.5f);
//    initial_steps      = (uint32_t)(total_mm * Z_STEPS_PER_MM + 0.5f);
//
//    if (steps_per_position == 0U || initial_steps == 0U)
//    {
//        printf("DIVERGENCE_BAD_STEPS\r\n");
//        return 0U;
//    }
//
//    actual_step_mm  = ((float)steps_per_position) / Z_STEPS_PER_MM;
//    actual_total_mm = ((float)initial_steps) / Z_STEPS_PER_MM;
//
//    // Exactly 10 scan positions:
//    // 270, 260, 250, ... 180
//    n_positions = 10U;
//
//    divergence_busy = 1U;
//
//    printf("DIVERGENCE_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
//           actual_step_mm, actual_total_mm, (unsigned long)n_positions);
//
//    // -------------------------------------------------
//    // 1) Home
//    // -------------------------------------------------
//    z_home_simple(Z_HOME_DIR);
//    z_current_mm = 0.0f;
//    printf("ZHOME_DONE,z_mm=%.3f\r\n", z_plot_mm());
//
//    // -------------------------------------------------
//    // 2) Move to 270 mm away from home
//    // -------------------------------------------------
//    z_stepper_wake();
//    HAL_Delay(5);
//    z_set_dir(Z_AWAY_DIR);
//    HAL_Delay(5);
//
//    z_move_steps(initial_steps);
//    z_current_mm = actual_total_mm;
//
//    printf("Z_INITIAL_MOVE_DONE,z_mm=%.3f\r\n", z_plot_mm());
//    HAL_Delay(Z_SETTLE_MS);
//
//    // -------------------------------------------------
//    // 3) Scan 10 positions total
//    // -------------------------------------------------
//    for (uint32_t i = 0U; i < n_positions; i++)
//    {
//        printf("DIV_POS_BEGIN,index=%lu,z_mm=%.3f\r\n",
//               (unsigned long)i, z_plot_mm());
//
//        t0_batch_total_axes = 2U;
//        t0_batch_begin(PROFILE_AXIS_M1, 10U);
//
//        while (t0_batch_active)
//        {
//            t0_batch_update();
//            live_power_update_service();
//        }
//
//        printf("DIV_POS_DONE,index=%lu,z_mm=%.3f\r\n",
//               (unsigned long)i, z_plot_mm());
//
//        // Move back 10 mm after every scan except the last one
//        if (i < (n_positions - 1U))
//        {
//            z_stepper_wake();
//            HAL_Delay(5);
//            z_set_dir(Z_HOME_DIR);
//            HAL_Delay(5);
//
//            z_move_steps(steps_per_position);
//
//            if (z_current_mm >= actual_step_mm)
//                z_current_mm -= actual_step_mm;
//            else
//                z_current_mm = 0.0f;
//
//            printf("Z_STEP_BACK_DONE,index=%lu,z_mm=%.3f\r\n",
//                   (unsigned long)(i + 1U), z_plot_mm());
//
//            HAL_Delay(Z_SETTLE_MS);
//        }
//    }
//
//    printf("DIVERGENCE_DONE,final_z_mm=%.3f\r\n", z_plot_mm());
//
//    z_stepper_disable();
//    divergence_busy = 0U;
//    return 1U;
//}

//static uint8_t run_divergence_measurement(float step_mm, float total_mm)
//{
//    uint32_t n_positions;
//
//    if (step_mm <= 0.0f || total_mm <= 0.0f)
//    {
//        printf("DIVERGENCE_BAD_ARGS\r\n");
//        return 0U;
//    }
//
//    n_positions = (uint32_t)(total_mm / step_mm + 0.5f);
//
//    divergence_busy = 1U;
//
//    printf("DIVERGENCE_BEGIN,step_mm=%.3f,total_mm=%.3f,positions=%lu\r\n",
//           step_mm,
//           total_mm,
//           (unsigned long)n_positions);
//
//    z_home_simple(Z_HOME_DIR);
//    if (!home_pressed())
//    {
//        printf("DIVERGENCE_HOME_FAIL\r\n");
//        divergence_busy = 0U;
//        return 0U;
//    }
//
//    // Clear the switch before starting divergence moves
//    z_stepper_wake();
//    HAL_Delay(20);
//    z_set_dir(Z_AWAY_DIR);
//    HAL_Delay(10);
//
//    // Step until switch releases or safety limit
//    uint32_t clear_steps = 0U;
//    while (home_pressed() && clear_steps < 2000U)
//    {
//        z_step_pulse();
//        HAL_Delay(Z_STEP_DELAY_MS);
//        clear_steps++;
//    }
//
//    printf("SWITCH_CLEAR_STEPS=%lu,PIN=%u\r\n",
//           (unsigned long)clear_steps,
//           (unsigned int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13));
//
//    z_stepper_disable();
//    HAL_Delay(Z_SETTLE_MS);
//
////    printf("PRE_MOVE_HOME_PIN=%u\r\n",
////           (unsigned int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13));
////    printf("PRE_MOVE_DIR=%u\r\n", (unsigned int)Z_AWAY_DIR);
////    HAL_Delay(Z_SETTLE_MS);
////
////    // After z_home_simple, before the loop
////    printf("MOVING_AWAY_TEST\r\n");
////    z_move_mm_dir(Z_AWAY_DIR, 10.0f);   // just move 2mm away
////    printf("POST_2MM_HOME_PIN=%u\r\n",
////           (unsigned int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13));
//
//    for (uint32_t i = 0U; i < n_positions; i++)
//    {
//        z_move_mm_dir(Z_AWAY_DIR, step_mm);
//        HAL_Delay(Z_SETTLE_MS);
//
//        if (!divergence_capture_both_axes_at_z(i + 1U))
//        {
//            printf("DIVERGENCE_ABORT,z_mm=%.3f,step=%lu\r\n",
//                   z_current_mm,
//                   (unsigned long)(i + 1U));
//            divergence_busy = 0U;
//            return 0U;
//        }
//    }
//
//    printf("DIVERGENCE_DONE,final_z_mm=%.3f\r\n", z_current_mm);
//
//    divergence_busy = 0U;
//    return 1U;
//}

// -----------------------------
// Extract latest N samples from circular ADC buffer
// Maintains correct ordering even across wrap-around
// -----------------------------
static void profile_extract_latest_window(uint16_t *dst, uint32_t nsamp)
{
    if (nsamp > ADC_BUF_LEN) nsamp = ADC_BUF_LEN;

    uint32_t wr = adc_dma_write_index();

    // Compute start index (wrap-safe)
    uint32_t start = (wr + ADC_BUF_LEN - nsamp) % ADC_BUF_LEN;

    for (uint32_t i = 0; i < nsamp; i++)
    {
        dst[i] = adc_buf[(start + i) % ADC_BUF_LEN];
    }
}

// -----------------------------
// Convert raw ADC window → beam profile samples
// Includes gain auto-ranging during acquisition
// -----------------------------
static void profile_update_power_buffer_from_adc_window(const uint16_t *src, uint32_t nsamp)
{
    // Reset output buffer
    profile_point_count = 0;

    // Process ADC data in decimated chunks (reduces noise + data size)
    for (uint32_t i = 0; i + PROFILE_DECIM <= nsamp && profile_point_count < PROFILE_MAX_POINTS; i += PROFILE_DECIM)
    {
        // Average small chunk of ADC samples
        float raw_avg = mean_u16(&src[i], PROFILE_DECIM);

        // Convert to voltage
        float v_meas = adc_counts_to_voltage(raw_avg);

        // Store processed sample
        uint32_t k = profile_point_count++;

        profile_raw_avg[k]   = (uint16_t)(raw_avg + 0.5f);
        profile_v_avg[k]     = v_meas;
        profile_power_mw[k]     = voltage_to_power_mw(v_meas);
        profile_stage_avg[k] = (uint8_t)current_gain;
    }
}

// Find the sample index where the absolute power derivative is largest.
// This is used to locate t0 during the initial beam-profile scan.
static uint32_t find_t0_index_from_profile(void)
{
    // Need at least 3 points for a centered derivative.
    if (profile_point_count < 3U)
        return 0U;

    // Track the largest absolute derivative found so far.
    float max_abs_dpdt = 0.0f;
    uint32_t best_idx = 1U;

    // Time spacing between decimated profile points.
    const float dt = ((float)PROFILE_DECIM / ADC_FS_HZ);

    // Compute centered derivative across the whole scan.
    for (uint32_t i = 1; i < profile_point_count - 1; i++)
    {
        float dpdt = (profile_power_mw[i + 1] - profile_power_mw[i - 1]) / (2.0f * dt);
        float abs_dpdt = fabsf(dpdt);

        // Keep the index of the maximum absolute derivative.
        if (abs_dpdt > max_abs_dpdt)
        {
            max_abs_dpdt = abs_dpdt;
            best_idx = i;
        }
    }

    return best_idx;
}



static void profile_find_initial_power_bounds(float *pmin_mw, float *pmax_mw)
{
    float pmin = 0.0f;
    float pmax = 0.0f;

    if (profile_point_count == 0U)
    {
        if (pmin_mw) *pmin_mw = 0.0f;
        if (pmax_mw) *pmax_mw = 0.0f;
        return;
    }

    pmin = profile_power_mw[0];
    pmax = profile_power_mw[0];

    for (uint32_t i = 1; i < profile_point_count; i++)
    {
        if (profile_power_mw[i] < pmin) pmin = profile_power_mw[i];
        if (profile_power_mw[i] > pmax) pmax = profile_power_mw[i];
    }

    if (pmin_mw) *pmin_mw = pmin;
    if (pmax_mw) *pmax_mw = pmax;
}


// -----------------------------
// Build and transmit one beam profile scan
// Output format:
// BM,axis,scan_no,t_ms,deg,x_mm,stage,raw,v,p_mw
// -----------------------------
static void profile_build_emit_window(uint8_t scan_no, float ts_s, float omega_rad_s)
{
	// Converts requested scan duration in number of ADC samples
    uint32_t nsamp = (uint32_t)(ts_s * ADC_FS_HZ + 0.5f);

    static uint16_t win_buf[ADC_BUF_LEN];

    // Ensure minimum valid samples
    if (nsamp < PROFILE_DECIM * 3U) nsamp = PROFILE_DECIM * 3U;
    if (nsamp > ADC_BUF_LEN) nsamp = ADC_BUF_LEN;

//    // Extract latest ADC window
//    profile_extract_latest_window(win_buf, nsamp);

    // Extract the exact ADC window that started when beam_profile_start_window()
    // was called, rather than whatever samples happen to be latest right now.
    profile_extract_window_from_index(win_buf, profiler.window_start_idx, nsamp);

    // Convert ADC → power with gain handling
    profile_update_power_buffer_from_adc_window(win_buf, nsamp);

    // Emit each processed point
    for (uint32_t k = 0; k < profile_point_count; k++)
    {
        // Convert index → time
    	// Time axis is local to the start of this extracted scan window
        float t_s = ((float)(k * PROFILE_DECIM) / ADC_FS_HZ);

        // Convert time → angle (rad/s → deg)
        float deg = omega_rad_s * t_s * (180.0f / (float)M_PI);
        float x_mm = ARM_LENGTH_MM * (omega_rad_s * t_s);

        profile_t_ms[k] = t_s * 1000.0f;
        profile_deg[k]  = deg;

        printf("BM,%u,%u,%.3f,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
        		(unsigned int)profiler.axis,
               scan_no,
               profile_t_ms[k],
               profile_deg[k],
               x_mm,
               profile_stage_avg[k],
               profile_raw_avg[k],
               profile_v_avg[k],
               profile_power_mw[k]);
    }

    // End marker for Python parser
    printf("BM_DONE\r\n");
}

static void profile_build_emit_window_from_raw_buffers(uint8_t scan_no)
{
    uint32_t k = 0U;
    float counts_per_rev = profile_get_active_counts_per_rev();
    profile_point_count = 0U;

    if (profile_raw_count < PROFILE_DECIM)
        return;

    for (uint32_t i = 0U; i < profile_raw_count; i += PROFILE_DECIM)
    {
        if (k >= PROFILE_MAX_POINTS)
            break;

        float v = adc_counts_to_voltage((float)profile_raw_power_buf[i]);
        float p = voltage_to_power_mw(v);

        int32_t denc = enc_delta_counts(profile_raw_enc_buf[i], profiler.scan_start_enc_count);
        float deg = ((float)denc / counts_per_rev) * 360.0f;
        float x_mm = profile_counts_to_x_mm(profile_raw_enc_buf[i], profiler.scan_start_enc_count, counts_per_rev);
        float t_ms = (float)i * ADC_DT_S * 1000.0f;

        profile_raw_avg[k]   = profile_raw_power_buf[i];
        profile_v_avg[k]     = v;
        profile_power_mw[k]  = p;
        profile_stage_avg[k] = (uint8_t)current_gain;
        profile_t_ms[k]      = t_ms;
        profile_deg[k]       = deg;

        printf("BM,%u,%u,%.3f,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
               (unsigned int)profiler.axis,
               scan_no,
               profile_t_ms[k],
               profile_deg[k],
               x_mm,
               profile_stage_avg[k],
               profile_raw_avg[k],
               profile_v_avg[k],
               profile_power_mw[k]);

        k++;
    }

    profile_point_count = k;
    printf("BM_DONE\r\n");
}

// -----------------------------
// Beam profile scheduler
// Compute beam profiling timing parameters
// Based on motor angular velocity and aperture geometry
// -----------------------------
//static void beam_profile_compute_timing(float omega_rad_s)
//{
//    // Prevent divide-by-zero or extremely slow motion
//    if (omega_rad_s < 0.01f)
//        omega_rad_s = 0.01f;
//
//    profiler.omega_rad_s = omega_rad_s;
//
//    // Angular width of beam crossing (rad)
//    profiler.theta_scan_rad = APERTURE_MM / ARM_LENGTH_MM;
//
//    // Time for blade to cross beam
//    profiler.t_cross_s = profiler.theta_scan_rad / profiler.omega_rad_s;
//
//    // Scan window (slightly larger than crossing)
//    profiler.ts_s = TS_SCALE * profiler.t_cross_s;
//
//    // Time to reach 90° position
//    profiler.t90_s = ((float)M_PI * 0.5f) / profiler.omega_rad_s;
//
//    // Time between scans
//    profiler.t_wait_s = profiler.t90_s - profiler.ts_s;
//
//    if (profiler.t_wait_s < 0.0f)
//        profiler.t_wait_s = 0.0f;
//
//    // Convert to milliseconds
//    profiler.scan_time_ms = (uint32_t)(profiler.ts_s * 1000.0f);
//}
// Compute all beam-profile timing values from the measured motor speed.
// The initial scan lasts pi/3 divided by angular speed.
// The repeated scan width is based on blade crossing time times TS_SCALE.
// The wait between repeated scans is t90 - ts.
static void beam_profile_compute_timing(float omega_rad_s)
{
    // Prevent divide-by-zero and unrealistically small speed values.
    if (omega_rad_s < 0.01f)
        omega_rad_s = 0.01f;

    // Store the angular speed used for this profile run.
    profiler.omega_rad_s = omega_rad_s;

    // Blade crossing angle through the beam aperture.
    profiler.theta_scan_rad = PROFILE_BLADE_CROSS_RAD;

    // Time for the blade to cross the aperture once.
    profiler.t_cross_s = profiler.theta_scan_rad / profiler.omega_rad_s;

    // Long initial scan window: pi/3 worth of rotation
    profiler.ts_init_s = PROFILE_THETA0_RAD / profiler.omega_rad_s;

    // Short repeated scans: slightly larger than crossing time
    profiler.ts_repeat_s = TS_SCALE * profiler.t_cross_s;

    // Time for the rotating blade to reach 90 degrees.
    profiler.t90_s = PROFILE_T90_RAD / profiler.omega_rad_s;

    // Wait time between repeated scans.
    profiler.t_wait_s = profiler.t90_s - profiler.ts_repeat_s;
    if (profiler.t_wait_s < 0.0f)
        profiler.t_wait_s = 0.0f;

    // Keep ms version only for repeated scan window
    profiler.scan_time_ms = (uint32_t)(profiler.ts_repeat_s * 1000.0f + 0.5f);
}

// -----------------------------
// Start a timed sampling window
// -----------------------------
static void beam_profile_start_window(uint32_t duration_ms)
{
    profiler.active_window_ms = duration_ms;
    profiler.state_start_ms = HAL_GetTick();

    // Snapshot where the ADC circular buffer is when this window begins
    profiler.window_start_idx = adc_dma_write_index();

    // Remember count at scan start
    profiler.scan_start_enc_count = enc_read_axis_count(profiler.axis);
}

// Starts scan window by encoder count span
static void beam_profile_start_window_counts(int32_t window_counts)
{
    int32_t dir = profile_get_active_count_direction();

    profiler.window_start_idx     = adc_dma_write_index();
    profiler.scan_start_enc_count = enc_read_axis_count(profiler.axis);
    profiler.scan_stop_enc_target = profiler.scan_start_enc_count + (dir * window_counts);
    profiler.scan_end_enc_count   = profiler.scan_start_enc_count;

    profile_raw_count = 0U;
    profile_last_adc_idx = profiler.window_start_idx;
    profile_capture_stride_counter = 0U;
}

//// Collects power + encoder during active position based scan
//static void beam_profile_sample_position_window(void)
//{
//    if (profile_raw_count >= PROFILE_RAW_MAX_SAMPLES)
//        return;
//
//    profile_raw_power_buf[profile_raw_count] = latest_raw_meas;
//    profile_raw_enc_buf[profile_raw_count]   = enc_read_axis_count(profiler.axis);
//    profile_raw_count++;
//}

//static void beam_profile_sample_position_window(void)
//{
//    uint32_t wr = adc_dma_write_index();
//
//    while (profile_last_adc_idx != wr)
//    {
//        uint16_t raw = adc_buf[profile_last_adc_idx];
//        uint8_t keep_sample = 0U;
//
//        if ((profile_capture_stride_counter % PROFILE_CAPTURE_STORE_STRIDE) == 0U)
//            keep_sample = 1U;
//        profile_capture_stride_counter++;
//
//        if (keep_sample)
//        {
//            if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
//                break;
//
//            profile_raw_power_buf[profile_raw_count] = raw;
//            profile_raw_enc_buf[profile_raw_count]   = enc_read_axis_count(profiler.axis);
//            profile_raw_stage_buf[profile_raw_count] = (uint8_t)current_gain;
//            profile_raw_count++;
//        }
//
//        profile_last_adc_idx++;
//        if (profile_last_adc_idx >= ADC_BUF_LEN)
//            profile_last_adc_idx = 0U;
//    }
//
//    profiler.scan_end_enc_count = enc_read_axis_count(profiler.axis);
//}


//// Scan done check by encoder counts
static uint8_t beam_profile_window_done_counts(void)
{
    int32_t enc_now = enc_read_axis_count(profiler.axis);
    int32_t err = enc_delta_counts(enc_now, profiler.scan_stop_enc_target);
    int32_t dir = profile_get_active_count_direction();

    if (dir < 0)
        err = -err;

    if (err >= 0)
    {
        profiler.scan_end_enc_count = enc_now;
        return 1U;
    }
    return 0U;
}

static void beam_profile_sample_position_window(void)
{
    uint32_t wr = adc_dma_write_index();
    int32_t dir = profile_get_active_count_direction();

    while (profile_last_adc_idx != wr)
    {
        int32_t enc_now = enc_read_axis_count(profiler.axis);
        int32_t remaining = enc_delta_counts(profiler.scan_stop_enc_target, enc_now);

        if (dir < 0)
            remaining = -remaining;

        // Stop immediately once the encoder window is complete.
        if (remaining <= 0)
        {
            profiler.scan_end_enc_count = enc_now;
            break;
        }

        uint16_t raw = adc_buf[profile_last_adc_idx];
        uint8_t keep_sample = 0U;

        if ((profile_capture_stride_counter % PROFILE_CAPTURE_STORE_STRIDE) == 0U)
            keep_sample = 1U;
        profile_capture_stride_counter++;

        if (keep_sample)
        {
            if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
                break;

            profile_raw_power_buf[profile_raw_count] = raw;
            profile_raw_enc_buf[profile_raw_count]   = enc_now;
            profile_raw_stage_buf[profile_raw_count] = (uint8_t)current_gain;
            profile_raw_count++;
        }

        profile_last_adc_idx++;
        if (profile_last_adc_idx >= ADC_BUF_LEN)
            profile_last_adc_idx = 0U;
    }

    profiler.scan_end_enc_count = enc_read_axis_count(profiler.axis);
}

// -----------------------------
// Check if current sampling window is complete
// -----------------------------
static uint8_t beam_profile_window_done(void)
{
    return ((HAL_GetTick() - profiler.state_start_ms) >= profiler.active_window_ms) ? 1U : 0U;
}

static float profile_counts_to_deg(int32_t enc_now, int32_t enc_start, float counts_per_rev)
{
    float dcounts = (float)enc_delta_counts(enc_now, enc_start);
    return (dcounts * 360.0f) / counts_per_rev;
}

static float profile_counts_to_x_mm(int32_t enc_now, int32_t enc_start, float counts_per_rev)
{
    float deg = profile_counts_to_deg(enc_now, enc_start, counts_per_rev);
    float rad = deg * ((float)M_PI / 180.0f);
    return ARM_LENGTH_MM * rad;
}

static int32_t profile_get_active_count_direction(void)
{
    int32_t delta = (profiler.axis == PROFILE_AXIS_M1) ? dc1.delta_count : dc2.delta_count;

    if (delta < 0)
        return -1;

    return 1;
}

#if PROFILE_SYNTHETIC_RAW_ENABLE
#if PROFILE_SYNTH_REPLAY_ENABLE
static const float synth_profile_init_mw[PROFILE_SYNTH_REPLAY_LUT_LEN] = {
    0.223564f, 0.333731f, 0.276017f, 0.083045f, 0.017496f, 0.003294f, 0.002208f, 0.002598f,
    0.003726f, 0.009532f, 0.040834f, 0.112181f, 0.039451f, 0.012217f, 0.003464f, 0.002494f,
    0.002697f, 0.004111f, 0.011938f, 0.056654f, 0.111559f, 0.013564f, 0.008993f, 0.002846f,
    0.000869f, 0.002970f, 0.006521f, 0.029800f, 0.107307f, 0.025750f, 0.010171f, 0.003150f,
    0.002121f, 0.003244f, 0.005770f, 0.025897f, 0.101047f, 0.040595f, 0.009748f, 0.003118f,
    0.002457f, 0.003031f, 0.005976f, 0.029126f, 0.108716f, 0.017174f, 0.008815f, 0.003274f,
    0.002242f, 0.003066f, 0.007411f, 0.040601f, 0.112869f, 0.011020f, 0.006593f, 0.002853f,
    0.002546f, 0.003660f, 0.011472f, 0.070644f, 0.100395f, 0.009859f, 0.004301f, 0.002339f,
};

static const float synth_profile_repeat_mw[PROFILE_SYNTH_REPLAY_LUT_LEN] = {
    0.279482f, 0.223839f, 0.266695f, 0.338918f, 0.267392f, 0.207001f, 0.256452f, 0.298842f,
    0.217509f, 0.267507f, 0.330037f, 0.269355f, 0.199244f, 0.258969f, 0.317072f, 0.216341f,
    0.267965f, 0.327289f, 0.271048f, 0.200023f, 0.263589f, 0.322095f, 0.216143f, 0.265570f,
    0.324078f, 0.270182f, 0.199639f, 0.260505f, 0.323291f, 0.216049f, 0.266183f, 0.322486f,
    0.271894f, 0.203513f, 0.263671f, 0.325368f, 0.214076f, 0.263639f, 0.310951f, 0.208482f,
    0.060572f, 0.044620f, 0.075821f, 0.002890f, 0.003806f, 0.042250f, 0.015254f, 0.003103f,
    0.006993f, 0.082536f, 0.006154f, 0.002680f, 0.019011f, 0.013932f, 0.002405f, 0.003739f,
    0.062800f, 0.011440f, 0.002707f, 0.005961f, 0.081505f, 0.006629f, 0.002592f, 0.011849f,
};

static float synth_profile_replay_lookup_mw(uint8_t scan_no, float x_mm, float span_mm)
{
    const float *lut = (scan_no == 0U) ? synth_profile_init_mw : synth_profile_repeat_mw;
    float frac;
    float pos;
    uint32_t i0;
    uint32_t i1;
    float t;

    if (span_mm < 1e-6f)
        return lut[0];

    frac = fabsf(x_mm) / span_mm;
    if (frac < 0.0f)
        frac = 0.0f;
    if (frac > 1.0f)
        frac = 1.0f;

    pos = frac * (float)(PROFILE_SYNTH_REPLAY_LUT_LEN - 1U);
    i0 = (uint32_t)pos;
    if (i0 >= (PROFILE_SYNTH_REPLAY_LUT_LEN - 1U))
        return lut[PROFILE_SYNTH_REPLAY_LUT_LEN - 1U];

    i1 = i0 + 1U;
    t = pos - (float)i0;
    return lut[i0] + t * (lut[i1] - lut[i0]);
}
#endif

#if !PROFILE_SYNTH_REPLAY_ENABLE
static float synth_profile_center_mm(uint8_t scan_no, float span_mm)
{
    float frac = (scan_no == 0U) ? PROFILE_SYNTH_INIT_CENTER_FRAC : PROFILE_SYNTH_REPEAT_CENTER_FRAC;
    return frac * span_mm;
}

static float synth_profile_sigma_mm(uint8_t scan_no)
{
    return (scan_no == 0U) ? PROFILE_SYNTH_INIT_SIGMA_MM : PROFILE_SYNTH_REPEAT_SIGMA_MM;
}

static float synth_profile_position_mm(float x_mm, float span_mm)
{
    float pos_mm = fabsf(x_mm);

    if (pos_mm > span_mm)
        pos_mm = span_mm;

    return pos_mm;
}

static float synth_profile_noise_mw(uint8_t axis, uint8_t scan_no, uint32_t sample_idx, float pos_mm)
{
    float noise_frac = (scan_no == 0U) ? PROFILE_SYNTH_INIT_NOISE_FRAC : PROFILE_SYNTH_REPEAT_NOISE_FRAC;
    float a = 0.61f * sinf(0.85f * pos_mm + 0.013f * (float)sample_idx + 0.4f * (float)axis);
    float b = 0.39f * sinf(2.35f * pos_mm + 0.041f * (float)sample_idx + 1.2f + 0.2f * (float)axis);

    if (noise_frac <= 0.0f)
        return 0.0f;

    return PROFILE_SYNTH_AMPLITUDE_MW * noise_frac * (a + b);
}
#endif

static float synth_profile_power_mw(uint8_t axis, uint8_t scan_no, uint32_t sample_idx, float x_mm, float span_mm)
{
#if PROFILE_SYNTH_REPLAY_ENABLE
    (void)axis;
    (void)sample_idx;
    return synth_profile_replay_lookup_mw(scan_no, x_mm, span_mm);
#else
    float pos_mm = synth_profile_position_mm(x_mm, span_mm);
    float sigma_mm = synth_profile_sigma_mm(scan_no);
    float x0_mm = synth_profile_center_mm(scan_no, span_mm);
    float z;
    float edge;
    float p_mw;

    if (sigma_mm < 0.05f)
        sigma_mm = 0.05f;

    z = (pos_mm - x0_mm) / (((float)sqrt(2.0)) * sigma_mm);
    edge = 0.5f * (1.0f - erff(z));

    p_mw = PROFILE_SYNTH_BASELINE_MW + PROFILE_SYNTH_AMPLITUDE_MW * edge;
    p_mw += synth_profile_noise_mw(axis, scan_no, sample_idx, pos_mm);

    if (p_mw < 0.0f)
        p_mw = 0.0f;

    return p_mw;
#endif
}

static uint16_t synth_profile_power_to_adc_counts(float p_mw, uint8_t *stage_used, float *v_out)
{
    float v = (p_mw / PROFILE_SYNTH_POWER_FULL_SCALE_MW) * (0.92f * ADC_VREF);
    float counts_f;

    if (v < 0.0f)
        v = 0.0f;
    if (v > ADC_VREF)
        v = ADC_VREF;

    counts_f = (v / ADC_VREF) * ADC_MAX_COUNTS;
    if (counts_f < 0.0f)
        counts_f = 0.0f;
    if (counts_f > ADC_MAX_COUNTS)
        counts_f = ADC_MAX_COUNTS;

    if (stage_used != NULL)
        *stage_used = (uint8_t)GAIN_STAGE_4;
    if (v_out != NULL)
        *v_out = v;

    return (uint16_t)(counts_f + 0.5f);
}
#endif

static void host_emit_raw_scan(uint8_t axis, uint8_t scan_no)
{
    float counts_per_rev = (axis == PROFILE_AXIS_M1) ? ENC1_COUNTS_PER_REV : ENC2_COUNTS_PER_REV;
    uint32_t emitted_count = 0U;
#if PROFILE_SYNTHETIC_RAW_ENABLE
    float span_mm = profile_counts_to_x_mm(profiler.scan_end_enc_count,
                                           profiler.scan_start_enc_count,
                                           counts_per_rev);
    if (span_mm < 0.0f)
        span_mm = -span_mm;
#endif

    for (uint32_t i = 0U; i < profile_raw_count; i += PROFILE_HOST_EMIT_STRIDE)
        emitted_count++;

    printf("RAW_SCAN_BEGIN,axis=%u,scan=%u,n=%lu,synthetic=%u,stride=%lu,emit_stride=%lu\r\n",
           (unsigned int)axis,
           (unsigned int)scan_no,
           (unsigned long)emitted_count,
           (unsigned int)PROFILE_SYNTHETIC_RAW_ENABLE,
           (unsigned long)PROFILE_CAPTURE_STORE_STRIDE,
           (unsigned long)PROFILE_HOST_EMIT_STRIDE);

    for (uint32_t i = 0; i < profile_raw_count; i += PROFILE_HOST_EMIT_STRIDE)
    {
        uint8_t saved_stage = profile_raw_stage_buf[i];
        float v;
        float p_mw;
        float deg;
        float x_mm;
        int32_t enc_now = (int32_t)profile_raw_enc_buf[i];

#if !PROFILE_SYNTHETIC_RAW_ENABLE
        float rf;
        switch (saved_stage)
        {
            case GAIN_STAGE_4: rf = R4; break;
            case GAIN_STAGE_3: rf = R3; break;
            case GAIN_STAGE_2: rf = R2; break;
            case GAIN_STAGE_1:
            default:           rf = R1; break;
        }
#endif

        deg = profile_counts_to_deg(enc_now, profiler.scan_start_enc_count, counts_per_rev);
        x_mm = profile_counts_to_x_mm(enc_now, profiler.scan_start_enc_count, counts_per_rev);

#if PROFILE_SYNTHETIC_RAW_ENABLE
        p_mw = synth_profile_power_mw(axis, scan_no, i, x_mm, span_mm);
        profile_raw_power_buf[i] = synth_profile_power_to_adc_counts(p_mw, &saved_stage, &v);
        profile_raw_stage_buf[i] = saved_stage;
#else
        v = adc_counts_to_voltage((float)profile_raw_power_buf[i]);
        p_mw = ((v / rf) / 0.00213f) * 1000.0f;
#endif

        printf("RAW_SCAN,%u,%u,%lu,%ld,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned long)i,
               (long)enc_now,
               deg,
               x_mm,
               (unsigned int)saved_stage,
               (unsigned int)profile_raw_power_buf[i],
               v,
               p_mw);
    }

    printf("RAW_SCAN_DONE,axis=%u,scan=%u,n=%lu\r\n",
           (unsigned int)axis,
           (unsigned int)scan_no,
           (unsigned long)emitted_count);
}

static void host_repeat_buffer_reset(void)
{
    memset(host_repeat_buf_start, 0, sizeof(host_repeat_buf_start));
    memset(host_repeat_buf_count, 0, sizeof(host_repeat_buf_count));
    memset(host_repeat_buf_emit_stride, 0, sizeof(host_repeat_buf_emit_stride));
    memset(host_repeat_buf_scan_start_enc, 0, sizeof(host_repeat_buf_scan_start_enc));
    memset(host_repeat_buf_scan_end_enc, 0, sizeof(host_repeat_buf_scan_end_enc));
    host_repeat_buf_used = 0U;
    host_repeat_buf_valid_scans = 0U;
}

static void host_arm_repeat_sequence(uint8_t reset_buffer)
{
    int32_t dir = profile_get_active_count_direction();
    int32_t current_enc = enc_read_axis_count(profiler.axis);
    int32_t period_counts =
        dir * (int32_t)(host_repeat_scan_width_counts + host_repeat_wait_counts);

    host_cmd_run_repeat_seq = 0U;

    if (reset_buffer)
        host_repeat_buffer_reset();

    host_repeat_scan_index = 0U;

    if (host_local_t0_ready)
    {
        host_first_wait_counts = profiler.xdelay_counts;
        host_next_scan_target =
            profiler.scan_end_enc_count + (dir * host_first_wait_counts);
    }
    else
    {
        host_next_scan_target = current_enc + (dir * host_first_wait_counts);
    }

    // If the first target is already behind us, skip to the next future period.
    while (1)
    {
        int32_t err = enc_delta_counts(current_enc, host_next_scan_target);
        if (dir < 0)
            err = -err;

        if (err < 0)
            break;

        host_next_scan_target += period_counts;
    }

    printf("REPEAT_SEQ_ARMED,axis=%u,current=%ld,first_wait=%ld,target=%ld,num=%lu,scan_width=%lu,wait=%ld\r\n",
           (unsigned int)profiler.axis,
           (long)current_enc,
           (long)host_first_wait_counts,
           (long)host_next_scan_target,
           (unsigned long)host_repeat_num_scans,
           (unsigned long)host_repeat_scan_width_counts,
           (long)host_repeat_wait_counts);

    host_profile_state = HOST_CAPTURE_REPEAT;
}

static uint8_t host_buffer_current_repeat_scan(uint8_t scan_no)
{
    uint32_t emit_stride = PROFILE_HOST_EMIT_STRIDE;
    uint16_t scan_slot;
    uint16_t start;
    uint16_t count = 0U;

    if (scan_no > PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS)
        return 0U;

    while ((((profile_raw_count + emit_stride) - 1U) / emit_stride) > PROFILE_REPEAT_BUFFER_MAX_POINTS_PER_SCAN)
        emit_stride += PROFILE_HOST_EMIT_STRIDE;

    scan_slot = (uint16_t)scan_no;
    start = host_repeat_buf_used;

    for (uint32_t i = 0U; i < profile_raw_count; i += emit_stride)
    {
        if ((count >= PROFILE_REPEAT_BUFFER_MAX_POINTS_PER_SCAN) ||
            (host_repeat_buf_used >= PROFILE_REPEAT_BUFFER_TOTAL_POINTS))
        {
            return 0U;
        }

        host_repeat_buf_enc[host_repeat_buf_used] = profile_raw_enc_buf[i];
        host_repeat_buf_raw[host_repeat_buf_used] = profile_raw_power_buf[i];
        host_repeat_buf_stage[host_repeat_buf_used] = profile_raw_stage_buf[i];
        host_repeat_buf_used++;
        count++;
    }

    host_repeat_buf_start[scan_slot] = start;
    host_repeat_buf_count[scan_slot] = count;
    host_repeat_buf_emit_stride[scan_slot] = (uint16_t)emit_stride;
    host_repeat_buf_scan_start_enc[scan_slot] = profiler.scan_start_enc_count;
    host_repeat_buf_scan_end_enc[scan_slot] = profiler.scan_end_enc_count;
    if ((uint8_t)(scan_slot + 1U) > host_repeat_buf_valid_scans)
        host_repeat_buf_valid_scans = (uint8_t)(scan_slot + 1U);

    return 1U;
}

static void host_emit_buffered_repeat_scans(uint8_t axis)
{
    float counts_per_rev = (axis == PROFILE_AXIS_M1) ? ENC1_COUNTS_PER_REV : ENC2_COUNTS_PER_REV;

    for (uint8_t scan_slot = 0U; scan_slot < host_repeat_buf_valid_scans; scan_slot++)
    {
        uint16_t count = host_repeat_buf_count[scan_slot];
        uint16_t start = host_repeat_buf_start[scan_slot];
        uint16_t emit_stride = host_repeat_buf_emit_stride[scan_slot];
        uint8_t scan_no = scan_slot;
        int32_t scan_start_enc = host_repeat_buf_scan_start_enc[scan_slot];
        int32_t scan_end_enc = host_repeat_buf_scan_end_enc[scan_slot];
#if PROFILE_SYNTHETIC_RAW_ENABLE
        float span_mm = profile_counts_to_x_mm(scan_end_enc, scan_start_enc, counts_per_rev);
        if (span_mm < 0.0f)
            span_mm = -span_mm;
#endif

        if (count == 0U)
            continue;

        printf("RAW_SCAN_BEGIN,axis=%u,scan=%u,n=%u,synthetic=%u,stride=%lu,emit_stride=%u\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned int)count,
               (unsigned int)PROFILE_SYNTHETIC_RAW_ENABLE,
               (unsigned long)PROFILE_CAPTURE_STORE_STRIDE,
               (unsigned int)emit_stride);

        for (uint16_t j = 0U; j < count; j++)
        {
            uint16_t raw = host_repeat_buf_raw[start + j];
            uint8_t saved_stage = host_repeat_buf_stage[start + j];
            int32_t enc_now = host_repeat_buf_enc[start + j];
            float v;
            float p_mw;
            float deg = profile_counts_to_deg(enc_now, scan_start_enc, counts_per_rev);
            float x_mm = profile_counts_to_x_mm(enc_now, scan_start_enc, counts_per_rev);

#if PROFILE_SYNTHETIC_RAW_ENABLE
            p_mw = synth_profile_power_mw(axis, scan_no, (uint32_t)j * emit_stride, x_mm, span_mm);
            raw = synth_profile_power_to_adc_counts(p_mw, &saved_stage, &v);
#else
            float rf;
            switch (saved_stage)
            {
                case GAIN_STAGE_4: rf = R4; break;
                case GAIN_STAGE_3: rf = R3; break;
                case GAIN_STAGE_2: rf = R2; break;
                case GAIN_STAGE_1:
                default:           rf = R1; break;
            }
            v = adc_counts_to_voltage((float)raw);
            p_mw = ((v / rf) / 0.00213f) * 1000.0f;
#endif

            printf("RAW_SCAN,%u,%u,%u,%ld,%.6f,%.6f,%u,%u,%.6f,%.6f\r\n",
                   (unsigned int)axis,
                   (unsigned int)scan_no,
                   (unsigned int)j,
                   (long)enc_now,
                   deg,
                   x_mm,
                   (unsigned int)saved_stage,
                   (unsigned int)raw,
                   v,
                   p_mw);
        }

        printf("RAW_SCAN_DONE,axis=%u,scan=%u,n=%u\r\n",
               (unsigned int)axis,
               (unsigned int)scan_no,
               (unsigned int)count);

        if (scan_no == 0U)
        {
            printf("INIT_SCAN_READY,axis=%u,n=%u\r\n",
                   (unsigned int)axis,
                   (unsigned int)count);
        }
        else
        {
            printf("REPEAT_SCAN_READY,axis=%u,scan=%u,n=%u\r\n",
                   (unsigned int)axis,
                   (unsigned int)scan_no,
                   (unsigned int)count);
        }
    }
}

static float host_profile_sample_power_mw(uint8_t axis,
                                          uint8_t scan_no,
                                          uint32_t sample_idx,
                                          uint16_t raw,
                                          uint8_t saved_stage,
                                          int32_t enc_now,
                                          int32_t scan_start_enc,
                                          int32_t scan_end_enc)
{
#if PROFILE_SYNTHETIC_RAW_ENABLE
    float counts_per_rev = (axis == PROFILE_AXIS_M1) ? ENC1_COUNTS_PER_REV : ENC2_COUNTS_PER_REV;
    float span_mm = profile_counts_to_x_mm(scan_end_enc, scan_start_enc, counts_per_rev);
    float x_mm = profile_counts_to_x_mm(enc_now, scan_start_enc, counts_per_rev);

    if (span_mm < 0.0f)
        span_mm = -span_mm;

    return synth_profile_power_mw(axis, scan_no, sample_idx, x_mm, span_mm);
#else
    float rf;

    switch (saved_stage)
    {
        case GAIN_STAGE_4: rf = R4; break;
        case GAIN_STAGE_3: rf = R3; break;
        case GAIN_STAGE_2: rf = R2; break;
        case GAIN_STAGE_1:
        default:           rf = R1; break;
    }

    return ((adc_counts_to_voltage((float)raw) / rf) / 0.00213f) * 1000.0f;
#endif
}

static void host_profile_learn_clear_threshold_from_initial_scan(void)
{
    if (profile_raw_count == 0U)
    {
        profiler.p_min_init_mw = 0.0f;
        profiler.p_max_init_mw = 0.0f;
        profiler.p_clear_thresh_mw = 0.0f;
        profiler.clear_confirm_count = 0U;
        return;
    }

    profiler.p_min_init_mw = host_profile_sample_power_mw((uint8_t)profiler.axis,
                                                          0U,
                                                          0U,
                                                          profile_raw_power_buf[0],
                                                          profile_raw_stage_buf[0],
                                                          profile_raw_enc_buf[0],
                                                          profiler.scan_start_enc_count,
                                                          profiler.scan_end_enc_count);
    profiler.p_max_init_mw = profiler.p_min_init_mw;

    for (uint32_t i = 1U; i < profile_raw_count; i++)
    {
        float p_mw = host_profile_sample_power_mw((uint8_t)profiler.axis,
                                                  0U,
                                                  i,
                                                  profile_raw_power_buf[i],
                                                  profile_raw_stage_buf[i],
                                                  profile_raw_enc_buf[i],
                                                  profiler.scan_start_enc_count,
                                                  profiler.scan_end_enc_count);

        if (p_mw < profiler.p_min_init_mw)
            profiler.p_min_init_mw = p_mw;
        if (p_mw > profiler.p_max_init_mw)
            profiler.p_max_init_mw = p_mw;
    }

    profiler.p_clear_thresh_mw = profiler.p_min_init_mw
                               + PROFILE_CLEAR_THRESH_FRAC
                                 * (profiler.p_max_init_mw - profiler.p_min_init_mw);
    profiler.clear_confirm_count = 0U;

    printf("PROFILE_CLEAR_LEARN,axis=%u,pmin=%.4f,pmax=%.4f,pclear=%.4f\r\n",
           (unsigned int)profiler.axis,
           profiler.p_min_init_mw,
           profiler.p_max_init_mw,
           profiler.p_clear_thresh_mw);
}

static float host_profile_get_live_clear_power_mw(void)
{
#if PROFILE_SYNTHETIC_RAW_ENABLE
    float counts_per_rev = (profiler.axis == PROFILE_AXIS_M1) ? ENC1_COUNTS_PER_REV : ENC2_COUNTS_PER_REV;
    int32_t enc_now = enc_read_axis_count(profiler.axis);
    float x_rel_mm = profile_counts_to_x_mm(enc_now, profiler.t0_enc_count, counts_per_rev);
    float sigma_mm = PROFILE_SYNTH_REPEAT_SIGMA_MM;
    float p_span = profiler.p_max_init_mw - profiler.p_min_init_mw;
    float norm;

    if (x_rel_mm < 0.0f)
        x_rel_mm = -x_rel_mm;
    if (sigma_mm < 0.05f)
        sigma_mm = 0.05f;
    if (p_span < 0.0f)
        p_span = 0.0f;

    norm = 1.0f - expf(-(x_rel_mm * x_rel_mm) / (2.0f * sigma_mm * sigma_mm));
    if (norm < 0.0f)
        norm = 0.0f;
    if (norm > 1.0f)
        norm = 1.0f;

    return profiler.p_min_init_mw + p_span * norm;
#else
    return latest_p_mw;
#endif
}

static void clear_active_axis_out_of_beam(void)
{
    uint32_t clear_start_ms = HAL_GetTick();
    DCMotor_t *active_motor = (profiler.axis == PROFILE_AXIS_M1) ? &dc1 : &dc2;
    uint8_t axis_no = (uint8_t)profiler.axis;

    profiler.clear_confirm_count = 0U;

    while (1)
    {
        float p_live;

        live_power_update_service();

        uint32_t now = HAL_GetTick();
        if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;

            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

            // keep only the active motor running here
            dc_motor_auto_correct_counts(active_motor);

            last_dc_speed_ms = now;
        }

        p_live = host_profile_get_live_clear_power_mw();

        if (p_live > profiler.p_clear_thresh_mw)
        {
            if (profiler.clear_confirm_count < 255U)
                profiler.clear_confirm_count++;
        }
        else
        {
            profiler.clear_confirm_count = 0U;
        }

        if (profiler.clear_confirm_count >= PROFILE_CLEAR_CONFIRM_COUNT)
        {
            dc_motor_set_duty(active_motor, 0);
            printf("AXIS_CLEAR_DONE,axis=%u,plive=%.4f,pclear=%.4f,pmin=%.4f,pmax=%.4f\r\n",
                   (unsigned int)axis_no,
                   p_live,
                   profiler.p_clear_thresh_mw,
                   profiler.p_min_init_mw,
                   profiler.p_max_init_mw);
            return;
        }

        if ((HAL_GetTick() - clear_start_ms) >= PROFILE_CLEAR_TIMEOUT_MS)
        {
            dc_motor_set_duty(active_motor, 0);
            printf("AXIS_CLEAR_TIMEOUT,axis=%u,plive=%.4f,pclear=%.4f,pmin=%.4f,pmax=%.4f\r\n",
                   (unsigned int)axis_no,
                   p_live,
                   profiler.p_clear_thresh_mw,
                   profiler.p_min_init_mw,
                   profiler.p_max_init_mw);
            return;
        }
    }
}

// Maps extracted raw ADC to encoder count across initial scan
static uint32_t profile_extract_initial_raw_power_and_encoder(uint16_t *raw_dst,
                                                              int32_t *enc_dst,
                                                              uint32_t max_n)
{
    uint32_t nsamp = (uint32_t)(profiler.ts_init_s * ADC_FS_HZ + 0.5f);
    static uint16_t win_buf[ADC_BUF_LEN];

    if (nsamp > ADC_BUF_LEN)
        nsamp = ADC_BUF_LEN;
    if (nsamp > max_n)
        nsamp = max_n;
    if (nsamp < 5U)
        return 0U;

    profile_extract_window_from_index(win_buf, profiler.window_start_idx, nsamp);

    profiler.scan_end_enc_count = enc_read_axis_count(profiler.axis);

    int32_t enc0 = profiler.scan_start_enc_count;
    int32_t denc = enc_delta_counts(profiler.scan_end_enc_count, enc0);

    for (uint32_t i = 0; i < nsamp; i++)
    {
        raw_dst[i] = win_buf[i];

        float frac = (nsamp > 1U) ? ((float)i / (float)(nsamp - 1U)) : 0.0f;
        enc_dst[i] = enc0 + (int32_t)(frac * (float)denc + 0.5f);
    }

    return nsamp;
}

// -----------------------------
// Extract a window of nsamp samples starting from a specific
// ADC DMA circular-buffer index
// -----------------------------
static void profile_extract_window_from_index(uint16_t *dst, uint32_t start_idx, uint32_t nsamp)
{
    if (nsamp == 0U)
        return;

    if (nsamp > ADC_BUF_LEN)
        nsamp = ADC_BUF_LEN;

    start_idx %= ADC_BUF_LEN;

    for (uint32_t i = 0; i < nsamp; i++)
    {
        uint32_t idx = (start_idx + i) % ADC_BUF_LEN;
        dst[i] = adc_buf[idx];
    }
}

// -----------------------------
// Initialize beam profiling sequence
// Called when "PROFILE" command is received
// -----------------------------
//static void beam_profile_begin(void)
//{
//    // Compute timing using current motor speed
//    beam_profile_compute_timing(fabsf(dc1.omega));
//
//    profiler.active = 1U;
//    profiler.scan_count = 0U;
//
//    profiler.t0_idx = 0U;
//    profiler.t0_s = 0.0f;
//
//    // Start with initial scan to find beam center
//    profiler.state = PROFILE_INITIAL_SCAN;
//
//    beam_profile_start_window(profiler.scan_time_ms);
//
//    // Send configuration info to Python
//    printf("PROFILE_BEGIN,omega=%.4f,ts_ms=%lu,t90_s=%.4f,twait_s=%.4f\r\n",
//           profiler.omega_rad_s,
//           (unsigned long)profiler.scan_time_ms,
//           profiler.t90_s,
//           profiler.t_wait_s);
//}
// Initialize the beam-profile sequence after the DC motors have spun up.
// Uses the measured motor angular speed to compute all timing values.
//static void beam_profile_begin(void)
//{
//
//    beam_profile_compute_timing(beam_profile_get_active_omega());
//
//    profiler.active = 1U;
//    profiler.scan_count = 0U;
//    profiler.axis_sequence_done = 0U;
//
//    beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
//    profiler.state = PROFILE_INITIAL_SCAN;
//
//    printf("PROFILE_BEGIN,axis=%u,init_counts=%d,repeat_counts=%d,event_counts=%d\r\n",
//           (unsigned int)profiler.axis,
//           PROFILE_INIT_COUNTS,
//           PROFILE_REPEAT_COUNTS,
//           PROFILE_EVENT_COUNTS);
////	float omega_to_use = profile_locked_omega_rad_s;
////
////	if (omega_to_use <= 1e-6f)
////	    omega_to_use = beam_profile_get_active_omega();
////
////	beam_profile_compute_timing(omega_to_use);
////
////    // Mark the profiler as active and reset scan counters/results.
////    profiler.active = 1U;
////    profiler.scan_count = 0U;
////    profiler.t0_idx = 0U;
////    profiler.t0_s = 0.0f;
////    profiler.t_wait_first_s = 0.0f;
////
////    profiler.p_min_init_mw = 0.0f;
////    profiler.p_max_init_mw = 0.0f;
////    profiler.p_clear_thresh_mw = 0.0f;
////    profiler.clear_confirm_count = 0U;
////
////    // Start with the initial long scan used to find t0.
////    profiler.state = PROFILE_INITIAL_SCAN;
////    beam_profile_start_window((uint32_t)(profiler.ts_init_s * 1000.0f + 0.5f));
////
////    // Send timing information to the host for logging/debugging.
////    printf("PROFILE_BEGIN,axis=%u,omega=%.4f,initial_ms=%lu,ts_ms=%.1f,t90_s=%.4f,twait_s=%.4f\r\n",
////           (unsigned int)profiler.axis,
////    		profiler.omega_rad_s,
////           profiler.ts_init_s * 1000.0f,
////           profiler.ts_repeat_s * 1000.0f,
////           profiler.t90_s,
////           profiler.t_wait_s);
//}

static void host_profile_update(void)
{
    switch (host_profile_state)
    {
        case HOST_IDLE:
            break;

        case HOST_WAIT_STABLE:
        {
            uint32_t now = HAL_GetTick();

            if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
            {
                float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;

                dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
                dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

                dc_motor_auto_correct_active_axis();
                last_dc_speed_ms = now;

                if (dc_active_axis_rpm_is_stable())
                {
                    profile_locked_rpm = dc_get_active_rpm();
                    profile_locked_omega_rad_s = beam_profile_get_active_omega();

                    printf("AXIS_STABLE,axis=%u,rpm=%.4f,omega=%.6f\r\n",
                           (unsigned int)profiler.axis,
                           profile_locked_rpm,
                           profile_locked_omega_rad_s);

                    beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
                    host_profile_state = HOST_CAPTURE_INIT;
                }
            }

            if ((now - profile_start_request_ms) >= PROFILE_STABILIZE_TIMEOUT_MS)
            {
                profile_locked_rpm = dc_get_active_rpm();
                profile_locked_omega_rad_s = beam_profile_get_active_omega();

                if (profile_locked_rpm < PROFILE_MIN_VALID_RPM)
                {
                    dc_motors_stop_all();
                    printf("AXIS_START_FAILED,axis=%u,rpm=%.4f,omega=%.6f,duty=%u\r\n",
                           (unsigned int)profiler.axis,
                           profile_locked_rpm,
                           profile_locked_omega_rad_s,
                           (unsigned int)((profiler.axis == PROFILE_AXIS_M1) ? dc1.duty : dc2.duty));
                    host_profile_state = HOST_IDLE;
                    break;
                }

                printf("AXIS_STABLE_TIMEOUT,axis=%u,rpm=%.4f,omega=%.6f\r\n",
                       (unsigned int)profiler.axis,
                       profile_locked_rpm,
                       profile_locked_omega_rad_s);

                beam_profile_start_window_counts(PROFILE_INIT_COUNTS);
                host_profile_state = HOST_CAPTURE_INIT;
            }
        }
        break;

        case HOST_CAPTURE_INIT:
            beam_profile_sample_position_window();

            if (beam_profile_window_done_counts())
            {
                host_profile_compute_local_t0();

                if (host_local_t0_ready)
                {
                    // ARM FIRST, immediately
                    host_arm_repeat_sequence(0U);

                    // Then save scan 0 for later dump
                    host_repeat_buffer_reset();

                    if (!host_buffer_current_repeat_scan(0U))
                    {
                        printf("PROFILE_REPEAT_BUFFER_OVERFLOW,axis=%u,scan=0,n=%lu\r\n",
                               (unsigned int)profiler.axis,
                               (unsigned long)profile_raw_count);
                        dc_motors_stop_all();
                        host_profile_state = HOST_DONE;
                    }
                }
                else
                {
                    host_repeat_buffer_reset();

                    if (!host_buffer_current_repeat_scan(0U))
                    {
                        printf("PROFILE_REPEAT_BUFFER_OVERFLOW,axis=%u,scan=0,n=%lu\r\n",
                               (unsigned int)profiler.axis,
                               (unsigned long)profile_raw_count);
                        dc_motors_stop_all();
                        host_profile_state = HOST_DONE;
                    }
                    else
                    {
                        host_emit_buffered_repeat_scans((uint8_t)profiler.axis);
                        host_profile_state = HOST_IDLE;
                    }
                }
            }
            else if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
            {
                printf("PROFILE_CAPTURE_OVERFLOW,axis=%u,scan=0,target=%ld,n=%lu\r\n",
                       (unsigned int)profiler.axis,
                       (long)profiler.scan_stop_enc_target,
                       (unsigned long)profile_raw_count);
                host_profile_state = HOST_IDLE;
            }
            break;

        case HOST_CAPTURE_REPEAT:
        {
            int32_t now = enc_read_axis_count(profiler.axis);
            int32_t dir = profile_get_active_count_direction();
            int32_t err = enc_delta_counts(now, host_next_scan_target);

            if (dir < 0)
                err = -err;

            if (err >= 0)
            {
                // Save the target that triggered this scan
                int32_t fired_target = host_next_scan_target;

                printf("REPEAT_TRIGGER,axis=%u,scan=%lu,now=%ld,target=%ld,err=%ld\r\n",
                       (unsigned int)profiler.axis,
                       (unsigned long)(host_repeat_scan_index + 1U),
                       (long)now,
                       (long)host_next_scan_target,
                       (long)err);

                beam_profile_start_window_counts(host_repeat_scan_width_counts);

                while (!beam_profile_window_done_counts())
                {
                    live_power_update_service();
                    beam_profile_sample_position_window();

                    if (profile_raw_count >= PROFILE_CAPTURE_MAX_SAMPLES)
                    {
                        printf("PROFILE_CAPTURE_OVERFLOW,axis=%u,scan=%lu,target=%ld,n=%lu\r\n",
                               (unsigned int)profiler.axis,
                               (unsigned long)(host_repeat_scan_index + 1U),
                               (long)profiler.scan_stop_enc_target,
                               (unsigned long)profile_raw_count);
                        dc_motors_stop_all();
                        host_profile_state = HOST_DONE;
                        break;
                    }
                }

                if (host_profile_state == HOST_DONE)
                    break;

                host_repeat_scan_index++;

                if (!host_buffer_current_repeat_scan((uint8_t)host_repeat_scan_index))
                {
                    printf("PROFILE_REPEAT_BUFFER_OVERFLOW,axis=%u,scan=%lu,n=%lu\r\n",
                           (unsigned int)profiler.axis,
                           (unsigned long)host_repeat_scan_index,
                           (unsigned long)profile_raw_count);
                    dc_motors_stop_all();
                    host_profile_state = HOST_DUMP_REPEAT;
                    break;
                }

                if (host_repeat_scan_index < host_repeat_num_scans)
                {
                    int32_t period_counts =
                        dir * (int32_t)(host_repeat_scan_width_counts + host_repeat_wait_counts);
                    int32_t now2 = enc_read_axis_count(profiler.axis);

                    // Anchor the next target to the target that fired this scan,
                    // not to the current late physical position
                    host_next_scan_target = fired_target + period_counts;

                    // Catch up only if that next target is already behind current position
                    while (1)
                    {
                        int32_t e = enc_delta_counts(now2, host_next_scan_target);
                        if (dir < 0)
                            e = -e;

                        if (e < 0)
                            break;

                        host_next_scan_target += period_counts;
                    }

                    printf("REPEAT_NEXT,axis=%u,scan=%lu,fired=%ld,next=%ld,now=%ld\r\n",
                           (unsigned int)profiler.axis,
                           (unsigned long)host_repeat_scan_index,
                           (long)fired_target,
                           (long)host_next_scan_target,
                           (long)now2);
                }

                if (host_repeat_scan_index >= host_repeat_num_scans)
                {
                    if (profiler.axis == PROFILE_AXIS_M1)
                        host_profile_state = HOST_CLEAR_AXIS1;
                    else
                        host_profile_state = HOST_CLEAR_AXIS2;
                }
                else
                {
                    host_profile_state = HOST_CAPTURE_REPEAT;
                }
            }
        }
        break;

        case HOST_CLEAR_AXIS1:
            clear_active_axis_out_of_beam();
            dc_motors_stop_all();
            host_profile_state = HOST_DUMP_REPEAT;
            break;

        case HOST_CLEAR_AXIS2:
            clear_active_axis_out_of_beam();
            dc_motors_stop_all();
            host_profile_state = HOST_DUMP_REPEAT;
            break;

        case HOST_DUMP_REPEAT:
            host_emit_buffered_repeat_scans((uint8_t)profiler.axis);
            if (profiler.axis == PROFILE_AXIS_M1)
            {
                printf("AXIS_DONE_READY_FOR_NEXT,axis=0\r\n");
                host_profile_state = HOST_IDLE;
            }
            else
            {
                printf("PROFILE_DONE,axis=1\r\n");
                host_profile_state = HOST_DONE;
            }
            break;

        case HOST_DONE:
        default:
            break;
    }
}

void steppers_run_continuous(void)
{
    static uint32_t last_step_us = 0;

    // adjust speed here (microseconds between steps)
    const uint32_t step_period_us = 2000;   // 2 ms → ~500 steps/sec

    uint32_t now = DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);

    if ((now - last_step_us) >= step_period_us)
    {
        last_step_us = now;


        // Step motor 2 (Z)
//        z_step_pulse();

        step_pulse();

    }
}


// -----------------------------
// Main beam profiling state machine
// Controls timing of scans relative to blade motion
// -----------------------------
// Main non-blocking beam-profile state machine.
// Sequence:
// 1) initial long scan
// 2) find t0 from maximum derivative
// 3) wait until the blade reaches 90 degrees
// 4) repeat wait + scan for NUM_PROFILE_SCANS
// 5) stop motors and finish
//static void beam_profile_update(void)
//{
//    if (!profiler.active)
//        return;
//
//    switch (profiler.state)
//    {
//        case PROFILE_IDLE:
//            break;
//
////        case PROFILE_INITIAL_SCAN:
////            // Wait for initial long scan to complete once per axis
////            if (beam_profile_window_done())
////            {
////                profiler.state = PROFILE_FIND_T0;
////            }
////            break;
//        case PROFILE_INITIAL_SCAN:
//        	beam_profile_sample_position_window();
//
//        	if(beam_profile_window_done_counts()){
//        		profiler.state = PROFILE_FIND_T0;
//        	}
//        	break;
//
//        case PROFILE_FIND_T0:
//        {
//            float fit_x0_idx;
//
//            // Emit initial scan 0 to Python from raw buffers
//            profile_build_emit_window_from_raw_buffers(0U);
//
//            if (profile_raw_count < 5U)
//            {
//                printf("PROFILE_T0_FAIL,axis=%u,reason=too_few_samples\r\n",
//                       (unsigned int)profiler.axis);
//                profiler.state = PROFILE_DONE;
//                break;
//            }
//
//            fit_x0_idx = profile_find_t0_fit_index_from_raw_power(profile_raw_power_buf, profile_raw_count);
//
//            profiler.t0_idx = (uint32_t)(fit_x0_idx + 0.5f);
//            if (profiler.t0_idx >= profile_raw_count)
//                profiler.t0_idx = profile_raw_count - 1U;
//
//            profiler.t0_enc_count = interp_encoder_count_at_index(profile_raw_enc_buf, profile_raw_count, fit_x0_idx);
//            profiler.x0_counts = enc_delta_counts(profiler.t0_enc_count, profiler.scan_start_enc_count);
//
//            profiler.t0_s = fit_x0_idx * ADC_DT_S;
//
//            // Count-based first delay:
//            // xdelay = xwait + xs/2 - (xend - x0)
//            profiler.xdelay_counts =
//                (int32_t)(PROFILE_WAIT_COUNTS
//                        + (PROFILE_REPEAT_COUNTS / 2)
//                        - (PROFILE_INIT_COUNTS - profiler.x0_counts));
//
//            while (profiler.xdelay_counts < 0)
//            {
//                profiler.xdelay_counts += PROFILE_EVENT_COUNTS;
//            }
//
//            profiler.next_scan_enc_target = profiler.t0_enc_count + profiler.xdelay_counts;
//
//            printf("PROFILE_T0_ENC,axis=%u,idx=%lu,t0_enc=%ld,x0=%ld,xdelay=%ld,next=%ld\r\n",
//                   (unsigned int)profiler.axis,
//                   (unsigned long)profiler.t0_idx,
//                   (long)profiler.t0_enc_count,
//                   (long)profiler.x0_counts,
//                   (long)profiler.xdelay_counts,
//                   (long)profiler.next_scan_enc_target);
//
//            profiler.state = PROFILE_WAIT_NEXT_SCAN;
//            break;
//        }
//
////        case PROFILE_FIND_T0:
////        {
////            float t_end_s;
////            float T_event_s;
////
////            // Build initial long scan (scan 0)
////            profile_build_emit_window(0U, profiler.ts_init_s, profiler.omega_rad_s);
////
////            // Find T0 once from initial scan
////            profiler.t0_idx = profile_find_t0_from_profile_buffer();
////
////            if (profiler.t0_idx >= profile_point_count)
////                profiler.t0_idx = 0U;
////
////            // Time where T0 was found inside initial scan
////            profiler.t0_s = profile_t_ms[profiler.t0_idx] / 1000.0f;
////
////            // End of initial scan window
////            t_end_s = profiler.ts_init_s;
////
////            // Whiteboard timing:
////            // t_wait_first = t_wait + ts_repeat/2 - (t_end_init - t0)
////            profiler.t_wait_first_s = profiler.t_wait_s
////                                    + 0.5f * profiler.ts_repeat_s
////                                    - (t_end_s - profiler.t0_s);
////
////            // Repeat/event period
////            T_event_s = profiler.ts_repeat_s + profiler.t_wait_s;
////
////            while (profiler.t_wait_first_s < 0.0f)
////            {
////                profiler.t_wait_first_s += T_event_s;
////            }
////
////            // Learn clear threshold from initial scan
////            profiler.p_min_init_mw = profile_power_mw[0];
////            profiler.p_max_init_mw = profile_power_mw[0];
////
////            for (uint32_t i = 1U; i < profile_point_count; i++)
////            {
////                if (profile_power_mw[i] < profiler.p_min_init_mw)
////                    profiler.p_min_init_mw = profile_power_mw[i];
////                if (profile_power_mw[i] > profiler.p_max_init_mw)
////                    profiler.p_max_init_mw = profile_power_mw[i];
////            }
////
////            profiler.p_clear_thresh_mw = profiler.p_min_init_mw
////                                       + 0.80f * (profiler.p_max_init_mw - profiler.p_min_init_mw);
////
////            printf("PROFILE_T0,axis=%u,idx=%lu,t0_s=%.6f,t0_ms=%.3f,p_mw=%.6f\r\n",
////                   (unsigned int)profiler.axis,
////                   (unsigned long)profiler.t0_idx,
////                   profiler.t0_s,
////                   profiler.t0_s * 1000.0f,
////                   profile_power_mw[profiler.t0_idx]);
////
////            printf("PROFILE_TIMING,axis=%u,tend_init_s=%.6f,ts_init_s=%.6f,ts_repeat_s=%.6f,twait_s=%.6f,twait_first_s=%.6f\r\n",
////                   (unsigned int)profiler.axis,
////                   t_end_s,
////                   profiler.ts_init_s,
////				   profiler.ts_repeat_s,
////                   profiler.t_wait_s,
////                   profiler.t_wait_first_s);
////
////            // First repeated scan uses corrected first wait
////            profiler.wait_ms = (uint32_t)(profiler.t_wait_first_s * 1000.0f +0.5f);
////            profiler.state_start_ms = HAL_GetTick();
////            profiler.state = PROFILE_WAIT_NEXT_SCAN;
////            break;
////        }
//
//			case PROFILE_WAIT_NEXT_SCAN:
//			{
//				int32_t enc_now = enc_read_axis_count(profiler.axis);
//				int32_t err = enc_delta_counts(enc_now, profiler.next_scan_enc_target);
//
//				if (err >= 0)
//				{
//					beam_profile_start_window_counts(PROFILE_REPEAT_COUNTS);
//					profiler.state = PROFILE_CAPTURE_SCAN;
//
//					printf("PROFILE_SCAN_START,axis=%u,scan=%u,enc_now=%ld,target=%ld\r\n",
//						   (unsigned int)profiler.axis,
//						   (unsigned int)(profiler.scan_count + 1U),
//						   (long)enc_now,
//						   (long)profiler.next_scan_enc_target);
//
//					printf("PROFILE_LOCKED,axis=%u,rpm=%.3f,omega=%.6f,duty=%u,delta=%ld\r\n",
//					       (unsigned int)profiler.axis,
//					       profile_locked_rpm,
//					       profile_locked_omega_rad_s,
//					       (profiler.axis == PROFILE_AXIS_M1) ? dc1.duty : dc2.duty,
//					       (long)((profiler.axis == PROFILE_AXIS_M1) ? dc1.delta_count : dc2.delta_count));
//				}
//			}
//			break;
////        case PROFILE_WAIT_NEXT_SCAN:
////            // Wait before next repeated short scan
////            if ((HAL_GetTick() - profiler.state_start_ms) >= profiler.wait_ms)
////            {
////                beam_profile_start_window((uint32_t)(profiler.ts_repeat_s * 1000.0f + 0.5f));
////                profiler.state = PROFILE_CAPTURE_SCAN;
////            }
////            break;
//
//
//			case PROFILE_CAPTURE_SCAN:
//			    beam_profile_sample_position_window();
//
//			    if (beam_profile_window_done_counts())
//			    {
//			        profiler.scan_count++;
//
//			        profile_build_emit_window_from_raw_buffers(profiler.scan_count);
//
//			        printf("PROFILE_SCAN_DONE,axis=%u,n=%u\r\n",
//			               (unsigned int)profiler.axis,
//			               profiler.scan_count);
//			        printf("SCAN_STATS,axis=%u,scan=%u,delta=%ld\r\n",
//			               (unsigned int)profiler.axis,
//			               profiler.scan_count,
//			               (long)((profiler.axis == PROFILE_AXIS_M1) ? dc1.delta_count : dc2.delta_count));
//
//			        if (profiler.scan_count >= NUM_PROFILE_SCANS)
//			        {
//			            if (profiler.axis == PROFILE_AXIS_M1)
//			            {
//			            	profiler.state_start_ms = HAL_GetTick();
//			                profiler.clear_confirm_count = 0U;
//			                profiler.state = PROFILE_CLEAR_BEAM_POWER;
//			            }
//			            else
//			            {
//			                profiler.state = PROFILE_DONE;
//			            }
//			        }
//			        else
//			        {
//			            profiler.next_scan_enc_target += PROFILE_EVENT_COUNTS;
//			            profiler.state = PROFILE_WAIT_NEXT_SCAN;
//			        }
//			    }
//			    break;
//
////        case PROFILE_CAPTURE_SCAN:
////            // Wait until repeated short scan finishes
////            if (beam_profile_window_done())
////            {
////                profiler.scan_count++;
////
////                // Emit repeated scan numbers 1..NUM_PROFILE_SCANS
////                profile_build_emit_window(profiler.scan_count,
////                                          profiler.ts_repeat_s,
////                                          profiler.omega_rad_s);
////                // new
////                if (profiler.scan_count == 1U && profile_point_count > 0U)
////                {
////                    profiler.p_min_init_mw = profile_power_mw[0];
////                    profiler.p_max_init_mw = profile_power_mw[0];
////
////                    for (uint32_t i = 1U; i < profile_point_count; i++)
////                    {
////                        if (profile_power_mw[i] < profiler.p_min_init_mw)
////                            profiler.p_min_init_mw = profile_power_mw[i];
////                        if (profile_power_mw[i] > profiler.p_max_init_mw)
////                            profiler.p_max_init_mw = profile_power_mw[i];
////                    }
////
////                    profiler.p_clear_thresh_mw = profiler.p_min_init_mw
////                                               + PROFILE_CLEAR_THRESH_FRAC *
////                                                 (profiler.p_max_init_mw - profiler.p_min_init_mw);
////
////                    printf("PROFILE_CLEAR_LEARN,axis=%u,pmin=%.4f,pmax=%.4f,pclear=%.4f\r\n",
////                           (unsigned int)profiler.axis,
////                           profiler.p_min_init_mw,
////                           profiler.p_max_init_mw,
////                           profiler.p_clear_thresh_mw);
////                }
////
////                printf("PROFILE_SCAN_DONE,axis=%u,n=%u\r\n",
////                       (unsigned int)profiler.axis,
////                       profiler.scan_count);
////
////                if (profiler.scan_count >= NUM_PROFILE_SCANS)
////                {
////                    if (profiler.axis == PROFILE_AXIS_M1)
////                    {
////                        profiler.state_start_ms = HAL_GetTick();
////                        profiler.clear_confirm_count = 0U;
////                        profiler.state = PROFILE_CLEAR_BEAM_POWER;
////                    }
////                    else
////                    {
////                        profiler.state = PROFILE_DONE;
////                    }
////                }
////                else
////                {
////                    // After first repeated scan, use steady repeat wait
////                    profiler.wait_ms = (uint32_t)(profiler.t_wait_s * 1000.0f + 0.5f);
////                    profiler.state_start_ms = HAL_GetTick();
////                    profiler.state = PROFILE_WAIT_NEXT_SCAN;
////                }
////            }
////            break;
//
//        case PROFILE_CLEAR_BEAM_POWER:
//        {
//            float p_live = latest_p_mw;
//
//            if (p_live > profiler.p_clear_thresh_mw)
//            {
//                if (profiler.clear_confirm_count < 255U)
//                    profiler.clear_confirm_count++;
//            }
//            else
//            {
//                profiler.clear_confirm_count = 0U;
//            }
//
//            if (profiler.clear_confirm_count >= PROFILE_CLEAR_CONFIRM_COUNT)
//            {
//                dc_motors_stop_all();
//
//                profiler.active = 0U;
//                profiler.state = PROFILE_IDLE;
//
//                profiler.axis = PROFILE_AXIS_M2;
//
//                // --- RESET PROFILER STATE FOR NEW AXIS ---
//                profiler.scan_count = 0U;
//
//                profiler.scan_start_enc_count = 0;
//                profiler.scan_end_enc_count = 0;
//                profiler.scan_stop_enc_target = 0;
//
//                profiler.t0_enc_count = 0;
//                profiler.x0_counts = 0;
//                profiler.xdelay_counts = 0;
//                profiler.next_scan_enc_target = 0;
//
//                profiler.t0_idx = 0;
//                profiler.t0_s = 0.0f;
//
//                profile_raw_count = 0U;
//
//                profiler.axis_sequence_done = 0U;
//                profile_locked_rpm = 0.0f;
//                profile_locked_omega_rad_s = 0.0f;
//                profile_rpm_stable_count = 0U;
//
////              // Scheduler starts next axis motor here
//                dc_motor_start_profile_axis(PROFILE_AXIS_M2);
//                last_dc_speed_ms = HAL_GetTick();
//
//                profile_spinup_pending = 1U;
//                profile_start_request_ms = HAL_GetTick();
//
//
//                printf("PROFILE_AXIS_DONE,axis=0\r\n");
//                printf("PROFILE_CLEAR_DONE,axis=0,plive=%.4f,pclear=%.4f,pmin=%.4f,pmax=%.4f\r\n",
//                       p_live,
//                       profiler.p_clear_thresh_mw,
//                       profiler.p_min_init_mw,
//                       profiler.p_max_init_mw);
//            }
//            else if ((HAL_GetTick() - profiler.state_start_ms) >= PROFILE_CLEAR_TIMEOUT_MS)
//            {
//                dc_motors_stop_all();
//
//                profiler.active = 0U;
//                profiler.state = PROFILE_IDLE;
//
//                profiler.axis = PROFILE_AXIS_M2;
//
//                // --- RESET PROFILER STATE FOR NEW AXIS ---
//                profiler.scan_count = 0U;
//
//                profiler.scan_start_enc_count = 0;
//                profiler.scan_end_enc_count = 0;
//                profiler.scan_stop_enc_target = 0;
//
//                profiler.t0_enc_count = 0;
//                profiler.x0_counts = 0;
//                profiler.xdelay_counts = 0;
//                profiler.next_scan_enc_target = 0;
//
//                profiler.t0_idx = 0;
//                profiler.t0_s = 0.0f;
//
//                profile_raw_count = 0U;
//
//                profiler.axis_sequence_done = 0U;
//                profile_locked_rpm = 0.0f;
//                profile_locked_omega_rad_s = 0.0f;
//                profile_rpm_stable_count = 0U;
//
//                // Scheduler starts next axis motor here
//                dc_motor_start_profile_axis(PROFILE_AXIS_M2);
//                last_dc_speed_ms = HAL_GetTick();
//
//                profile_spinup_pending = 1U;
//                profile_start_request_ms = HAL_GetTick();
//
//                printf("PROFILE_AXIS_DONE,axis=0\r\n");
//                printf("PROFILE_CLEAR_TIMEOUT,axis=0,plive=%.4f,pclear=%.4f,pmin=%.4f,pmax=%.4f\r\n",
//                       p_live,
//                       profiler.p_clear_thresh_mw,
//                       profiler.p_min_init_mw,
//                       profiler.p_max_init_mw);
//            }
//        }
//        break;
//
//        case PROFILE_DONE:
//            dc_motors_stop_all();
//
//            profiler.active = 0U;
//            profiler.state = PROFILE_IDLE;
//            profiler.axis_sequence_done = 1U;
//            profile_spinup_pending = 0U;
//            start_profile_flag = 0U;
//            profiler.clear_confirm_count = 0U;
//
//            printf("PROFILE_DONE,axis=1\r\n");
//            break;
//
//        default:
//            break;
//    }
//}
//static void beam_profile_update(void)
//{
//    if (!profiler.active)
//        return;
//
//    switch (profiler.state)
//    {
//        case PROFILE_INITIAL_SCAN:
//            if (beam_profile_window_done())
//            {
//                profiler.state = PROFILE_FIND_T0;
//            }
//            break;
//
//        case PROFILE_FIND_T0:
//            profile_build_emit_window(0, profiler.ts_s, profiler.omega_rad_s);
//
//            profiler.t0_idx = find_t0_index_from_profile();
//
//            if (profiler.t0_idx < profile_point_count)
//                profiler.t0_s = profile_t_ms[profiler.t0_idx] / 1000.0f;
//            else
//                profiler.t0_s = 0.0f;
//
//            printf("PROFILE_T0,idx=%lu,t0_s=%.6f,stage=%u,p_mw=%.4f\r\n",
//                   (unsigned long)profiler.t0_idx,
//                   profiler.t0_s,
//                   (unsigned int)profile_stage_avg[profiler.t0_idx],
//                   profile_power_mw[profiler.t0_idx]);
//
//            if (profiler.t90_s > profiler.t0_s)
//                profiler.wait_ms = (uint32_t)((profiler.t90_s - profiler.t0_s) * 1000.0f);
//            else
//                profiler.wait_ms = 0U;
//
//            profiler.state_start_ms = HAL_GetTick();
//            profiler.state = PROFILE_WAIT_END_90;
//            break;
//
//        case PROFILE_WAIT_END_90:
//            if ((HAL_GetTick() - profiler.state_start_ms) >= profiler.wait_ms)
//            {
//                profiler.wait_ms = (uint32_t)(profiler.t_wait_s * 1000.0f);
//                profiler.state_start_ms = HAL_GetTick();
//                profiler.state = PROFILE_WAIT_NEXT_SCAN;
//            }
//            break;
//
//        case PROFILE_WAIT_NEXT_SCAN:
//            if ((HAL_GetTick() - profiler.state_start_ms) >= profiler.wait_ms)
//            {
//                beam_profile_start_window((uint32_t)(profiler.ts_s * 1000.0f));
//                profiler.state = PROFILE_CAPTURE_SCAN;
//            }
//            break;
//
//        case PROFILE_CAPTURE_SCAN:
//            if (beam_profile_window_done())
//            {
//                profiler.scan_count++;
//
//                profile_build_emit_window(profiler.scan_count,
//                                          profiler.ts_s,
//                                          profiler.omega_rad_s);
//
//                printf("PROFILE_SCAN_DONE,n=%u\r\n", profiler.scan_count);
//
//                if (profiler.scan_count >= NUM_PROFILE_SCANS)
//                {
//                    profiler.state = PROFILE_DONE;
//                }
//                else
//                {
//                    profiler.wait_ms = (uint32_t)(profiler.t_wait_s * 1000.0f);
//                    profiler.state_start_ms = HAL_GetTick();
//                    profiler.state = PROFILE_WAIT_NEXT_SCAN;
//                }
//            }
//            break;
//
//        case PROFILE_DONE:
//            profiler.active = 0U;
//            profiler.state = PROFILE_IDLE;
//            profile_spinup_pending = 0;
//            start_profile_flag = 0;
//            dc_motors_stop_all();
//            printf("PROFILE_DONE\r\n");
//            break;
//
//        default:
//            break;
//    }
//}

static inline uint32_t step3_step_period_us(void)
{
    return STEP3_STEP_PERIOD_US;
}

static float wavelength_nm_from_peak_hz(float f_hz)
{
    if (f_hz <= 0.0f)
        return 0.0f;

    /* lambda = WL_PATH_FACTOR * v / f
       v is in mm/s, so convert mm -> nm by *1e6
    */
    return (WL_PATH_FACTOR * STEP3_SPEED_MM_S * 1.0e6f) / f_hz;
}

// Replace delay_us(step_us) with a chunk-aware wait
static void step_delay_with_fft(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
    	if (fft_chunk_half_ready)
    	{
    	    fft_chunk_half_ready = 0U;

    	    memcpy(fft_last_chunk_snap,
    	           &fft_chunk_buf[0],
    	           FFT_CHUNK_HALF * sizeof(uint16_t));

    	    fft_process_chunk(fft_last_chunk_snap, FFT_CHUNK_HALF);

    	    fft_chunks_processed++;
    	    chunks_this_run++;
    	}

    	if (fft_chunk_full_ready)
    	{
    	    fft_chunk_full_ready = 0U;

    	    memcpy(fft_last_chunk_snap,
    	           &fft_chunk_buf[FFT_CHUNK_HALF],
    	           FFT_CHUNK_HALF * sizeof(uint16_t));

    	    fft_process_chunk(fft_last_chunk_snap, FFT_CHUNK_HALF);

    	    fft_chunks_processed++;
    	    chunks_this_run++;
    	}
    }
}

static float fft_find_peak_hz_and_accumulate(const uint16_t *src, uint32_t n, uint8_t clear_accum)
{
    uint32_t k;
    uint32_t peak_bin = FFT_MIN_BIN;
    float peak_mag = 0.0f;
    float mean = 0.0f;

    if ((src == NULL) || (n != FFT_SAMPLES))
        return 0.0f;

    if (clear_accum)
    {
        for (k = 0U; k < (n / 2U); k++)
            fft_mag_accum[k] = 0.0f;
    }

    for (k = 0U; k < n; k++)
        mean += (float)src[k];
    mean /= (float)n;

    /* Remove DC offset */
    for (k = 0U; k < n; k++)
        fft_time_buf[k] = (float)src[k] - mean;

    arm_rfft_fast_f32(&fft_inst, fft_time_buf, fft_out_buf, 0);

    /* Fast RFFT output layout:
       bin 0   = out[0]
       bin N/2 = out[1]
       bins 1..N/2-1 use pairs [2k], [2k+1]
    */
    for (k = 1U; k < (n / 2U); k++)
    {
        float mag;

        if (k == (n / 2U))
        {
            mag = fabsf(fft_out_buf[1]);
        }
        else
        {
            float re = fft_out_buf[2U * k];
            float im = fft_out_buf[2U * k + 1U];
            mag = sqrtf((re * re) + (im * im));
        }

        fft_mag_accum[k] += mag;

        if ((k >= FFT_MIN_BIN) && (mag > peak_mag))
        {
            peak_mag = mag;
            peak_bin = k;
        }
    }

    return ((float)peak_bin * FFT_ADC_FS_HZ) / (float)n;
}

static float fft_find_avg_peak_hz_from_accum(uint32_t n_avg, uint32_t n_fft)
{
    uint32_t k;
    uint32_t peak_bin = FFT_MIN_BIN;
    float peak_mag = 0.0f;

    if ((n_avg == 0U) || (n_fft == 0U))
        return 0.0f;

    for (k = FFT_MIN_BIN; k < (n_fft / 2U); k++)
    {
        float mag_avg = fft_mag_accum[k] / (float)n_avg;
        if (mag_avg > peak_mag)
        {
            peak_mag = mag_avg;
            peak_bin = k;
        }
    }

    return ((float)peak_bin * FFT_ADC_FS_HZ) / (float)n_fft;
}

static void fft_process_chunk(const uint16_t *src, uint32_t n)
{
    float    mean = 0.0f;
    uint32_t k;

    if (n == 0U || n > FFT_SAMPLES)
        return;

    // DC removal
    for (k = 0U; k < n; k++)
        mean += (float)src[k];
    mean /= (float)n;

    for (k = 0U; k < n; k++)
        fft_time_buf[k] = (float)src[k] - mean;

    // Zero-pad up to FFT_SAMPLES for full frequency resolution
    for (k = n; k < FFT_SAMPLES; k++)
        fft_time_buf[k] = 0.0f;

    // Run FFT
    arm_rfft_fast_f32(&fft_inst, fft_time_buf, fft_out_buf, 0);

    // Accumulate magnitudes into fft_mag_accum
    for (k = FFT_MIN_BIN; k < (FFT_SAMPLES / 2U); k++)
    {
        float re  = fft_out_buf[2U * k];
        float im  = fft_out_buf[2U * k + 1U];
        fft_mag_accum[k] += sqrtf((re * re) + (im * im));
    }

//    fft_chunks_processed++;
}

// DMA WINDOW HELPER FOR WAVELENGTH
static void wl_process_dma_for_window(uint32_t window_id,
                                      uint32_t *chunk_id,
                                      uint32_t *samples_this_window)
{
    uint32_t remaining;
    uint32_t n_send;

    if (chunk_id == NULL || samples_this_window == NULL)
        return;

    if (*samples_this_window >= WL_WINDOW_SAMPLES)
        return;

    if (fft_chunk_half_ready)
    {
        fft_chunk_half_ready = 0U;

        remaining = WL_WINDOW_SAMPLES - *samples_this_window;
        n_send = (remaining >= FFT_CHUNK_HALF) ? FFT_CHUNK_HALF : remaining;

        wl_send_raw_packet(window_id, *chunk_id, &fft_chunk_buf[0], n_send);

        (*chunk_id)++;
        *samples_this_window += n_send;
    }

    if (*samples_this_window >= WL_WINDOW_SAMPLES)
        return;

    if (fft_chunk_full_ready)
    {
        fft_chunk_full_ready = 0U;

        remaining = WL_WINDOW_SAMPLES - *samples_this_window;
        n_send = (remaining >= FFT_CHUNK_HALF) ? FFT_CHUNK_HALF : remaining;

        wl_send_raw_packet(window_id, *chunk_id, &fft_chunk_buf[FFT_CHUNK_HALF], n_send);

        (*chunk_id)++;
        *samples_this_window += n_send;
    }
}


//// OLD Wavelength process
//static void do_wavelength_fft_measurement(void)
//{
//    uint8_t dir = 1U;
//    float pos_mm = 0.0f;
//
//    uint32_t window_id = 0U;
//
//    const uint32_t step_period_us = step3_step_period_us();
//    const uint32_t steps_to_wall_total =
//        (uint32_t)(WL_LIMIT_MM * STEP3_STEPS_PER_MM + 0.5f);
//
//    fft_busy = 1U;
//    fft_abort_flag = 0U;
//
//    printf("WL_RAW_BEGIN,fs_hz=%.1f,window_samples=%lu,window_s=%.6f,"
//           "speed_mm_s=%.3f,limit_mm=%.3f\r\n",
//           FFT_ADC_FS_HZ,
//           (unsigned long)WL_WINDOW_SAMPLES,
//           WL_WINDOW_TIME_S,
//           STEP3_SPEED_MM_S,
//           WL_LIMIT_MM);
//
//    stepper_sleep_wake();
//    HAL_Delay(10);
//
//    for (uint32_t leg = 0U; leg < WL_NUM_RUNS; leg++)
//    {
//        uint32_t steps_taken = 0U;
//
//        drv2_set_dir(dir);
//        HAL_Delay(10);
//
//        printf("WL_LEG_BEGIN,leg=%lu,dir=%u,pos_mm=%.3f\r\n",
//               (unsigned long)(leg + 1U),
//               (unsigned int)dir,
//               pos_mm);
//
//        while (steps_taken < steps_to_wall_total)
//        {
//            float distance_to_wall_mm;
//            float time_to_wall_s;
//
//            if (fft_abort_flag)
//                goto wl_done;
//
//            if (dir)
//                distance_to_wall_mm = WL_LIMIT_MM - pos_mm;
//            else
//                distance_to_wall_mm = pos_mm;
//
//            if (distance_to_wall_mm < 0.0f)
//                distance_to_wall_mm = 0.0f;
//
//            time_to_wall_s = distance_to_wall_mm / STEP3_SPEED_MM_S;
//
//            /*
//             * Do NOT start a new 0.3 s capture if it would run into the wall.
//             */
//            if (time_to_wall_s < WL_WALL_GUARD_S)
//            {
//                printf("WL_NEAR_WALL_SKIP_WINDOW,leg=%lu,dir=%u,pos_mm=%.3f,"
//                       "time_to_wall_s=%.6f\r\n",
//                       (unsigned long)(leg + 1U),
//                       (unsigned int)dir,
//                       pos_mm,
//                       time_to_wall_s);
//                break;
//            }
//
//            /*
//             * Capture one exact 60000-sample window.
//             */
//            uint32_t chunk_id = 0U;
//            uint32_t samples_this_window = 0U;
//
//            fft_chunk_half_ready = 0U;
//            fft_chunk_full_ready = 0U;
//
//            printf("WL_WINDOW_BEGIN,window=%lu,leg=%lu,dir=%u,pos_mm=%.3f,"
//                   "target_samples=%lu\r\n",
//                   (unsigned long)window_id,
//                   (unsigned long)(leg + 1U),
//                   (unsigned int)dir,
//                   pos_mm,
//                   (unsigned long)WL_WINDOW_SAMPLES);
//
//            if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)fft_chunk_buf, FFT_CHUNK_SIZE) != HAL_OK)
//            {
//                printf("WL_ERR,adc_start_failed\r\n");
//                goto wl_done;
//            }
//
//            if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
//            {
//                printf("WL_ERR,tim6_start_failed\r\n");
//                HAL_ADC_Stop_DMA(&hadc1);
//                goto wl_done;
//            }
//
//            uint32_t scan_start_ms = HAL_GetTick();
//
//            while (samples_this_window < WL_WINDOW_SAMPLES)
//            {
//                /*
//                 * Step continuously during the ADC scan window.
//                 * At 5 mm/s and 100 kHz, 60000 samples should take 0.6 s,
//                 * so this should move about 3 mm per scan window.
//                 */
//                step2_pulse();
//                delay_us(step_period_us);
//
//                steps_taken++;
//
//                if (dir)
//                    pos_mm += (1.0f / STEP3_STEPS_PER_MM);
//                else
//                    pos_mm -= (1.0f / STEP3_STEPS_PER_MM);
//
//                if (pos_mm < 0.0f)
//                    pos_mm = 0.0f;
//                if (pos_mm > WL_LIMIT_MM)
//                    pos_mm = WL_LIMIT_MM;
//
//                wl_process_dma_for_window(window_id, &chunk_id, &samples_this_window);
//                CDC_TX_Kick();
//
//                /*
//                 * Safety only: if we somehow reach the wall during a scan,
//                 * stop stepping but continue servicing DMA until 60000 samples.
//                 */
//                if (steps_taken >= steps_to_wall_total)
//                {
//                    uint32_t wait_start_ms = HAL_GetTick();
//
//                    printf("WL_HIT_WALL_DURING_WINDOW,window=%lu,samples=%lu,chunks=%lu,pos_mm=%.3f\r\n",
//                           (unsigned long)window_id,
//                           (unsigned long)samples_this_window,
//                           (unsigned long)chunk_id,
//                           pos_mm);
//
//                    while (samples_this_window < WL_WINDOW_SAMPLES)
//                    {
//                        wl_process_dma_for_window(window_id, &chunk_id, &samples_this_window);
//                        CDC_TX_Kick();
//
//                        if ((HAL_GetTick() - wait_start_ms) > 1500U)
//                        {
//                            printf("WL_SAMPLE_TIMEOUT,window=%lu,samples=%lu,chunks=%lu,pos_mm=%.3f\r\n",
//                                   (unsigned long)window_id,
//                                   (unsigned long)samples_this_window,
//                                   (unsigned long)chunk_id,
//                                   pos_mm);
//                            break;
//                        }
//                    }
//
//                    break;
//                }
//
//                if ((HAL_GetTick() - scan_start_ms) > 1500U)
//                {
//                    printf("WL_WINDOW_TIMEOUT,window=%lu,samples=%lu,chunks=%lu,pos_mm=%.3f\r\n",
//                           (unsigned long)window_id,
//                           (unsigned long)samples_this_window,
//                           (unsigned long)chunk_id,
//                           pos_mm);
//                    break;
//                }
//            }
//
//            HAL_TIM_Base_Stop(&htim6);
//            HAL_ADC_Stop_DMA(&hadc1);
//
//            /*
//             * Catch any final DMA flag after stopping.
//             */
//            wl_process_dma_for_window(window_id, &chunk_id, &samples_this_window);
//
//            printf("WL_WINDOW_END,window=%lu,samples=%lu,chunks=%lu,pos_mm=%.3f\r\n",
//                   (unsigned long)window_id,
//                   (unsigned long)samples_this_window,
//                   (unsigned long)chunk_id,
//                   pos_mm);
//
//            window_id++;
//        }
//
//        /*
//         * Move remaining distance to the wall without ADC sampling.
//         */
//        while (steps_taken < steps_to_wall_total)
//        {
//            step2_pulse();
//            delay_us(step_period_us);
//
//            steps_taken++;
//
//            if (dir)
//                pos_mm += (1.0f / STEP3_STEPS_PER_MM);
//            else
//                pos_mm -= (1.0f / STEP3_STEPS_PER_MM);
//
//            if (pos_mm < 0.0f)
//                pos_mm = 0.0f;
//            if (pos_mm > WL_LIMIT_MM)
//                pos_mm = WL_LIMIT_MM;
//        }
//
//        if (dir)
//            pos_mm = WL_LIMIT_MM;
//        else
//            pos_mm = 0.0f;
//
//        printf("WL_WALL_HIT,leg=%lu,dir=%u,pos_mm=%.3f,total_windows=%lu\r\n",
//               (unsigned long)(leg + 1U),
//               (unsigned int)dir,
//               pos_mm,
//               (unsigned long)window_id);
//
//        /*
//         * Pause here so USB can finish sending queued binary data.
//         */
//        CDC_TX_Kick();
//        HAL_Delay(WL_USB_FLUSH_DELAY_MS);
//
//        dir ^= 1U;
//        drv2_set_dir(dir);
//        HAL_Delay(50);
//
//        printf("WL_DIRECTION_FLIP,leg=%lu,new_dir=%u,pos_mm=%.3f\r\n",
//               (unsigned long)(leg + 1U),
//               (unsigned int)dir,
//               pos_mm);
//
//        HAL_Delay(50);
//    }
//
//wl_done:
//    HAL_TIM_Base_Stop(&htim6);
//    HAL_ADC_Stop_DMA(&hadc1);
//
//    stepper_sleep_disable();
//
//    printf("WL_DONE,total_windows=%lu\r\n", (unsigned long)window_id);
//    CDC_TX_Kick();
//
//    fft_busy = 0U;
//}

// NEW WAVELENGTH: ONE 0.6s scan back and forth 10 times
static void do_wavelength_fft_measurement(void)
{
    uint32_t window_id = 0U;
    const uint32_t step_period_us = step3_step_period_us();

    fft_busy = 1U;
    fft_abort_flag = 0U;

    printf("WL_RAW_BEGIN,fs_hz=%.1f,window_samples=%lu,window_s=%.6f,"
           "speed_mm_s=%.3f,runs=%lu\r\n",
           FFT_ADC_FS_HZ,
           (unsigned long)WL_WINDOW_SAMPLES,
           WL_WINDOW_TIME_S,
           STEP3_SPEED_MM_S,
           (unsigned long)WL_NUM_RUNS);

    stepper_sleep_wake();
    HAL_Delay(10);

    for (uint32_t run = 0U; run < WL_NUM_RUNS; run++)
    {
        uint32_t chunk_id = 0U;
        uint32_t samples_this_window = 0U;
        uint32_t steps_taken = 0U;
        uint8_t dir = (run % 2U) ? 0U : 1U;

        drv2_set_dir(dir);
        HAL_Delay(10);

        fft_chunk_half_ready = 0U;
        fft_chunk_full_ready = 0U;

        printf("WL_WINDOW_BEGIN,window=%lu,run=%lu,dir=%u,target_samples=%lu\r\n",
               (unsigned long)window_id,
               (unsigned long)(run + 1U),
               (unsigned int)dir,
               (unsigned long)WL_WINDOW_SAMPLES);

        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)fft_chunk_buf, FFT_CHUNK_SIZE) != HAL_OK)
        {
            printf("WL_ERR,adc_start_failed\r\n");
            goto wl_done;
        }

        if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        {
            printf("WL_ERR,tim6_start_failed\r\n");
            HAL_ADC_Stop_DMA(&hadc1);
            goto wl_done;
        }

        uint32_t scan_start_ms = HAL_GetTick();

        while (samples_this_window < WL_WINDOW_SAMPLES)
        {
            if (fft_abort_flag)
                goto wl_done;

            step2_pulse();
            delay_us(step_period_us);
            steps_taken++;

            wl_process_dma_for_window(window_id, &chunk_id, &samples_this_window);

            if ((HAL_GetTick() - scan_start_ms) > 1500U)
            {
                printf("WL_WINDOW_TIMEOUT,window=%lu,samples=%lu,chunks=%lu,steps=%lu\r\n",
                       (unsigned long)window_id,
                       (unsigned long)samples_this_window,
                       (unsigned long)chunk_id,
                       (unsigned long)steps_taken);
                break;
            }
        }

        HAL_TIM_Base_Stop(&htim6);
        HAL_ADC_Stop_DMA(&hadc1);

        wl_process_dma_for_window(window_id, &chunk_id, &samples_this_window);
        CDC_TX_Kick();

        printf("WL_WINDOW_END,window=%lu,samples=%lu,chunks=%lu,steps=%lu,dir=%u\r\n",
               (unsigned long)window_id,
               (unsigned long)samples_this_window,
               (unsigned long)chunk_id,
               (unsigned long)steps_taken,
               (unsigned int)dir);

        window_id++;

        HAL_Delay(100);
    }

wl_done:
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);

    stepper_sleep_disable();

    printf("WL_DONE,total_windows=%lu\r\n", (unsigned long)window_id);
    CDC_TX_Kick();

    fft_busy = 0U;
}


// -----------------------------
// UART command parser (USB CDC input)
// Handles:
//   START / STOKES → polarization scan
//   PROFILE → beam profiling
//   ABORT → stop profiling
// -----------------------------
void process_cmd_byte(uint8_t b)
{
    // Ignore carriage return
    if (b == '\r')
        return;

    // End of command
    if (b == '\n')
    {
        cmd_buf[cmd_idx] = '\0';

        // Start Stokes scan
        if ((strcmp(cmd_buf, "START") == 0) || (strcmp(cmd_buf, STOKES_CMD_STR) == 0))
        {
            start_stokes_flag = 1;
        }

        else if((strcmp(cmd_buf, "HOME") == 0) || (strcmp(cmd_buf, HOME_STEPPER) == 0))
        {
        	z_home_start_flag = 1;
        }

        else if (strcmp(cmd_buf, DIVERGENCE_CMD_STR) == 0)
        {
            printf("DIVERGENCE CMD RECEIVED\r\n");
            start_divergence_flag = 1U;
        }

        else if (strcmp(cmd_buf, M2_CMD_STR) == 0)
        {
            printf("M2 CMD RECEIVED\r\n");
            start_m2_flag = 1U;
        }

        else if (strncmp(cmd_buf, M2_POS_CMD_PREFIX, strlen(M2_POS_CMD_PREFIX)) == 0)
        {
            float target_mm = 0.0f;
            unsigned long step_index = 0UL;
            int parsed = sscanf(cmd_buf, "M2POS,%f,%lu", &target_mm, &step_index);

            if (parsed >= 1)
            {
                m2_target_z_mm = target_mm;
                m2_target_step_index = (parsed >= 2) ? (uint32_t)step_index : 0U;
                printf("M2POS CMD RECEIVED,target_z_mm=%.3f,step=%lu\r\n",
                       m2_target_z_mm,
                       (unsigned long)m2_target_step_index);
                start_m2_position_flag = 1U;
            }
            else
            {
                printf("M2POS BAD CMD\r\n");
            }
        }

        // Start beam profiling
        else if (strcmp(cmd_buf, PROFILE_CMD_STR) == 0)
        {
            printf("PROFILE CMD RECEIVED\r\n");
            start_profile_flag = 1;
        }

        // START DIVERGENCE
        else if (strcmp(cmd_buf, STEP_DIV_CMD_STR) == 0)
        {
            printf("STEPDIV CMD RECEIVED\r\n");
            start_step_div_flag = 1U;
        }

        // Start wavelength
        else if (strcmp(cmd_buf, WAVELENGTH_CMD_STR) == 0)
        {
        	printf("WAVELENGTH CMD RECEIVED\r\n");
            start_fft_flag = 1U;
        }

        else if(strcmp(cmd_buf, T0_M1_CMD_STR) == 0)
        {
        	printf("T0 CMD RECEIVED, axis=0\r\n");
        	start_t0_m1_flag = 1;
        }

        else if(strcmp(cmd_buf, T0_M2_CMD_STR) == 0)
        {
        	printf("T0 CMD RECEIVED, axis=1\r\n");
        	start_t0_m2_flag = 1;
        }

        else if (strcmp(cmd_buf, STEP_PROFILE_CMD_STR) == 0)
        {
            printf("STEP_PROFILE CMD RECEIVED\r\n");
            start_step_profile_flag = 1U;
        }

        else if (strncmp(cmd_buf, "START_AXIS0", 11) == 0)
        {
            host_cmd_start_axis0 = 1U;
        }
        else if (strncmp(cmd_buf, "START_AXIS1", 11) == 0)
        {
            host_cmd_start_axis1 = 1U;
        }
        else if (strncmp(cmd_buf, "RUN_REPEAT,", 11) == 0)
        {
            // Format:
            // RUN_REPEAT,<first_wait_counts>,<num_scans>,<scan_width>,<wait_counts>
            int fw = 0, ns = 0, sw = 0, ww = 0;
            if (sscanf(cmd_buf, "RUN_REPEAT,%d,%d,%d,%d", &fw, &ns, &sw, &ww) == 4)
            {
                (void)sw;
                (void)ww;

                if (!host_local_t0_ready)
                    host_first_wait_counts = fw;

                host_repeat_num_scans = (ns > 0) ? (uint32_t)ns : 10U;
                if ((host_profile_state != HOST_IDLE) || (host_repeat_scan_index > 0U) || (host_repeat_buf_valid_scans > 0U))
                {
                    printf("RUN_REPEAT_IGNORED,axis=%u,state=%u,idx=%lu,buf_scans=%u\r\n",
                           (unsigned int)profiler.axis,
                           (unsigned int)host_profile_state,
                           (unsigned long)host_repeat_scan_index,
                           (unsigned int)host_repeat_buf_valid_scans);
                }
                else
                {
                    if (host_repeat_num_scans > PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS)
                        host_repeat_num_scans = PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS;
                    host_repeat_scan_width_counts = PROFILE_REPEAT_COUNTS;
                    host_repeat_wait_counts = PROFILE_WAIT_COUNTS;
                    host_cmd_run_repeat_seq = 1U;
                }
            }
        }
        else if (strncmp(cmd_buf, "ABORT", 5) == 0)
        {
            host_cmd_abort_profile = 1U;
        }

        // Reset parser buffer
        cmd_idx = 0;
        cmd_buf[0] = '\0';
        return;
    }

    // Store incoming character
    if (cmd_idx < (sizeof(cmd_buf) - 1U))
    {
        cmd_buf[cmd_idx++] = (char)b;
        cmd_buf[cmd_idx] = '\0';
    }
    else
    {
        // Overflow protection
        cmd_idx = 0;
        cmd_buf[0] = '\0';
    }
}

static void clear_beam_on_startup(ProfileAxis_t axis)
{
    uint32_t clear_start_ms = HAL_GetTick();
    uint8_t clear_count = 0U;
    DCMotor_t *active_motor;

    printf("STARTUP_CLEAR_BEGIN,axis=%u\r\n", (unsigned int)axis);

    profiler.axis = axis;
    active_motor = (axis == PROFILE_AXIS_M1) ? &dc1 : &dc2;

    // Start only this axis motor
    dc_motor_start_profile_axis(axis);
    HAL_Delay(PROFILE_SPINUP_MS);

    last_dc_speed_ms = HAL_GetTick();

    while (1)
    {
        live_power_update_service();

        uint32_t now = HAL_GetTick();

        // keep RPM control active
        if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
        {
            float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;

            dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
            dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

            dc_motor_auto_correct_counts(active_motor);

            last_dc_speed_ms = now;
        }

        // LOW power = beam not blocked
        if (latest_p_mw >= 0.2f)   // <-- adjust if needed
        {
            if (clear_count < 50U)
                clear_count++;
        }
        else
        {
            clear_count = 0U;
        }

        if (clear_count >= 50U)
        {
            dc_motor_set_duty(active_motor, 0);

            printf("STARTUP_CLEAR_DONE,axis=%u,plive=%.4f\r\n",
                   (unsigned int)axis,
                   latest_p_mw);
            return;
        }

        if ((HAL_GetTick() - clear_start_ms) >= 5000U)
        {
            dc_motor_set_duty(active_motor, 0);

            printf("STARTUP_CLEAR_TIMEOUT,axis=%u,plive=%.4f\r\n",
                   (unsigned int)axis,
                   latest_p_mw);
            return;
        }
    }
}

static void clear_beam_on_startup_both(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t phase_start_ms;
    uint32_t now;
    uint8_t clear_count = 0U;
    ProfileAxis_t axis = PROFILE_AXIS_M1;
    DCMotor_t *active_motor;

    printf("STARTUP_CLEAR_BOTH_BEGIN\r\n");

    dc_motors_stop_all();
    HAL_Delay(50);

    while ((HAL_GetTick() - start_ms) < 8000U)
    {
        axis = (axis == PROFILE_AXIS_M1) ? PROFILE_AXIS_M2 : PROFILE_AXIS_M1;
        active_motor = (axis == PROFILE_AXIS_M1) ? &dc1 : &dc2;

        printf("STARTUP_CLEAR_TRY,axis=%u\r\n", (unsigned int)axis);

        dc_motor_start_profile_axis(axis);
        HAL_Delay(PROFILE_SPINUP_MS);

        phase_start_ms = HAL_GetTick();
        last_dc_speed_ms = HAL_GetTick();
        clear_count = 0U;

        while ((HAL_GetTick() - phase_start_ms) < 1500U)
        {
            now = HAL_GetTick();

            live_power_update_service();

            if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
            {
                float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;

                dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
                dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);

                dc_motor_auto_correct_counts(active_motor);

                last_dc_speed_ms = now;
            }

            // HIGH power = beam path is clear, so neither motor is blocking
            if (latest_p_mw >= 0.2f)
            {
                if (clear_count < 50U)
                    clear_count++;
            }
            else
            {
                clear_count = 0U;
            }

            if (clear_count >= 50U)
            {
                dc_motors_stop_all();
                printf("STARTUP_CLEAR_BOTH_DONE,plive=%.4f\r\n", latest_p_mw);
                return;
            }
        }

        dc_motor_set_duty(active_motor, 0);
        HAL_Delay(50);
    }

    dc_motors_stop_all();
    printf("STARTUP_CLEAR_BOTH_TIMEOUT,plive=%.4f\r\n", latest_p_mw);
}

static void clear_beam_both_motors_startup(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint8_t cleared = 0U;

    printf("STARTUP_CLEAR_BOTH_BEGIN\r\n");

    dc_start_continuous();

    while ((HAL_GetTick() - start_ms) < PROFILE_CLEAR_TIMEOUT_MS)
    {
        live_power_update_service();

        float p = latest_p_mw;

        printf("STARTUP_CLEAR_PWR,%.4f\r\n", p);

        // LOW POWER = beam blocked / blade in path
        if (p < PROFILE_CLEAR_THRESHOLD_MW)
        {
            cleared = 1U;
            printf("STARTUP_CLEAR_SUCCESS,p=%.4f\r\n", p);
            break;
        }

        HAL_Delay(5);
    }

    if (!cleared)
    {
        printf("STARTUP_CLEAR_TIMEOUT\r\n");
    }

    dc_motors_stop_all();

    printf("STARTUP_CLEAR_BOTH_DONE\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // enable DWT for microsecond delay
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USB_Device_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);

  if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  gain_init_stage1();
  dc_motors_init();

  HAL_Delay(2);
//  z_set_microstep(2);

  last_dc_speed_ms = HAL_GetTick();

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_Delay(200);

  printf("ADC_STREAM_READY\r\n");
  printf("STOKES_READY\r\n");
  printf("PROFILE_READY\r\n");
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(z_home_start_flag)
//	  {
//		  z_home_start_flag = 0; //clear flag
//		  z_home_simple(Z_HOME_DIR);
//		  printf("ZHOME_DONE\r\n");
//
//		  stepper_sleep_wake();
//		  HAL_Delay(5);
//		  z_set_dir(Z_AWAY_DIR);
//		  HAL_Delay(5);
//		  z_move_steps(30000);
//		  stepper_sleep_disable();
//	  }


//	  stepper_sleep_wake();
//	  drv_set_dir(1);
//	  move_steps(5000);
//	  drv_set_dir(0);
//	  move_steps(5000);
//	  stepper_sleep_disable();

//	  steppers_run_continuous();


      live_power_update_service();

//      stepper_sleep_wake();
//      drv_set_dir(1);
//      HAL_Delay(5);
//      move_steps(5000);
//      HAL_Delay(5);
//      drv_set_dir(0);
//      HAL_Delay(5);
//      move_steps(5000);
//      HAL_Delay(5);
//      z_stepper_disable();

      // ✅ ONE-TIME startup beam clear
//      if (!startup_clear_done)
//      {
//          clear_beam_on_startup_both();
//          startup_clear_done = 1U;
//      }

//      if (!startup_clear_done)
//      {
//          clear_beam_on_startup(PROFILE_AXIS_M1);
//          HAL_Delay(100);
//
//          clear_beam_on_startup(PROFILE_AXIS_M2);
//
//          startup_clear_done = 1U;
//      }

      if (!startup_clear_done)
      {
          clear_beam_both_motors_startup();
          startup_clear_done = 1U;
      }

      if (z_home_start_flag && !stokes_busy && !fft_busy &&
          !divergence_busy && !profiler.active && !profile_spinup_pending)
      {
          z_home_start_flag = 0U;
          z_home_simple(Z_HOME_DIR);
          printf("ZHOME_DONE,z_mm=%.3f\r\n", z_current_mm);
          continue;
      }

      if (start_divergence_flag && !stokes_busy && !t0_batch_active &&
          !fft_busy && !divergence_busy && !profiler.active && !profile_spinup_pending)
      {
          start_divergence_flag = 0U;
          run_divergence_measurement(DIVERGENCE_STEP_MM, DIVERGENCE_TOTAL_MM);
          continue;
      }

//	  // Start Stokes
	  if (start_stokes_flag && !stokes_busy & !profiler.active && !profile_spinup_pending && !divergence_busy)
	  {
		  start_stokes_flag = 0U;
		  stokes_busy = 1U;

		  printf("STOKES_BEGIN\r\n");
		  do_stokes_scan();
		  printf("STOKES_END\r\n");

		  stokes_busy = 0U;
		  continue;
	  }

      if (start_m2_flag && !stokes_busy && !t0_batch_active &&
          !fft_busy && !divergence_busy && !profiler.active && !profile_spinup_pending)
      {
          start_m2_flag = 0U;
          run_m2_measurement(M2_STEP_MM, M2_TOTAL_MM);
          continue;
      }

      if (start_m2_position_flag && !stokes_busy && !t0_batch_active &&
          !fft_busy && !divergence_busy && !profiler.active && !profile_spinup_pending)
      {
          uint32_t step_index = m2_target_step_index;
          float target_mm = m2_target_z_mm;

          start_m2_position_flag = 0U;
          if (step_index == 0U)
              step_index = 1U;

          run_m2_capture_at_position(target_mm, step_index);
          continue;
      }

	    if (start_fft_flag && !fft_busy && !stokes_busy && !profiler.active && !divergence_busy)
	    {
	        start_fft_flag = 0U;
	        do_wavelength_fft_measurement();
	    }

//	  if (start_t0_m1_flag && !stokes_busy && !t0_batch_active)
//	  {
//		  start_t0_m1_flag = 0U;
//		  t0_batch_total_axes = 2U;
//		  t0_batch_begin(PROFILE_AXIS_M1, 10U);
//	  }
//
//	  if (start_t0_m2_flag && !stokes_busy && !t0_batch_active)
//	  {
//		  start_t0_m2_flag = 0U;
//		  t0_batch_begin(PROFILE_AXIS_M2, 10U);
//		  t0_batch_total_axes = 2U;
//	  }

	  // Run the non-blocking T0 scheduler every loop
//	  t0_batch_update();

//
//	      uint32_t now = HAL_GetTick();
//	      if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	      {
//	          float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
//
//	          dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	          dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	          if (profile_spinup_pending || profiler.active)
//	              dc_motor_auto_correct_active_axis();
//	          else
//	              dc_motors_auto_correct_counts();
//
//	          last_dc_speed_ms = now;
//	      }
//
//
//	  if (host_cmd_start_axis0)
//	  {
//	      host_cmd_start_axis0 = 0U;
//
//	      profiler.axis = PROFILE_AXIS_M1;
//	      host_local_t0_ready = 0U;
//	      profiler.t0_idx = 0U;
//	      profiler.t0_enc_count = 0;
//	      profiler.x0_counts = 0;
//	      profiler.xdelay_counts = 0;
//	      profiler.next_scan_enc_target = 0;
//	      profile_locked_rpm = 0.0f;
//	      profile_locked_omega_rad_s = 0.0f;
//	      profile_rpm_stable_count = 0U;
//	      host_cmd_run_repeat_seq = 0U;
//	      host_first_wait_counts = 0;
//	      host_repeat_num_scans = PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS;
//	      host_repeat_scan_width_counts = PROFILE_REPEAT_COUNTS;
//	      host_repeat_wait_counts = PROFILE_WAIT_COUNTS;
//	      host_repeat_buffer_reset();
//	      host_repeat_scan_index = 0U;
//
//	      dc_motor_start_profile_axis(PROFILE_AXIS_M1);
//	      last_dc_speed_ms = HAL_GetTick();
//	      profile_start_request_ms = HAL_GetTick();
//
//	      printf("AXIS_STARTING,axis=0\r\n");
//	      host_profile_state = HOST_WAIT_STABLE;
//	  }
//
//	  if (host_cmd_start_axis1)
//	  {
//	      host_cmd_start_axis1 = 0U;
//
//	      profiler.axis = PROFILE_AXIS_M2;
//	      host_local_t0_ready = 0U;
//	      profiler.t0_idx = 0U;
//	      profiler.t0_enc_count = 0;
//	      profiler.x0_counts = 0;
//	      profiler.xdelay_counts = 0;
//	      profiler.next_scan_enc_target = 0;
//	      profile_locked_rpm = 0.0f;
//	      profile_locked_omega_rad_s = 0.0f;
//	      profile_rpm_stable_count = 0U;
//	      host_cmd_run_repeat_seq = 0U;
//	      host_first_wait_counts = 0;
//	      host_repeat_num_scans = PROFILE_REPEAT_BUFFER_MAX_REPEAT_SCANS;
//	      host_repeat_scan_width_counts = PROFILE_REPEAT_COUNTS;
//	      host_repeat_wait_counts = PROFILE_WAIT_COUNTS;
//	      host_repeat_buffer_reset();
//	      host_repeat_scan_index = 0U;
//
//	      dc_motor_start_profile_axis(PROFILE_AXIS_M2);
//	      last_dc_speed_ms = HAL_GetTick();
//	      profile_start_request_ms = HAL_GetTick();
//
//	      printf("AXIS_STARTING,axis=1\r\n");
//	      host_profile_state = HOST_WAIT_STABLE;
//	  }
//
//	  if (host_cmd_run_repeat_seq)
//	  {
//	      host_arm_repeat_sequence(1U);
//	  }
//
//	  if (host_cmd_abort_profile)
//	  {
//	      host_cmd_abort_profile = 0U;
//	      dc_motors_stop_all();
//	      host_profile_state = HOST_IDLE;
//	      printf("PROFILE_ABORTED\r\n");
//	  }
//
//	  host_profile_update();
//

//	      if (start_t0_m1_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_t0_m1_flag = 0U;
//	          printf("T0_BEGIN,axis=0\r\n");
//	          profile_find_t0_only(PROFILE_AXIS_M1);
//	          printf("T0_END,axis=0\r\n");
//	          continue;
//	      }
//
//	      if (start_t0_m2_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_t0_m2_flag = 0U;
//	          printf("T0_BEGIN,axis=1\r\n");
//	          profile_find_t0_only(PROFILE_AXIS_M2);
//	          printf("T0_END,axis=1\r\n");
//	          continue;
//	      }
//
//	      if (start_t0raw_m1_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_t0raw_m1_flag = 0U;
//	          printf("T0RAW_BEGIN_CMD,axis=0\r\n");
//	          profile_stream_raw_scan(PROFILE_AXIS_M1);
//	          printf("T0RAW_END_CMD,axis=0\r\n");
//	          continue;
//	      }
//
//	      if (start_t0raw_m2_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_t0raw_m2_flag = 0U;
//	          printf("T0RAW_BEGIN_CMD,axis=1\r\n");
//	          profile_stream_raw_scan(PROFILE_AXIS_M2);
//	          printf("T0RAW_END_CMD,axis=1\r\n");
//	          continue;
//	      }
//
//	      if (start_profile_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_profile_flag = 0U;
//
//	          // choose axis 0 first unless you set it elsewhere
//	          profiler.axis = PROFILE_AXIS_M1;
//	          profile_locked_rpm = 0.0f;
//	          profile_locked_omega_rad_s = 0.0f;
//	          profile_rpm_stable_count = 0U;
//
//	          dc_motor_start_profile_axis(profiler.axis);
//	          profile_spinup_pending = 1U;
//	          profile_start_request_ms = HAL_GetTick();
//
//	          printf("PROFILE_SPINUP_BEGIN,axis=%u\r\n", (unsigned int)profiler.axis);
//	          continue;
//	      }
//
//	      if (profile_spinup_pending)
//	      {
//	          uint32_t now = HAL_GetTick();
//
//	          if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	          {
//	              float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
//
//	              dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	              dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	              dc_motor_auto_correct_active_axis();
//
//	              last_dc_speed_ms = now;
//
//	              if (dc_active_axis_rpm_is_stable())
//	              {
//	                  profile_locked_rpm = dc_get_active_rpm();
//	                  profile_locked_omega_rad_s = beam_profile_get_active_omega();
//
//	                  printf("PROFILE_LOCKED,axis=%u,rpm=%.3f,omega=%.6f,duty=%u,delta=%ld\r\n",
//	                         (unsigned int)profiler.axis,
//	                         profile_locked_rpm,
//	                         profile_locked_omega_rad_s,
//	                         (profiler.axis == PROFILE_AXIS_M1) ? dc1.duty : dc2.duty,
//	                         (long)((profiler.axis == PROFILE_AXIS_M1) ? dc1.delta_count : dc2.delta_count));
//
//	                  profile_spinup_pending = 0U;
//	                  beam_profile_begin();
//	              }
//	          }
//
//	          if ((now - profile_start_request_ms) >= PROFILE_STABILIZE_TIMEOUT_MS)
//	          {
//	              profile_locked_rpm = dc_get_active_rpm();
//	              profile_locked_omega_rad_s = beam_profile_get_active_omega();
//
//	              profile_spinup_pending = 0U;
//	              beam_profile_begin();
//
//	              printf("PROFILE_STABILIZE_TIMEOUT_START,axis=%u,rpm=%.3f,omega=%.6f\r\n",
//	                     (unsigned int)profiler.axis,
//	                     profile_locked_rpm,
//	                     profile_locked_omega_rad_s);
//	          }
//	      }
//
//	      if (profiler.active)
//	      {
//	          beam_profile_update();
//	      }

	  // CONTINUOUS MOTOR RUNNING
//	  live_power_update_service();   // keep ADC + gain switching running
//
//	      uint32_t now = HAL_GetTick();
//
//	      if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	      {
//	          float dt_s = (now - last_dc_speed_ms) / 1000.0f;
//
//	          // Update encoder speeds
//	          dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	          dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	          // 🔥 THIS IS THE KEY PART
//	          dc_motors_auto_correct_counts();
//
//	          last_dc_speed_ms = now;
//	      }
//
//	      // Optional debug print
//	      static uint32_t last_print = 0;
//	      if ((now - last_print) > 500)
//	      {
//	          dc_print_status();
//	          last_print = now;
//	      }

//
//	      // Keep continuous ADC / DMA / gain switching alive
//	      live_power_update_service();
//
//	      // Periodic motor speed update + closed-loop correction
//	      uint32_t now = HAL_GetTick();
//	      if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	      {
//	          float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
//
//	          dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	          dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	          // If profile state machine is actively controlling one axis,
//	          // only correct the active axis. Otherwise correct both.
//	          if (profiler.active)
//	              dc_motor_auto_correct_active_axis();
//	          else
//	              dc_motors_auto_correct_counts();
//
//	          last_dc_speed_ms = now;
//	      }
//
//	      // -----------------------------
//	      // One-shot commands
//	      // -----------------------------
//	      if (start_stokes_flag && !stokes_busy)
//	      {
//	          start_stokes_flag = 0U;
//	          stokes_busy = 1U;
//
//	          printf("STOKES_BEGIN\r\n");
//	          do_stokes_scan();
//	          printf("STOKES_END\r\n");
//
//	          stokes_busy = 0U;
//	          continue;
//	      }
//
//	      if (start_t0_m1_flag)
//	      {
//	          start_t0_m1_flag = 0U;
//	          printf("T0_BEGIN,axis=0\r\n");
//	          profile_find_t0_only(PROFILE_AXIS_M1);
//	          printf("T0_END,axis=0\r\n");
//	          continue;
//	      }
//
//	      if (start_t0_m2_flag)
//	      {
//	          start_t0_m2_flag = 0U;
//	          printf("T0_BEGIN,axis=1\r\n");
//	          profile_find_t0_only(PROFILE_AXIS_M2);
//	          printf("T0_END,axis=1\r\n");
//	          continue;
//	      }
//
//	      if (start_t0raw_m1_flag)
//	      {
//	          start_t0raw_m1_flag = 0U;
//	          printf("T0RAW_BEGIN_CMD,axis=0\r\n");
//	          profile_stream_raw_scan(PROFILE_AXIS_M1);
//	          printf("T0RAW_END_CMD,axis=0\r\n");
//	          continue;
//	      }
//
//	      if (start_t0raw_m2_flag)
//	      {
//	          start_t0raw_m2_flag = 0U;
//	          printf("T0RAW_BEGIN_CMD,axis=1\r\n");
//	          profile_stream_raw_scan(PROFILE_AXIS_M2);
//	          printf("T0RAW_END_CMD,axis=1\r\n");
//	          continue;
//	      }
//
//	      // -----------------------------
//	      // Profile sequence command
//	      // -----------------------------
//	      if (start_profile_flag && !profiler.active && !profile_spinup_pending)
//	      {
//	          start_profile_flag = 0U;
//	          beam_profile_begin();
//	          continue;
//	      }
//
//	      // Keep profile state machine running
//	      if (profiler.active || profile_spinup_pending)
//	      {
//	          beam_profile_update();
//	      }


//	       WORKING FIND T0
//	     keep updating ADC + gain

//	    if (start_t0_m1_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	    {
//	        start_t0_m1_flag = 0U;
//	        profile_find_t0_only(PROFILE_AXIS_M1);
//	    }
//
//	    if (start_t0_m2_flag && !stokes_busy && !profiler.active && !profile_spinup_pending)
//	    {
//	        start_t0_m2_flag = 0U;
//	        profile_find_t0_only(PROFILE_AXIS_M2);
//	    }
//


	  // Continuous gain switching from ADC4 circular DMA stream.
//	  if (!burst_capture_active)
//	  {
//	      if (adc_half_ready)
//	      {
//	          adc_half_ready = 0;
//
//	          float avg_counts = mean_u16(&adc_buf[0], ADC_HALF_LEN);
//	          float v_meas = adc_counts_to_voltage(avg_counts);
//
//	          if (update_gain_stage(v_meas))
//	          {
//	              gain_settle_skip_live = 1U;
//	          }
//	          else if (gain_settle_skip_live > 0U)
//	          {
//	              gain_settle_skip_live--;
//	          }
//	          else
//	          {
//	              latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
//	              latest_v_meas = v_meas;
//	              latest_p_mw = voltage_to_power_mw(v_meas);
//	          }
//	      }
//
//	      if (adc_full_ready)
//	      {
//	          adc_full_ready = 0;
//
//	          float avg_counts = mean_u16(&adc_buf[ADC_HALF_LEN], ADC_HALF_LEN);
//	          float v_meas = adc_counts_to_voltage(avg_counts);
//
//	          if (update_gain_stage(v_meas))
//	          {
//	              gain_settle_skip_live = 1U;
//	          }
//	          else if (gain_settle_skip_live > 0U)
//	          {
//	              gain_settle_skip_live--;
//	          }
//	          else
//	          {
//	              latest_raw_meas = (uint16_t)(avg_counts + 0.5f);
//	              latest_v_meas = v_meas;
//	              latest_p_mw = voltage_to_power_mw(v_meas);
//	          }
//	      }
//	  }
//
//
//	  uint32_t now = HAL_GetTick();
//
//	  // DC motor speed update and existing counts-based auto-correct.
//	  if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	  {
//	      float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
//	      last_dc_speed_ms = now;
//
//	      // Update measured motor speed from encoder counts
//	      dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	      dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	      // Keep existing counts-based auto-correct during spinup and profiling
//	      if (profile_spinup_pending || profiler.active)
//	      {
//	          dc_motor_auto_correct_active_axis();
//	      }
//
//	      //dc_print_status();
//	  }
//
//	  // Start motors only if a new PROFILE command arrived
//	  if (start_profile_flag && !profiler.active && !profile_spinup_pending)
//	  {
//	      start_profile_flag = 0;
//
//	      profiler.axis = PROFILE_AXIS_M1;
//	      profiler.axis_sequence_done = 0U;
//
//	      dc_motor_start_profile_axis(PROFILE_AXIS_M1);
//	      profile_start_request_ms = HAL_GetTick();
//	      profile_spinup_pending = 1U;
//	  }
//
//	  // After spin-up delay, measure speed and begin profiling
//	  if (profile_spinup_pending)
//	  {
//	      if ((HAL_GetTick() - profile_start_request_ms) >= PROFILE_SPINUP_MS)
//	      {
//	          uint8_t locked = dc_wait_for_active_axis_stable_rpm(PROFILE_STABILIZE_TIMEOUT_MS);
//
//	          printf("RPM_LOCK,axis=%u,ok=%u,rpm=%.4f,omega=%.6f\r\n",
//	                 (unsigned int)profiler.axis,
//	                 (unsigned int)locked,
//	                 profile_locked_rpm,
//	                 profile_locked_omega_rad_s);
//
//	          profile_spinup_pending = 0U;
//	          beam_profile_begin();
//	      }
//	  }
//
//	  if (start_stokes_flag && !stokes_busy)
//	  {
//	      start_stokes_flag = 0;
//	      stokes_busy = 1;
//
//	      do_stokes_scan();
//
//	      stokes_busy = 0;
//
//	      if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
//	      {
//	          Error_Handler();
//	      }
//
//	      if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
//	      {
//	          Error_Handler();
//	      }
//	  }
//
//	  beam_profile_update();

	    // ----------------------------------------
	    // Existing code you likely already have:
	    // - update DC motor speed every 100 ms
	    // - auto-correct motor duty
	    // - print motor status
	    // - run Stokes scan if start flag set
	    // - run beam profile state machine
	    // ----------------------------------------

//	   uint32_t now = HAL_GetTick();
//
//	    if ((now - last_dc_speed_ms) >= DC_SPEED_SAMPLE_MS)
//	    {
//	        float dt_s = (float)(now - last_dc_speed_ms) / 1000.0f;
//	        last_dc_speed_ms = now;
//
//	        dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	        dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//
//	        if (profiler.active)
//	        {
//	            dc_motors_auto_correct_counts();
//	        }
//
//	        //dc_print_status();
//	    }
//
//	    // Start motors only if a new PROFILE command arrived
//	    if (start_profile_flag && !profiler.active && !profile_spinup_pending)
//	    {
//	        start_profile_flag = 0;
//
//	        dc_motors_start_profile_mode();
//	        profile_start_request_ms = HAL_GetTick();
//	        profile_spinup_pending = 1;
//	    }
//
//	    // After spin-up delay, measure speed and begin profiling
//	    if (profile_spinup_pending)
//	    {
//	        if ((HAL_GetTick() - profile_start_request_ms) >= 300U)
//	        {
//	            uint32_t now2 = HAL_GetTick();
//	            float dt_s = (float)(now2 - last_dc_speed_ms) / 1000.0f;
//
//	            if (dt_s > 0.0f)
//	            {
//	                dc_update_speed(&dc1, ENC1_COUNTS_PER_REV, dt_s);
//	                dc_update_speed(&dc2, ENC2_COUNTS_PER_REV, dt_s);
//	                last_dc_speed_ms = now2;
//	            }
//
//	            profile_spinup_pending = 0;
//	            beam_profile_begin();
//	        }
//	    }
//
//	    if (start_stokes_flag && !stokes_busy)
//	    {
//	        start_stokes_flag = 0;
//	        stokes_busy = 1;
//
//	        do_stokes_scan();
//
//	        stokes_busy = 0;
//
//	        if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
//	        {
//	            Error_Handler();
//	        }
//
//	        if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
//	        {
//	            Error_Handler();
//	        }
//	    }
//
//	    beam_profile_update();
//
//	    static uint32_t last_idle_print_ms = 0;
//	    uint32_t now_idle = HAL_GetTick();
//
//	    if (!stokes_busy && !profiler.active && !profile_spinup_pending)
//	    {
//	        if ((now_idle - last_idle_print_ms) >= 100U)
//	        {
//	            last_idle_print_ms = now_idle;
//
//	            uint32_t wr = adc_dma_write_index();
//	            uint32_t idx = (wr == 0U) ? (ADC_BUF_LEN - 1U) : (wr - 1U);
//
//	            uint16_t raw = adc_buf[idx];
//	            float v = adc_counts_to_voltage((float)raw);
//	            update_gain_stage(v);
//	            float p_mw = voltage_to_power_mw(v);
//
//	            printf("IDLE,ADC=%u,V=%.4f,P=%.4f,STAGE=%u\r\n",
//	                   raw, v, p_mw, (unsigned int)current_gain);
//	        }
//	    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 479;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 95;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC4)
    {
        if (!burst_capture_active)
            adc_half_ready = 1U;
    }
    else if (hadc->Instance == ADC1)
    {
        fft_chunk_half_ready = 1U;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC4)
    {
        if (burst_capture_active)
            adc_dma_done = 1U;
        else
            adc_full_ready = 1U;
    }
    else if (hadc->Instance == ADC1)
    {
        fft_chunk_full_ready = 1U;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

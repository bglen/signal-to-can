/* adc_module.c
 *
 * Implementation of a non-blocking, round-robin ADC sampler for STM32F042.
 * - Single conversion per channel, polled with zero-timeout to avoid blocking.
 * - Each enabled channel is sampled once per "cycle"; cycle period = 1 / sample_rate_hz.
 * - Converts raw counts to pin voltage using vref_mv and ADC resolution.
 * - Applies a per-channel external scaling gain to produce "scaled voltage".
 *
 * Integration:
 *   1) Call ADC_Module_Init(&hadc, 3300, 100);
 *   2) Enable desired channels (ADC_Module_Set_Enable_Mask or ADC_Module_Enable_Channel).
 *   3) In main loop, call ADC_Module_Task() frequently.
 *   4) Read values via getters.
 */

#include "adc_module.h"
#include <string.h>

/* ===== Private state ===== */

static ADC_HandleTypeDef *s_adc = NULL;

static uint8_t  s_enable_mask = 0u;  /* bit i enables channel i */
static uint16_t s_raw[ADC_MODULE_CHANNEL_COUNT];
static float    s_pin_v[ADC_MODULE_CHANNEL_COUNT];
static float    s_scaled_v[ADC_MODULE_CHANNEL_COUNT];
static float    s_scale_gain[ADC_MODULE_CHANNEL_COUNT];

static uint32_t s_vref_mv = 3300u;

static uint16_t s_sample_hz = ADC_MODULE_DEFAULT_RATE_HZ;
static uint32_t s_cycle_interval_ms = 10u; /* computed from s_sample_hz */

static uint32_t s_cycle_start_ms = 0u;
static uint8_t  s_scan_active = 0u;
static uint8_t  s_conv_in_progress = 0u;
static uint8_t  s_active_channel = 0xFFu;

/* Map logical channel 0..7 to HAL channel constants.
 * Adjust if your hardware routes signals differently.
 */
static const uint32_t s_hal_channel_map[ADC_MODULE_CHANNEL_COUNT] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};

/* ===== Helpers ===== */

/**
 * @brief Return max ADC count based on configured resolution.
 */
static uint32_t adc_max_count(void)
{
    if (s_adc == NULL) {
        return 4095u;
    }

    /* Map HAL enum to numeric maximum. */
    switch (s_adc->Init.Resolution) {
    case ADC_RESOLUTION_6B:  return 63u;
    case ADC_RESOLUTION_8B:  return 255u;
    case ADC_RESOLUTION_10B: return 1023u;
    case ADC_RESOLUTION_12B:
    default:                 return 4095u;
    }
}

/**
 * @brief Configure the ADC to convert the given HAL channel in single mode.
 */
static HAL_StatusTypeDef adc_select_channel(uint32_t hal_channel)
{
    ADC_ChannelConfTypeDef s_config;
    memset(&s_config, 0, sizeof(s_config));

    s_config.Channel      = hal_channel;
    s_config.Rank         = ADC_RANK_CHANNEL_NUMBER; /* F0 uses channel number as rank in some HAL versions */
    s_config.SamplingTime = ADC_MODULE_SAMPLETIME;

    /* Some HAL versions require ADC_REGULAR_RANK_1; fallback if available. */
#ifdef ADC_REGULAR_RANK_1
    s_config.Rank = ADC_REGULAR_RANK_1;
#endif

    return HAL_ADC_ConfigChannel(s_adc, &s_config);
}

/**
 * @brief Start conversion for the next enabled channel after 'start_index'.
 *        If start_index is 0xFF, begin from channel 0.
 * @return HAL_OK if a conversion was started, HAL_BUSY if none enabled, or error.
 */
static HAL_StatusTypeDef start_next_enabled_conversion(uint8_t start_index)
{
    if (s_adc == NULL) {
        return HAL_ERROR;
    }

    uint8_t idx = (start_index == 0xFFu) ? 0u : (uint8_t)(start_index + 1u);

    /* Search across all channels once. */
    for (uint8_t n = 0u; n < ADC_MODULE_CHANNEL_COUNT; n++) {
        uint8_t ch = (uint8_t)((idx + n) % ADC_MODULE_CHANNEL_COUNT);
        if ((s_enable_mask & (1u << ch)) == 0u) {
            continue;
        }

        HAL_StatusTypeDef st = adc_select_channel(s_hal_channel_map[ch]);
        if (st != HAL_OK) {
            return st;
        }

        s_active_channel = ch;

        st = HAL_ADC_Start(s_adc);
        if (st == HAL_OK) {
            s_conv_in_progress = 1u;
        }
        return st;
    }

    /* No channel enabled. */
    s_active_channel   = 0xFFu;
    s_conv_in_progress = 0u;
    return HAL_BUSY;
}

/**
 * @brief Finish an in-progress conversion if ready, store results, and advance.
 */
static void service_conversion_state(void)
{
    if (s_conv_in_progress == 0u || s_adc == NULL) {
        return;
    }

    /* Zero-timeout poll to avoid blocking. */
    if (HAL_ADC_PollForConversion(s_adc, 0u) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(s_adc);
        (void)HAL_ADC_Stop(s_adc);  /* Stop single conversion */

        uint8_t ch = s_active_channel;
        if (ch < ADC_MODULE_CHANNEL_COUNT) {
            s_raw[ch] = (uint16_t)(raw & 0xFFFFu);

            /* Convert to pin voltage in volts. */
            float pin_v = ((float)raw * (float)s_vref_mv) / ((float)adc_max_count() * 1000.0f);
            s_pin_v[ch]    = pin_v;
            s_scaled_v[ch] = pin_v * s_scale_gain[ch];
        }

        s_conv_in_progress = 0u;

        /* Start the next enabled channel in this cycle, if any. */
        if (start_next_enabled_conversion(ch) != HAL_OK) {
            /* No more channels or error; end of scan for this cycle. */
            s_scan_active = 0u;
            s_active_channel = 0xFFu;
        }
    }
}

/**
 * @brief Recompute cycle interval (ms) from sample rate.
 */
static void recompute_cycle_interval(void)
{
    uint16_t hz = (s_sample_hz == 0u) ? 1u : s_sample_hz;
    /* Round to nearest integer ms to keep timing stable. */
    s_cycle_interval_ms = (uint32_t)((1000u + (hz / 2u)) / hz);
}

/* ===== Public API ===== */

HAL_StatusTypeDef ADC_Module_Init(ADC_HandleTypeDef *hadc, uint32_t vref_mv, uint16_t sample_hz)
{
    if (hadc == NULL) {
        return HAL_ERROR;
    }

    s_adc = hadc;
    s_vref_mv = (vref_mv == 0u) ? 3300u : vref_mv;

    /* Default all disabled, unity scale, zero readings. */
    s_enable_mask = 0u;
    for (uint8_t i = 0u; i < ADC_MODULE_CHANNEL_COUNT; i++) {
        s_scale_gain[i] = 1.0f;
        s_raw[i] = 0u;
        s_pin_v[i] = 0.0f;
        s_scaled_v[i] = 0.0f;
    }

    /* Optional calibration (supported on F0). */
#ifdef HAL_ADCEx_Calibration_Start
    (void)HAL_ADCEx_Calibration_Start(s_adc);
#endif

    s_sample_hz = (sample_hz == 0u) ? ADC_MODULE_DEFAULT_RATE_HZ : sample_hz;
    recompute_cycle_interval();

    s_cycle_start_ms   = HAL_GetTick();
    s_scan_active      = 0u;
    s_conv_in_progress = 0u;
    s_active_channel   = 0xFFu;

    return HAL_OK;
}

void ADC_Module_Task(void)
{
    if (s_adc == NULL) {
        return;
    }

    /* Always try to progress any in-flight conversion. */
    service_conversion_state();

    uint32_t now = HAL_GetTick();

    /* Start a new cycle when interval elapsed and no scan is active. */
    if (s_scan_active == 0u) {
        if ((now - s_cycle_start_ms) >= s_cycle_interval_ms) {
            s_cycle_start_ms = now;
            s_scan_active = 1u;
            s_conv_in_progress = 0u;
            s_active_channel = 0xFFu;

            /* Begin with the first enabled channel. If none enabled, stay idle. */
            if (start_next_enabled_conversion(0xFFu) != HAL_OK) {
                s_scan_active = 0u; /* nothing to do this cycle */
            }
        }
    }
}

void ADC_Module_Enable_Channel(uint8_t channel_index, uint8_t enable)
{
    if (channel_index >= ADC_MODULE_CHANNEL_COUNT) {
        return;
    }
    if (enable) {
        s_enable_mask |= (uint8_t)(1u << channel_index);
    } else {
        s_enable_mask &= (uint8_t)~(1u << channel_index);
    }
}

void ADC_Module_Set_Enable_Mask(uint8_t enable_mask)
{
    s_enable_mask = enable_mask;
}

uint8_t ADC_Module_Get_Enable_Mask(void)
{
    return s_enable_mask;
}

void ADC_Module_Set_Scale(uint8_t channel_index, float scale_gain)
{
    if (channel_index >= ADC_MODULE_CHANNEL_COUNT) {
        return;
    }
    s_scale_gain[channel_index] = scale_gain;
}

void ADC_Module_Set_Vref_Mv(uint32_t vref_mv)
{
    if (vref_mv == 0u) {
        return;
    }
    s_vref_mv = vref_mv;
}

void ADC_Module_Set_Sample_Rate(uint16_t sample_hz)
{
    if (sample_hz == 0u) {
        sample_hz = 1u;
    }
    if (sample_hz > 2000u) {
        sample_hz = 2000u;
    }
    s_sample_hz = sample_hz;
    recompute_cycle_interval();
}

uint16_t ADC_Module_Get_Sample_Rate(void)
{
    return s_sample_hz;
}

uint16_t ADC_Module_Get_Raw(uint8_t channel_index)
{
    if (channel_index >= ADC_MODULE_CHANNEL_COUNT) {
        return 0u;
    }
    return s_raw[channel_index];
}

float ADC_Module_Get_Pin_Voltage(uint8_t channel_index)
{
    if (channel_index >= ADC_MODULE_CHANNEL_COUNT) {
        return 0.0f;
    }
    return s_pin_v[channel_index];
}

float ADC_Module_Get_Scaled_Voltage(uint8_t channel_index)
{
    if (channel_index >= ADC_MODULE_CHANNEL_COUNT) {
        return 0.0f;
    }
    return s_scaled_v[channel_index];
}

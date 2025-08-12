/* adc_module.h
 *
 *  - Initialize ADC (CubeMX does base config; this module calibrates and prepares runtime state)
 *  - Enable/disable ADC inputs on the fly (channels 0..7)
 *  - Sample all enabled channels at a target sample rate (Hz) in a non-blocking task called from main loop
 *  - Adjust sample rate on the fly
 *  - Convert raw ADC readings to pin voltage, with optional per-channel external scaling (e.g., voltage divider)
 *  - Provide getters for raw, pin-voltage, and scaled-voltage per channel
 *
 * Notes:
 *  - Uses single-conversion, one channel at a time, round-robin across enabled channels.
 *  - Call ADC_Module_Task() frequently from the main loop.
 *  - No use of <stdbool.h>; all flags are uint8_t.
 */

#ifndef ADC_MODULE_H
#define ADC_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* Number of logical channels managed by this module (0..7). */
#define ADC_MODULE_CHANNEL_COUNT 8u

/* Compile-time override for per-conversion sampling time. Choose a valid F0 value. */
#ifndef ADC_MODULE_SAMPLETIME
#define ADC_MODULE_SAMPLETIME ADC_SAMPLETIME_239CYCLES_5
#endif

/* Default sample rate if not set explicitly (Hz). */
#ifndef ADC_MODULE_DEFAULT_RATE_HZ
#define ADC_MODULE_DEFAULT_RATE_HZ 100u
#endif

/* ===== Public API ===== */

/**
 * @brief Initialize the ADC helper module.
 *        Performs optional calibration (if supported by HAL), resets internal buffers,
 *        sets default enable mask (all disabled), default scaling (1.0), and target sample rate.
 *        The ADC base configuration (clock, resolution, alignment) must be done via CubeMX.
 *
 * @param hadc        Pointer to the ADC handle configured by CubeMX.
 * @param vref_mv     Reference voltage in millivolts (e.g., 3300 for 3.3 V).
 * @param sample_hz   Desired per-channel sample frequency in Hz (each enabled channel sampled once per cycle).
 * @return HAL_OK on success, otherwise HAL error code.
 */
HAL_StatusTypeDef ADC_Module_Init(ADC_HandleTypeDef *hadc, uint32_t vref_mv, uint16_t sample_hz);

/**
 * @brief Task function to be called frequently from the main loop.
 *        Non-blocking state machine that schedules conversions and collects results.
 */
void ADC_Module_Task(void);

/**
 * @brief Enable or disable a single logical channel (0..7).
 *
 * @param channel_index  Channel index 0..7.
 * @param enable         0 to disable, nonzero to enable.
 */
void ADC_Module_Enable_Channel(uint8_t channel_index, uint8_t enable);

/**
 * @brief Set the enable bitmask for channels.
 *
 * @param enable_mask Bit i enables channel i when set (i=0..7).
 */
void ADC_Module_Set_Enable_Mask(uint8_t enable_mask);

/**
 * @brief Get the current enable bitmask.
 *
 * @return Bitmask of enabled channels.
 */
uint8_t ADC_Module_Get_Enable_Mask(void);

/**
 * @brief Update the per-channel external scaling factor (e.g., undo a divider).
 *        Scaled voltage is pin_voltage * scale_gain.
 *
 * @param channel_index Channel index 0..7.
 * @param scale_gain    Multiplicative gain (float). Use 1.0f for no scaling.
 */
void ADC_Module_Set_Scale(uint8_t channel_index, float scale_gain);

/**
 * @brief Set the ADC reference voltage in millivolts used for conversion to volts.
 *
 * @param vref_mv Reference voltage in millivolts.
 */
void ADC_Module_Set_Vref_Mv(uint32_t vref_mv);

/**
 * @brief Change the per-channel sample rate on the fly.
 *
 * @param sample_hz New sample rate in Hz (per channel). Clamped to [1..2000].
 */
void ADC_Module_Set_Sample_Rate(uint16_t sample_hz);

/**
 * @brief Get the current per-channel sample rate in Hz.
 */
uint16_t ADC_Module_Get_Sample_Rate(void);

/**
 * @brief Get the latest raw ADC sample for a channel.
 *
 * @param channel_index Channel index 0..7.
 * @return Raw ADC counts (resolution-dependent). Returns 0 if index invalid.
 */
uint16_t ADC_Module_Get_Raw(uint8_t channel_index);

/**
 * @brief Get the latest pin voltage (volts) for a channel (before external scaling).
 *
 * @param channel_index Channel index 0..7.
 * @return Pin voltage in volts. Returns 0.0f if index invalid.
 */
float ADC_Module_Get_Pin_Voltage(uint8_t channel_index);

/**
 * @brief Get the latest scaled voltage (volts) for a channel (after external scaling).
 *
 * @param channel_index Channel index 0..7.
 * @return Scaled voltage in volts. Returns 0.0f if index invalid.
 */
float ADC_Module_Get_Scaled_Voltage(uint8_t channel_index);

#ifdef __cplusplus
}
#endif

#endif /* ADC_MODULE_H */

#ifndef PROCESS_SIGNALS_H
#define PROCESS_SIGNALS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"

/**
 * @file process_signals.h
 * @brief High-level ADC-to-device-voltage processing + CAN publishing.
 *
 * Channel indexing is 0..7 corresponding to ADC_IN0..ADC_IN7.
 * Converted values are stored in millivolts of the DEVICE INPUT (post-divider).
 */

/* Number of channels handled (ADC_IN0..ADC_IN7). */
#define PS_NUM_CHANNELS 8u

/* ADC characteristics (can be overridden before init if needed). */
#ifndef PS_ADC_VREF_V
#define PS_ADC_VREF_V        3.3f     /* ADC reference voltage (V) */
#endif
#ifndef PS_ADC_FULL_SCALE
#define PS_ADC_FULL_SCALE    4095.0f  /* 12-bit */
#endif

/* Default out-of-range thresholds in DEVICE INPUT volts. */
#ifndef PS_DEFAULT_MIN_V
#define PS_DEFAULT_MIN_V     0.5f
#endif
#ifndef PS_DEFAULT_MAX_V
#define PS_DEFAULT_MAX_V     4.5f
#endif

/**
 * @brief Initialize processing state with defaults.
 *
 * - Sets per-channel divider gain/offset to unity (no scaling).
 * - Sets out-of-range thresholds to [0.5 V, 4.5 V] for device input.
 */
void Process_Signals_Init(void);

/**
 * @brief Update internal snapshots:
 *        - Read all raw ADC samples (DMA buffer)
 *        - Compute pin voltages (V_pin)
 *        - Compute device input voltages (V_in = gain * V_pin + offset)
 *        - Compute device input millivolts array
 *        - Update out-of-range bitmask
 *
 * Call this at any rate; it is non-blocking and uses the latest DMA values.
 */
void Process_Signals_Update(void);

/**
 * @brief Get latest raw ADC counts for all channels.
 * @param out_raw Pointer to array of length PS_NUM_CHANNELS.
 */
void Process_Signals_Get_All_Raw(uint16_t *out_raw);

/**
 * @brief Get the last computed device input voltage for a channel (volts).
 * @param ch Channel index 0..7
 * @return Voltage (V). Returns 0.0f if ch is out of range.
 */
float Process_Signals_Get_Input_V(uint8_t ch);

/**
 * @brief Get the last computed device input voltage for a channel (millivolts).
 * @param ch Channel index 0..7
 * @return Millivolts (uint16_t), saturated to 0..65535.
 */
uint16_t Process_Signals_Get_Input_mV(uint8_t ch);

/**
 * @brief Get a pointer to the latest device-input millivolts array.
 * @return Pointer to array[PS_NUM_CHANNELS] of uint16_t mV values.
 */
const uint16_t* Process_Signals_Get_All_Input_mV(void);

/**
 * @brief Returns a bitmask of channels that are out of the configured range.
 *        Bit i corresponds to channel i. 1 means out-of-range.
 */
uint8_t Process_Signals_Get_OutOfRange_Mask(void);

/**
 * @brief Set per-channel out-of-range thresholds (device-input domain, volts).
 * @param ch Channel index 0..7
 * @param v_min Minimum allowed (V)
 * @param v_max Maximum allowed (V)
 */
void Process_Signals_Set_MinMax(uint8_t ch, float v_min, float v_max);

/**
 * @brief Get per-channel thresholds (device-input domain, volts).
 * @param ch Channel index 0..7
 * @param v_min_out Output min (V), optional (can be NULL)
 * @param v_max_out Output max (V), optional
 */
void Process_Signals_Get_MinMax(uint8_t ch, float *v_min_out, float *v_max_out);

/**
 * @brief Configure a simple resistive divider for a channel.
 *        V_in = V_pin * (R_top + R_bottom)/R_bottom
 * @param ch Channel 0..7
 * @param r_top_ohm    Top resistor (ohms)
 * @param r_bottom_ohm Bottom resistor (ohms)
 */
void Process_Signals_Set_Divider(uint8_t ch, float r_top_ohm, float r_bottom_ohm);

/**
 * @brief Directly set the affine calibration for a channel:
 *        V_in = gain * V_pin + offset
 * @param ch Channel 0..7
 * @param gain   Gain applied to pin voltage
 * @param offset Offset added (volts)
 */
void Process_Signals_Set_GainOffset(uint8_t ch, float gain, float offset);

/**
 * @brief Send two CAN frames with converted device-input millivolts.
 *
 * Frame 1: StdID = CAN_Module_Get_Node_Id() + 0x1, DLC=8, mV for ch 0..3
 * Frame 2: StdID = CAN_Module_Get_Node_Id() + 0x2, DLC=8, mV for ch 4..7
 *
 * Each channel encoded big-endian (high byte first).
 *
 * @param timeout_ms  Per-frame TX mailbox timeout.
 * @return HAL status. If either frame fails, returns that error.
 */
HAL_StatusTypeDef Process_Signals_Send_Can(uint32_t timeout_ms);

/**
 * @brief Helper that rate-limits Process_Signals_Send_Can using HAL_GetTick().
 *
 * Call frequently (e.g., in your main loop). When period_ms has elapsed since
 * the last successful send, it pushes the two frames.
 *
 * @param period_ms   Desired period between sends.
 * @param timeout_ms  Per-frame timeout passed to Process_Signals_Send_Can.
 * @return HAL_OK if sent this call, HAL_BUSY if not due yet, or an error from CAN.
 */
HAL_StatusTypeDef Process_Signals_Send_Can_If_Due(uint32_t period_ms, uint32_t timeout_ms);

#endif /* PROCESS_SIGNALS_H */

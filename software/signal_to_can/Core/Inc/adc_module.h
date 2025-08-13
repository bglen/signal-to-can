#ifndef ADC_MODULE_H
#define ADC_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f0xx_hal.h"  // pulls in ADC and RCC HAL

// Number of ADC channels sampled (ADC_IN0 .. ADC_IN7)
#define ADC_MODULE_NUM_CHANNELS 8U

/**
 * Initialize ADC1 for multi-channel continuous conversion with DMA.
 * - Enables HSI14 for ADC
 * - Calibrates ADC
 * - Configures channels 0..7
 * - Starts ADC with DMA in circular mode
 *
 * Pass a pointer to the CubeMX-created ADC handle (usually &hadc or &hadc1).
 *
 * Returns HAL_OK on success.
 */
HAL_StatusTypeDef ADC_Module_Init(ADC_HandleTypeDef *hadc_handle);

/**
 * Get the latest raw ADC value for a channel index [0..7].
 * Returns 0 if index is out of range.
 */
uint16_t ADC_Module_Get_Raw(uint8_t channel_index);

/**
 * Get a pointer to the live DMA-backed buffer [length = ADC_MODULE_NUM_CHANNELS].
 * Buffer values update continuously as DMA writes new conversions.
 */
const volatile uint16_t* ADC_Module_Get_Buffer(void);

/**
 * Stop ADC + DMA (optional helper if you need to halt sampling cleanly).
 */
HAL_StatusTypeDef ADC_Module_Stop(ADC_HandleTypeDef *hadc_handle);

#ifdef __cplusplus
}
#endif

#endif // ADC_MODULE_H

/*
 * adc_module.h
 *
 *  Created on: Aug 6, 2025
 *      Author: bglen
 */

/* === adc_module.h === */
#ifndef ADC_MODULE_H
#define ADC_MODULE_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

// Configuration for each ADC channel
// channel: use ADC_CHANNEL_x macro (e.g. ADC_CHANNEL_0)
// scale: factor to apply to raw ADC reading
typedef struct {
    uint32_t channel;
    float    scale;
} ADC_ChannelConfig_t;

// Initialize ADC module with an array of channel configs
// configs: pointer to ADC_ChannelConfig_t array
// numChannels: number of channels in the array
HAL_StatusTypeDef ADC_Module_Init(ADC_ChannelConfig_t *configs, uint8_t numChannels);

// Poll all configured channels and output scaled values
// outValues: array of floats of length numChannels
HAL_StatusTypeDef ADC_Module_Poll(float *outValues);

#endif // ADC_MODULE_H

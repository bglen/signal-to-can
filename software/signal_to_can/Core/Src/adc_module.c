/*
 * adc_module.c
 *
 *  Created on: Aug 6, 2025
 *      Author: bglen
 */

/* === adc_module.c === */
#include "adc_module.h"
#include "main.h"      // for extern ADC_HandleTypeDef hadc
#include <string.h>

extern ADC_HandleTypeDef hadc;

static ADC_ChannelConfig_t *adcConfigs = NULL;
static uint8_t adcNumChannels = 0;

HAL_StatusTypeDef ADC_Module_Init(ADC_ChannelConfig_t *configs, uint8_t numChannels) {
    adcConfigs = configs;
    adcNumChannels = numChannels;
    // Assume HAL_ADC_Init(&hadc) was called by MX_ADC_Init()
    return HAL_OK;
}

HAL_StatusTypeDef ADC_Module_Poll(float *outValues) {
    HAL_StatusTypeDef ret;
    ADC_ChannelConfTypeDef sConfig;

    for (uint8_t i = 0; i < adcNumChannels; i++) {
        // Configure channel
        memset(&sConfig, 0, sizeof(sConfig));
        sConfig.Channel      = adcConfigs[i].channel;
        sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
        sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

        ret = HAL_ADC_ConfigChannel(&hadc, &sConfig);
        if (ret != HAL_OK) {
            return ret;
        }
        // Start conversion
        ret = HAL_ADC_Start(&hadc);
        if (ret != HAL_OK) {
            return ret;
        }
        // Wait for conversion complete (timeout 10 ms)
        ret = HAL_ADC_PollForConversion(&hadc, 10);
        if (ret != HAL_OK) {
            HAL_ADC_Stop(&hadc);
            return ret;
        }
        uint32_t raw = HAL_ADC_GetValue(&hadc);
        // Scale and store
        outValues[i] = raw * adcConfigs[i].scale;
        // Stop ADC
        HAL_ADC_Stop(&hadc);
    }
    return HAL_OK;
}

#include "adc_module.h"
#include "stm32f0xx_hal_rcc.h"

// Static DMA target buffer (one sample per channel)
static volatile uint16_t s_adc_raw[ADC_MODULE_NUM_CHANNELS] = {0};

HAL_StatusTypeDef ADC_Module_Init(ADC_HandleTypeDef *hadc_handle)
{
    if (hadc_handle == NULL) {
        return HAL_ERROR;
    }

    // Ensure the dedicated ADC clock is on (HSI14)
    __HAL_RCC_HSI14_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI14RDY) == RESET) {
    	// wait until HSI14 is ready
    }

    HAL_Delay(1);

    if (HAL_ADCEx_Calibration_Start(hadc_handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // Continuous mode and scan-forward should be set in Init
    hadc_handle->Init.ContinuousConvMode = ENABLE;
    hadc_handle->Init.ScanConvMode       = ADC_SCAN_DIRECTION_FORWARD;
    hadc_handle->Init.EOCSelection       = ADC_EOC_SEQ_CONV;

    // Re-init to apply changes to the HAL state machine if needed
    if (HAL_ADC_Init(hadc_handle) != HAL_OK) return HAL_ERROR;

    // Enable DMA from ADC and set ADC to circular DMA mode
    hadc_handle->Instance->CFGR1 |= (ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | ADC_CFGR1_OVRMOD);

    // Configure channels ADC_IN0 .. ADC_IN7 into the regular sequence.
    // On STM32F0, "rank equals channel number"; setting Rank to ADC_RANK_CHANNEL_NUMBER
    // adds/removes the channel from the CHSELR. SamplingTime here applies to the group.
    hadc_handle->Instance->CHSELR = 0;
    ADC_ChannelConfTypeDef s_config = {0};
    s_config.Rank = ADC_RANK_CHANNEL_NUMBER;
    s_config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    // Add channels 0..7
    for (uint32_t ch = 0; ch < ADC_MODULE_NUM_CHANNELS; ++ch) {
        s_config.Channel = ADC_CHANNEL_0 + ch; // sequential enum for CH0..CH7
        if (HAL_ADC_ConfigChannel(hadc_handle, &s_config) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    // Start ADC in DMA circular mode
    // Ensure your CubeMX DMA config for ADC is:
    // - Direction: Peripheral-to-Memory
    // - Mode: Circular
    // - Peripheral increment: Disable
    // - Memory increment: Enable
    // - Data alignment: Halfword (16-bit)
    if (HAL_ADC_Start_DMA(hadc_handle, (uint32_t*)s_adc_raw, ADC_MODULE_NUM_CHANNELS) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

uint16_t ADC_Module_Get_Raw(uint8_t channel_index)
{
    if (channel_index >= ADC_MODULE_NUM_CHANNELS) {
        return 0;
    }
    return s_adc_raw[channel_index];
}

const volatile uint16_t* ADC_Module_Get_Buffer(void)
{
    return s_adc_raw;
}

HAL_StatusTypeDef ADC_Module_Stop(ADC_HandleTypeDef *hadc_handle)
{
    if (hadc_handle == NULL) {
        return HAL_ERROR;
    }
    if (HAL_ADC_Stop_DMA(hadc_handle) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

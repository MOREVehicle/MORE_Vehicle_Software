/**
 * @file adc.c
 * @brief Source file for adc functions
 * used to calculate current from shunt resistors
 */
#include "adc.h"

ADC_HandleTypeDef ADC_INTERFACE;

void ADC_init(void) {
    ADC_INTERFACE.Instance = ADC3;
    ADC_INTERFACE.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_INTERFACE.Init.Resolution = ADC_RESOLUTION_12B;
    ADC_INTERFACE.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_INTERFACE.Init.GainCompensation = 0;
    ADC_INTERFACE.Init.ScanConvMode = ADC_SCAN_DISABLE;
    ADC_INTERFACE.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    ADC_INTERFACE.Init.LowPowerAutoWait = DISABLE;
    ADC_INTERFACE.Init.ContinuousConvMode = DISABLE;
    ADC_INTERFACE.Init.NbrOfConversion = 1;
    ADC_INTERFACE.Init.DiscontinuousConvMode = DISABLE;
    ADC_INTERFACE.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    ADC_INTERFACE.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    ADC_INTERFACE.Init.DMAContinuousRequests = DISABLE;
    ADC_INTERFACE.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    ADC_INTERFACE.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&ADC_INTERFACE) != HAL_OK) {
        ADC_error();
    }

    ADC_MultiModeTypeDef multimode; 
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&ADC_INTERFACE, &multimode) != HAL_OK)
    {
        ADC_error();
    }

    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&ADC_INTERFACE, &sConfig) != HAL_OK)
    {
        ADC_error();
    }
}

uint32_t ADC_read(void) {
    uint32_t raw = 0;

    HAL_ADC_Start(&ADC_INTERFACE);
    if (HAL_ADC_PollForConversion(&ADC_INTERFACE, ADC_DELAY) == HAL_OK) { 
        raw = HAL_ADC_GetValue(&ADC_INTERFACE);
    } 
    else {
        ADC_error();
    }
    HAL_ADC_Stop(&ADC_INTERFACE);
    return raw;
}

float ADC_calculateCurrent(uint32_t raw) {
    float current = ((raw - ADC_OFFSET) / ADC_RESOLUTION * ADC_V_REF) / (ADC_A_V * ADC_R_SENSE);
    return (current >= 0) ? current : 0;
}

float ADC_readCurrent() {
    return ADC_calculateCurrent(ADC_read());
}

/**
 * @warning AUTOGEN
 * @brief needs to be adjusted for correct pin used
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if(adcHandle->Instance==ADC3) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
        PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            ADC_error();
        }

        __HAL_RCC_ADC345_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @warning AUTOGEN
 * @brief needs to be adjusted for correct pin used
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
    if(adcHandle->Instance==ADC3) {
        __HAL_RCC_ADC345_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
    }
}

void ADC_error() {
    while(1);
}

#include "adc.h"

ADC_HandleTypeDef hadc1;

void ADC_init(void) {

}

float ADC_readCurrent(void) {
    float current = 0;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc1);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    current = (ADC_V_REF - (raw * ADC_A_V)) / (ADC_A_V * ADC_R_SENSE);

    return current;
}

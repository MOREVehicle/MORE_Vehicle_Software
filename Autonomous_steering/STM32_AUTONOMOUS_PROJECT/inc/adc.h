#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include <stm32g4xx_hal_adc.h>

extern ADC_HandleTypeDef hadc1;

#define ADC_V_REF       3.3
#define ADC_A_V         19.8
#define ADC_R_SENSE     0.0002

/**
* @brief Initializes ADC peripheral
*/
void ADC_init(void);

/**
 * @brief reads adc and returns the current based on the configurations listed upstair
 * @return current with 1 number behind decimal
 */
float ADC_readCurrent(void);

#ifdef __cplusplus
extern "C" }
#endif

#endif

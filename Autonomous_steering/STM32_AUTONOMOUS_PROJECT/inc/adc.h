/**
 * @file adc.h
 * @brief Header file for library definitions
 * used to calculate current from shunt resistor
 */
#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_adc.h>

extern ADC_HandleTypeDef ADC_INTERFACE;

#define ADC_RESOLUTION  4096.0f
#define ADC_OFFSET      235.0f//90.0f
#define ADC_V_REF       3.3f
#define ADC_A_V         70.0f//39.4f
#define ADC_R_SENSE     0.00067f

/**
 * @brief ADC_delay used for polling maximum polling a result
 * delay in ms
 */
#define ADC_DELAY       10

/**
* @brief Initializes ADC peripheral
* @warning it has to be properly initialised these are just defaults
*/
void ADC_init(void);


/**
 * @brief reads adc
 * @return raw adc value
 * @warning CALCULATION IS WRONG
 */
uint32_t ADC_read(void);

/**
 * @brief reads adc and returns the current based on the configurations listed upstair
 * @return current with 1 number behind decimal
 */
float ADC_readCurrent(void);

/**
 * @brief converts raw adc value to current
 * @return current with 1 number behind decimal
 * @warning CALCULATION IS WRONG
 */
float ADC_calculateCurrent(uint32_t raw);

/**
* @brief handles errors of adc (currently just while(1))
*/
void ADC_error();

#ifdef __cplusplus
}
#endif

#endif

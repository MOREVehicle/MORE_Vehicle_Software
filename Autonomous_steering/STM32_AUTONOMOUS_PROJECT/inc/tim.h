/**
 * @file tim.h
 * @brief file sets up safety timer used in the main code
 */

#ifndef TIM_H
#define TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_tim.h>
#include <stdint.h>

/**
 * @brief used TIM interface
 */
extern TIM_HandleTypeDef TIM_TIMER_INTERFACE;

/**
 * @brief global variable used to check if interrupt occured
 */
extern volatile uint8_t TIM_interuptflag;

/**
 * @brief settings for 100ms timer, used for escaping from fault situation, in cause of unresponsive angle sensor
*/
void TIM_init(void);

#ifdef __cplusplus
}
#endif

#endif 

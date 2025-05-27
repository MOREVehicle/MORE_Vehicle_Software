#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"

#define HAL_UART_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

extern UART_HandleTypeDef huart2;

void UART_init(void);

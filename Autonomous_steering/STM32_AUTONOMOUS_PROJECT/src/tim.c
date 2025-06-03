#include "tim.h"

TIM_HandleTypeDef TIM_TIMER_INTERFACE;
volatile uint8_t TIM_interuptflag;

/**
 * @brief settings for 100ms timer, used for escaping from fault situation, in cause of unresponsive angle sensor
*/
void TIM_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_TIMER_INTERFACE.Instance = TIM2;
    TIM_TIMER_INTERFACE.Init.Prescaler = 8499;
    TIM_TIMER_INTERFACE.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_TIMER_INTERFACE.Init.Period = 999;
    TIM_TIMER_INTERFACE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_TIMER_INTERFACE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&TIM_TIMER_INTERFACE);
    HAL_TIM_Base_Start_IT(&TIM_TIMER_INTERFACE);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief setup IRQ for Timer
*/
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&TIM_TIMER_INTERFACE);
}

/**
 * @brief Interrupt for saftety timer which ensures the systems shutsdown after some time, if the angle sensor isn't working anymnore
 * It just sets a flag on, the CAN_error function in main then handles the situationo
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        TIM_interuptflag = 1;
    }
}


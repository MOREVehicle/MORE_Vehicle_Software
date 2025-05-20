/**
 * @file    main.c
 * @brief   main loop of the steering box
 *
 * The main file initialises the modules and then loops wait for an event to occur.
 * 1: The angle sensor send data to CAN 2 
 * 2: The car's main can bus has send data to CAN 1 
 * 3: event 1 hasn't occured for X amount of time
 *
 * Event 1 means data is ready so it gathers all other data and sends it over CAN 1 to the rest of the car.
 * Event 2 means that a command has been send from the car's CAN bus, once this happens this command is processed.
 * Event 3 means that the angle sensor is unresponsive, so it sends a error message (and shut the system down???)
 */
#include "../Inc/drv8703.h"
#include "../Inc/CAN.h"
#include "../Inc/tmp112.h"
#include "../Inc/ADC.h"
#include "../Inc/TIM.h"

#define MESSAGE_BUFFER 64
volatile uint8_t timeout_flag = 0;
TIM_HandleTypeDef TIM_TIMER_INTERFACE;

int main () {
    /**
     * @note Might wanna add initialisation checks.
     */
    ADC_init();
    DRV_init();
    TMP_init();
    TIM_init();
    CAN_init();

    while (1) {
        if (CAN_ANGLE_SENSOR_FLAG) {
            uint8_t CAN_message[MESSAGE_BUFFER] = {0};
            uint8_t DRV_message = 0;

            DRV_read(DRV_FAULT_STATUS_GDF_MASK, &DRV_message);
            DRV_message &= DRV_FAULT_STATUS_GDF_MASK;

            int16_t angle = CAN_getAngle();
            uint16_t angleSpeed = CAN_getAngleSpeed();
            int16_t temperature = TMP_getTemperature(); 
            int16_t current     = ADC_getCurrent();
            
            CAN_format();
            CAN_write(1);

            __HAL_TIM_SET_COUNTER(&TIM_TIMER_INTERFACE, 0);
            CAN_FLAG &= ~(CAN_ANGLE_SENSOR_FLAG);
        }
        if (CAN_MAIN_BUS_FLAG) {
            CAN_read(1);
            CAN_process();
            CAN_FLAG &= ~(CAN_MAIN_BUS_FLAG);
        }
        if (timeout_flag) {
            CAN_error();
            /**
             * @warning I believe this should be determined by some central cpu of the car if so comment the line
            */
            break;
        }
    }

    return 0;
}

/**
 * @brief Interrupt for saftety timer which ensures the systems shutsdown after some time, if the angle sensor isn't working anymnore
 * It just sets a flag on, the CAN_error function in main then handles the situationo
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timeout_flag = 1;
    }
}

/**
 * @brief settings for 100ms timer, used for escaping from fault situation, in cause of unresponsive angle sensor
*/
void TIM_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_TIMER_INTERFACE.Instance = TIM2;
    TIM_TIMER_INTERFACE.Init.Prescaler = 16999;
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
    HAL_TIM_IRQHandler(&htim2);
}

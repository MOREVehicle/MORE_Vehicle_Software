#include "../Inc/drv8703.h"
#include "../Inc/CAN.h"
#include "../Inc/tmp112.h"

#define MESSAGE_BUFFER 64
volatile uint8_t timeout_flag = 0;
TIM_HandleTypeDef TIM_TIMER_INTERFACE;

int main () {
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
            CAN_FLAG &= ~(CAN_ANGLE_SENSOR_FLAG);
        }
        if (CAN_MAIN_BUS_FLAG) {
            CAN_read(1);
            CAN_process();
            CAN_FLAG &= ~(CAN_MAIN_BUS_FLAG);
        }
        if (timeout_flag) {
            CAN_error();
            break;
        }
    }

    return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timeout_flag = 1;
    }
}

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

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

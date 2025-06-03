// #include <stdio.h>
// #include "adc.h"
// #include "tim.h"
// #include "uart.h"
//
// void SystemClock_Config(void);
// void Error_Handler(void);
//
// int main(void) {
//     HAL_Init();
//     SystemClock_Config();
//
//     ADC_init();
//     UART_init();
//     TIM_init();
//
//     while (1) {
//         if (TIM_interuptflag) {
//             TIM_interuptflag = 0;
//             uint8_t str[64] = {0};
//             HAL_UART_Transmit(&huart2, str, sizeof(str), HAL_MAX_DELAY);
//         }
//     }
// }
//
// void Error_Handler() {
//     while (1);
// }

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
#include "drv8703.h"
// #include "../inc/CAN.h"
#include "tmp112.h"
#include "adc.h"
#include "tim.h"

#define MESSAGE_BUFFER 64

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
            int16_t current     = ADC_readCurrent();
            
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
        if (TIM_interuptflag) {
            TIM_interuptflag = 0;
            CAN_error();
            /**
             * @warning I believe this should be determined by some central cpu of the car if so comment the line
            */
            break;
        }
    }

    return 0;
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

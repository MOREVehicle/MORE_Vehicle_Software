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
#include "can.h"
#include "tmp112.h"
#include "adc.h"
#include "tim.h"

#define MESSAGE_BUFFER 64
void systemclock_init(void);

int main () {
    /**
     * @note Might wanna add initialisation checks.
     */
    HAL_Init();
    systemclock_init();
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
            uint16_t angle_speed = CAN_getAngleSpeed();
            float temperature = TMP_getTemperature();  // should probably read the two from the currently active bank instead.
            float current     = ADC_readCurrent();
            
            CAN_format(CAN_message, angle, angle_speed, temperature, current);
            CAN_write(&CAN_MAIN_BUS, CAN_message, CAN_MESSAGE_SIZE);

            __HAL_TIM_SET_COUNTER(&TIM_TIMER_INTERFACE, 0);
            CAN_BUS_FLAG &= ~(CAN_ANGLE_SENSOR_FLAG);
        }
        if (CAN_MAIN_BUS_FLAG) {
            /*
             *@warning random size for data, should be based on DBC
            */
            uint8_t data[8] = {0};
            CAN_read(&CAN_MAIN_BUS, data, sizeof(data));
            /*
             *@warning this is undefined it should receive commands from main bus and process however said commands are currently unkown
            */
            // CAN_process();
            CAN_BUS_FLAG &= ~(CAN_MAIN_BUS_FLAG);
        }
        if (TIM_interuptflag) {
            TIM_interuptflag = 0;
            CAN_error();
            /**
             * @warning I believe the error should be send to a central processor which should decide what to do but for now it sends message and enters hardfault
            */
            break;
        }
    }

    return 0;
}

/**
 * @brief sets up nucleo clock to 170MHZ 
 * @warning currently 64MHZ. change it. if you want to utilize full speed of nucleo
 */
void systemclock_init(void) {
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
        CAN_error();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        CAN_error();
    }
}

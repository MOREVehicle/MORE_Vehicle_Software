/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT 1000
#define NT_MSG_END_LENGTH 3
#define DATA_LENGTH 8
#define UART_TEMP_LENGTH 4
#define UART_MAIN_LENGTH 20
#define BCO_RED 63488
#define BCO_GRAY 50712
#define Throttle_Id 0x003
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
const uint8_t NextionMSG_END[NT_MSG_END_LENGTH] = {0xff,0xff,0xff};
volatile uint8_t rxComplete = 0;
uint8_t TxData[DATA_LENGTH];
uint8_t RxData[DATA_LENGTH];
uint8_t tempUART_Rx[UART_TEMP_LENGTH];
uint8_t mainUART_Rx[UART_MAIN_LENGTH];
uint16_t oldPos = 0, newPos = 0;
const uint32_t DriveSelectID = 0x195;
uint32_t TxMailbox = 0;
const double Pi = 3.14159;

const char dpObj_SpeedGauge[] = "VG";
const char dpObj_SpeedMeter[] = "VM";
const char dpObj_Cruise[] = "sw0";
const char dpObj_FrontAng[] = "FrAng";
const char dpObj_RearAng[] = "RAng";
const char dpObj_Park[] = "b2";
const char dpObj_Reverse[] = "b3";
const char dpObj_Neutral[] = "b4";
const char dpObj_Drive[] = "b5";
const char dpObj_ThrottleL[] = "j0";
const char dpObj_ThrottleR[] = "j1";
const char dpObj_ThrottleErr[] = "Err";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void NEXTION_ChangeVal(const char *objName, uint32_t value){
	char buf[30];
	unsigned int len = sprintf(buf, "%s.val=%u", objName, value);
	HAL_UART_Transmit(&huart3, buf, len,UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, NextionMSG_END, NT_MSG_END_LENGTH,UART_TIMEOUT);
}

void NEXTION_Changefloat(const char *objName, float value){
	char buf[20];
	unsigned int len = sprintf(buf, "%s.val=%d", objName, (int)value);
	HAL_UART_Transmit(&huart3, buf, len,UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, NextionMSG_END, NT_MSG_END_LENGTH,UART_TIMEOUT);
}

void NEXTION_UpdateDriveSelectButton(const char *objName, uint32_t value){
	char buf[20];
	unsigned int len = sprintf(buf, "%s.bco=%u", objName, (uint32_t)value);
	HAL_UART_Transmit(&huart3, buf, len,UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, NextionMSG_END, NT_MSG_END_LENGTH,UART_TIMEOUT);
}

void NEXTION_SendErrorMessage(char *objName, char *string){
	char buffer[50];
	int len = sprintf(buffer, "%s.txt=\"%s\"", objName, string);
	HAL_UART_Transmit(&huart3, buffer, len,UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, NextionMSG_END, NT_MSG_END_LENGTH,UART_TIMEOUT);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	//HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(RxHeader.StdId == 0x00 && RxHeader.IDE == CAN_ID_STD) {
		//HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
		int temp = RxData[0] << 8 | RxData[1];
		if(temp >= 16383) {
			temp-= 65536;
		}
		float value = (float)temp * 0.1274;
		NEXTION_Changefloat(dpObj_FrontAng, value);
	}

	else if(RxHeader.StdId == 0x180 && RxHeader.IDE == CAN_ID_STD) {
		//HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
		int temp = RxData[1] << 8 | RxData[0];
		if(temp >= 16383){
			temp -=65536;
		}
		float value = temp * 0.0637;
		value += 200;
		NEXTION_Changefloat(dpObj_RearAng, value);

	}
	else if(RxHeader.StdId == DriveSelectID && RxHeader.IDE == CAN_ID_STD)
	{
		//Park
		if(RxData[0] == 8)
		{
			NEXTION_UpdateDriveSelectButton(dpObj_Park,BCO_RED);
			NEXTION_UpdateDriveSelectButton(dpObj_Reverse,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Neutral,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Drive,BCO_GRAY);
		}

		//Reverse
		else if(RxData[0] == 4)
		{
			NEXTION_UpdateDriveSelectButton(dpObj_Park,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Reverse,BCO_RED);
			NEXTION_UpdateDriveSelectButton(dpObj_Neutral,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Drive,BCO_GRAY);
		}

		//Neutral
		else if(RxData[0] == 2)
		{
			NEXTION_UpdateDriveSelectButton(dpObj_Park,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Reverse,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Neutral,BCO_RED);
			NEXTION_UpdateDriveSelectButton(dpObj_Drive,BCO_GRAY);
		}

		//Drive
		else if(RxData[0] == 1)
		{
			NEXTION_UpdateDriveSelectButton(dpObj_Park,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Reverse,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Neutral,BCO_GRAY);
			NEXTION_UpdateDriveSelectButton(dpObj_Drive,BCO_RED);
		}
	}
	else if(RxHeader.StdId == 0x01 && RxHeader.IDE == CAN_ID_STD)
	{
		uint16_t WheelSpeedRight = RxData[0] << 8 | RxData[1];
		uint16_t WheelSpeedLeft = RxData[2] << 8 | RxData[3];

		WheelSpeedRight *= 2 * Pi / 3;
		WheelSpeedLeft *= 2 * Pi / 3;


		uint32_t Velocity = (WheelSpeedRight + WheelSpeedLeft) / 2 * 3.6;

		int GaugeVal = Velocity * 3;
		if(GaugeVal > 270)
			GaugeVal = 270;
		NEXTION_ChangeVal(dpObj_SpeedMeter, Velocity);
		NEXTION_ChangeVal(dpObj_SpeedGauge, GaugeVal);
	}
	else if(RxHeader.StdId == Throttle_Id && RxHeader.IDE == CAN_ID_STD)
	{
		uint16_t WheelThrottleLeft = RxData[0] << 8 | RxData[1];
		uint16_t WheelThrottleRight = RxData[2] << 8 | RxData[3];

		WheelThrottleRight = WheelThrottleRight / 10.2375;
		WheelThrottleLeft = WheelThrottleLeft / 10.2375;

		if ((WheelThrottleRight < 9) || (WheelThrottleLeft < 9))
		{
			NEXTION_SendErrorMessage(dpObj_ThrottleErr, "Throttle Open Loop!");
			int ErrFlag = 1;
		}
		else if ((WheelThrottleRight > 98) || (WheelThrottleLeft > 98))
		{
			NEXTION_SendErrorMessage(dpObj_ThrottleErr, "Throttle Short to 3.3V!");
			ErrFlag = 1;
		}
		else
		{
			if(ErrFlag == 0)
			{
				NEXTION_SendErrorMessage(dpObj_ThrottleErr, "");
			}
			uint16_t MappedWheelThrottleLeft = ((WheelThrottleLeft - 11) * (100 - 0)) / (97 - 11) + 0;
			uint16_t MappedWheelThrottleRight = ((WheelThrottleRight - 11) * (100 - 0)) / (97 - 11) + 0;
			NEXTION_ChangeVal(dpObj_ThrottleL, MappedWheelThrottleLeft);
			NEXTION_ChangeVal(dpObj_ThrottleR, MappedWheelThrottleRight);

		}
	}

}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size){

	/* start the DMA again */
	if(huart == &huart3)
	{

		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,tempUART_Rx,UART_TEMP_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

		for(int i = 0; i < size; i++)
		{
			if(tempUART_Rx[i] == 'P')
			{
				//HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
				TxHeader.DLC = 8;
				TxHeader.StdId = DriveSelectID;
				TxHeader.RTR = CAN_RTR_DATA;
				TxData[0] = 8;
				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			}

			if(tempUART_Rx[i] == 'R')
			{
				TxHeader.DLC = 8;
				TxHeader.StdId = DriveSelectID;
				TxHeader.RTR = CAN_RTR_DATA;
				TxData[0] = 4;
				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			}

			if(tempUART_Rx[i] == 'N')
			{
				TxHeader.DLC = 8;
				TxHeader.StdId = DriveSelectID;
				TxHeader.RTR = CAN_RTR_DATA;
				TxData[0] = 2;
				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			}

			if(tempUART_Rx[i] == 'D')
			{
				TxHeader.DLC = 8;
				TxHeader.StdId = DriveSelectID;
				TxHeader.RTR = CAN_RTR_DATA;
				TxData[0] = 1;
				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			}
		}
	}


	else if(huart == &huart1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,mainUART_Rx,UART_MAIN_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		HAL_UART_Transmit(&huart3, mainUART_Rx, UART_MAIN_LENGTH,UART_TIMEOUT);
		HAL_UART_Transmit(&huart3, NextionMSG_END, NT_MSG_END_LENGTH,UART_TIMEOUT);

	}
}







/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN_Init();
	MX_USART3_UART_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,tempUART_Rx,UART_TEMP_LENGTH);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,mainUART_Rx,UART_MAIN_LENGTH);

	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);


	TxHeader.DLC = 8;
	TxHeader.StdId = DriveSelectID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxData[0] = 8;
	HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{


		//	  TxHeader.DLC = 8;
		//	  TxHeader.IDE = CAN_ID_EXT;
		//	  TxHeader.ExtId = 0x19B50100;
		//	  TxHeader.RTR = CAN_RTR_DATA;
		//	  TxData[0] = 80;
		//	  TxData[1] = 81;
		//	  TxData[2] = 82;
		//	  TxData[3] = 83;
		//	  TxData[4] = 84;
		//	  TxData[5] = 85;
		//	  TxData[6] = 86;
		//	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
		//	  HAL_Delay(200);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	// canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 256000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_2_Pin */
	GPIO_InitStruct.Pin = LED_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  ******************************************************************************
  * @file           : CAN.c
  * @brief          : Source file for CAN communication functions
  *                  Provides initialization, transmission, and reception routines
  *                  for FDCAN1 and FDCAN2.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
/**
  * @brief FDCAN flag.
  */
volatile uint8_t CAN_BUS_FLAG = 0;

/**
 * @brief FDCAN receive message header structure.
 *
 * Stores metadata from received FDCAN messages, such as ID, timestamp, and frame info.
 */
FDCAN_RxHeaderTypeDef rxHeader;

/**
 * @brief Buffer used for transmitting or receiving CAN data.
 *
 * This array stores up to 5 bytes of CAN data for communication.
 */
uint8_t CAN_Data[5];

/**
 * @brief Handle for the FDCAN1 peripheral.
 *
 * Used to configure and manage operations on FDCAN1.
 */
FDCAN_HandleTypeDef CAN_ANGLE_BUS;

/**
 * @brief Handle for the FDCAN2 peripheral.
 *
 * Used to configure and manage operations on FDCAN2.
 */
FDCAN_HandleTypeDef CAN_MAIN_BUS;

/**
 * @brief Initializes the FDCAN1 peripheral.
 *
 * Configures FDCAN1 with parameters such as clock, filters, and operational mode.
 * This function is typically called during system initialization.
 */
static void MX_FDCAN1_Init(void);

/**
 * @brief Initializes the FDCAN2 peripheral.
 *
 * Configures FDCAN2 with parameters such as clock, filters, and operational mode.
 * This function is typically called during system initialization.
 */
static void MX_FDCAN2_Init(void);

/**
  * @brief Reads CAN data from a global buffer into the provided data array.
  * @param hfdcan Pointer to the FDCAN handle (e.g., &hfdcan1 or &hfdcan2). (Currently unused in this function.)
  * @param data Pointer to the buffer where the received data will be copied.
  * @param Length Number of bytes to read from the global CAN_Data buffer.
  * @retval None
  * @note This function simply copies bytes from a global buffer `CAN_Data` to the provided `data` buffer.
  *       It does not interact with the FDCAN peripheral directly. Ensure `CAN_Data` is filled elsewhere using
  *       `HAL_FDCAN_GetRxMessage` or another appropriate function.
  */
void CAN_read(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, int Length)
{
  for (int32_t i = 0; i < Length; i++) {
    data[i] = CAN_Data[i];
  }
}

/**
  * @brief Transmits a CAN FD message using the specified FDCAN peripheral.
  * @param hfdcan Pointer to the FDCAN handle (e.g., &hfdcan1 or &hfdcan2).
  * @param data Pointer to the data buffer to be transmitted (up to 32 bytes based on configured DLC).
  * @param device_on_can Standard ID (11-bit) of the target CAN device.
  * @retval None
  * @note This function configures the FDCAN_TxHeaderTypeDef with standard settings for CAN FD,
  *       including 32-byte data length, bit rate switching, and FD frame format. It adds the message
  *       to the transmission FIFO queue. The `CAN_error()` function is called if transmission fails.
  */
void CAN_write(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint16_t device_on_can)
{
  FDCAN_TxHeaderTypeDef txHeader;

  // Configure TX Header for FDCAN1
  txHeader.Identifier = device_on_can;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_32;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_ON;
  txHeader.FDFormat = FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data)!= HAL_OK) {
	  CAN_error();
  }
}

/**
  * @brief  FDCAN1 RX FIFO0 Callback function.
  * @param  hfdcan: Pointer to the FDCAN handle.
  * @param  RxFifo0ITs: Specifies the interrupt sources for FIFO0.
  * @retval None
  * @note   Sets the CAN_MAIN_BUS_FLAG when a new message is received in FIFO0.
  *         Reactivates the FIFO0 notification for further interrupts.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
	  CAN_BUS_FLAG |= CAN_MAIN_BUS_FLAG;
	if (HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &rxHeader, CAN_Data) != HAL_OK) {
		CAN_error();
	}
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      /* Notification Error */
        CAN_error();
    }
  }
}

/**
  * @brief  FDCAN2 RX FIFO1 Callback function.
  * @param  hfdcan: Pointer to the FDCAN handle.
  * @param  RxFifo1ITs: Specifies the interrupt sources for FIFO1.
  * @retval None
  * @note   Sets the CAN_ANGLE_SENSOR_FLAG when a new message is received in FIFO1.
  *         Reactivates the FIFO1 notification for further interrupts.
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
	  CAN_BUS_FLAG |= CAN_ANGLE_SENSOR_FLAG;
	if (HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &rxHeader, CAN_Data) != HAL_OK) {
		CAN_error();
	}
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
      /* Notification Error */
        CAN_error();
    }
  }
}

/**
  * @brief  Sends the steering wheel angle sensor data via CAN using the rxdata.
  * @param  angle_data: The 16-bit steering wheel angle data (-780 to 780).
  * @retval Angle value.
  */
uint16_t CAN_getAngle( ) {

	uint16_t Angle_value;

  // Prepare data
  Angle_value = ((CAN_Data[0] << 8) & 0xFF00) | (CAN_Data[1] & 0x00FF);
  return Angle_value;
}

/**
  * @brief  Sends the steering wheel angle sensor data via CAN using the rxdata.
  * @param  speed_data: The 8-bit steering speed data (0 to 1016).
  * @retval speed.
  */
uint8_t CAN_getAngleSpeed(){
	uint8_t Speed;
	Speed = CAN_Data[2]; // Steering speed
	return Speed;
}
/**
 * @brief Sends a calibration command to the LWS (Lane Watch Sensor).
 *
 * This function sends a command to the sensor via CAN to perform calibration-related actions.
 * The command is encoded in the first byte with only the lower 3 bits used.
 *
 * @param ccw_value The calibration command word (only bits 0–2 are used).
 */
void LWS_Send_Calibration_Command(uint8_t ccw_value) {

    // CCW value in Byte 0 (bits 0–2)
	CAN_Data[0] = ccw_value & 0x07; // Mask to 3 bits
	CAN_Data[1] = 0x00; // Reserved
  CAN_write(&CAN_ANGLE_BUS,CAN_Data,0x7C0);
}
/**
 * @brief Performs a full calibration sequence for the LWS sensor.
 *
 * This function first sends a reset calibration command and then initiates a new calibration.
 * Delays are inserted to allow the sensor time to process each step.
 */
void Calibrate_LWS_Sensor(void) {
    // Step 1: Reset calibration
    LWS_Send_Calibration_Command(0x05);
    HAL_Delay(100); // Wait for sensor to process

    // Step 2: Start new calibration
    LWS_Send_Calibration_Command(0x03);
    HAL_Delay(100); // Wait for sensor to process
}


/**
  * @brief Initializes the FDCAN1 and FDCAN2 peripheral with predefined settings and STart CAN and activate thr notification
  * @retval None
  * @note Configures nominal and data timing parameters and enables reception filters.
  */
void CAN_init(void)
{
   MX_FDCAN1_Init();
   MX_FDCAN2_Init();

  // STart FDCAN1
  if(HAL_FDCAN_Start(&CAN_ANGLE_BUS)!= HAL_OK) {
	  CAN_error();
  }

  // STart FDCAN2
  if(HAL_FDCAN_Start(&CAN_MAIN_BUS)!= HAL_OK) {
	  CAN_error();
  }

  // Activate the notification for new data in FIFO0 for FDCAN1
  if (HAL_FDCAN_ActivateNotification(&CAN_ANGLE_BUS, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    /* Notification Error */
    CAN_error();
  }

  // Activate the notification for new data in FIFO1 for FDCAN2
  if (HAL_FDCAN_ActivateNotification(&CAN_MAIN_BUS, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
    /* Notification Error */
    CAN_error();
  }
}
/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{
CAN_ANGLE_BUS.Instance = FDCAN1;
  CAN_ANGLE_BUS.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  CAN_ANGLE_BUS.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  CAN_ANGLE_BUS.Init.Mode = FDCAN_MODE_NORMAL;
  CAN_ANGLE_BUS.Init.AutoRetransmission = DISABLE;
  CAN_ANGLE_BUS.Init.TransmitPause = DISABLE;
  CAN_ANGLE_BUS.Init.ProtocolException = ENABLE;
  CAN_ANGLE_BUS.Init.NominalPrescaler = 2;
  CAN_ANGLE_BUS.Init.NominalSyncJumpWidth = 3;
  CAN_ANGLE_BUS.Init.NominalTimeSeg1 = 20;
  CAN_ANGLE_BUS.Init.NominalTimeSeg2 = 3;
  CAN_ANGLE_BUS.Init.DataPrescaler = 2;
  CAN_ANGLE_BUS.Init.DataSyncJumpWidth = 3;
  CAN_ANGLE_BUS.Init.DataTimeSeg1 = 20;
  CAN_ANGLE_BUS.Init.DataTimeSeg2 = 3;
  CAN_ANGLE_BUS.Init.StdFiltersNbr = 1;
  CAN_ANGLE_BUS.Init.ExtFiltersNbr = 0;
  CAN_ANGLE_BUS.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&CAN_ANGLE_BUS) != HAL_OK)
  {
    CAN_error();
  }
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x2;
  sFilterConfig.FilterID2 = 0x2;
  //sFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&CAN_ANGLE_BUS, &sFilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    CAN_error();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{
  CAN_MAIN_BUS.Instance = FDCAN2;
  CAN_MAIN_BUS.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  CAN_MAIN_BUS.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  CAN_MAIN_BUS.Init.Mode = FDCAN_MODE_NORMAL;
  CAN_MAIN_BUS.Init.AutoRetransmission = DISABLE;
  CAN_MAIN_BUS.Init.TransmitPause = DISABLE;
  CAN_MAIN_BUS.Init.ProtocolException = ENABLE;
  CAN_MAIN_BUS.Init.NominalPrescaler = 2;
  CAN_MAIN_BUS.Init.NominalSyncJumpWidth = 3;
  CAN_MAIN_BUS.Init.NominalTimeSeg1 = 20;
  CAN_MAIN_BUS.Init.NominalTimeSeg2 = 3;
  CAN_MAIN_BUS.Init.DataPrescaler = 2;
  CAN_MAIN_BUS.Init.DataSyncJumpWidth = 3;
  CAN_MAIN_BUS.Init.DataTimeSeg1 = 20;
  CAN_MAIN_BUS.Init.DataTimeSeg2 = 3;
  CAN_MAIN_BUS.Init.StdFiltersNbr = 1;
  CAN_MAIN_BUS.Init.ExtFiltersNbr = 0;
  CAN_MAIN_BUS.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&CAN_MAIN_BUS) != HAL_OK)
  {
    CAN_error();
  }
  
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x1;
  sFilterConfig.FilterID2 = 0x1;
  if (HAL_FDCAN_ConfigFilter(&CAN_MAIN_BUS, &sFilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    CAN_error();
  }
}

void CAN_format(uint8_t* buffer, int16_t angle, uint16_t angle_speed, float temperature, float current) {
    uint8_t gate_drive_fault = 0;
    uint8_t overtemp_flag = 0;
    uint16_t temp_encoded   = (uint16_t)(temperature / 0.1f);
    uint16_t angle_encoded  = (uint16_t)(angle + 780);
    uint16_t speed_encoded  = angle_speed;
    uint8_t current_encoded = (uint8_t)(current + 80);

    buffer[0] |= (gate_drive_fault & 0x01) << 0;
    buffer[0] |= (overtemp_flag    & 0x01) << 1;
    buffer[0] |= (temp_encoded     & 0x3F) << 2;
    buffer[1]  = (temp_encoded >> 6) & 0x0F;
    buffer[1] |= (angle_encoded & 0x0F) << 4;
    buffer[2]  = (angle_encoded >> 4) & 0xFF;
    buffer[3]  = speed_encoded & 0xFF;
    buffer[4]  = ((speed_encoded >> 8) & 0x03) | (current_encoded << 2);
    buffer[5]  = (current_encoded >> 6) & 0x03;
    buffer[6]  = 0;
    buffer[7]  = 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void CAN_error(void)
{
    __disable_irq();
    uint8_t error_code = 0b1111111;
    CAN_write(&CAN_MAIN_BUS, (uint8_t*)error_code, 0x100);
    while (1) {}
}


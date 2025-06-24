/**
 ******************************************************************************
 * @file           : CAN.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 */
#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_fdcan.h>

/**
 * @defgroup CAN_Exported_Functions CAN Exported Functions
 * @brief    Functions for CAN initialization, transmission, and reception
 * @{
 */

/**
 * @brief  Initializes the CAN peripheral.
 * @param  None
 * @retval None
 */
void CAN_init(void);

/**
 * @brief Performs a full calibration sequence for the LWS sensor.
 *
 * This function first sends a reset calibration command and then initiates a new calibration.
 * Delays are inserted to allow the sensor time to process each step.
 */
void LWS_Send_Calibration_Command(uint8_t ccw_value);
/**
 * @brief Initiates a full calibration sequence for the LWS sensor.
 *
 * Performs a reset of any existing calibration and starts a new calibration cycle.
 */
void Calibrate_LWS_Sensor(void);
/**
  * @brief Reads CAN data from a global buffer into the provided data array.
  * @param hfdcan Pointer to the FDCAN handle (e.g., &hfdcan1 or &hfdcan2). (Currently unused in this function.)
  * @param data Pointer to the buffer where the received data will be copied.
  * @param Length Number of bytes to read from the global CAN_Data buffer.
  * @retval None
  * @note This function assumes `CAN_Data` is populated externally.
  */
void CAN_read(FDCAN_HandleTypeDef *hfdcan,uint8_t *data, int Length);

/**
  * @brief Transmits a CAN FD message using the specified FDCAN peripheral.
  * @param hfdcan Pointer to the FDCAN handle (e.g., &hfdcan1 or &hfdcan2).
  * @param data Pointer to the data buffer to be transmitted (up to 32 bytes).
  * @param device_on_can Standard ID (11-bit) of the target CAN device.
  * @retval None
  * @note Configures and sends a 32-byte CAN FD frame. Calls Error_Handler on failure.
  */
void CAN_write(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint16_t device_on_can);

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void CAN_error(void);
/**
  * @brief  Sends the steering wheel angle sensor data via CAN.
  * @param  angle_data: The 16-bit steering wheel angle data (-780 to 780).
  * @retval Angle value.
  */
uint16_t CAN_getAngle( );
/**
  * @brief  Sends the steering wheel angle speed data via CAN using the rxdata.
  * @param  speed_data: The 8-bit steering speed data (0 to 1016).
  * @retval speed.
  */
uint8_t CAN_getAngleSpeed();

/**
 * @brief formats the data accordinly
 * @param All data in the steering box
 * @warning currently format is hardcoded according to the proposed dbc in the this repository ../../MORE_AUTONOMOUS_CAN.dbc
 */
void CAN_format(uint8_t* buffer, int16_t anlge, uint16_t angle_speed, float temperature, float current);

/**
 * @brief Global CAN bus status flag.
 *
 * Each bit in this flag represents the status of a different CAN-connected device.
 * Use predefined masks to check specific device status.
 */
extern volatile uint8_t CAN_BUS_FLAG;

/**
 * @brief Check if the angle sensor device is active on the CAN bus.
 *
 * Bit 0 of CAN_BUS_FLAG indicates the status of the angle sensor.
 */
#define CAN_ANGLE_SENSOR_FLAG        (CAN_BUS_FLAG & 0b00000001)  // First bit mask
/**
 * @brief Check if the main bus device is active on the CAN bus.
 *
 * Bit 1 of CAN_BUS_FLAG indicates the status of the main CAN device.
 */
#define CAN_MAIN_BUS_FLAG            (CAN_BUS_FLAG & 0b00000010)  // Second bit mask

#define CAN_MESSAGE_SIZE             48

/**
 * @brief Buffer used for storing CAN data to be transmitted.
 *
 * This buffer holds up to 5 bytes of CAN data, typically formatted according to 
 * the requirements of the specific CAN message being sent.
 */
extern uint8_t CAN_Data[5];

/**
 * @brief Handle for the first FDCAN peripheral (FDCAN1).
 *
 * Used to configure and manage communication through the FDCAN1 interface.
 */
extern FDCAN_HandleTypeDef CAN_ANGLE_BUS;

/**
 * @brief Handle for the second FDCAN peripheral (FDCAN2).
 *
 * Used to configure and manage communication through the FDCAN2 interface.
 */
extern FDCAN_HandleTypeDef CAN_MAIN_BUS;


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */

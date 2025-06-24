/**
 * @file tmp112.h
 * @brief Driver for TMP112 Temperature Sensor
 * 
 * This file provides functions to interact with the TMP112 temperature sensor
 * over I2C, including reading temperature values and configuring sensor settings.
 * Everything is MSB to LSB
 */

#ifndef TMP112_H
#define TMP112_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

#define TMP_REG_MAP_SIZE       0x04

/**
 * @def device registers
 * @brief This is all registers on the TMP112. 
 * All of them ar 2-byte.
 * All except TMP_CONFIG_REG are single data field.
 * All execpt TMP_CONFIG_REG are read only.
 */
#define TMP_TEMPERATURE        0x00
#define TMP_CONFIG             0x01
#define TMP_TLOW               0x02
#define TMP_THIGH              0x03

/**
 * @def Config
 * @brief Config register on driver's fields
 * READ-WRITE
 */ 
#define TMP_CONFIG_ONES_SHOT_MASK           (1U << 15) 
#define TMP_CONFIG_CONVERTER_MASK(x)        ((((uint8_t)x) & 0x03) << 13) //read only
#define TMP_CONFIG_FAULT_QUEUE_MASK(x)      ((((uint8_t)x) & 0x03) << 11) 
#define TMP_CONFIG_POLARITY_MASK            (1U << 10) 
#define TMP_CONFIG_THERMOSTAT_MASK          (1U << 9) 
#define TMP_CONFIG_SHUTDOWN_MASK            (1U << 8) 
#define TMP_CONFIG_CONVERSION_RATE_MASK(x)  ((((uint8_t)x) & 0x03) << 6)
#define TMP_CONFIG_ALERT_MASK               (1U << 5)                     //read only
#define TMP_CONFIG_RESOLUTION_MASK          (1U << 4)                     //0 for 12, 1 for 13 bit resolution

/**12 bit resolution, two's complement, left justified*/
#define TMP_RESOLUTION         12           //can be set to 13
#define TMP_MAX_DEGREE         125          //this is max supported; absolute max is 128
#define TMP_MIN_DEGREE         -40          //this is max supported; absolute max is -129
#define TMP_ABSOLUTE_RANGE     256
#define TMP_DEGREE_RESOLUTION  (float)(TMP_ABSOLUTE_RANGE / (float)(1<<TMP_RESOLUTION))

/**
 * @def I2C macros
 * @brief general macros for complying to drivers I2C format when using I2C.
 * message format - 4 bytes:
 * MSB --- LSB 
 * |DEV_ADDR_6|DEV_ADDR_5|DEV_ADDR_4|DEV_ADDR_3|DEV_ADDR_2|DEV_ADDR_1|DEV_ADDR_0|READ/WRITE|
 * |         0|         0|         0|         0|         0|         0|REG_ADDR_1|REG_ADDR_0|
 * |   DATA_15|   DATA_14|   DATA_13|   DATA_12|   DATA_11|   DATA_10|    DATA_9|    DATA_8|
 * |    DATA_7|    DATA_6|    DATA_5|    DATA_4|    DATA_3|    DATA_2|    DATA_1|    DATA_0|
 *
 * DEV_ADDR_6 to DEV_ADDR_2 can be subsituted for 10010 as it is the TMP_DEFAULT_ADDR
 * DEV_ADDR_2 to DEV_ADDR_1 are varying depending on the CONF register
 */

#define TMP_ADDR_DEFAULT       0x48
#define TMP_ADDR_SIZE          2
#define TMP_DEV_ADDR_MASK(x)   ((((uint8_t)x) & 0x7F) << 1) //byte #1
#define TMP_READWRITE_MASK     (1U << 0)                    //byte #1
#define TMP_REG_ADDR_MASK(x)   ((((uint8_t)x) & 0x03) << 0) //byte #2
#define TMP_DATA_1_MASK(x)     ((((uint8_t)x) & 0xFF) << 0) //byte #3
#define TMP_DATA_2_MASK(x)     ((((uint8_t)x) & 0xFF) << 0) //byte #4


/**Used address defined by project, default is default TMP112 address * 
 * number is refering to what sensor is used 
 */
#define TMP_ADDR_AMOUNT             4
#define TMP_ADDR_HIGHSIDE_LEFT      0
#define TMP_ADDR_HIGHSIDE_RIGHT     1
#define TMP_ADDR_LOWSIDE_LEFT       2
#define TMP_ADDR_LOWSIDE_RIGHT      3
#define TMP_ADDR(x)                 ((uint8_t)(TMP_ADDR_DEFAULT + x))

/** @brief used i2c channel on mcu.*/
extern I2C_HandleTypeDef 	   TMP_I2C_INTERFACE;

/**
 * @brief Initializes the TMP112 sensor.
 * Configures the sensor for default settings and prepares it for communication.
 * @return 0 if initialization is successful, non-zero if an error occurs.
 */
void TMP_init(void);

/**
 * @brief Reads the current temperature from the TMP112 sensor.
 * Reads the 16-bit temperature value from the TMP112 and converts it to Celsius.
 * @return Temperature in Celsius as a 16-bit signed integer.
 */
float TMP_getTemperatureSingle(uint8_t address);

/**
 * @brief Reads all the temperature sensors
 * @return Average temperature of all sensors
 */
float TMP_getTemperature();

/**
 * @brief basic function to write to TMP112 
 * @param reg: register to be written
 * @param data: data to be written to register
 * @return success
 */
uint8_t TMP_write(uint8_t address, uint8_t reg, uint16_t data);

/**
 * @brief basic function to read to TMP112 
 * @param reg: register to be read
 * @return data or 0xFFFF if fail
 */
uint16_t TMP_read(uint8_t address, uint8_t reg);

/**
 * @brief wrapper of write to set the TLOW and THIGH regisers 
 * return success 
 */
uint8_t TMP_setTLow(uint8_t address, float celcius);
uint8_t TMP_setTHigh(uint8_t address, float celcius);

/** 
 * @brief wrapper of write to write to config register
 * return success
 */
uint8_t TMP_setConfig(uint8_t address, uint16_t data);

/**
* @brief Converts digital data to degrees celcius 
* @param TMP_RESOLUTION bits of sensor data in two's complement
* @return temperature in degree celcius
*/
float TMP_data_to_celcius(uint16_t data);

/**
* @brief Converts celcius to digital data 
* @param temperature in degree celcius between TMP_MAX_DEGREE AND TMP_MIN_DEGREE
* @return digital data in TMP_RESOLUTION amount of bits
*/
uint16_t TMP_celcius_to_data(float celcius);

/**
 * Error state function
 */
void TMP_error(void);

/**
 * AUTOGEN DO NOT CHANGE THIS
 */
void MX_I2C_Init(void);

#ifdef __cplusplus
}
#endif

#endif 

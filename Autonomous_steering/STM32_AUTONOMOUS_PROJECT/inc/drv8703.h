/**
 * @file drv8703.h
 * @brief Header file for library definitions
 * Everything is MSB to LSB
 */

#ifndef __DRV8703_H_
#define __DRV8703_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

/**
 * @def Register Map
 * @brief general macro's about register map for the driver
 */
#define DRV_REG_MAP_SIZE                (uint8_t)0x05

/** 
 * @def Fault status
 * @brief Fault status register on Driver.
 * Provides information regarding general errors and warnings
 * READ-ONLY
 */ 
#define DRV_FAULT_STATUS                (uint8_t)(0x00) 
#define DRV_FAULT_STATUS_FAULT_MASK     (1U<<7) 
#define DRV_FAULT_STATUS_WDFLT_MASK     (1U<<6) 
#define DRV_FAULT_STATUS_GDF_MASK       (1U<<5) 
#define DRV_FAULT_STATUS_OCP_MASK       (1U<<4) 
#define DRV_FAULT_STATUS_VM_UVFL_MASK   (1U<<3) 
#define DRV_FAULT_STATUS_VCP_UVFL_MASK  (1U<<2) 
#define DRV_FAULT_STATUS_OTSD_MASK      (1U<<1) 
#define DRV_FAULT_STATUS_OTW_MASK       (1U<<0) 

/** 
 * @def VDS and GDF
 * @brief VDS and GDF register on Driver
 * Provides information regarding errors and warnings for the MOSFETs
 * READ-ONLY
 */ 
#define DRV_VDS_GDF                     (uint8_t)(0x01) 
#define DRV_VDS_GDF_H2_GDF_MASK         (1U<<7)
#define DRV_VDS_GDF_L2_GDF_MASK         (1U<<6)
#define DRV_VDS_GDF_H1_GDF_MASK         (1U<<5)
#define DRV_VDS_GDF_L1_GDF_MASK         (1U<<4)
#define DRV_VDS_GDF_H2_VDS_MASK         (1U<<3)
#define DRV_VDS_GDF_L2_VDS_MASK         (1U<<2)
#define DRV_VDS_GDF_H1_VDS_MASK         (1U<<1)
#define DRV_VDS_GDF_L1_VDS_MASK         (1U<<0)

/** 
 * @def Main
 * @brief Main register on Driver
 * Used for settings and clearing fault 
 * READ-WRITE
 */ 
#define DRV_MAIN                        (uint8_t)(0x02)
#define DRV_MAIN_RESERVED_MASK(x)       ((((uint8_t)x) & 0x03) << 6)
#define DRV_MAIN_LOCK_MASK(x)           ((((uint8_t)x) & 0x07) << 3)
#define DRV_MAIN_UNLOCK					DRV_MAIN_LOCK_MASK((0b011))
#define DRV_MAIN_LOCK					DRV_MAIN_LOCK_MASK((0b110))
#define DRV_MAIN_IN1_PH_MASK            (1U<<2)
#define DRV_MAIN_IN2_EN_MASK            (1U<<1)
#define DRV_MAIN_CLR_FLT_MASK           (1U<<0)

/** 
 * @def IDRIVE and WD
 * @brief IDRIVE and WD register on Driver.
 * Used for settings deadtime, watchdogs timeout and peak source and peak sink current.
 * READ-WRITE
 */ 
#define DRV_IDRIVE_WD                   (uint8_t)(0x03)
#define DRV_IDRIVE_WD_TDEAD_MASK(x)     ((((uint8_t)x) & 0x03) << 6)
#define DRV_IDRIVE_WD_WD_EN_MASK        (1U<<5)
#define DRV_IDRIVE_WD_WD_DLY_MASK(x)    ((((uint8_t)x) & 0x03) << 3)
#define DRV_IDRIVE_WD_IDRIVE_MASK(x)    ((((uint8_t)x) & 0x07) << 0) 

/** 
 * @def VDS
 * @brief VDS register on Driver
 * READ-WRITE
 */ 
#define DRV_VDS                         (uint8_t)(0x04)
#define DRV_VDS_SO_LIM_MASK             (1U<<7)
#define DRV_VDS_VDS_MASK(x)             ((((uint8_t)x) & 0x07) << 4)
#define DRV_VDS_DIS_H2_VDS_MASK         (1U<<3)
#define DRV_VDS_DIS_L2_VDS_MASK         (1U<<2)
#define DRV_VDS_DIS_H1_VDS_MASK         (1U<<1)
#define DRV_VDS_DIS_L1_VDS_MASK         (1U<<0)

/**
 * @def Config
 * @brief Config register on Driver.
 * Used to set Vds, limit SO output and turning of monitors
 * READ-WRITE
 */ 
#define DRV_CONFIG                      (uint8_t)(0x05)
#define DRV_CONFIG_TOFF_MASK(x)         ((((uint8_t)x) & 0x03) << 6)
#define DRV_CONFIG_CHOP_IDS_MASK        (1U<<5)
#define DRV_CONFIG_VREF_SCL_MASK(x)     ((((uint8_t)x) & 0x03) << 3)
#define DRV_CONFIG_SH_EN_MASK           (1U<<2)
#define DRV_CONFIG_GAIN_CS_MASK(x)      ((((uint8_t)x) & 0x03) << 0) 

/**
* @def SPI macros
* @brief general macros for complying to drivers spi format when using spi.
* Configure PWM, VREF, shunt amplifier gain.
* message format - 2 bytes:
* MSB --- LSB
* |READ/WRITE|  ADRESS_3|  ADRESS_2|  ADRESS_1|  ADRESS_0| 		 X_1|       X_1|       X_1|
* |    DATA_7|    DATA_6|    DATA_5|    DATA_4|    DATA_3|    DATA_2|    DATA_1|    DATA_0|
*/ 
#define DRV_MSG_SIZE                    (uint8_t)8
#define DRV_DATA_SIZE                   (uint8_t)8
#define DRV_ADDR_SIZE                   (uint8_t)4
#define DRV_READWRITE_MASK              (1U<<7)						 //byte #1
#define DRV_ADDR_MASK(x)                ((((uint8_t)x) & 0x0F) << 3) //byte #1
#define DRV_RESERVED_MASK(x)            ((((uint8_t)x) & 0x0F) << 0) //byte #1
#define DRV_DATA_MASK(x)                ((((uint8_t)x) & 0xFF) << 0) //byte #2
#define DRV_SPI_TIMEOUT                 (uint8_t)10  

/**
 * @def Spi interface
 * @brief used spi channel on mcu.
 * Rest of the settings for setup of driver should be here too?
 */
extern SPI_HandleTypeDef 				DRV_SPI_INTERFACE;

/**
* @brief Initialisation according to SPI settings defined earlier
*/
void DRV_init(void);

/**
* @brief General low level abstraction to communicate with drivers.
* Basically wrappers for HAL_SPI_... functions
* @param address: 4 bits  
* @param data: 8 bit.
* For write register write value
* For read returns value of register
* @return success
*/
bool DRV_write(uint8_t address, uint8_t data);
bool DRV_read(uint8_t address, uint8_t* data);

/**
* @brief modifies register with only bits mentioned and the rest unchanged.
* @param address: 4 bits
* @param data: 8 bit.
* For set function, sets masked bits to 1
* For reset function, sets masked bits to 0
* @return success
*/
bool DRV_modify_set(uint8_t address, uint8_t data);
bool DRV_modify_reset(uint8_t address, uint8_t data);

/**
* @brief Write/Read all registers and returns them from highest address to lowest address
* @param data: data must be big enough to hold all filtered registers. data array will be loaded into this.
* @param filter: bit mask to filter out unwanted registers.
* 2 MSB are don't cares. in case of write, 2 LSB are don't care too.
* 0 = means filter in
* 1 = filter out
* based on MSB to LSB of registry map.
* @return success
*/
bool DRV_writeRegister(uint8_t* data, uint8_t filter);
bool DRV_readRegister(uint8_t* data, uint8_t filter);

int MX_SPI_init();
void MX_GPIO_init();

#ifdef __cplusplus
}
#endif

#endif

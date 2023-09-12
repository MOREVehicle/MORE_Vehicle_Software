/*
 * Steering Wheel Angle Sensor decoding functions
 * What: This header file include the function prototypes for extracting values
 * of the CAN Message from the BMS. For more information on the CAN
 * frame format look at:MORE shared/22-23 Sem2/6. Embedded Systems/Quick Reference
 *  Resources/BMS
 * 
 *   
 * 
*/

#ifndef BMS_CAN_H
#define BMS_CAN_H

#include<stdlib.h>
#include <stdint.h>
#include "main.h"

//Decodes message 1 frames and returns the decoded data in the correct format
void Overall_Parameters_Decoder(uint8_t *data);
void Diagnostic_Codes_Decoder(uint8_t *data);
void Battery_Voltage_Decoder(uint8_t *data);
void Cell_Module_Temperature_Decoder(uint8_t *data);
void Cell_Temperature_Decoder(uint8_t *data);
void Cell_Balancing_Rate_Decoder(uint8_t *data);
void Individual_Cell_Voltages_Decoder(uint8_t *data);
void Individual_Cell_Module_Temperatures_Decoder(uint8_t *data);
void Individual_Cell_Temperatures_Decoder(uint8_t *data);
void Individual_Cell_Balancing_Rate_Decoder(uint8_t *data);
void State_Of_Charge_Decoder(uint8_t *data);
void Configuration_Parameters_Decoder(uint8_t *data);


enum ERROR_CODE {
     NO_ERR,
     ERR_0, // No cell communication at the start of charging or communication lost during Pre-charging (using CAN charger), cannot charge;
     ERR_1, // No cell communication (using non-CAN charger), cannot charge;
     ERR_2, // Maximum charging stage duration expired;
     ERR_3, // Cell communication lost during Main Charging or Balancing stage (using CAN charger), cannot continue charging;
     ERR_4, // Cannot set cell module balancing threshold;
     ERR_5, // Cell or cell module temperature too high;
     ERR_6, // Cell communication lost during Pre-heating stage (using CAN charger);
     ERR_7, // Number of cells mismatch;
     ERR_8, // Cell over-voltage;
     ERR_9, // Cell protection event occurred, see “Diagnostic Codes” message for determining specific protection reason;
	 ERR_10
	 };

enum CHARGING_STATE {
	Disconnected,
	Pre_heating,
	Pre_charging,
	Main_charging,
	Balancing,
	Charging_finished,
	Charging_error,
	No_Charge_Error
	};

enum PROTECTION_FLAGS {
	Under_voltage,
	Over_voltage,
	Discharge_Over_current,
	Charge_Over_current,
	Cell_Module_Overheat,
	Leakage,
	No_Cell_Communication,
	Cell_Overheat,
	PROTECTION_OK
	};

enum WARNING_FLAGS {
	Low_voltage,
	High_current,
	High_temperature,
	WARNING_OK
	};

enum BATTERY_CHARGING_STATUS {
	Inactive,
	Active
	};

enum VALIDITY {
	Invalid,
	Valid
	};




#endif

#include "BMS_CAN.h"
#include "main.h"
#include<stdlib.h>

//void Overall_Parameters_Decoder(uint8_t *data){
//
//
//
//	 enum ERROR_CODE Last_Charging_Error;
//	 enum CHARGING_STATE Charging_State;
//
//	Input_Signals = data[0];
//	Output_Signals = data[1];
//	Number_Of_Live_Cells = data[7] | (data[2] << 8);
//
//	(data[3] == 0 ? Charging_State = Disconnected      :  No_Charge_Error);
//	(data[3] == 1 ? Charging_State = Pre_heating       :  No_Charge_Error);
//	(data[3] == 2 ? Charging_State = Pre_charging      :  No_Charge_Error);
//	(data[3] == 3 ? Charging_State = Main_charging     :  No_Charge_Error);
//	(data[3] == 4 ? Charging_State = Balancing         :  No_Charge_Error);
//	(data[3] == 5 ? Charging_State = Charging_finished :  No_Charge_Error);
//	(data[3] == 6 ? Charging_State = Charging_error    :  No_Charge_Error);
//
//
//	Charging_Stage_Duration = data[5] | (data[4] << 8);
//
//	//check for LAST CHARGING ERROR on byte 6th
//	(data[6] == 0 ? Last_Charging_Error = ERR_0 :  NO_ERR);
//	(data[6] == 1 ? Last_Charging_Error = ERR_1 :  NO_ERR);
//	(data[6] == 2 ? Last_Charging_Error = ERR_2 :  NO_ERR);
//	(data[6] == 3 ? Last_Charging_Error = ERR_3 :  NO_ERR);
//	(data[6] == 4 ? Last_Charging_Error = ERR_4 :  NO_ERR);
//	(data[6] == 5 ? Last_Charging_Error = ERR_5 :  NO_ERR);
//	(data[6] == 6 ? Last_Charging_Error = ERR_6 :  NO_ERR);
//	(data[6] == 7 ? Last_Charging_Error = ERR_7 :  NO_ERR);
//	(data[6] == 8 ? Last_Charging_Error = ERR_8 :  NO_ERR);
//	(data[6] == 9 ? Last_Charging_Error = ERR_9 :  NO_ERR);
//	(data[6] == 10? Last_Charging_Error = ERR_10:  NO_ERR);
//
//}

const char dpObj_Current[] = "x2";
const char dpObj_EstimatedCharge[] = "x1";
const char dpObj_EstimatedSOC[] = "x0";
const char dpObj_Cell1[] = "x3";
const char dpObj_Cell2[] = "x4";
const char dpObj_Cell3[] = "x5";
const char dpObj_Cell4[] = "x6";
const char dpObj_Cell5[] = "x7";
const char dpObj_Cell6[] = "x8";
const char dpObj_Cell7[] = "x9";
const char dpObj_Cell8[] = "x10";
const char dpObj_Cell9[] = "x11";
const char dpObj_Cell10[] = "x12";
const char dpObj_Cell11[] = "x13";
const char dpObj_Cell12[] = "x14";
const char dpObj_Cell13[] = "x15";
const char dpObj_Cell14[] = "x16";
const char dpObj_Cell15[] = "x17";
const char dpObj_Cell16[] = "x18";
const char dpObj_Cell17[] = "x19";
const char dpObj_Cell18[] = "x20";
const char dpObj_Cell19[] = "x21";
const char dpObj_Cell20[] = "x22";
const char dpObj_Cell21[] = "x23";
const char dpObj_Cell22[] = "x24";
const char dpObj_Battery[] = "BatB";

void Diagnostic_Codes_Decoder(uint8_t *data){

	enum PROTECTION_FLAGS Protection_Flags[8];
	enum WARNING_FLAGS Warning_Flags[3];

	uint8_t Input_Signals;
	uint8_t Output_Signals;
	uint16_t Number_Of_Live_Cells;
	uint16_t Charging_Stage_Duration;
	enum VALIDITY Cell_Voltage,Cell_Module_Temperatures,Cell_Balancing_Rate,Battery_charging_finished,Cell_Temperatures;

	(data[0] & (1<<0)) ? Protection_Flags[0] = Under_voltage      :  PROTECTION_OK;
	(data[0] & (1<<1)) ? Protection_Flags[1] = Over_voltage       :  PROTECTION_OK;
	(data[0] & (1<<2)) ? Protection_Flags[2] = Discharge_Over_current  :  PROTECTION_OK;
	(data[0] & (1<<3)) ? Protection_Flags[3] = Charge_Over_current     :  PROTECTION_OK;
	(data[0] & (1<<4)) ? Protection_Flags[4] = Cell_Module_Overheat    :  PROTECTION_OK;
	(data[0] & (1<<5)) ? Protection_Flags[5] = Leakage 			  :  PROTECTION_OK;
	(data[0] & (1<<6)) ? Protection_Flags[6] = No_Cell_Communication   :  PROTECTION_OK;
	(data[2] & (1<<3)) ? Protection_Flags[7] = Cell_Overheat      :  PROTECTION_OK;

	(data[1] & (1<<0)) ? Warning_Flags[0] = Low_voltage      :  WARNING_OK;
	(data[1] & (1<<1)) ? Warning_Flags[1] = High_current     :  WARNING_OK;
	(data[1] & (1<<2)) ? Warning_Flags[2] = High_temperature :  WARNING_OK;

	(data[3] & (1<<0)) ? Cell_Voltage = Valid                :  Invalid;
	(data[3] & (1<<1)) ? Cell_Module_Temperatures = Valid    :  Invalid;
	(data[3] & (1<<2)) ? Cell_Balancing_Rate = Valid         :  Invalid;
	(data[3] & (1<<3)) ? Number_Of_Live_Cells = Valid        :  Invalid;
	(data[3] & (1<<4)) ? Battery_charging_finished = Active  :  Inactive;
	(data[3] & (1<<5)) ? Cell_Temperatures = Valid           :  Invalid;

}

void Individual_Cell_Voltages_G0_Decoder(uint8_t *data){

	uint8_t Voltage_Cell_1;
	uint8_t Voltage_Cell_2;
	uint8_t Voltage_Cell_3;
	uint8_t Voltage_Cell_4;
	uint8_t Voltage_Cell_5;
	uint8_t Voltage_Cell_6;
	uint8_t Voltage_Cell_7;
	uint8_t Voltage_Cell_8;

	Voltage_Cell_1=(data[0]+200)/100;
	Voltage_Cell_2=(data[1]+200)/100;
	Voltage_Cell_3=(data[2]+200)/100;
	Voltage_Cell_4=(data[3]+200)/100;
	Voltage_Cell_5=(data[4]+200)/100;
	Voltage_Cell_6=(data[5]+200)/100;
	Voltage_Cell_7=(data[6]+200)/100;
	Voltage_Cell_8=(data[7]+200)/100;

	NEXTION_ChangeVal(dpObj_Cell1,(uint32_t)Voltage_Cell_1);
	NEXTION_ChangeVal(dpObj_Cell2,(uint32_t)Voltage_Cell_2);
	NEXTION_ChangeVal(dpObj_Cell3,(uint32_t)Voltage_Cell_3);
	NEXTION_ChangeVal(dpObj_Cell4,(uint32_t)Voltage_Cell_4);
	NEXTION_ChangeVal(dpObj_Cell5,(uint32_t)Voltage_Cell_5);
	NEXTION_ChangeVal(dpObj_Cell6,(uint32_t)Voltage_Cell_6);
	NEXTION_ChangeVal(dpObj_Cell7,(uint32_t)Voltage_Cell_7);
	NEXTION_ChangeVal(dpObj_Cell8,(uint32_t)Voltage_Cell_8);
}

void Individual_Cell_Voltages_G1_Decoder(uint8_t *data){

	uint8_t Voltage_Cell_9;
	uint8_t Voltage_Cell_10;
	uint8_t Voltage_Cell_11;
	uint8_t Voltage_Cell_12;
	uint8_t Voltage_Cell_13;
	uint8_t Voltage_Cell_14;
	uint8_t Voltage_Cell_15;
	uint8_t Voltage_Cell_16;

	Voltage_Cell_9=(data[0]+200)/100;
	Voltage_Cell_10=(data[1]+200)/100;
	Voltage_Cell_11=(data[2]+200)/100;
	Voltage_Cell_12=(data[3]+200)/100;
	Voltage_Cell_13=(data[4]+200)/100;
	Voltage_Cell_14=(data[5]+200)/100;
	Voltage_Cell_15=(data[6]+200)/100;
	Voltage_Cell_16=(data[7]+200)/100;

	NEXTION_ChangeVal(dpObj_Cell9,(uint32_t)Voltage_Cell_9);
	NEXTION_ChangeVal(dpObj_Cell10,(uint32_t)Voltage_Cell_10);
	NEXTION_ChangeVal(dpObj_Cell11,(uint32_t)Voltage_Cell_11);
	NEXTION_ChangeVal(dpObj_Cell12,(uint32_t)Voltage_Cell_12);
	NEXTION_ChangeVal(dpObj_Cell13,(uint32_t)Voltage_Cell_13);
	NEXTION_ChangeVal(dpObj_Cell14,(uint32_t)Voltage_Cell_14);
	NEXTION_ChangeVal(dpObj_Cell15,(uint32_t)Voltage_Cell_15);
	NEXTION_ChangeVal(dpObj_Cell16,(uint32_t)Voltage_Cell_16);

}

void Individual_Cell_Voltages_G2_Decoder(uint8_t *data){

	uint8_t Voltage_Cell_17;
	uint8_t Voltage_Cell_18;
	uint8_t Voltage_Cell_19;
	uint8_t Voltage_Cell_20;
	uint8_t Voltage_Cell_21;
	uint8_t Voltage_Cell_22;

	Voltage_Cell_17=(data[0]+200)/100;
	Voltage_Cell_18=(data[1]+200)/100;
	Voltage_Cell_19=(data[2]+200)/100;
	Voltage_Cell_20=(data[3]+200)/100;
	Voltage_Cell_21=(data[4]+200)/100;
	Voltage_Cell_22=(data[5]+200)/100;

	NEXTION_ChangeVal(dpObj_Cell17,(uint32_t)Voltage_Cell_17);
	NEXTION_ChangeVal(dpObj_Cell18,(uint32_t)Voltage_Cell_18);
	NEXTION_ChangeVal(dpObj_Cell19,(uint32_t)Voltage_Cell_19);
	NEXTION_ChangeVal(dpObj_Cell20,(uint32_t)Voltage_Cell_20);
	NEXTION_ChangeVal(dpObj_Cell21,(uint32_t)Voltage_Cell_21);
	NEXTION_ChangeVal(dpObj_Cell22,(uint32_t)Voltage_Cell_22);

}
void State_Of_Charge_Decoder(uint8_t *data){
	uint16_t Current;
	uint16_t Estimated_Charge;
	uint8_t  Estimated_Soc;

	Current = (data[1] | (data[0] << 8)) / 10;
	Estimated_Charge = (data[3] | (data[2] << 8)) / 10;
	Estimated_Soc = data[6];

	NEXTION_ChangeVal(dpObj_Current,(uint32_t)Current);
	NEXTION_ChangeVal(dpObj_EstimatedCharge,(uint32_t)Estimated_Charge);
	NEXTION_ChangeVal(dpObj_EstimatedSOC,(uint32_t)Estimated_Soc);
	NEXTION_ChangeVal(dpObj_Battery,(uint32_t)Estimated_Soc);




}

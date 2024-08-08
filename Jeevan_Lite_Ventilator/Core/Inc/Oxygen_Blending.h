/*
 * Oxygen_Blending.h
 *
 *  Created on: Mar 30, 2022
 *      Author: asus
 */

#ifndef INC_OXYGEN_BLENDING_H_
#define INC_OXYGEN_BLENDING_H_


#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



typedef struct
{

	int oxygen_Acheived;
	int Servo;
	uint16_t O2_DAC;
	int O2_process;
	uint8_t O2_percentage;
	uint32_t _AVG_CirusO2Sensor_value;
	uint32_t count;
	float _FIO2_Val_float;
	float O2_percentage_float;
	float Fio2_Acheived_Percentage;
	float Fio2_Acheived_Percentage2;
	int _Pressure_Base;
	int _Flow_Base;
	uint8_t _PIP_Val;
	uint16_t _VT_Val;
	int fio2_check;
	int fio2_check2;
	float Result;
	float _AVG_CirusO2Sensor;
	uint8_t _FIO2_Val;
    uint8_t FiO2_old;
}O2_PARAMETER;





typedef struct
{
	float _AVG_O2_Flow_Sensor;
	uint16_t Raw_O2_Flow;
	uint16_t O2_milli_volt;
	uint16_t O2_Flow_raw1;
	uint16_t O2_raw1_new;
	uint16_t O2_raw1_old;
	uint16_t O2_raw1;
	uint16_t O2_Flow_raw1_new;
	uint16_t O2_Flow_raw1_old;
	uint16_t O2_Flow_Offset;
	float O2_flow_raw1;
	uint16_t O2_flow_raw1_new;
	uint16_t O2_flow_raw_old;
	uint16_t O2_volt2;
	uint16_t O2_volt_new;
	uint16_t O2_volt_old;
	uint16_t Raw_O2;
	float O2_p;
	float O2_kpa;
	int O2_kpa1;
} O2_FLOW_PARAMETER;


O2_PARAMETER O2;
O2_FLOW_PARAMETER O2_F;


void Oxygen_Task (void *argument);
void Get_Oxygen();
void O2_Parameter();
void O2_Flow_offset();
uint16_t O2_milli_volt1(uint16_t);
void O2_Flow_Func();
#endif /* INC_OXYGEN_BLENDING_H_ */

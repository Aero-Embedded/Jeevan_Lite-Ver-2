/*
 * Sensors_Data.h
 *
 *  Created on: Mar 7, 2022
 *      Author: asus
 */

#ifndef INC_PRESSURE_SENSORS_DATA_H_
#define INC_PRESSURE_SENSORS_DATA_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"




typedef struct
{
	uint16_t _Pressure_Sensor_Offset_Val;
	uint16_t  _Runtime_Pressure_Val;
	uint8_t   _Pressure_Val;
	float     _Pressure_Mv_Val_;
	float     _P_cmh2o_Val;
}Pressure;



Pressure Pressure_sensor;

void Pressure_Sensor_offset();
uint16_t Pressure_Sensor_Mv(uint16_t P_value);
void Pressure_Sensor_Value();



#endif /* INC_PRESSURE_SENSORS_DATA_H_ */

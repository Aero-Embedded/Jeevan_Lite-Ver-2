/*
 * Sensors_Data.c
 *
 *  Created on: Mar 7, 2022
 *      Author: asus
 */

#include "Pressure_Sensors_Data.h"


uint16_t Pressure_Sensor_Mv(uint16_t P_value)
{
 	  return (((P_value)*5000)/4095);
}

void Pressure_Sensor_offset(void)
{
  	for(int n=0;n<1000;n++)
  	{
  		  Pressure_sensor._Pressure_Sensor_Offset_Val=Pressure_Sensor_Mv(Pressure_Sensor_Pin);
  		  vTaskDelay(1);
  	}
}

void Pressure_Sensor_Value()
{
     const float gP_sensitivity=44.13;

     Pressure_sensor._Runtime_Pressure_Val=Pressure_Sensor_Mv(Pressure_Sensor_Pin);

     Pressure_sensor._Pressure_Mv_Val_=(Pressure_sensor._Runtime_Pressure_Val-Pressure_sensor._Pressure_Sensor_Offset_Val);

     Pressure_sensor._P_cmh2o_Val=(Pressure_sensor._Pressure_Mv_Val_/gP_sensitivity);

     Pressure_sensor._Pressure_Val=Pressure_sensor._P_cmh2o_Val;


}

/*
 * Flow_Sensors_Data.h
 *
 *  Created on: Mar 7, 2022
 *      Author: asus
 */

#ifndef INC_FLOW_SENSORS_DATA_H_
#define INC_FLOW_SENSORS_DATA_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



#define ADS1115_ADDRESS 0x48
extern I2C_HandleTypeDef hi2c2;




typedef struct
{
	uint16_t AW_Flow_Offset;
	uint16_t AW_flow_raw;
	uint16_t AW_flow_raw_filtered;
	uint16_t AW_flow_milli_volt;
	unsigned char ADSwrite[3];
	unsigned char ADSread[2];
	uint16_t AW_flow_raw1_new;
	uint16_t AW_flow_raw_old;
	uint8_t fault;

}Flow;

typedef struct
{
	float delp_flow1;
	float dp;
	float dp1;
	double Flow1;
	int readings[15];
	int readIndex;
	long total ;
	long average;
	int _Flow_Val;
}Flow2;

typedef struct
{
	float Flow_Volume;
	float Volume;
	int  Volume_Val;
}Volume;




Flow Flow_sensor;
Flow2 Flow_Sensor_cal;
Volume vol;


float AW_flow_raw_Volt(uint16_t r);
uint16_t AW_flow_moving_average(uint16_t value);
uint16_t ADS1115_AW_flow_sensor();
void Get_AW_Flow(void);
long adj(long x, long in_min, long in_max, long out_min, long out_max);
void _Flow();
void Flow_Sensor_7002_offset();
void Flow_Sensor_Value();
void Get_Volume();


#endif /* INC_FLOW_SENSORS_DATA_H_ */

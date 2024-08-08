/*
 * Nebuliser.h
 *
 *  Created on: Apr 6, 2022
 *      Author: asus
 */

#ifndef INC_NEBULISER_H_
#define INC_NEBULISER_H_


#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"




typedef struct  __attribute__((packed))
{
	uint16_t  _header ; // 2
	uint8_t   _length ; // 1
	uint8_t   NEBULISER_SYNC ;
	uint8_t   NEBULISER_ON ;
	uint8_t   TAB_CHARGER;
	uint8_t   FIND_MY_DEVICE  ;
	uint8_t   SHUT_DOWN  ;
	uint8_t   ALERT_SNOOZE  ;
	uint8_t   INSPIRATORY_HOLD_EXPIRATORY_HOLD  ;
	uint8_t   NON_INVARSIVE ;
	uint8_t   READ_SENSOR_OFFSET ;
	uint8_t   dummy7  ;
	uint8_t   dummy8  ;
	uint8_t   dummy9  ;
	uint8_t   dummy10 ;
	uint8_t   _CRC8  ;
} NEBULISER_RANGE_PACKET ;


typedef struct
{
	uint8_t _RANGE_NEBULISER_ON_Val;
	uint8_t _RANGE_NEBULISER_SYNC_ON_Val;
	uint8_t _TAB_CHARGER;
	uint8_t _FIND_MY_DEVICE;
	uint8_t _SHUT_DOWN;
	uint8_t _ALERT_SNOOZE;
	uint8_t _INSPIRATORY_HOLD;
	uint8_t _EXPIRATORY_HOLD;
	uint8_t _NON_INVARSIVE;
	uint8_t _INVARSIVE;
	uint8_t _READ_SENSOR_OFFSET;

}NEBULISER_FLAGS;



typedef struct  __attribute__((packed))
{
	uint16_t _header;
	uint8_t  _length;
	uint8_t  _Dummy_1;
	int16_t  _Pressure_Sensor_Voltage_Val;
	int16_t  _Flow_Sensor_Voltage_Val;
	int16_t  _O2_Flow_Sensor_Voltage_Val;
	uint8_t  _Dummy_2;
	uint8_t  _CRC8;
} SENSOR_VOLTAGES ;


NEBULISER_FLAGS N;
SENSOR_VOLTAGES Sensor_Voltage;

void Nebuliser_Task(void *argument);
void NEBULISER_COMMAND_HANDLER(NEBULISER_RANGE_PACKET * );
void Nebuliser_Func();
void Call_Shutdown_Func();
void Sensor_Voltage_Transmit();
uint8_t chksum8_cal_neb(const unsigned char *buff, size_t len);
void Shutdown_Task(void *argument);



#endif /* INC_NEBULISER_H_ */

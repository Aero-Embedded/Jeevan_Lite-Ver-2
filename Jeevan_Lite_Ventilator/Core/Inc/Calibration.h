/*
 * Calibration.h
 *
 *  Created on: Apr 2, 2022
 *      Author: asus
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_



#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"





typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _BLOWER;
	uint8_t  _PRESSURE_SENSOR;
	uint8_t  _FLOW_SENSOR_7002;
	uint8_t  _LEAK_VALVE_TEST ;
	uint8_t  _O2_CHECK ;
	uint8_t  _ALARAM_TEST;
	uint8_t  _BATTERY_TEST;
	uint8_t  NC1;
	uint8_t  NC2;
	uint8_t  NC3;
	uint8_t  NC4;
	uint8_t  NC5;
	uint8_t  NC6;
	uint8_t  _CRC8;

} REQUEST_CALIBRATION_PACKET_tst;


typedef struct __attribute__((packed))
{
	    uint16_t _header ;
		uint8_t  _length ;
		uint8_t  _REPORT0;
		uint8_t   _PRESSURE;
		uint8_t   _FLOW;
		uint8_t   _LEAK;
		uint8_t   _O2FLOW;
		uint8_t   NCBYTE;
		uint8_t   _BATTERY;
		uint8_t   NCBYTE1;
		uint8_t  _CRC8;

} RESPOND_CALIBRATION_PACKET_tst;


typedef struct
{
	uint8_t BLOWER;
	uint8_t	PRESSURE_SENSOR;
	uint8_t	FLOW_SENSOR_7002;
	uint8_t	LEAK_VALVE_TEST ;
	uint8_t	O2_CHECK ;
	uint8_t	ALARAM_TEST;
	uint8_t	BATTERY_TEST;
	uint8_t Uart_Calib;
	int error_count;
	float temp_Pressure_Val1;
	float temp_Pressure_Val2;
	float total_temp_Pressure_Val;
}CALIBRARTION_NAME;

RESPOND_CALIBRATION_PACKET_tst _RESPOND_CALIBRATION_PACKET ;
CALIBRARTION_NAME C;

void CALIBRATION_COMMAND_HANDLER(REQUEST_CALIBRATION_PACKET_tst *);
void CALIBRATION_Task(void *argument);
void SEND_REPORT_PACKET();
void CALIBRATION_Blower(void);
void CALIBRATION_Pressure_Sensor();
void CALIBRATION_Proximal_Flow_Sensor();
void CALIBRATION_Exp_valve();
void CALIBRATION_Oxygen();
void CALIBRATION_Led();
void CALIBRATION_Battery();
void cal_Battery();
uint8_t chksum8_cal(const unsigned char *buff, size_t len);

#endif /* INC_CALIBRATION_H_ */

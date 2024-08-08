/*
 * Service.h
 *
 *  Created on: Apr 5, 2022
 *      Author: asus
 */

#ifndef INC_SERVICE_H_
#define INC_SERVICE_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"

typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  SERVICE_BLOWER;
	uint8_t  SERVICE_ADS1115;
	uint8_t  SERVICE_PRESSURE_SENSOR;
	uint8_t  SERVICE_FLOW_SENSOR ;
	uint8_t  SERVICE_EXPIRATORY_VALVE ;
	uint8_t  SERVICE_LEAK;
	uint8_t  SERVICE_SERVO_MOTOR;;
	uint8_t  SERVICE_O2;
	uint8_t  SERVICE_NEBULISER;
	uint8_t  REMOVE_TEST_LUNG;
	uint8_t  HAND_LOCK;
	uint8_t  NC3;
	uint8_t  NC4;
	uint8_t  _CRC8;

} REQUEST_SERVICE_PACKET_tst;


typedef struct __attribute__((packed))
{
	    uint16_t _header ;
		uint8_t  _length ;
		uint8_t  _REPORT0_FLAGS;
		uint8_t  _PRESSURE1;
		uint8_t  _PRESSURE2;
		uint8_t  _FLOW;
		uint8_t  _O2_PERCENTAGE;
		uint16_t  _RESULT;
		uint8_t  _DUMMY1;
		uint8_t  _CRC8;

} RESPOND_SERVICE_PACKET_tst;


typedef struct
{
   int Uart_Service;
   int error_count;
   int temp_Pressure_Val1;
   int temp_Pressure_Val2;
   int total_temp_Pressure_Val;
   uint8_t Blower;
   uint8_t ADS1115;
   uint8_t Pressure_Sensor;
   uint8_t Flow_Sensor;
   uint8_t Expiratory_Valve;
   uint8_t E_Valve_use_hand;
   uint8_t Connect_E_valve;
   uint8_t Service_Leak;
   uint8_t Servo;
   uint8_t O2;
   uint8_t Nebuliser;
   int error_count_2;
   uint8_t Blower_Problem;
   uint8_t Step_One;
   uint8_t Step_Two;
   uint8_t Blower_Base_Problem;
   uint16_t dac_last;
   uint8_t Remove_Test_Lung;
   uint8_t Hand_Lock;
   uint8_t Leak_first_Test;
   uint8_t servo_step_one;
   uint16_t O2_Acheived_Count;

}SERVICE_NAME;


SERVICE_NAME S;
RESPOND_SERVICE_PACKET_tst _RESPOND_SERVICE_PACKET;
void SERVICE_COMMAND_HANDLER(REQUEST_SERVICE_PACKET_tst *);
void SERVICE_Task(void *argument);
void SEND_SERVICE_PACKET();

void SERVICE_Blower();
void SERVICE_ADS1115();
void SERVICE_Pressure_Sensor();
void SERVICE_Flow_Sensor();
void SERVICE_Expiratory_Valve();
void SERVICE_Leak();
void SERVICE_Servo();
void SERVICE_O2();
void SERVICE_Nebuliser();

#endif /* INC_SERVICE_H_ */

/*
 * Alert.h
 *
 *  Created on: Mar 30, 2022
 *      Author: asus
 */

#ifndef INC_ALERT_H_
#define INC_ALERT_H_


#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"




typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_PRESSURE_LMT:1;
	volatile uint8_t _ALERT_TAB_BATTERY:1;
	volatile uint8_t _ALERT_OXYGEN_SUPPLY:1;
	volatile uint8_t _ALERT_PATIENT_CIRCUIT:1;
	volatile uint8_t _ALERT_SPO2:1;
	volatile uint8_t _ALERT_APNEA:1;
	volatile uint8_t _ALERT_BAT_DRAIN:1;
	volatile uint8_t _ALERT_POWER:1;
} FIRST_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_FLOW_HL:1;
	volatile uint8_t _ALERT_FLOW_YN:1;
	volatile uint8_t _ALERT_PIP_HL:1;
	volatile uint8_t _ALERT_PIP_YN:1;
	volatile uint8_t _ALERT_OXY_HL:1;
	volatile uint8_t _ALERT_OXY_YN:1;
	volatile uint8_t _ALERT_MINT_VOLUME_HL:1;
	volatile uint8_t _ALERT_MINT_VOLUME_YN:1;

} SECOND_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_PULSE_HL:1;
	volatile uint8_t _ALERT_PULSE_YN:1;
	volatile uint8_t _ALERT_PEEP_HL:1;
	volatile uint8_t _ALERT_PEEP_YN:1;
	volatile uint8_t _ALERT_VT_HL:1;
	volatile uint8_t _ALERT_VT_YN:1;
	volatile uint8_t _ALERT_T_RR_HL:1;
	volatile uint8_t _ALERT_T_RR_YN:1;

} THIRD_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_BAT_DRAIN_50_PERC:1;
	volatile uint8_t _ALERT_FLOW_SENSOR_REVERSED:1;
	volatile uint8_t _ALERT_FLOW_SENSOR_FOR_WATER:1;
	volatile uint8_t _ALERT_FLOW_SENSOR_CALIBRATION_NEED:1;
	volatile uint8_t _ALERT_FIND_DEVICE:1;
	volatile uint8_t _ALERT_LEAK_HL:1;
	volatile uint8_t _ALERT_BLUETOOTH_CONNECTION_ERROR:1;
	volatile uint8_t _ALERT_NEBULIZER:1;



} FOURTH_FRAME ;

typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_INVASIVE_NON_INVASIVE_VENTILATION:1;
	volatile uint8_t _ALERT_HFNC:1;
	volatile uint8_t _ALERT_PRESSURE_:1;
	volatile uint8_t _ALERT_SUCTION:1;
	volatile uint8_t _ALERT_EXPIRATORY_HOLD:1;
	volatile uint8_t _ALERT_INSPIRATORY_HOLD:1;
	volatile uint8_t _ALERT_SPO2_SENSOR_FAILURE:1;
	volatile uint8_t _ALERT_SPO2_MONITORING_INITIATED:1;
} FIFTH_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t RESERVED2:1;
	volatile uint8_t RESERVED1:1;
	volatile uint8_t _ALERT_WIFI_CONNECTION:1;
	volatile uint8_t _ALERT_O2_SENSOR_NOT_AVAILABLE:1;
	volatile uint8_t _ALERT_SYSTEM_CALIBRATION_NEEDED:1;
	volatile uint8_t _ALERT_TEMPERATURE:1;
	volatile uint8_t _ALERT_OXYGEN_SENSOR_REPLACEMENT:1;
	volatile uint8_t _ALERT_BATTERY_NOT_AVAILABLE:1;
} SIXTH_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_BATTERY_PERCENTAGE:8;

} SEVETH_FRAME ;

typedef struct __attribute__((packed)) {

	volatile uint8_t RESRVED:5;
	volatile uint8_t ALERT:1;
	volatile uint8_t BKUPMODE:1;
	volatile uint8_t MODE:1;


} EIGHT_FRAME ;

typedef struct __attribute__((packed)) {
	uint16_t _header; // 2
    uint8_t  _length; // 1
	union {
	 volatile unsigned char FIRST_BYTES;
	 FIRST_FRAME FRAMEBits ;
	}FIRST_FRAME_UN;

	union {
	 volatile unsigned char SECOND_BYTES;
	 SECOND_FRAME FRAMEBits ;
	}SECOND_FRAME_UN;

	union {
	 volatile unsigned char THIRD_BYTES;
	 THIRD_FRAME FRAMEBits ;
	}THIRD_FRAME_UN;

	union {
	 volatile unsigned char FOURTH_BYTES;
	 FOURTH_FRAME FRAMEBits ;
	}FOURTH_FRAME_UN;

	union {
	 volatile unsigned char FIFTH_BYTES;
	 FIFTH_FRAME FRAMEBits ;
	}FIFTH_FRAME_UN;

	union {
	  volatile unsigned char SIXTH_BYTES;
	  SIXTH_FRAME FRAMEBits ;
	 }SIXTH_FRAME_UN;

	 union {
	 	volatile unsigned char SEVENTH_BYTES;
	 	SEVETH_FRAME FRAMEBits ;
	 }SEVENTH_FRAME_UN;

	//8th BYTE
	 union {
		 	volatile unsigned char EIGHT_BYTES;
		 	EIGHT_FRAME FRAMEBits ;
		 }EIGHT_FRAME_UN;

	volatile uint8_t _CRC8;

}ALERT_RESPONSE_PACKET;


typedef struct  __attribute__((packed))
{
	uint16_t  _header ; // 2
	uint8_t   _length ; // 1
	uint8_t _RANGE_MODE          ;
	uint8_t _RANGE_VT_MIN        ;
	uint8_t _RANGE_VT_MAX        ;
	uint8_t _RANGE_PIP_MIN       ;
	uint8_t _RANGE_PIP_MAX       ;
	uint8_t _RANGE_RR_MIN        ;
	uint8_t _RANGE_RR_MAX        ;
	uint8_t _RANGE_MINT_VOL_MIN  ;
	uint8_t _RANGE_MINT_VOL_MAX  ;
	uint8_t _RANGE_SPO2_MIN      ;
	uint8_t _RANGE_SPO2_MAX      ;
	uint8_t _RANGE_PULSE_MIN     ;
	uint8_t _RANGE_PULSE_MAX     ;
	uint8_t _CRC8;
} ALERT_RANGE_PACKET;


typedef struct
{
	uint8_t _RANGE_MODE_Val;
	uint16_t _RANGE_VT_MIN_Val;
	uint16_t _RANGE_VT_MAX_Val;
	uint8_t _RANGE_PIP_MIN_Val;
	uint8_t _RANGE_PIP_MAX_Val;
	uint8_t _RANGE_RR_MIN_Val;
	uint8_t _RANGE_RR_MAX_Val;
	uint16_t _RANGE_MINT_VOL_MIN_Val;
	uint16_t _RANGE_MINT_VOL_MAX_Val;
	uint8_t _RANGE_SPO2_MIN_Val;
	uint8_t _RANGE_SPO2_MAX_Val;
	uint8_t _RANGE_PULSE_MIN_Val;
	uint8_t _RANGE_PULSE_MAX_Val;
}ALERT_RANGE_PACKET_RECEIVE;

typedef struct
{
	uint8_t Alert_check_done;
	uint8_t Pip_Alert;
	uint8_t Alert;
	uint8_t Alert_Now;
	uint8_t Apnea_UART_alert;
	uint8_t PEEP_VAL;
	uint8_t Peep_Alert;
	uint8_t Respiratory_Rate;
	float _Max_Volume_Val;
	float _Max_Volume_Val_Ml;
	uint8_t Mint_Vol_Alert;
	float _Max_Tidal_Volume;
	uint8_t Tidal_Volume_Alert;
	uint16_t RR_I_TIME;
	uint16_t RR_E_TIME;
	float Acheived_RR;
	uint8_t Wait_4_cycle;
	uint8_t Fio2_Supply_Error;
	uint8_t Oxygen_Alert;
	uint8_t Fio2_Value_Set;
	int Leak;
	int Insp_Volume;
	int Exp_Volume;
	int Leak_Error_Count;
	int Patient_Circuit_disconnected;
	int Proximal_Flow_Sensor_reversed;
	uint8_t Red_Led_Alert;

}ALERT_FLAGS;





ALERT_RESPONSE_PACKET _ALERT_RESPONSE_PKT ;
ALERT_FLAGS A;
ALERT_RANGE_PACKET_RECEIVE AR;

void COMMAND_HANDLER_ALERT(ALERT_RANGE_PACKET *RX_ALERT_RANGE_PACKET);
void Alert_Task (void *argument);
void Alert_Func();
void SEND_ALERT_PACKET();
void Pip_Alert_Func();
void Peep_Alert_Func();
void Minite_Volume_Alert_Func();
void Tidal_Volume_Alert_Func();
void Respiratory_Rate_Alert_Func();
void Oxygen_Alert_Func();
void Leak_Alert_Func();
void Patient_Circuit_Disconnected_Alert_Func();
void Proximal_Flow_Sensor_Reverse_Direction();
void Alert_I_Time_Parameter();
void Alert_E_Time_Parameter();
void Alert_Receiving_Parameter();
void Main_Supply_or_Battery();
void Battery_Alert_Func();
void Main_Supply_or_Battery_Indication();
void Battery_Status();
void Led_Alert();
void Battery_Information();
void data_request_Check();

#endif /* INC_ALERT_H_ */

/*
 * Uart.h
 *
 *  Created on: Mar 8, 2022
 *      Author: asus
 */

#ifndef INC_UART_H_
#define INC_UART_H_




#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



typedef struct  __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _Pressure_Val;
	int16_t  _Flow_Val;
	int16_t _Volume_Val;
	uint8_t _Control_Byte;
	uint8_t _SPO2;
	uint8_t _Heart_BPM;
	uint8_t  _CRC8;
} SET_CYCLIC_PACKET ;

typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _mode;
	uint8_t  _PIP;
	uint8_t  _PEEP;
	uint16_t _VTI ;    //_T_HIGH
	uint8_t  _I_E ;
	uint8_t  _RR;
	uint8_t  _FIO2;
	uint8_t _RiseTime_TRIG_TIME ;
	uint8_t  _FlowRate;
	uint8_t  _APNEA;
	uint8_t  _TRIG_TYPE_TRIG_LMT;
	uint8_t  _T_HIGH;
	uint8_t  _CRC8;

} SET_PARAM_CMD_PACKET ;


SET_CYCLIC_PACKET _CYCLIC_TRANSMIT_PKT ;


void COMMAND_HANDLER(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void COMMAND_HANDLER_GRAPH(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);

void PC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void VC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void PC_SIMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void VC_SIMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void PSV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void CPAP_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void BIPAP_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void APRV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void BACKUP_PC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void BACKUP_VC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);

void HFNC_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
void SUCTION_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Uart_Receiver_Task (void *argument);
uint8_t chksum8(const unsigned char *buff, size_t len);
void Uart_Transmit_Task (void *argument);
void UART_Transmit();
void Clear_All_Alert_Bits();
void UART5_Init();
void UART6_Init();
void alter_uart_data();
#endif /* INC_UART_H_ */

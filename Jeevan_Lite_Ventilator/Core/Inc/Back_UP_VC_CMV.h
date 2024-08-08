/*
 * Back_UP_VC_CMV.h
 *
 *  Created on: Mar 28, 2022
 *      Author: asus
 */

#ifndef INC_BACK_UP_VC_CMV_H_
#define INC_BACK_UP_VC_CMV_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



typedef struct
{
	uint8_t _Tidal_Volume;
	uint8_t _PEEP_Val;
	uint32_t CycleTime;
	uint32_t I_Time;
	uint32_t E_Time;
	uint16_t _Flow_Rate;
	uint8_t  RR;
	uint8_t _FIO2_Val;
	uint16_t _VT_Val;
}BACKUP_VC_SIMV_Mode_Parameter;



typedef struct
{
	uint32_t _I_TIMER_HOLD;
	uint32_t _E_TIMER_HOLD;
}BACKUP_VC_SIMV_Mode_Calculated_Parameter;




typedef struct
{
	int max_flow_acheived;
	int constant_dac_done;
	int Reached_flow_val;
	int sensordata_done;
	int Volume_acheived;
	int volume_reached;

}BACKUP_VC_SIMV_Mode_Flags_Set;


typedef struct
{
	int F_max;
	uint16_t temp_dac;
	uint16_t temp_dac_new;
	uint16_t _DAC_VAL0;
	int PID_task_delay;


}BACKUP_VC_SIMV_Mode_DAC_Control;

typedef struct
{
	int peep_process_done;
	int lock;
	uint16_t Lock_delay;


}BACKUP_VC_SIMV_Mode_Common_Parameter;



BACKUP_VC_SIMV_Mode_Parameter D1;
BACKUP_VC_SIMV_Mode_Calculated_Parameter D2;
BACKUP_VC_SIMV_Mode_Flags_Set D3;
BACKUP_VC_SIMV_Mode_DAC_Control D4;
BACKUP_VC_SIMV_Mode_Common_Parameter D5;



void Back_Up_VC_CMV_Mode_Task(void *argument);
void PID_Back_Up_VC_CMV_Mode_Task(void *argument);
void volume_task_BACKUP_SIMV();
void BACKUP_VC_SIMV_Pulse_I_Parameter();
void BACKUP_VC_SIMV_Pulse_E_Parameter();
void Peep_E_Valve_Lock_delay_BACKUP_VC();

#endif /* INC_BACK_UP_VC_CMV_H_ */

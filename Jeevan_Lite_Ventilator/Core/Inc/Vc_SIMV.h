/*
 * Vc_SIMV.h
 *
 *  Created on: Mar 17, 2022
 *      Author: asus
 */

#ifndef INC_VC_SIMV_H_
#define INC_VC_SIMV_H_

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
	uint8_t Assist_mode2;

}VC_SIMV_Mode_Parameter;



typedef struct
{
	uint32_t _I_TIMER_HOLD;
	uint32_t _E_TIMER_HOLD;
}VC_SIMV_Mode_Calculated_Parameter;


typedef struct
{
	int max_flow_acheived;
	int constant_dac_done;
	int Reached_flow_val;
	int sensordata_done;
	int Volume_acheived;
	int volume_reached;

}VC_SIMV_Mode_Flags_Set;


typedef struct
{
	int F_max;
	uint16_t temp_dac;
	uint16_t temp_dac_new;
	uint16_t _DAC_VAL0;
	int PID_task_delay;


}VC_SIMV_Mode_DAC_Control;

typedef struct
{
	int peep_process_done;
	int lock;
	uint16_t Lock_delay;


}VC_SIMV_Mode_Common_Parameter;




typedef struct
{
	uint16_t  _TRIG_TYPE;
	uint16_t _TRIG_LMT;
	uint16_t  _TRIG_TIME;
	uint16_t _CALC_TRIG_VAL;
	uint16_t _TOLERANCE_EWAVE;
	uint16_t _TRIG_WINDOW;
	float simv_trigger_offset;
	float simv_trigger_offset2;
}VC_SIMV_Trigger_Control;


VC_SIMV_Mode_Parameter R1;
VC_SIMV_Mode_Calculated_Parameter R2;
VC_SIMV_Mode_Flags_Set R3;
VC_SIMV_Mode_DAC_Control R4;
VC_SIMV_Mode_Common_Parameter R5;
VC_SIMV_Trigger_Control R6;

void VC_SIMV_Task(void *argument);
void VC_SIMV_PID_Task(void *argument);
void VC_SIMV_Pulse_I_Parameter();
void VC_SIMV_Pulse_E_Parameter();
void volume_task_SIMV();
void Peep_E_Valve_Lock_delay_Vc_simv();
void Switch_TASK_I_CYCLE();

#endif /* INC_VC_SIMV_H_ */

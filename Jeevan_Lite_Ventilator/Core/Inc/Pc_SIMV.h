/*
 * Pc_SIMV.h
 *
 *  Created on: Mar 17, 2022
 *      Author: asus
 */

#ifndef INC_PC_SIMV_H_
#define INC_PC_SIMV_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"




typedef struct
{

	uint8_t _Mode_Val;
	uint8_t _Pause;
	uint8_t _PIP_Val;
	uint8_t _PEEP_Val;
	uint32_t CycleTime;
	uint32_t I_Time;
	uint32_t E_Time;
	uint16_t RT_Value;
	uint8_t  RR;
	uint8_t  Assist_mode;


}PC_SIMV_Mode_Parameter;


typedef struct
{
	uint32_t _I_TIMER_HOLD;
	uint32_t _E_TIMER_HOLD;
	uint16_t  ramp_time;
	float     ramp_time_percentage;




}PC_SIMV_Mode_Calculated_Parameter;



typedef struct
{

	int ok;
	int cycle_done;
	int Pip_Acheived_Flag;

}PC_SIMV_Mode_Flags_Set;


typedef struct
{
	float starting_DAC;
	uint16_t _DAC_VAL0;
	uint16_t Last_DAC;
	float result1;
	float last_result1;
	float result2;
	float last_result2;
	uint16_t ten_ms;
	uint16_t Acheived_ms;
	uint16_t Acheived_ten_ms;
	int pmax_error2;
	int pmax_error1;
	float Ending_Dac;
	float incrementing_Dac_value_10ms;
	int nack;
	int PID_task_delay;
	int PID_task_delay_lock;
	int result1_error;


}PC_SIMV_Mode_DAC_Control;


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


}PC_SIMV_Trigger_Control;






PC_SIMV_Mode_Parameter T1;
PC_SIMV_Mode_Calculated_Parameter T2;
PC_SIMV_Mode_Flags_Set T3;
PC_SIMV_Mode_DAC_Control T4;
PC_SIMV_Trigger_Control T5;

void PC_SIMV_Task (void *argument);
void PC_SIMV_PID_Task(void *argument);
void PC_SIMV_Pulse_I_Parameter();
void PC_SIMV_Pulse_E_Parameter();
void DAC_Value_Correction_SIMV();
void Ending_Dac_value_correction_SIMV();
void pip_value_correction_SIMV();
void Pip_Acheived_Early_SIMV();
void Pip_Acheived_Slowly_SIMV();
void Pip_Not_Acheived_SIMV();
void Pip_Acheived_Normally_SIMV();
void Peep_E_Valve_Lock_delay_Pc_SIMV();
void Switch_TASK_I_CYCLE();

#endif /* INC_PC_SIMV_H_ */

/*
 * pc_cmv.h
 *
 *  Created on: Mar 11, 2022
 *      Author: asus
 */

#ifndef INC_PC_CMV_H_
#define INC_PC_CMV_H_



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


}PC_CMV_Mode_Parameter;


typedef struct
{
	uint32_t _I_TIMER_HOLD;
	uint32_t _E_TIMER_HOLD;
	uint16_t  ramp_time;
	float     ramp_time_percentage;




}PC_CMV_Mode_Calculated_Parameter;



typedef struct
{

	int ok;
	int cycle_done;
	int Pip_Acheived_Flag;

}PC_CMV_Mode_Flags_Set;


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


}PC_CMV_Mode_DAC_Control;








PC_CMV_Mode_Parameter S1;
PC_CMV_Mode_Calculated_Parameter S2;
PC_CMV_Mode_Flags_Set S3;
PC_CMV_Mode_DAC_Control S4;


void PC_CMV_Task (void *argument);
void PC_CMV_PID_Task(void *argument);
void PC_CMV_Pulse_I_Parameter();
void PC_CMV_Pulse_E_Parameter();
void DAC_Value_Correction();
void Ending_Dac_value_correction();
void pip_value_correction();
void Pip_Acheived_Early();
void Pip_Acheived_Slowly();
void Pip_Not_Acheived();
void Pip_Acheived_Normally();
void Peep_E_Valve_Lock_delay_Pc_cmv();
void adjust_servo();

#endif /* INC_PC_CMV_H_ */

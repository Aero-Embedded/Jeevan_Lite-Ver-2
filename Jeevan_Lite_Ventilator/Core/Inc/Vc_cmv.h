/*
 * Vc_cmv.h
 *
 *  Created on: Mar 14, 2022
 *      Author: asus
 */

#ifndef INC_VC_CMV_H_
#define INC_VC_CMV_H_

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

}VC_CMV_Mode_Parameter;



typedef struct
{
	uint32_t _I_TIMER_HOLD;
	uint32_t _E_TIMER_HOLD;
}VC_CMV_Mode_Calculated_Parameter;


typedef struct
{
	int max_flow_acheived;
	int constant_dac_done;
	int Reached_flow_val;
	int sensordata_done;
	int Volume_acheived;
	int volume_reached;

}VC_CMV_Mode_Flags_Set;


typedef struct
{
	int F_max;
	uint16_t temp_dac;
	uint16_t temp_dac_new;
	uint16_t _DAC_VAL0;
	int PID_task_delay;


}VC_CMV_Mode_DAC_Control;

typedef struct
{
	int peep_process_done;
	int lock;
	uint16_t Lock_delay;


}VC_CMV_Mode_Common_Parameter;




VC_CMV_Mode_Parameter V1;
VC_CMV_Mode_Calculated_Parameter V2;
VC_CMV_Mode_Flags_Set V3;
VC_CMV_Mode_DAC_Control V4;
VC_CMV_Mode_Common_Parameter V5;




void Vc_Cmv_Task (void *argument);
void Vc_cmv_PID_Task (void *argument);
void VC_CMV_Pulse_I_Parameter();
void VC_CMV_Pulse_E_Parameter();
void volume_task();
void Peep_E_Valve_Lock_delay_Vc_cmv();

#endif /* INC_VC_CMV_H_ */

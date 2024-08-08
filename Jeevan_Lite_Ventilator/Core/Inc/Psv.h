/*
 * Psv.h
 *
 *  Created on: Mar 18, 2022
 *      Author: asus
 */

#ifndef INC_PSV_H_
#define INC_PSV_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



typedef struct
{
	uint16_t PIP_PSV_Val;
	uint8_t  PEEP_PSV_Val;
	uint8_t  PEEP_CPAP_Val;
	uint16_t _Apnea_counter_trigger_check_time;
	uint16_t  _TRIG_TYPE;
    uint16_t _TRIG_LMT;
	uint16_t  _TRIG_TIME;
	uint16_t _CALC_TRIG_VAL;
	uint16_t _TOLERANCE_EWAVE;
	uint16_t _TRIG_WINDOW;
	float    simv_trigger_offset;
	float  simv_trigger_offset2;
	uint16_t  PEEP_PSV_DAC_Val;
	uint16_t  PIP_PSV_DAC_Val;
	uint16_t PEEP_CPAP_DAC_Val;
	uint16_t _APNEA_TIME;
	uint16_t _APNEA_COUNTER;
	uint8_t  IPAP_Val;
	uint8_t  EPAP_Val;
	uint8_t  T_HIGH;
	uint16_t  IPAP_DAC_Val;
	uint16_t  EPAP_DAC_Val;
	uint16_t T_HIGH_VAL;
	uint8_t  T_LOW;
	uint16_t  T_LOW_VAL;
	uint8_t  P_HIGH;
	uint8_t  P_LOW;
	uint16_t  P_HIGH_DAC_VAL;
	uint16_t  P_LOW_DAC_VAL;
	uint16_t P_HIGH_TIMER;
	uint16_t P_LOW_TIMER;
	uint8_t Apnea_Mode;

}PSV_Mode_Parameters;





PSV_Mode_Parameters P1;

void PSV_Mode_Task (void *argument);
void PSV_PID_Task (void *argument);
uint16_t call_PSV_DAC_Value(int);
void Dac_Value_Boost(void *argument);

#endif /* INC_PSV_H_ */

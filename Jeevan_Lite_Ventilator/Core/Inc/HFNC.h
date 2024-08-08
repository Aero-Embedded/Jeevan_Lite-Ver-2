/*
 * HFNC.h
 *
 *  Created on: Oct 15, 2022
 *      Author: asus
 */

#ifndef INC_HFNC_H_
#define INC_HFNC_H_



#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



typedef struct
{
	uint16_t _Flow_Rate;
	uint8_t _FIO2_Val;

}HFNC_Mode_Parameter;


void HFNC_Task (void *argument);
void Suction_Task(void *argument);

HFNC_Mode_Parameter H1;


#endif /* INC_HFNC_H_ */

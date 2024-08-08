/*
 * APRV.h
 *
 *  Created on: Apr 1, 2022
 *      Author: asus
 */

#ifndef INC_APRV_H_
#define INC_APRV_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"


void APRV_Mode_Task (void *argument);
void APRV_Mode_One_Time_Task (void *argument);
void Peep_E_Valve_Lock_delay_APRV();

#endif /* INC_APRV_H_ */

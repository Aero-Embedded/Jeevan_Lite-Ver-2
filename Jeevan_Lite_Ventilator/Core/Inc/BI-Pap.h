/*
 * BI-Pap.h
 *
 *  Created on: Mar 31, 2022
 *      Author: asus
 */

#ifndef INC_BI_PAP_H_
#define INC_BI_PAP_H_


#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"



void BIPAP_Mode_Task(void *argument);
void BIPAP_PID_Mode_Task (void *argument);



#endif /* INC_BI_PAP_H_ */

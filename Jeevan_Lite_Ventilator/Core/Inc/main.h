/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "Pc_cmv.h"
#include "Vc_cmv.h"
#include "Pressure_Sensors_Data.h"
#include "Flow_Sensors_Data.h"
#include "Typedef.h"
#include "Uart.h"
#include "Pc_SIMV.h"
#include "Vc_SIMV.h"
#include "Psv.h"
#include "Cpap.h"
#include "Back_UP_PC_CMV.h"
#include "Back_UP_VC_CMV.h"
#include "Oxygen_Blending.h"
#include "Alert.h"
#include "BI-Pap.h"
#include "APRV.h"
#include "Calibration.h"
#include "Service.h"
#include "Nebuliser.h"
#include "pid.h"
#include "HFNC.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


#define Pressure_Sensor_Pin AdcData[2]
#define UART 6
#define Sensor_Read_Wait_Delay 2
#define  Dac_value 100
#define E_TIME_TOLERANCE 30u


#define ExpValve_OPEN()  (GPIOB->ODR&=(~(1<<13)))
#define ExpValve_CLOSE() (GPIOB->ODR|=((1<<13)))

#define Nebuliser_OFF()  (GPIOB->ODR&=(~(1<<15)))
#define Nebuliser_ON() (GPIOB->ODR|=((1<<15)))

#define Tab_Charger_OFF()  (GPIOD->ODR&=(~(1<<15)))
#define Tab_Charger_ON() (GPIOD->ODR|=((1<<15)))

#define Blower_ON()  GPIOE->ODR|=(1<<5)
#define Blower_OFF()  GPIOE->ODR&=(~(1<<5))

#define Servo_Power_ON() (GPIOB->ODR|=((1<<12)))
#define Servo_Power_OFF() (GPIOB->ODR&=(~(1<<12)))

#define Power_Led_ON()  GPIOE->ODR|=(1<<1)
#define Uart_Receive_Debug_Toggle_Led()  GPIOD->ODR^=(1<<1)

#define Blower_Signal(X)  DAC1->DHR12R1=X;
#define Parkar_valve_Signal(Y)  DAC1->DHR12R2=Y;
#define Servo_Angle(Z)  TIM12->CCR1=Z;


#define Red_Led_ON()  GPIOD->ODR|=(1<<4)
#define Red_Led_OFF()  GPIOD->ODR&=(~(1<<4))
#define Blue_Led_ON()  GPIOD->ODR|=(1<<3)
#define Blue_Led_OFF()  GPIOD->ODR&=(~(1<<3))
#define Green_Led_ON()  GPIOD->ODR|=(1<<5)
#define Green_Led_OFF()  GPIOD->ODR&=(~(1<<5))
#define Buzzer1_ON()  GPIOD->ODR|=(1<<6)
#define Buzzer1_OFF()  GPIOD->ODR&=(~(1<<6))
#define Buzzer2_ON()  GPIOD->ODR|=(1<<7)
#define Buzzer2_OFF()  GPIOD->ODR&=(~(1<<7))

#define SET_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 1 )
#define CLEAR_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 0 )
#define _ALERT_RESPONSE_PKT_length 8

SemaphoreHandle_t  binarysem;
xQueueHandle Uart_Receive;
xTaskHandle One_Time_Handler;

xTaskHandle pc_mode_Handler;
xTaskHandle Pc_cmv_Pid_Handler;


xTaskHandle Vc_mode_Handler;
xTaskHandle Vc_cmv_Pid_Handler;


xTaskHandle Pc_simv_Mode_Handler;
xTaskHandle Pc_simv_Mode_Pid_Handler;

xTaskHandle Vc_simv_mode_Handler;
xTaskHandle Vc_simv_Pid_Handler;

xTaskHandle Psv_Handler;
xTaskHandle Psv_Pid_Handler;

xTaskHandle Back_Up_PC_CMV_Mode_Handler;
xTaskHandle PID_Back_Up_PC_CMV_Mode_Handler;


xTaskHandle Back_Up_VC_CMV_Mode_Handler;
xTaskHandle PID_Back_Up_VC_CMV_Mode_Handler;

xTaskHandle Cpap_Handler;

xTaskHandle BiPap_Handler;
xTaskHandle BiPap_Pid_Handler;

xTaskHandle APRV_Handler;
xTaskHandle APRV_one_Handler;

xTaskHandle Oxygen_Handler;
xTaskHandle alert_Handler;
xTaskHandle Calibration_Handler;
xTaskHandle Service_Handler;
xTaskHandle Nebuliser_Handler;

xTaskHandle HFNC_Handler;
xTaskHandle Suction_Handler;
xTaskHandle Shutdown_Handler;

//xSemaphoreHandle binarysem;

void One_Time_Task (void *argument);


UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DAC_HandleTypeDef hdac;

uint8_t UART_RX_BUF[17];
uint16_t AdcData[5];
uint8_t  _Control_Byte;
uint32_t _E_TIMER;
uint32_t _I_TIMER;
uint16_t _I_TIMER_ACHEIVED;
uint16_t _E_TIMER_ACHEIVED;
uint16_t check_dev;
int check_count,vol_check_count;
uint16_t Acheived_Volume;
int Ach_vol;
uint16_t battery_raw_value;
float battery1;
uint8_t battery;
uint8_t battery_new,battery_old;

int Trigger;
int Trigger_Count;
int now_update;
uint16_t PIP_PSV_DAC_Val;
uint16_t PEEP_CPAP_DAC_Val;
uint16_t IPAP_EPAP_DAC_Val;
uint16_t P_HIGH_LOW_DAC_Val;
int V_max;

float Bat_Avg_val,Bat_Avg_count,Bat_Avg;
int Mode_data_error;
uint8_t UART_RX_BUF_CURR[17];
uint8_t Mode_Not_Start;

uint8_t Trigger_Flag,Trigger_Flag2;
uint8_t cpap_volume_flag_set;






double Temp, PIDOut, TempSetpoint,Temp_Flow;





/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

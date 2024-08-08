/*
 * Uart.c
 *
 *  Created on: Mar 8, 2022
 *      Author: asus
 */

#include "Uart.h"

uint8_t T=0;
uint16_t Uart_Delay;
uint8_t _RX_CRC8 ;



int Received_APP_data_found,Received_index_Number,Received_index_number2;
extern uint8_t Breath;
extern uint8_t new_Breath;

int arrangecount;

void UART6_Init()
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 57600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
}

void UART5_Init()
{
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 57600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
	   Error_Handler();
	 }
}

void Uart_Transmit_Task (void *argument)
{
	while (1)
	{

		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);

	  if(T==2)
	  {

		if(N._READ_SENSOR_OFFSET == 1)
     	{

		}

		else if(C.Uart_Calib==1)
		{
			SEND_REPORT_PACKET();
		}
		else if(S.Uart_Service==1)
		{
			SEND_SERVICE_PACKET();
		}
		else if(C.Uart_Calib==0 && S.Uart_Service==0)
		{
			if(A.Alert_Now==0)
			{
				UART_Transmit();
			}
			else if(A.Alert_Now==1)
			{
				if(A.Apnea_UART_alert==1)
				{
					SEND_ALERT_PACKET();
					CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_APNEA);
					A.Apnea_UART_alert=0;
				}


				else if (S1._Mode_Val == 1  || S1._Mode_Val == 2 || S1._Mode_Val == 3 || S1._Mode_Val == 4 || _CurrentBackupMode == VCCMV_BACKUP ||_CurrentBackupMode == PCCMV_BACKUP  || _CurrentMode == 6 || _CurrentMode == 7 || _CurrentMode == 8 ||_CurrentMode == 5)
				{
					SEND_ALERT_PACKET();
					Clear_All_Alert_Bits();
				}

				else if(S1._Mode_Val==0)
				{
					SEND_ALERT_PACKET();
					Clear_All_Alert_Bits();
				}
				A.Alert_Now=0;

			}
		}
	  }
		     else if(T==0)
			 {
				Uart_Delay=4000;
				T=1;
			 }
			 else if(T==1)
			 {
				 T=2;

#if UART==6
		 UART6_Init();
         HAL_UART_Receive_IT(&huart6,(uint8_t *) UART_RX_BUF, sizeof( UART_RX_BUF));
#endif
#if UART==5
         UART5_Init();
         HAL_UART_Receive_IT(&huart5,(uint8_t *) UART_RX_BUF, sizeof( UART_RX_BUF));
#endif
				Uart_Delay=20;
			 }


	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		vTaskDelay(Uart_Delay);
    }
}


void Clear_All_Alert_Bits()
{
	    CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_PIP_HL);
		CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_PIP_YN);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_PEEP_HL);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_PEEP_YN);
		CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_MINT_VOLUME_HL);
		CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_MINT_VOLUME_YN);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_YN);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_YN);
		CLEAR_ALERT_BIT(FOURTH_FRAME_UN,_ALERT_FLOW_SENSOR_REVERSED);
		CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PATIENT_CIRCUIT);
		CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_OXYGEN_SUPPLY);
		CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_HL);
		CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_YN);
		CLEAR_ALERT_BIT(FOURTH_FRAME_UN, _ALERT_LEAK_HL);
		CLEAR_ALERT_BIT(FOURTH_FRAME_UN, _ALERT_BAT_DRAIN_50_PERC);
		CLEAR_ALERT_BIT(EIGHT_FRAME_UN,ALERT);
		CLEAR_ALERT_BIT(EIGHT_FRAME_UN,BKUPMODE);
		CLEAR_ALERT_BIT(EIGHT_FRAME_UN,MODE);
}

uint8_t chksum8(const unsigned char *buff, size_t len)
{
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

void UART_Transmit()
{


	               if(cpap_volume_flag_set==1)
			  		{
	            	   vol.Volume = 0;
			  		   _Control_Byte &= (uint8_t) (~(0x80));
			  			cpap_volume_flag_set=0;
			  		}

	            _CYCLIC_TRANSMIT_PKT._header          = 0x5052 ;
		  		_CYCLIC_TRANSMIT_PKT._length          = sizeof(_CYCLIC_TRANSMIT_PKT)-4 ;
		  		_CYCLIC_TRANSMIT_PKT._Pressure_Val    = Pressure_sensor._Pressure_Val ;
		  		_CYCLIC_TRANSMIT_PKT._Flow_Val        = Flow_Sensor_cal._Flow_Val ;
		  		_CYCLIC_TRANSMIT_PKT._Volume_Val      = vol.Volume_Val ;
		  		_CYCLIC_TRANSMIT_PKT._Control_Byte    = _Control_Byte ;
		  		_CYCLIC_TRANSMIT_PKT._SPO2            = O2.O2_percentage;
		  		_CYCLIC_TRANSMIT_PKT._Heart_BPM       = new_Breath ;
		  		_CYCLIC_TRANSMIT_PKT._CRC8            = chksum8(&_CYCLIC_TRANSMIT_PKT._Pressure_Val,_CYCLIC_TRANSMIT_PKT._length);
#if UART==6
		  	HAL_UART_Transmit_IT(&huart6,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));
#endif
#if UART==5
		  	HAL_UART_Transmit_IT(&huart5,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));
#endif
		  	CDC_Transmit_FS((uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));

}






void COMMAND_HANDLER_GRAPH(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

	       S1._Mode_Val =( 0x0F & (RX_PARAM_CMD_PACKET->_mode) ) ;
	       S1._Pause = (0x10 & (RX_PARAM_CMD_PACKET->_mode))>>4;
	       if(S1._Pause==1)
	       {
	    	   vTaskSuspend(pc_mode_Handler);
	    	   vTaskSuspend(Pc_cmv_Pid_Handler);
	    	   vTaskSuspend(Vc_mode_Handler);
	    	   vTaskSuspend(Vc_cmv_Pid_Handler);
	    	   vTaskSuspend(Pc_simv_Mode_Handler);
	    	   vTaskSuspend(Pc_simv_Mode_Pid_Handler);
	    	   vTaskSuspend(Vc_simv_mode_Handler);
	    	   vTaskSuspend(Vc_simv_Pid_Handler);
	    	   vTaskSuspend(Psv_Handler);
	    	   vTaskSuspend(Psv_Pid_Handler);
	    	   vTaskSuspend(Cpap_Handler);
	    	   vTaskSuspend(BiPap_Handler);
	    	   vTaskSuspend(BiPap_Pid_Handler);
	    	   vTaskSuspend(APRV_Handler);
	    	   vTaskSuspend(APRV_one_Handler);
	    	   vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
	    	   vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
	    	   vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
	    	   vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
	    	   vTaskSuspend(alert_Handler);
	    	   vTaskSuspend(HFNC_Handler);
	    	   vTaskSuspend(Suction_Handler);
	    	   vTaskSuspend(Oxygen_Handler);
	    	   vTaskSuspend(Nebuliser_Handler);
	    	   ExpValve_OPEN();
	    	   Blower_Signal( 0);
	    	   Parkar_valve_Signal(0);
	    	   Nebuliser_OFF();
			   vol.Volume=0;
			   A.Red_Led_Alert=0;
			   Blue_Led_ON();
			   Red_Led_OFF();
			   Green_Led_OFF();
	       }
	       else
	       {
				switch(S1._Mode_Val)
				{
					case 1:
						PC_CMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 2:
						VC_CMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 3:
						PC_SIMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 4:
						VC_SIMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 5:
						APRV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 6:
						PSV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 7:
						CPAP_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 8:
						BIPAP_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 10:
						BACKUP_PC_CMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 11:
						BACKUP_VC_CMV_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 12:
						HFNC_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					case 13:
						SUCTION_PARAMETERS((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
						break;
					default:
						break;
				}
	       }


}

void COMMAND_HANDLER(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

	Uart_Receive_Debug_Toggle_Led();
	_RX_CRC8 = chksum8(&UART_RX_BUF[3],((((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_length)));
	if(_RX_CRC8 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_CRC8)
	{
		Mode_data_error=0;
		if ((0x5053 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
		{
			 Mode_Not_Start=1;
			 COMMAND_HANDLER_GRAPH((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
		}
		else if((0x5054 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
		{
			COMMAND_HANDLER_ALERT((ALERT_RANGE_PACKET*) (UART_RX_BUF));
		}
		else if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
		{
			CALIBRATION_COMMAND_HANDLER((REQUEST_CALIBRATION_PACKET_tst*) (UART_RX_BUF));
		}
		else if((0x5055 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
		{
			SERVICE_COMMAND_HANDLER((REQUEST_SERVICE_PACKET_tst*) (UART_RX_BUF));
		}
		else if((0x5057 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
		{
			NEBULISER_COMMAND_HANDLER((NEBULISER_RANGE_PACKET*) (UART_RX_BUF));
		}


	}

	else
	{
		Mode_data_error=1;
		alter_uart_data();


		        if ((0x5053 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					 COMMAND_HANDLER_GRAPH((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
					 Mode_data_error=0;
				}
				else if((0x5054 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					COMMAND_HANDLER_ALERT((ALERT_RANGE_PACKET*) (UART_RX_BUF));
					Mode_data_error=0;
				}
				else if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					CALIBRATION_COMMAND_HANDLER((REQUEST_CALIBRATION_PACKET_tst*) (UART_RX_BUF));
					Mode_data_error=0;
				}
				else if((0x5055 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					SERVICE_COMMAND_HANDLER((REQUEST_SERVICE_PACKET_tst*) (UART_RX_BUF));
					Mode_data_error=0;
				}
				else if((0x5057 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					NEBULISER_COMMAND_HANDLER((NEBULISER_RANGE_PACKET*) (UART_RX_BUF));
					Mode_data_error=0;
				}

	}

}




void alter_uart_data()
{

	Received_APP_data_found=0;
	Received_index_Number=0;


	for(arrangecount=0;arrangecount<=16;arrangecount++)
	{
		if(UART_RX_BUF[arrangecount]==83)
		{
			if(UART_RX_BUF[arrangecount+1]==80)
			{
				Received_index_Number=arrangecount;
				Received_APP_data_found=1;
				break;
			}
		}
	}

	if(Received_APP_data_found==1)
	{
		for(arrangecount=0;arrangecount<=16;arrangecount++)
		{
			Received_index_number2  = Received_index_Number + arrangecount;

			if(Received_index_number2>16)
			{
				UART_RX_BUF_CURR[arrangecount]=UART_RX_BUF[Received_index_number2-17];
			}
			else
			{
				UART_RX_BUF_CURR[arrangecount]=UART_RX_BUF[Received_index_number2];
			}

		}


		for(arrangecount=0;arrangecount<=16;arrangecount++)
		{
			UART_RX_BUF[arrangecount]  =  UART_RX_BUF_CURR[arrangecount];
		}
	}

}


void Receiver_Task (void *argument)
{
	while (1)
	{

		if (xQueueReceive(Uart_Receive, &UART_RX_BUF, portMAX_DELAY) == pdTRUE)
		{
			COMMAND_HANDLER((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));

		}



   }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if UART==6
	HAL_UART_Receive_IT(&huart6,(uint8_t *) UART_RX_BUF, sizeof( UART_RX_BUF));
#endif
#if UART==5
	HAL_UART_Receive_IT(&huart5,(uint8_t *) UART_RX_BUF, sizeof( UART_RX_BUF));
#endif
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToFrontFromISR(Uart_Receive, &UART_RX_BUF, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}





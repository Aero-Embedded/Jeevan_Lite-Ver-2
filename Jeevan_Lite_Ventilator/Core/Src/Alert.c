/*
 * Alert.c
 *
 *  Created on: Mar 30, 2022
 *      Author: asus
 */


#include "Alert.h"



int Apnea_Patient_circuit_disconnected_Flag=0;
int RR_Count=0;
float Total_Tidal_volume=0;
int volume_alert_check=0;
int Total_Tidal_volume_int=0;

void COMMAND_HANDLER_ALERT(ALERT_RANGE_PACKET *RX_ALERT_RANGE_PACKET)
{


	AR._RANGE_MODE_Val = RX_ALERT_RANGE_PACKET->_RANGE_MODE;
	AR._RANGE_VT_MIN_Val= (RX_ALERT_RANGE_PACKET->_RANGE_VT_MIN)*10;
	AR._RANGE_VT_MAX_Val= (RX_ALERT_RANGE_PACKET->_RANGE_VT_MAX)*10;
	AR._RANGE_PIP_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_PIP_MIN;
	AR._RANGE_PIP_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_PIP_MAX;
	AR._RANGE_RR_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_RR_MIN;
	AR._RANGE_RR_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_RR_MAX;
	AR._RANGE_MINT_VOL_MIN_Val=(RX_ALERT_RANGE_PACKET->_RANGE_MINT_VOL_MIN);
	AR._RANGE_MINT_VOL_MAX_Val=(RX_ALERT_RANGE_PACKET->_RANGE_MINT_VOL_MAX);
	AR._RANGE_SPO2_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_SPO2_MIN;
	AR._RANGE_SPO2_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_SPO2_MAX;
	AR._RANGE_PULSE_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_PULSE_MIN;
	AR._RANGE_PULSE_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_PULSE_MAX;

}



void Battery_Information()
{
			Bat_Avg_val = Bat_Avg / Bat_Avg_count;
	        battery_new = Bat_Avg_val;

	    if( Mode_Not_Start!=0)
	    {
	        if(battery_new>battery_old)
	        {
	        	battery=battery_old;
	        }
	        else
	        {
	        	battery=battery_new;
	        }
	    }
	    else
	    {
	    	battery=battery_new;
	    }
	        battery_old=battery_new;
			Bat_Avg = 2;
			Bat_Avg_count = 1;
}

void SEND_ALERT_PACKET()
{

	     data_request_Check();
	     if( Mode_Not_Start!=0)
	     {
	    	 Battery_Information();
	     }
	     if(Trigger_Flag==1 || Trigger_Flag2==1)
	     {
	    	 Clear_All_Alert_Bits();
	     }


	    _ALERT_RESPONSE_PKT.SEVENTH_FRAME_UN.FRAMEBits._ALERT_BATTERY_PERCENTAGE = battery ;
		_ALERT_RESPONSE_PKT._header=0x5054;
		_ALERT_RESPONSE_PKT._length=8;
		_ALERT_RESPONSE_PKT._CRC8   = chksum8((unsigned char*)&_ALERT_RESPONSE_PKT.FIRST_FRAME_UN.FIRST_BYTES,_ALERT_RESPONSE_PKT_length);
#if UART==6
		HAL_UART_Transmit(&huart6,(uint8_t*)&_ALERT_RESPONSE_PKT,sizeof(_ALERT_RESPONSE_PKT),300);
#endif
#if UART==5
		HAL_UART_Transmit_IT(&huart5,(uint8_t*)&_ALERT_RESPONSE_PKT,sizeof(_ALERT_RESPONSE_PKT));
#endif

		CDC_Transmit_FS((uint8_t*)&_ALERT_RESPONSE_PKT,sizeof(_ALERT_RESPONSE_PKT));
}
void Alert_Task (void *argument)
{
	while(1)
	{

		if (S1._Mode_Val == 1 || S1._Mode_Val == 2 || S1._Mode_Val == 3 || S1._Mode_Val == 4 || _CurrentBackupMode == VCCMV_BACKUP ||_CurrentBackupMode == PCCMV_BACKUP ||    (R1.Assist_mode2==0 &&Trigger_Flag2==0 )  || (T1.Assist_mode==0 &&Trigger_Flag==0 ))
		{
			Alert_Func();
		}
		/*if(_CurrentMode == 6 || _CurrentMode == 7 || _CurrentMode == 8 || _CurrentMode==5)
		{
			Main_Supply_or_Battery_Indication();
		}*/



		vTaskDelay(2);
	}
}



void Alert_Func()
{
	Battery_Alert_Func();

	if (_CurrentComputationState == Compute_E_Wave)
	{
		if (_E_TIMER < 500)
		{
			Main_Supply_or_Battery_Indication();
			if (A.Alert_check_done == 1)
			{
				if(Trigger_Flag==0 && Trigger_Flag2==0)
				{
					Pip_Alert_Func();
					Peep_Alert_Func();
					Minite_Volume_Alert_Func();
					Tidal_Volume_Alert_Func();
					Respiratory_Rate_Alert_Func();
					Oxygen_Alert_Func();
					Leak_Alert_Func();
					A.Alert_check_done = 0;
				}
			}
		}
	}
	if (_CurrentComputationState == Compute_I_Wave)
	{
		if(A.Wait_4_cycle<=1)
		{
			if(Trigger_Flag==0 && Trigger_Flag2==0)
			{
				Patient_Circuit_Disconnected_Alert_Func();
				Proximal_Flow_Sensor_Reverse_Direction();
			}

		}
	}
}



void Battery_Alert_Func()
{
	if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))== 0)
	{
		if(S1._Mode_Val!=0)
		{
		   if(A.Wait_4_cycle<=1)
		   {
			if(battery_raw_value<2250)
			{
				SET_ALERT_BIT(FIRST_FRAME_UN, _ALERT_BAT_DRAIN);
				//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
				//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
				A.Alert=1;
				vTaskDelay(500);
			}
			else
			{
				CLEAR_ALERT_BIT(FIRST_FRAME_UN, _ALERT_BAT_DRAIN);
				//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,1);
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,0);
			}


			if(battery<50)
			{
				SET_ALERT_BIT(FOURTH_FRAME_UN, _ALERT_BAT_DRAIN_50_PERC);
			}
		   }
		}
	}
}

void Main_Supply_or_Battery_Indication()
{
	if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))== 0)
	{
		SET_ALERT_BIT(FIRST_FRAME_UN, _ALERT_POWER);
	}
	else
	{
		CLEAR_ALERT_BIT(FIRST_FRAME_UN, _ALERT_POWER);
	}
}

void Pip_Alert_Func()
{
				if (S5.P_Max > AR._RANGE_PIP_MIN_Val && S5.P_Max < AR._RANGE_PIP_MAX_Val)
				{
					CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_PIP_YN);
					A.Pip_Alert = 0;
				}
				else
				{
					A.Pip_Alert++;
					if (A.Pip_Alert > 3)
					{
						A.Alert = 1;
						SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_PIP_YN);
						if (AR._RANGE_PIP_MIN_Val > S5.P_Max)
						{
							CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_PIP_HL);
							A.Pip_Alert = 0;
						}
						else if (AR._RANGE_PIP_MAX_Val < S5.P_Max)
						{
							SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_PIP_HL);
							A.Pip_Alert = 0;
						}
					}
				}
}

void Peep_Alert_Func()
{

		if (S5._Peep_Avg_val_int < (A.PEEP_VAL+3)&& S5._Peep_Avg_val_int > (A.PEEP_VAL-3) )
		{
			CLEAR_ALERT_BIT(THIRD_FRAME_UN, _ALERT_PEEP_YN);
			A.Peep_Alert=0;
		}
		else
		{
			A.Peep_Alert++;
			if(A.Peep_Alert>3)
			{
				A.Alert=1;
				SET_ALERT_BIT(THIRD_FRAME_UN, _ALERT_PEEP_YN);
				if ((A.PEEP_VAL+3) > S5._Peep_Avg_val_int)
				{

					CLEAR_ALERT_BIT(THIRD_FRAME_UN, _ALERT_PEEP_HL);
					A.Peep_Alert=0;

				}
				else if ((A.PEEP_VAL-3) < S5._Peep_Avg_val_int)
				{
					SET_ALERT_BIT(THIRD_FRAME_UN, _ALERT_PEEP_HL);
					A.Peep_Alert=0;
				}
			}

		}

}

void Minite_Volume_Alert_Func()
{


	/*A._Max_Volume_Val_Ml = ((A._Max_Volume_Val * A.Respiratory_Rate) / 100);

	if (A._Max_Volume_Val_Ml > AR._RANGE_MINT_VOL_MIN_Val&& A._Max_Volume_Val_Ml < AR._RANGE_MINT_VOL_MAX_Val)
	{
		CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_MINT_VOLUME_YN);
		A.Mint_Vol_Alert=0;
	}
	else
	{
		A.Mint_Vol_Alert++;
		if(A.Mint_Vol_Alert>5)
		{
			A.Alert=1;
			SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_MINT_VOLUME_YN);
			if (AR._RANGE_MINT_VOL_MIN_Val > A._Max_Volume_Val_Ml)
			{
				CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_MINT_VOLUME_HL);
				A.Mint_Vol_Alert=0;
			}
			else if (AR._RANGE_MINT_VOL_MAX_Val < A._Max_Volume_Val_Ml)
			{
				SET_ALERT_BIT(SECOND_FRAME_UN,_ALERT_MINT_VOLUME_HL);
				A.Mint_Vol_Alert=0;
			}
		}

	}*/


	   if( volume_alert_check==1)
	   {

		   Total_Tidal_volume_int= (Total_Tidal_volume / 1000);

		if (Total_Tidal_volume_int > AR._RANGE_MINT_VOL_MIN_Val&& Total_Tidal_volume_int < AR._RANGE_MINT_VOL_MAX_Val)
		{
			CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_MINT_VOLUME_YN);
			volume_alert_check=0;
		}
		else
		{

				A.Alert=1;
				SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_MINT_VOLUME_YN);
				if (AR._RANGE_MINT_VOL_MIN_Val > Total_Tidal_volume_int)
				{
					CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_MINT_VOLUME_HL);
					volume_alert_check=0;
				}
				else if (AR._RANGE_MINT_VOL_MAX_Val < Total_Tidal_volume_int)
				{
					SET_ALERT_BIT(SECOND_FRAME_UN,_ALERT_MINT_VOLUME_HL);
					volume_alert_check=0;
				}
			}

		}

}


void Tidal_Volume_Alert_Func()
{
	if(V_max >AR._RANGE_VT_MIN_Val&&V_max<AR._RANGE_VT_MAX_Val)
	{
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_YN);
		A.Tidal_Volume_Alert=0;
	}
	else
	{
		A.Tidal_Volume_Alert++;
		if(A.Tidal_Volume_Alert>3)
		{
			A.Alert=1;
			SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_YN);
			if(AR._RANGE_VT_MIN_Val>V_max)
			{
				CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);
				A.Tidal_Volume_Alert=0;
			}
			else if(AR._RANGE_VT_MAX_Val<V_max)
			{
				SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);
				A.Tidal_Volume_Alert=0;
			}
		}

	  }
}


void Respiratory_Rate_Alert_Func()
{
	A.Acheived_RR=(60000/(A.RR_E_TIME+A.RR_I_TIME));

	if(A.Acheived_RR >AR._RANGE_RR_MIN_Val&&A.Acheived_RR<AR._RANGE_RR_MAX_Val)
	{
		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_YN);
	}
	else
	{
		if(A.Wait_4_cycle<=1)
		{
			A.Alert=1;
			SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_YN);
			if(AR._RANGE_RR_MIN_Val>A.Acheived_RR)
			{
				CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);
			}
			else if(AR._RANGE_RR_MAX_Val<A.Acheived_RR)
			{
				SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);
			}
		}
	}
}




void Oxygen_Alert_Func()
{
	if(O2._FIO2_Val>21)
	{
		if(O2.O2_percentage <=22)
		{
			A.Fio2_Supply_Error++;
		    if(A.Fio2_Supply_Error>5)
		    {
		    	  A.Alert=1;
		    	  SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_OXYGEN_SUPPLY);
		    	  A.Fio2_Supply_Error=0;
		    }
		 }
		else
		{
			A.Fio2_Supply_Error=0;
		}





		    	     if((O2.O2_percentage < (O2._FIO2_Val +5))&& (O2.O2_percentage > (O2._FIO2_Val-5)))
		    	     {
		    	     	CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_YN);
		    	     	A.Oxygen_Alert=0;
		    	     }
		    	     else
		    	     {
		    	    	if(A.Fio2_Value_Set<=1)
		    	    	{
		    	    	A.Oxygen_Alert++;
		    	     	if(A.Oxygen_Alert>3)
		    	     	{
		    	     		A.Alert=1;
		    	     		SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_YN);
		    	     		if (O2.O2_percentage > (O2._FIO2_Val +5))
		    	     		{
		    	     			SET_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_HL);
		    	     			A.Oxygen_Alert=0;
		    	     		}
		    	     		else if (O2.O2_percentage < (O2._FIO2_Val -5))
		    	     		{
		    	     			CLEAR_ALERT_BIT(SECOND_FRAME_UN, _ALERT_OXY_HL);
		    	     			A.Oxygen_Alert=0;
		    	     		}
		    	     	}
		    	    	}
		    	     }

	}

}



void Leak_Alert_Func()
{
	if(0>A.Leak)
	{
		 A.Leak_Error_Count++;
		 if(A.Leak_Error_Count>3)
		 {
		    A.Alert=1;
		    SET_ALERT_BIT(FOURTH_FRAME_UN, _ALERT_LEAK_HL);
		    A.Leak_Error_Count=0;
		  }
	}
}



void Patient_Circuit_Disconnected_Alert_Func()
{
	if(Pressure_sensor._Pressure_Val==0 && (Flow_Sensor_cal._Flow_Val==0 || Flow_Sensor_cal._Flow_Val==-1 || Flow_Sensor_cal._Flow_Val==-2))
	{
		if((DAC1->DHR12R1)>=300)
		{
				A.Patient_Circuit_disconnected++;
				if(A.Patient_Circuit_disconnected>=100)
				{
					Apnea_Patient_circuit_disconnected_Flag=1;
					A.Alert=1;
					SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PATIENT_CIRCUIT);
					A.Patient_Circuit_disconnected=0;
				}
		}
	}
}


void Proximal_Flow_Sensor_Reverse_Direction()
{
	if(_I_TIMER_ACHEIVED<500)
	{
		if((vol.Volume_Val<(-10)))
		{
			A.Proximal_Flow_Sensor_reversed++;
			{
				 if( A.Proximal_Flow_Sensor_reversed>=300)
				 {
				 	  A.Alert=1;
				 	  SET_ALERT_BIT(FOURTH_FRAME_UN,_ALERT_FLOW_SENSOR_REVERSED);
				 }
			}
		}
	}
}



void data_request_Check()
{
	if( AR._RANGE_VT_MIN_Val && AR._RANGE_VT_MAX_Val && AR._RANGE_PIP_MIN_Val &&AR._RANGE_PIP_MAX_Val &&AR._RANGE_RR_MIN_Val &&AR._RANGE_RR_MAX_Val&&AR._RANGE_MINT_VOL_MIN_Val&&AR._RANGE_MINT_VOL_MAX_Val&&AR._RANGE_SPO2_MIN_Val&&AR._RANGE_SPO2_MAX_Val&&AR._RANGE_PULSE_MIN_Val&&AR._RANGE_PULSE_MAX_Val )
	{
		/*if(now_check_alert==0)
		{
			SET_ALERT_BIT(EIGHT_FRAME_UN,ALERT);
			now_check_alert=1;
		}*/
	}

	if(S1._Mode_Val==6 || S1._Mode_Val==7 || S1._Mode_Val==8)
	{
		if(_CurrentBackupMode == IdleState)
		{
			//SET_ALERT_BIT(EIGHT_FRAME_UN,BKUPMODE);
		}
	}
	if(Mode_data_error==1)
	{
		//SET_ALERT_BIT(EIGHT_FRAME_UN,MODE);
	}
}

void Alert_I_Time_Parameter()
{
	A.Alert_Now=1;
	A.RR_E_TIME=_E_TIMER_ACHEIVED;
	if(A.Wait_4_cycle>=1)
	{
		A.Wait_4_cycle--;
	}
	if(A.Fio2_Value_Set>=1)
	{
		A.Fio2_Value_Set--;
	}
	A.Exp_Volume=vol.Volume;
	A.Exp_Volume=A.Insp_Volume-A.Exp_Volume;
	A.Leak=A.Insp_Volume-A.Exp_Volume;

	RR_Count++;


}

void Alert_E_Time_Parameter()
{
	A._Max_Volume_Val=(float)(vol.Volume_Val/10);
    if(RR_Count!=0)
    {
	  A._Max_Tidal_Volume+=V_max;
    }

	if(RR_Count==A.Respiratory_Rate)
	{
		Total_Tidal_volume=A._Max_Tidal_Volume;
		A._Max_Tidal_Volume=0;
		RR_Count=0;
		volume_alert_check=1;
	}

	A.Alert_check_done=1;
	A.RR_I_TIME=_I_TIMER_ACHEIVED;
	A.Insp_Volume=vol.Volume;
	A.Patient_Circuit_disconnected=0;
	A.Proximal_Flow_Sensor_reversed=0;
}

void Alert_Receiving_Parameter()
{
	A.Respiratory_Rate = (((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_RR);
	A.Wait_4_cycle=5;
	A.RR_I_TIME=2000;
	A.RR_E_TIME=2000;
	A.Pip_Alert = 0;
	A.Peep_Alert=0;
	A.Mint_Vol_Alert=0;
	A.Tidal_Volume_Alert=0;
	A.Fio2_Supply_Error=0;
	A.Oxygen_Alert=0;
	A.Leak_Error_Count=0;
	A.Patient_Circuit_disconnected=0;
	A.Proximal_Flow_Sensor_reversed=0;
	A._Max_Tidal_Volume=0;
	V_max=0;
	Total_Tidal_volume_int=0;
	RR_Count=0;
}



void Battery_Status()
{
	if(_CurrentComputationState==Compute_E_Wave || _CurrentMode == PSV ||  _CurrentMode == cPAP ||  _CurrentMode == BiPAP ||  _CurrentMode == APRV )
	{
		battery_raw_value=(AdcData[3]);
		battery1=((battery_raw_value-2250)*100)/(740.0);
		if(battery1>100)
			battery1=100;

		Bat_Avg+=battery1;
		Bat_Avg_count++;
	}



	    if( Mode_Not_Start==0 )
		{
			battery_raw_value=(AdcData[3]);
			battery1=((battery_raw_value-2250)*100)/(740.0);
			if(battery1>100)
				battery1=100;

			Bat_Avg+=battery1;
			Bat_Avg_count++;
			if(Bat_Avg_count>1000)
			{
				Battery_Information();
			}
		}
}

void Led_Alert()
{
	 if(A.Red_Led_Alert==1 && S1._Pause==0 )
	 {
		 Blue_Led_OFF();
		 Red_Led_ON();
		 Green_Led_OFF();
		 A.Alert=0;

	 }
	 else if(A.Red_Led_Alert==0 && S1._Pause==0)
	 {
		 Blue_Led_OFF();
		 Red_Led_OFF();
		 Green_Led_ON();

	 }

}

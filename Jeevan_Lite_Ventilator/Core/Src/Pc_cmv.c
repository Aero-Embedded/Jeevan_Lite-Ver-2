/*
 * Pc_cmv.c
 *
 *  Created on: Mar 11, 2022
 *      Author: asus
 */


#include "Pc_cmv.h"


PC_CMV_Mode_DAC_Control S4={400};


extern uint16_t _60_Seconds;
extern uint8_t now_check_breath;
extern uint8_t Breath;


void PC_CMV_Task (void *argument)
{

	while(1)
	{

		switch (_CurrentComputationState)
		{
				case Compute_I_Wave:
					ExpValve_CLOSE();
					Blower_Signal( S4._DAC_VAL0);
					S5.peep_process_done=0;
					S5.lock=1;
				break;

		      case Compute_E_Wave:
					S4._DAC_VAL0=0;
					Blower_Signal( S4._DAC_VAL0);
							if(S5.peep_process_done==1)
							{
									if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
									{
										Blower_Signal( S4._DAC_VAL0);
									}
									else
									{
										Blower_Signal( S4._DAC_VAL0);
									}
							 }
							if(Pressure_sensor._Pressure_Val<=(S5._Set_Peep))
							{
								if(S5.lock==1)
								{
									S5.lock=0;
									vTaskDelay(S5.Lock_delay);
								}
								S5.peep_process_done=1;
								ExpValve_CLOSE();
							}
							else if(S5.peep_process_done==0)
							{
								ExpValve_OPEN();
							}
				break;
				case NoComputeState:
				break;
				default:
				break;


		}

		vTaskDelay(2);


	}


}



void PC_CMV_Pulse_I_Parameter()
{
	   now_check_breath=1;
	   _Control_Byte &= (uint8_t) (~(0x80));
	   vol.Volume = 0;
	   V_max=0;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		_I_TIMER_ACHEIVED = 0;
		S5.peep_process_done = 0;
		S4._DAC_VAL0 = S4.starting_DAC;
	    S4.Last_DAC = S4._DAC_VAL0;
		S3.ok = 1;
		S3.cycle_done = 1;
		S4.last_result1 = S4.result1;
		Led_Alert();
		DAC_Value_Correction();
		Ending_Dac_value_correction();
		S5.P_Max = 0;
		S3.Pip_Acheived_Flag = 0;
		S5.now_check = 0;
		S5.P_Min = 60;
		S3.cycle_done = 1;
		Peep_E_Valve_Lock_delay_Pc_cmv();
		S4.ten_ms = 0;
		S4.Acheived_ten_ms = 0;
		Alert_I_Time_Parameter();
		_I_TIMER = S2._I_TIMER_HOLD;
		_CurrentWaveFormState = Generate_E_Wave;
		_CurrentComputationState = Compute_I_Wave ;


		vTaskDelay(S2._I_TIMER_HOLD);
}



void PC_CMV_Pulse_E_Parameter()
{

	 _Control_Byte |= (uint8_t) 0x80;
	 _E_TIMER = S2._E_TIMER_HOLD;
	 _E_TIMER_ACHEIVED = 0;
	 S5.lock = 1;
	 Alert_E_Time_Parameter();
	 adjust_servo();
	_CurrentWaveFormState = Generate_I_Wave;
	_CurrentComputationState = Compute_E_Wave ;

	vTaskDelay(S2._E_TIMER_HOLD);

}



void Ending_Dac_value_correction()
{

	if(S1.RR>=30 || S2.ramp_time>80 || (S1.E_Time>4))
    {
		    if(S5.P_Max>(S1._PIP_Val))
			{
		    	S4.pmax_error1++;
		    	S4.pmax_error2=0;
		    	if(S4.pmax_error1>2)
		    	{
		    	S4.Ending_Dac=S4.Ending_Dac-1;
				S4.pmax_error1=0;
					if(S4.Ending_Dac<=500)
						S4.Ending_Dac=500;
		    	}
			}
			else if(S5.P_Max<(S1._PIP_Val))
			{
				S4.pmax_error2++;
				S4.pmax_error1=0;
				if(S4.pmax_error2>2)
				{
				S4.Ending_Dac=S4.Ending_Dac+1;
				S4.pmax_error2=0;
				if(S4.Ending_Dac>=4095)
					 S4.Ending_Dac=4095;
				}
			}
			else
			{
				S4.Ending_Dac=S4.Ending_Dac;
			}
    }

 else if(S1.RR<30 || S2.ramp_time<=80)
 {
	if(S5._Pip_Avg_val_int>S1._PIP_Val)
	{
		S4.Ending_Dac=S4.Ending_Dac-5;
		if(S4.Ending_Dac<=500)
			S4.Ending_Dac=500;
	}
	else if(S5._Pip_Avg_val_int<S1._PIP_Val)
	{
		S4.Ending_Dac=S4.Ending_Dac+5;
		if(S4.Ending_Dac>=4095)
			S4.Ending_Dac=4095;
	}
	else
	{
		S4.Ending_Dac=S4.Ending_Dac;

	}
 }
}

void DAC_Value_Correction()
{
		 if(S4.Acheived_ms < (S2.ramp_time_percentage -40))
	     {
			 Pip_Acheived_Early();

	     }
	     else if(S4.Acheived_ms > (S2.ramp_time_percentage +40))
	     {
	    	  Pip_Acheived_Slowly();
	     }
	     else
	     {
	    	   Pip_Acheived_Normally();
	     }
		 Pip_Not_Acheived();
}

void pip_value_correction()
{
	if (Pressure_sensor._Pressure_Val >= (S1._PIP_Val ))
	{
		S3.cycle_done = 0;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		if(S3.ok==1)
		{
			S4.Acheived_ten_ms = S4.ten_ms;
			S4.Acheived_ms=(S4.Acheived_ten_ms*S4.PID_task_delay);
			S3.ok=0;
			S3.Pip_Acheived_Flag=1;
			S4._DAC_VAL0 = S4.Last_DAC;
		}
	}
	if (S3.cycle_done == 1)
	{
		if (S4.Last_DAC >= S4.Ending_Dac)
		{
			S4.Last_DAC = S4.Ending_Dac;
			S3.cycle_done = 0;
		}
		else if (S4.Last_DAC < S4.Ending_Dac)
		{
			S4.Last_DAC = S4._DAC_VAL0;
			S4._DAC_VAL0=S4.Last_DAC + (S4.incrementing_Dac_value_10ms );
		}
	}

}




void PC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

			S1._PIP_Val = RX_PARAM_CMD_PACKET->_PIP;
			S1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			S1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			S1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			S1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			S1.RT_Value =(0xF0 & RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME)>>4;
			S1.RR=RX_PARAM_CMD_PACKET->_RR;


			S2._I_TIMER_HOLD = (S1.I_Time * (S1.CycleTime / (S1.I_Time +S1.E_Time)));
			S2._E_TIMER_HOLD = (S1.E_Time * (S1.CycleTime / (S1.I_Time + S1.E_Time)));
			S2.ramp_time=(S1.RT_Value*10);
			S2.ramp_time_percentage = ((float)S2.ramp_time/100.00)*(S2._I_TIMER_HOLD) ;

			if(S2._I_TIMER_HOLD > 600)
			{
				if(S2.ramp_time_percentage <600)
				{
					S2.ramp_time_percentage =600;
				}
			}

			S4.Ending_Dac=17.1129 * (S1._PIP_Val) + 587.7390+((1/70)*200);
			S4.incrementing_Dac_value_10ms=70;
			S4.Acheived_ms=(S2.ramp_time_percentage*2);

			S3.Pip_Acheived_Flag=0;
			S4.nack=1;
			S4.PID_task_delay=10;
			S4.result1_error=0;
			S4.last_result1=0;
			S4.PID_task_delay_lock=0;
			S5._Set_Peep=S1._PEEP_Val/2;
			S5.peep_process_done=0;
			S5.lock=1;

			A.PEEP_VAL=S1._PEEP_Val;
			Alert_Receiving_Parameter();

			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._PIP_Val=S1._PIP_Val;
			O2._Pressure_Base=1;
			O2._Flow_Base=0;
			O2_Parameter();
			P1.Apnea_Mode=0;
			S5.Lock_delay=300;


			 _60_Seconds=60000;				//breath
			 now_check_breath=0;			//breath
			 Breath=0;


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

			vTaskSuspend(HFNC_Handler);

			vTaskSuspend(Suction_Handler);

			vTaskResume(pc_mode_Handler);
			vTaskResume(Pc_cmv_Pid_Handler);
			vTaskResume(One_Time_Handler);
			vTaskResume(alert_Handler);

			if(O2._FIO2_Val>21)
			{
				vTaskResume(Oxygen_Handler);
			}
			else if(O2._FIO2_Val<=21)
			{
				O2.O2_DAC=0;
				Parkar_valve_Signal(0);
				vTaskSuspend(Oxygen_Handler);
			}
			_CurrentMode=PCCMV;
			_CurrentBackupMode = IdleState;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_E_Wave ;




}



void PC_CMV_PID_Task(void *argument)
{

	while(1)
	{

		S4.ten_ms++;
			if(S1._Mode_Val == 1)
			 {
				if(_CurrentComputationState==Compute_I_Wave)
				{
					if(O2._FIO2_Val==100)
					{
						S4._DAC_VAL0=400;
					}
					else
					{
						pip_value_correction();
					}

				}
			 }


		vTaskDelay(S4.PID_task_delay);

	}

}




void Pip_Acheived_Early()
{

		     if(S3.Pip_Acheived_Flag==0)
			 {

			 }
			 else
			 {

			    S4.result1=(S4.Acheived_ms/S2.ramp_time_percentage)*100;
			    if(S4.result1<=70)
			    {
			    	S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms-5;
			    	if(S4.incrementing_Dac_value_10ms<=70)
			    	{
			    		S4.incrementing_Dac_value_10ms=70;
			    		S4.nack=0;
			    	}
			    }

			    else if(S4.result1<=80 && S4.result1>70)
			    {
			    	S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms-(0.5);
			    	if(S4.incrementing_Dac_value_10ms<=70)
			    	{
			    		S4.incrementing_Dac_value_10ms=70;
			    		S4.nack=0;
			    	}
			    }
			    else if(S4.result1<=90 && S4.result1>80)
			    {
			    	S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms-(0.1);
			    	if(S4.incrementing_Dac_value_10ms<=70)
			    	{
			    		S4.incrementing_Dac_value_10ms=70;
			    		S4.nack=0;
			    	}
			    }


			    if(S4.nack==0)
			    {

			    	if(S5.P_Max<S1._PIP_Val)
			    	{
			    		 S4.result1_error++;
			    		 if(S4.result1_error>=5)
			    		 {
			    			S4.PID_task_delay=S4.PID_task_delay;
			    			S4.result1_error=6;
			    		    S4.PID_task_delay_lock=1;
			    		 }
			    	}
			    	else if(S4.result1>S4.last_result1)
			    	{
			    		if(S4.PID_task_delay_lock==0)
			    		{
			    		  if(S4.result1<60)
			    		  {
			    			  S4.PID_task_delay=S4.PID_task_delay+10;
			    		  }
			    		  else if(S4.result1<70 && S4.result1>=60)
			    		  {
			    			  S4.PID_task_delay=S4.PID_task_delay+6;
			    		  }
			    		  else if(S4.result1<=80 && S4.result1>=70)
			    		  {
			    			  S4.PID_task_delay=S4.PID_task_delay+4;
			    		  }
			    		  else
			    		  {
			    			  S4.PID_task_delay=S4.PID_task_delay+1;
			    		  }
			    		}

			    	}

			    }
			 }

}



void Pip_Acheived_Slowly()
{
	               S4.result2=(S4.Acheived_ms/S2.ramp_time_percentage)*100;
		    	   S4.last_result2=S4.result2;

		    	   if(S4.result2 >= 130)
		    	   {
		    		    S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms+5;
		    	   	    if(S4.incrementing_Dac_value_10ms>=350)
		    	   	    {
		    	   	    	 S4.incrementing_Dac_value_10ms=350;
		    	   	    	 S4.nack=0;
		    	   	    }
		    	   }


		    	   else if(S4.result2 > 120 && S4.result2 <130)
		    	   {
		    		    S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms+(0.5);
		    	   		if(S4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			  S4.incrementing_Dac_value_10ms=350;
		    	   			  S4.nack=0;
		    	   		}
		    	   	}
		    	   	else if(S4.result2<=120 && S4.result2>=110)
		    	   	{
		    	   		S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms+(0.1);
		    	   		if(S4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			   S4.incrementing_Dac_value_10ms=350;
		    	   			   S4.nack=0;
		    	   		}
		    	   	}


}
void Pip_Acheived_Normally()
{
 	   S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms;
}


void Pip_Not_Acheived()
{
		   if(S3.Pip_Acheived_Flag==0)
		   {
			   S4.incrementing_Dac_value_10ms=S4.incrementing_Dac_value_10ms+1;
			   if(S4.incrementing_Dac_value_10ms>=350)
			   {
				   S4.incrementing_Dac_value_10ms=350;
				   S4.nack=0;
			   }
		   }
}




void Peep_E_Valve_Lock_delay_Pc_cmv()
{


	// added one extra if condition and Remove error count value 2 to 0 in below RR value 30.

	if(S1.RR<30)
	{

		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (S1.RR >= 30 || S2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;



		if (S5._Peep_Avg_val_int > (S1._PEEP_Val +8) )
		{

			S5.error_count++;
			if (S5.error_count > 0)
			{

				S5.Lock_delay = S5.Lock_delay + 100;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}



		else if( (S5._Peep_Avg_val_int > (S1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (S1._PEEP_Val+8)) )
		{

			S5.error_count++;
			if (S5.error_count > 0)
			{
				S5.Lock_delay = S5.Lock_delay + 40;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}


		else if( (S5._Peep_Avg_val_int > (S1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (S1._PEEP_Val+2)) )
		{

			S5.error_count++;
			if (S5.error_count > 0)
			{

				S5.Lock_delay = S5.Lock_delay + 10;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}


		else if ((S5._Peep_Avg_val_int == 0 ) )
		{


			S5.error_count2++;
			if (S5.error_count2 > 0)
			{
				S5._Set_Peep = S5._Set_Peep + 1;
				S5.Lock_delay = S5.Lock_delay - 10;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;

				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}



		else if ((S5._Peep_Avg_val_int < (S1._PEEP_Val -8)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 0)
			{
				S5.Lock_delay = S5.Lock_delay - 50;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;
					S5._Set_Peep = S5._Set_Peep + 0.5;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}



		else if ((S5._Peep_Avg_val_int >= (S1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (S1._PEEP_Val-2)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 0)
			{
				S5.Lock_delay = S5.Lock_delay - 30;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;
					S5._Set_Peep = S5._Set_Peep + 0.5;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}


		else if ((S5._Peep_Avg_val_int >= (S1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (S1._PEEP_Val)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 0)
			{

				S5.Lock_delay = S5.Lock_delay - 10;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5._Set_Peep = S5._Set_Peep + 0.5;
					S5.Lock_delay = 1;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}





		else
		{
			S5._Set_Peep = S5._Set_Peep;
		}
		S5._Peep_Avg_count = 0;
		S5._Peep_Avg = 0;

	}
	else
	{
		// added one extra if condition and Remove error count value 2 to 2 in Above RR value 30.

		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (S1.RR >= 30 || S2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;



		if (S5._Peep_Avg_val_int > (S1._PEEP_Val +8) )
		{

			S5.error_count++;
			if (S5.error_count > 2)
			{

				S5.Lock_delay = S5.Lock_delay + 100;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}



		else if( (S5._Peep_Avg_val_int > (S1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (S1._PEEP_Val+8)) )
		{

			S5.error_count++;
			if (S5.error_count > 2)
			{
				S5.Lock_delay = S5.Lock_delay + 40;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}


		else if( (S5._Peep_Avg_val_int > (S1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (S1._PEEP_Val+2)) )
		{

			S5.error_count++;
			if (S5.error_count > 2)
			{

				S5.Lock_delay = S5.Lock_delay + 10;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
				{
					S5.Lock_delay = 700;
					S5._Set_Peep = S5._Set_Peep - 0.5;
				}
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}


		else if ((S5._Peep_Avg_val_int == 0 ) )
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{
				S5._Set_Peep = S5._Set_Peep + 1;
				S5.Lock_delay = S5.Lock_delay - 10;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;

				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}



		else if ((S5._Peep_Avg_val_int < (S1._PEEP_Val -8)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{
				S5.Lock_delay = S5.Lock_delay - 50;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;
					S5._Set_Peep = S5._Set_Peep + 0.5;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}



		else if ((S5._Peep_Avg_val_int >= (S1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (S1._PEEP_Val-2)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{
				S5.Lock_delay = S5.Lock_delay - 30;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5.Lock_delay = 1;
					S5._Set_Peep = S5._Set_Peep + 0.5;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}


		else if ((S5._Peep_Avg_val_int >= (S1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (S1._PEEP_Val)))
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{

				S5.Lock_delay = S5.Lock_delay - 10;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
				{
					S5._Set_Peep = S5._Set_Peep + 0.5;
					S5.Lock_delay = 1;
				}
			}
			if (S5._Set_Peep > S1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}





		else
		{
			S5._Set_Peep = S5._Set_Peep;
		}
		S5._Peep_Avg_count = 0;
		S5._Peep_Avg = 0;
	}
}

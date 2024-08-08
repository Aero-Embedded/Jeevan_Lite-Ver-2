/*
 * Pc_SIMV.c
 *
 *  Created on: Mar 17, 2022
 *      Author: asus
 */

#include "Pc_SIMV.h"


PC_SIMV_Mode_DAC_Control T4={400};

extern uint16_t Dac1;
extern uint16_t Dac2;

extern uint8_t kp;
extern uint8_t ki;
extern double kd;
extern int Time;
extern int Speed;
extern int pressure_acheived;


extern uint16_t _60_Seconds;
extern uint8_t now_check_breath;
extern uint8_t Breath;

int trigger_valve_modification=0;


void PC_SIMV_Task (void *argument)
{

	while(1)
	{

		switch (_CurrentComputationState)
		{
				case Compute_I_Wave:
					ExpValve_CLOSE();
					Blower_Signal( T4._DAC_VAL0);
					S5.peep_process_done=0;
					S5.lock=1;
					pressure_acheived = 1;
				break;

		      case Compute_E_Wave:

					Blower_Signal( T4._DAC_VAL0);
							if(S5.peep_process_done==1)
							{


								if(T1.Assist_mode==0 )
								{
									if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
									{
										Blower_Signal( T4._DAC_VAL0);
									}
									else
									{

										Blower_Signal( T4._DAC_VAL0);
									}
								}
								else if(T1.Assist_mode==1 )
								{
									if(pressure_acheived == 1)
									{
										PID_Compute(&TPID);
										T4._DAC_VAL0=(int16_t)PIDOut;
										Blower_Signal( T4._DAC_VAL0);
									}

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

										if(_E_TIMER <= (T5._TRIG_WINDOW) &&  _E_TIMER != 0)
										{

											pressure_acheived =0;
											T4._DAC_VAL0=0;
											Blower_Signal( T4._DAC_VAL0);

											if(T1.Assist_mode==0 )
											{
												if(T5._TRIG_TYPE==1)
												{
													if((Pressure_sensor._Pressure_Val<(T5.simv_trigger_offset-T5._TRIG_LMT)))
													{
														trigger_valve_modification=1;
														Switch_TASK_I_CYCLE();
													}
												}
												else
												{
													if((Flow_Sensor_cal._Flow_Val>=(T5.simv_trigger_offset2+T5._TRIG_LMT)))
													{
														trigger_valve_modification=1;
														Switch_TASK_I_CYCLE();
													}
												}
											}

											else if(T1.Assist_mode==1 )
											{
												if(T5._TRIG_TYPE==1)
												{
													if((Pressure_sensor._Pressure_Val<(T5.simv_trigger_offset-T5._TRIG_LMT)))
													{
														trigger_valve_modification=1;
														Trigger_Flag=1;
														Switch_TASK_I_CYCLE();
													}
												}
												else
												{
													if((Flow_Sensor_cal._Flow_Val>=(T5.simv_trigger_offset2+T5._TRIG_LMT)))
													{
														trigger_valve_modification=1;
														Trigger_Flag=1;
														Switch_TASK_I_CYCLE();
													}
												}
											}
										}
										else
										{

											  if(Flow_Sensor_cal._Flow_Val==0 || (Flow_Sensor_cal._Flow_Val>=(-8) && Flow_Sensor_cal._Flow_Val<0))
											  {
												  T5.simv_trigger_offset=Pressure_sensor._Pressure_Val;
												  T5.simv_trigger_offset2=Flow_Sensor_cal._Flow_Val;
											  }

										}

										if(  (Pressure_sensor._Pressure_Val > (T1._PEEP_Val))   &&  (S5.peep_process_done==1))
										{
											pressure_acheived=0;
											T4._DAC_VAL0=0;
										    Blower_Signal( T4._DAC_VAL0);
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

void Switch_TASK_I_CYCLE()
{

	vTaskDelete(One_Time_Handler);
	_I_TIMER = 0 ;
	_E_TIMER = 0 ;
	_CurrentWaveFormState = Generate_I_Wave ;
	 xTaskCreate(One_Time_Task, "one-time-task", 256, NULL, 2, &One_Time_Handler);

}

void PC_SIMV_Pulse_I_Parameter()
{




	if(Trigger_Flag==0)
	{

		T5._TOLERANCE_EWAVE = T2._E_TIMER_HOLD - S5.Lock_delay;
		T5._TRIG_WINDOW = T5._TOLERANCE_EWAVE* (((float) T5._TRIG_TIME * 10.00) / 100.00);
		now_check_breath=1;
	    _Control_Byte &= (uint8_t) (~(0x80));
	    vol.Volume = 0;
	    V_max=0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		_I_TIMER_ACHEIVED = 0;
		S5.peep_process_done = 0;

			T4._DAC_VAL0 = T4.starting_DAC;
			T4.Last_DAC = T4._DAC_VAL0;
			T3.ok = 1;
			T3.cycle_done = 1;
			T4.last_result1 = T4.result1;
			Led_Alert();
			DAC_Value_Correction_SIMV();
			Ending_Dac_value_correction_SIMV();
			S5.P_Max = 0;
			T3.Pip_Acheived_Flag = 0;
			S5.now_check = 0;
			S5.P_Min = 60;

		T3.cycle_done = 1;
		Peep_E_Valve_Lock_delay_Pc_SIMV();
		T4.ten_ms = 0;
		T4.Acheived_ten_ms = 0;
		Alert_I_Time_Parameter();
		_I_TIMER = T2._I_TIMER_HOLD;
		_CurrentWaveFormState = Generate_E_Wave;
		_CurrentComputationState = Compute_I_Wave ;
		vTaskDelay(T2._I_TIMER_HOLD);
	}
	else if(Trigger_Flag==1)
	{
		T5._TOLERANCE_EWAVE = T2._E_TIMER_HOLD - S5.Lock_delay;
		T5._TRIG_WINDOW = T5._TOLERANCE_EWAVE* (((float) T5._TRIG_TIME * 10.00) / 100.00);
		now_check_breath=1;
		 _Control_Byte &= (uint8_t) (~(0x80));
		 vol.Volume = 0;
		 V_max=0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		_I_TIMER_ACHEIVED = 0;
		S5.peep_process_done = 0;
		Led_Alert();
		Peep_E_Valve_Lock_delay_Pc_SIMV();
		Alert_I_Time_Parameter();
		_I_TIMER = T2._I_TIMER_HOLD;
		_CurrentWaveFormState = Generate_E_Wave;
		_CurrentComputationState = Compute_I_Wave ;
		vTaskDelay(T2._I_TIMER_HOLD);

	}
}



void PC_SIMV_Pulse_E_Parameter()
{
	if(Trigger_Flag==0)
	{
		_Control_Byte |= (uint8_t) 0x80;
		T4._DAC_VAL0=0;
		_E_TIMER = T2._E_TIMER_HOLD;
		_E_TIMER_ACHEIVED = 0;
		S5.lock = 1;
		Alert_E_Time_Parameter();
		adjust_servo();
		_CurrentWaveFormState = Generate_I_Wave;
		_CurrentComputationState = Compute_E_Wave ;
		vTaskDelay(T2._E_TIMER_HOLD);
	}
	else if(Trigger_Flag==1)
	{
		_Control_Byte |= (uint8_t) 0x80;


		TempSetpoint = T1._PEEP_Val;
		PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
		PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&TPID, Time);
		PID_SetOutputLimits(&TPID, Dac1, Dac2);


		_E_TIMER = T2._E_TIMER_HOLD;
		_E_TIMER_ACHEIVED = 0;
		S5.lock = 1;
		Trigger_Flag=0;
		Alert_E_Time_Parameter();
		adjust_servo();
		_CurrentWaveFormState = Generate_I_Wave;
		_CurrentComputationState = Compute_E_Wave ;
		vTaskDelay(T2._E_TIMER_HOLD);
	}

}


void PC_SIMV_PID_Task(void *argument)
{

	while(1)
	{
		T4.ten_ms++;
			if(S1._Mode_Val == 3)
			 {
				if(_CurrentComputationState==Compute_I_Wave)
				{
					if(O2._FIO2_Val==100)
					{
						T4._DAC_VAL0=400;
					}
					else
					{
					    pip_value_correction_SIMV();
					}
				}
			 }
		vTaskDelay(T4.PID_task_delay);

	}

}


void DAC_Value_Correction_SIMV()
{



		 if(T4.Acheived_ms < (T2.ramp_time_percentage -40))
	     {
			 Pip_Acheived_Early_SIMV();

	     }


	       else if(T4.Acheived_ms > (T2.ramp_time_percentage +40))
	       {
	    	   Pip_Acheived_Slowly_SIMV();
	       }

	       else
	       {
	    	   Pip_Acheived_Normally_SIMV();

	       }

		     Pip_Not_Acheived_SIMV();

}

void pip_value_correction_SIMV()
{
	if (Pressure_sensor._Pressure_Val >= (T1._PIP_Val ))
	{
		T3.cycle_done = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		if(T3.ok==1)
		{
			T4.Acheived_ten_ms = T4.ten_ms;
			T4.Acheived_ms=(T4.Acheived_ten_ms*T4.PID_task_delay);
			T3.ok=0;
			T3.Pip_Acheived_Flag=1;
			T4._DAC_VAL0 = T4.Last_DAC;
		}
	}
	if (T3.cycle_done == 1)
	{
		if (T4.Last_DAC >= T4.Ending_Dac)
		{
			T4.Last_DAC = T4.Ending_Dac;
			T3.cycle_done = 0;
		}
		else if (T4.Last_DAC < T4.Ending_Dac)
		{
			T4.Last_DAC = T4._DAC_VAL0;
			T4._DAC_VAL0=T4.Last_DAC + (T4.incrementing_Dac_value_10ms );
		}
	}

}




void PC_SIMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{


			T1._PIP_Val = RX_PARAM_CMD_PACKET->_PIP;
			T1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			T1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			T1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			T1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			T1.RT_Value =(0xF0 & RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME)>>4;
			T1.RR=RX_PARAM_CMD_PACKET->_RR;



			T5._TRIG_TYPE = (0xF0 & RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT) >> 4;
			T5._TRIG_LMT = 0x0F & (RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT);
			T5._TRIG_TIME = 0x0F & (RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME);

			T1.Assist_mode = 0x0F & (RX_PARAM_CMD_PACKET->_T_HIGH);

			T2._I_TIMER_HOLD = (T1.I_Time * (T1.CycleTime / (T1.I_Time +T1.E_Time)));
			T2._E_TIMER_HOLD = (T1.E_Time * (T1.CycleTime / (T1.I_Time + T1.E_Time)));
			T2.ramp_time=(T1.RT_Value*10);
			T2.ramp_time_percentage = ((float)T2.ramp_time/100.00)*(T2._I_TIMER_HOLD) ;

			if(T2._I_TIMER_HOLD > 600)
			{
				if(T2.ramp_time_percentage <600)
				{
					T2.ramp_time_percentage =600;
				}
			}

			T4.Ending_Dac=17.1129 * (T1._PIP_Val) + 587.7390+((1/70)*200);
			T4.incrementing_Dac_value_10ms=70;
			T4.Acheived_ms=(T2.ramp_time_percentage*2);

			T5._CALC_TRIG_VAL = ((float) E_TIME_TOLERANCE / 100.00)* (T2._E_TIMER_HOLD);
			T5._TOLERANCE_EWAVE = T2._E_TIMER_HOLD - T5._CALC_TRIG_VAL;
			T5._TRIG_WINDOW = T5._TOLERANCE_EWAVE* (((float) T5._TRIG_TIME * 10.00) / 100.00);

			T3.Pip_Acheived_Flag=0;
			T4.nack=1;
			T4.PID_task_delay=10;
			T4.result1_error=0;
			T4.last_result1=0;
			T4.PID_task_delay_lock=0;
			S5._Set_Peep=T1._PEEP_Val/2;

			S5.peep_process_done=0;
			S5.lock=1;

			A.PEEP_VAL=T1._PEEP_Val;
			Alert_Receiving_Parameter();
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._PIP_Val=T1._PIP_Val;
			O2._Pressure_Base=1;
			O2._Flow_Base=0;
			O2_Parameter();
			P1.Apnea_Mode=0;
			S5.Lock_delay=300;

			trigger_valve_modification=0;

			Dac1=300;
			Dac2=1200;

			kp=10;
			ki=10;
			kd=1;
			Time=50;
			Speed=50;

			_60_Seconds=60000;				//breath
			 now_check_breath=0;			//breath
			 Breath=0;

			vTaskSuspend(Vc_mode_Handler);
			vTaskSuspend(Vc_cmv_Pid_Handler);

			vTaskSuspend(pc_mode_Handler);
			vTaskSuspend(Pc_cmv_Pid_Handler);

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

		    vTaskResume(Pc_simv_Mode_Handler);
			vTaskResume(Pc_simv_Mode_Pid_Handler);
			vTaskResume(One_Time_Handler);
			vTaskResume(alert_Handler);

			if(O2._FIO2_Val>21)							// FOR O2
			{
				vTaskResume(Oxygen_Handler);
			}
			else if(O2._FIO2_Val<=21)
			{
				O2.O2_DAC=0;
				Parkar_valve_Signal(0);
				vTaskSuspend(Oxygen_Handler);
			}
			_CurrentMode=SIMVPC;
			_CurrentBackupMode = IdleState;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_E_Wave ;




}







void Ending_Dac_value_correction_SIMV()
{

	if(T1.RR>=30 || T2.ramp_time>80 || (T1.E_Time>4))
    {
		    if(S5.P_Max>(T1._PIP_Val))
			{
		    	T4.pmax_error1++;
		    	T4.pmax_error2=0;
		    	if(T4.pmax_error1>2)
		    	{
		    	T4.Ending_Dac=T4.Ending_Dac-1;
				T4.pmax_error1=0;
					if(T4.Ending_Dac<=500)
						T4.Ending_Dac=500;
		    	}
			}
			else if(S5.P_Max<(T1._PIP_Val))
			{
				T4.pmax_error2++;
				T4.pmax_error1=0;
				if(T4.pmax_error2>2)
				{
				T4.Ending_Dac=T4.Ending_Dac+1;
				T4.pmax_error2=0;
				if(T4.Ending_Dac>=4095)
					 T4.Ending_Dac=4095;
				}
			}
			else
			{
				T4.Ending_Dac=T4.Ending_Dac;
			}
    }

 else if(T1.RR<30 || T2.ramp_time<=80)
 {
	if(S5._Pip_Avg_val_int>T1._PIP_Val)
	{
		T4.Ending_Dac=T4.Ending_Dac-5;
		if(T4.Ending_Dac<=500)
			T4.Ending_Dac=500;
	}
	else if(S5._Pip_Avg_val_int<T1._PIP_Val)
	{
		T4.Ending_Dac=T4.Ending_Dac+5;
		if(T4.Ending_Dac>=4095)
			T4.Ending_Dac=4095;
	}
	else
	{
		T4.Ending_Dac=T4.Ending_Dac;

	}
 }
}


void Pip_Acheived_Early_SIMV()
{

		     if(T3.Pip_Acheived_Flag==0)
			 {

			 }
			 else
			 {

			    T4.result1=(T4.Acheived_ms/T2.ramp_time_percentage)*100;
			    if(T4.result1<=70)
			    {
			    	T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms-5;
			    	if(T4.incrementing_Dac_value_10ms<=70)
			    	{
			    		T4.incrementing_Dac_value_10ms=70;
			    		T4.nack=0;
			    	}
			    }

			    else if(T4.result1<=80 && T4.result1>70)
			    {
			    	T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms-(0.5);
			    	if(T4.incrementing_Dac_value_10ms<=70)
			    	{
			    		T4.incrementing_Dac_value_10ms=70;
			    		T4.nack=0;
			    	}
			    }
			    else if(T4.result1<=90 && T4.result1>80)
			    {
			    	T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms-(0.1);
			    	if(T4.incrementing_Dac_value_10ms<=70)
			    	{
			    		T4.incrementing_Dac_value_10ms=70;
			    		T4.nack=0;
			    	}
			    }


			    if(T4.nack==0)
			    {

			    	if(S5.P_Max<T1._PIP_Val)
			    	{
			    		 T4.result1_error++;
			    		 if(T4.result1_error>=5)
			    		 {
			    			T4.PID_task_delay=T4.PID_task_delay;
			    			T4.result1_error=6;
			    		    T4.PID_task_delay_lock=1;
			    		 }
			    	}
			    	else if(T4.result1>T4.last_result1)
			    	{
			    		if(T4.PID_task_delay_lock==0)
			    		{
			    		  if(T4.result1<60)
			    		  {
			    			  T4.PID_task_delay=T4.PID_task_delay+10;
			    		  }
			    		  else if(T4.result1<70 && T4.result1>=60)
			    		  {
			    			  T4.PID_task_delay=T4.PID_task_delay+6;
			    		  }
			    		  else if(T4.result1<=80 && T4.result1>=70)
			    		  {
			    			  T4.PID_task_delay=T4.PID_task_delay+4;
			    		  }
			    		  else
			    		  {
			    			  T4.PID_task_delay=T4.PID_task_delay+1;
			    		  }
			    		}

			    	}

			    }
			 }

}



void Pip_Acheived_Slowly_SIMV()
{
	               T4.result2=(T4.Acheived_ms/T2.ramp_time_percentage)*100;
		    	   T4.last_result2=T4.result2;

		    	   if(T4.result2 >= 130)
		    	   {
		    		    T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms+5;
		    	   	    if(T4.incrementing_Dac_value_10ms>=350)
		    	   	    {
		    	   	    	 T4.incrementing_Dac_value_10ms=350;
		    	   	    	 T4.nack=0;
		    	   	    }
		    	   }


		    	   else if(T4.result2 > 120 && T4.result2 <130)
		    	   {
		    		    T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms+(0.5);
		    	   		if(T4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			  T4.incrementing_Dac_value_10ms=350;
		    	   			  T4.nack=0;
		    	   		}
		    	   	}
		    	   	else if(T4.result2<=120 && T4.result2>=110)
		    	   	{
		    	   		T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms+(0.1);
		    	   		if(T4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			   T4.incrementing_Dac_value_10ms=350;
		    	   			   T4.nack=0;
		    	   		}
		    	   	}


}
void Pip_Acheived_Normally_SIMV()
{
 	   T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms;
}


void Pip_Not_Acheived_SIMV()
{
		   if(T3.Pip_Acheived_Flag==0)
		   {
			   T4.incrementing_Dac_value_10ms=T4.incrementing_Dac_value_10ms+1;
			   if(T4.incrementing_Dac_value_10ms>=350)
			   {
				   T4.incrementing_Dac_value_10ms=350;
				   T4.nack=0;
			   }
		   }
}




void Peep_E_Valve_Lock_delay_Pc_SIMV()
{
	if (T1.RR < 30)
	{
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;

		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;



		if(trigger_valve_modification==1)
		{
			S5._Peep_Avg_val = S5._Peep_Avg_trigger / S5._Peep_Avg_count_trigger;
			S5._Peep_Avg_val_int = S5._Peep_Avg_val;
			trigger_valve_modification=0;
		}

		else
		{
			S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
			S5._Peep_Avg_val_int = S5._Peep_Avg_val;
		}

		if (T1.RR >= 30 || T2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;

		if (S5._Peep_Avg_val_int > (T1._PEEP_Val +8) )
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



		else if( (S5._Peep_Avg_val_int > (T1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (T1._PEEP_Val+8)) )
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

		else if( (S5._Peep_Avg_val_int > (T1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (T1._PEEP_Val+2)) )
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
			if (S5._Set_Peep > T1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}



				else if ((S5._Peep_Avg_val_int < (T1._PEEP_Val -8)))
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
					if (S5._Set_Peep > T1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int >= (T1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (T1._PEEP_Val-2)))
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
					if (S5._Set_Peep > T1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}


				else if ((S5._Peep_Avg_val_int >= (T1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (T1._PEEP_Val)))
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
					if (S5._Set_Peep > T1._PEEP_Val + 10)
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

				S5._Peep_Avg_trigger=0;
				S5._Peep_Avg_count_trigger=0;
	}

	else
	{
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;

		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;



		if(trigger_valve_modification==1)
		{
			S5._Peep_Avg_val = S5._Peep_Avg_trigger / S5._Peep_Avg_count_trigger;
			S5._Peep_Avg_val_int = S5._Peep_Avg_val;
			trigger_valve_modification=0;
		}

		else
		{
			S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
			S5._Peep_Avg_val_int = S5._Peep_Avg_val;
		}

		if (T1.RR >= 30 || T2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;

		if (S5._Peep_Avg_val_int > (T1._PEEP_Val +8) )
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



		else if( (S5._Peep_Avg_val_int > (T1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (T1._PEEP_Val+8)) )
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


		else if( (S5._Peep_Avg_val_int > (T1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (T1._PEEP_Val+2)) )
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
				if (S5._Set_Peep > T1._PEEP_Val + 10)
				{
					S5._Set_Peep = S5._Set_Peep;
				}
			}



			else if ((S5._Peep_Avg_val_int < (T1._PEEP_Val -8)))
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
				if (S5._Set_Peep > T1._PEEP_Val + 10)
				{
					S5._Set_Peep = S5._Set_Peep;
				}
			}



				else if ((S5._Peep_Avg_val_int >= (T1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (T1._PEEP_Val-2)))
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
					if (S5._Set_Peep > T1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}


				else if ((S5._Peep_Avg_val_int >= (T1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (T1._PEEP_Val)))
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
					if (S5._Set_Peep > T1._PEEP_Val + 10)
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

				S5._Peep_Avg_trigger=0;
				S5._Peep_Avg_count_trigger=0;
	}
}

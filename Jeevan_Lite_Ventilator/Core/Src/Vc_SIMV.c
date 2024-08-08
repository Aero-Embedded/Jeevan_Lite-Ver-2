/*
 * Vc_SIMV.c
 *
 *  Created on: Mar 17, 2022
 *      Author: asus
 */

#include "Vc_SIMV.h"



int Dac_set=0;
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


extern int trigger_valve_modification;

void VC_SIMV_Task(void *argument)
{


	while (1)
	{
		        switch (_CurrentComputationState)
				{
						case Compute_I_Wave:
							ExpValve_CLOSE();
							Blower_Signal( R4._DAC_VAL0);
							S5.peep_process_done=0;
							S5.lock=1;
							pressure_acheived = 1;
						break;

				      case Compute_E_Wave:

							Blower_Signal( R4._DAC_VAL0);
									if(S5.peep_process_done==1)
									{
										if(R1.Assist_mode2==0 )
										{
											if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
											{
												Blower_Signal( R4._DAC_VAL0);
											}
											else
											{
												Blower_Signal( R4._DAC_VAL0);
											}
										}

										else if(R1.Assist_mode2==1 )
										{
											if(pressure_acheived == 1)
											{
												PID_Compute(&TPID);
												R4._DAC_VAL0=(int16_t)PIDOut;
												Blower_Signal( R4._DAC_VAL0);
											}

										}


										if(Pressure_sensor._Pressure_Val > (R1._PEEP_Val))
										{
											pressure_acheived=0;
											R4._DAC_VAL0=0;
											Blower_Signal( R4._DAC_VAL0);
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
										uwTick=0;
										TPID.OutputSum=300;
										//Speed=100;
									}




									if (_E_TIMER <= (R6._TRIG_WINDOW) && _E_TIMER != 0)
									{

										pressure_acheived =0;
										R4._DAC_VAL0=0;
										Blower_Signal( R4._DAC_VAL0);

										if(R1.Assist_mode2==0 )
										{
											if (R6._TRIG_TYPE == 1)
											{
												if ((Pressure_sensor._Pressure_Val< (R6.simv_trigger_offset - R6._TRIG_LMT)))
												{
													trigger_valve_modification=1;
													Switch_TASK_I_CYCLE();
												}
											}
											else
											{
												if ((Flow_Sensor_cal._Flow_Val > (R6.simv_trigger_offset2+R6._TRIG_LMT)))
												{
													trigger_valve_modification=1;
													Switch_TASK_I_CYCLE();
												}
											}
										}


										else if(R1.Assist_mode2==1 )
										{
											if (R6._TRIG_TYPE == 1)
											{
												if ((Pressure_sensor._Pressure_Val< (R6.simv_trigger_offset - R6._TRIG_LMT)))
												{
													trigger_valve_modification=1;
													Trigger_Flag2=1;
													Switch_TASK_I_CYCLE();
												}
											}
											else
											{
												if ((Flow_Sensor_cal._Flow_Val > (R6.simv_trigger_offset2+R6._TRIG_LMT)))
												{
													trigger_valve_modification=1;
													Trigger_Flag2=1;
													Switch_TASK_I_CYCLE();
												}
											}
										}
									}
									else
									{

											if(Flow_Sensor_cal._Flow_Val==0 || (Flow_Sensor_cal._Flow_Val>=(-8) && Flow_Sensor_cal._Flow_Val<0))
											{
												R6.simv_trigger_offset = Pressure_sensor._Pressure_Val;
												R6.simv_trigger_offset2=Flow_Sensor_cal._Flow_Val;
											}

									}


			if(  (Pressure_sensor._Pressure_Val > (R1._PEEP_Val))   &&  (S5.peep_process_done==1))
			{
				pressure_acheived=0;
				R4._DAC_VAL0=0;
				Blower_Signal( R4._DAC_VAL0);
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





void VC_SIMV_PID_Task(void *argument)
{


	while (1)
	{
		if(S1._Mode_Val == 4 )
		{
			if(_CurrentComputationState==Compute_I_Wave && Trigger_Flag2==0)
			{
				if(O2._FIO2_Val==100)
				{
					R4._DAC_VAL0=400;
				}
				else
				{
					volume_task_SIMV();
				}

			}
			else if(_CurrentComputationState==Compute_E_Wave )
			{
				if(Ach_vol==1)
				{
					Acheived_Volume=vol.Volume;
					Ach_vol=0;
				}
				if(Dac_set==1)
				{
					R4._DAC_VAL0=0;
				}
				/*else
				{
					PID_Compute(&TPID);
					R4._DAC_VAL0=(int16_t)PIDOut;
					Blower_Signal( R4._DAC_VAL0);

				}*/
			}
		}
		vTaskDelay(R4.PID_task_delay);
	}
}






void VC_SIMV_Pulse_I_Parameter()
{

	if(Trigger_Flag2==0)
	{

		R6._TOLERANCE_EWAVE = R2._E_TIMER_HOLD - S5.Lock_delay;
		R6._TRIG_WINDOW = R6._TOLERANCE_EWAVE* (((float) R6._TRIG_TIME * 10.00) / 100.00);

		now_check_breath=1;
			 _Control_Byte &= (uint8_t) (~(0x80));
			 vol.Volume = 0;
			 V_max=0;
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
			 _I_TIMER_ACHEIVED = 0;
			 Led_Alert();
						   if (R4.F_max > (R1._Flow_Rate +10))
							{
								if (R4.F_max > (R1._Flow_Rate + 20))
								{
									R4.temp_dac = R4.temp_dac_new - 20;
								}
								else if ((R4.F_max <= (R1._Flow_Rate + 20))&& (R4.F_max >= (R1._Flow_Rate + 10)))
								{
									R4.temp_dac = R4.temp_dac_new - 5;
								}
								else if ((R4.F_max < (R1._Flow_Rate + 10)) && (R4.F_max >= (R1._Flow_Rate + 5)))
								{
									R4.temp_dac = R4.temp_dac_new - 1;
								}

								R4.temp_dac_new = R4.temp_dac;
								if (R4.temp_dac_new < 500)
								{
									R4.temp_dac_new = 500;
								}
							}
							else if (R4.F_max < (R1._Flow_Rate- 5))
							{
								if (R3.max_flow_acheived == 0)
								{
									R4.temp_dac = R4.temp_dac_new + 30;
									R4.temp_dac_new = R4.temp_dac;

										if (R4.temp_dac_new > 3500)
										{
											R4.temp_dac_new = 3500;
										}

								}

							}
							else
							{
								R4.temp_dac = R4.temp_dac_new;
								R4.temp_dac_new = R4.temp_dac;
							}


						   R3.max_flow_acheived = 0;
						   R3.constant_dac_done = 1;
						   R3.Reached_flow_val = 0;
						   R4.F_max = 0;
						   R3.sensordata_done = 1;
						   R3.Volume_acheived = 1;
						   R3.volume_reached = 0;


						   Peep_E_Valve_Lock_delay_Vc_simv();

						   Ach_vol=1;
						   if(Acheived_Volume>(R1._VT_Val+20))
						   {
								check_count++;
								if (check_count >= 3)
								{
									check_dev = check_dev + 5;
									check_count = 0;
								}
							}
						   S5.P_Max = 0;
			 Alert_I_Time_Parameter();
			 _I_TIMER = R2._I_TIMER_HOLD;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_I_Wave ;
			vTaskDelay(R2._I_TIMER_HOLD);
	}
	else if(Trigger_Flag2==1)
	{
		R6._TOLERANCE_EWAVE = R2._E_TIMER_HOLD - S5.Lock_delay;
		R6._TRIG_WINDOW = R6._TOLERANCE_EWAVE* (((float) R6._TRIG_TIME * 10.00) / 100.00);

		now_check_breath=1;
			 _Control_Byte &= (uint8_t) (~(0x80));
			 vol.Volume = 0;
			 V_max=0;

			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
			 _I_TIMER_ACHEIVED = 0;
			 Led_Alert();
			 R4._DAC_VAL0=0;
			 Peep_E_Valve_Lock_delay_Vc_simv();


			 _I_TIMER = R2._I_TIMER_HOLD;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_I_Wave ;
			vTaskDelay(R2._I_TIMER_HOLD);
	}
}


void VC_SIMV_Pulse_E_Parameter()
{
	if(Trigger_Flag2==0)
	{
		_Control_Byte |= (uint8_t) 0x80;
		Dac_set=1;
		R4._DAC_VAL0=0;
		_E_TIMER_ACHEIVED = 0;
		_E_TIMER = R2._E_TIMER_HOLD;
		Alert_E_Time_Parameter();
		 adjust_servo();
		_CurrentWaveFormState = Generate_I_Wave;
		_CurrentComputationState = Compute_E_Wave ;
		vTaskDelay(R2._E_TIMER_HOLD);
	}
	else if(Trigger_Flag2==1)
	{
		_Control_Byte |= (uint8_t) 0x80;
		Dac_set=0;
		Trigger_Flag2=0;
		_E_TIMER_ACHEIVED = 0;



		 TempSetpoint = R1._PEEP_Val;
		 PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
		 PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
		 PID_SetSampleTime(&TPID, Time);
		 PID_SetOutputLimits(&TPID, Dac1, Dac2);


		_E_TIMER = R2._E_TIMER_HOLD;
		Alert_E_Time_Parameter();
		 adjust_servo();
		_CurrentWaveFormState = Generate_I_Wave;
		_CurrentComputationState = Compute_E_Wave ;
		vTaskDelay(R2._E_TIMER_HOLD);
	}
}

void VC_SIMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{


			R1._VT_Val = RX_PARAM_CMD_PACKET->_VTI;
			R1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			R1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			R1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			R1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			R1._Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;
			R1.RR=RX_PARAM_CMD_PACKET->_RR;
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;


			R6._TRIG_TYPE = (0xF0 & RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT) >> 4;
			R6._TRIG_LMT = 0x0F & (RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT);
			R6._TRIG_TIME = 0x0F & (RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME);

			R1.Assist_mode2 = 0x0F & (RX_PARAM_CMD_PACKET->_T_HIGH);

			R2._I_TIMER_HOLD = (R1.I_Time * (R1.CycleTime / (R1.I_Time +R1.E_Time)));
			R2._E_TIMER_HOLD = (R1.E_Time * (R1.CycleTime / (R1.I_Time + R1.E_Time)));
			R4.PID_task_delay=10;


			R6._CALC_TRIG_VAL = ((float) E_TIME_TOLERANCE / 100.00)* (R2._E_TIMER_HOLD);
			R6._TOLERANCE_EWAVE = R2._E_TIMER_HOLD - R6._CALC_TRIG_VAL;
			R6._TRIG_WINDOW = R6._TOLERANCE_EWAVE* (((float) R6._TRIG_TIME * 10.00) / 100.00);

			S5.Lock_delay=10;
			S5._Set_Peep=R1._PEEP_Val;
			check_dev=0;

			S5.peep_process_done=0;
			S5.lock=1;

			if(R1._Flow_Rate>=60)
				R4.temp_dac_new = 900;
			else if(R1._Flow_Rate<60 && R1._Flow_Rate>=40 )
				R4.temp_dac_new = 800;
			else if(R1._Flow_Rate<40 && R1._Flow_Rate>10 )
				R4.temp_dac_new = 700;

			A.PEEP_VAL=R1._PEEP_Val;
			Alert_Receiving_Parameter();
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._VT_Val=R1._VT_Val;
			O2._Pressure_Base=0;
			O2._Flow_Base=1;
			O2_Parameter();
			P1.Apnea_Mode=0;
			S5.Lock_delay=200;

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



			vTaskSuspend(pc_mode_Handler);
			vTaskSuspend(Pc_cmv_Pid_Handler);

			vTaskSuspend(Pc_simv_Mode_Handler);
			vTaskSuspend(Pc_simv_Mode_Pid_Handler);

			vTaskSuspend(Vc_mode_Handler);
			vTaskSuspend(Vc_cmv_Pid_Handler);

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

			vTaskResume(Vc_simv_mode_Handler);
			vTaskResume(Vc_simv_Pid_Handler);
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

			_CurrentMode=SIMVVC;
			_CurrentBackupMode = IdleState;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_E_Wave ;




}




void volume_task_SIMV()
{

	        	if(_CurrentComputationState==Compute_I_Wave)
				{

					if(vol.Volume<(R1._VT_Val-check_dev))
					{


					 if(R3.volume_reached==0)
					 {
						//if(R3.constant_dac_done==0)
				        //{
								R4._DAC_VAL0=R4.temp_dac+3;
								R4.temp_dac=R4._DAC_VAL0;
							    if(R4._DAC_VAL0>4094)
							    {
							    	R4._DAC_VAL0=4094;
							    }
				       // }

						  if(Flow_Sensor_cal._Flow_Val>=R1._Flow_Rate)
						  {

							      R3.max_flow_acheived=1;
								  R3.Reached_flow_val=Flow_Sensor_cal._Flow_Val;
								  R3.constant_dac_done=0;
								  if(Flow_Sensor_cal._Flow_Val>R4.F_max)
								  {
									  R4.F_max=Flow_Sensor_cal._Flow_Val;
								  }





						  }
						  if(Flow_Sensor_cal._Flow_Val<R1._Flow_Rate)
						  {

							    if(R3.constant_dac_done==1)
							    {
							    	R4._DAC_VAL0=R4.temp_dac;

							    }

				     	  }
					 }

					}
					else
					{

						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						R3.volume_reached=1;
						R4._DAC_VAL0=0;
						R4.temp_dac=R4.temp_dac_new;

					}

				}

}





void Peep_E_Valve_Lock_delay_Vc_simv()
{
	if (R1.RR < 30)
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

		if (R1.RR >= 30 || R2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		if (S5._Peep_Avg_val_int >= (R1._PEEP_Val +6) )
				{

					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 150;
						S5.error_count = 0;
						if (S5.Lock_delay >= 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}



				else if( (S5._Peep_Avg_val_int > (R1._PEEP_Val+3))  && (S5._Peep_Avg_val_int < (R1._PEEP_Val+6)) )
				{

					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 50;
						S5.error_count = 0;
						if (S5.Lock_delay >= 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}


				else if( (S5._Peep_Avg_val_int > (R1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (R1._PEEP_Val+3)) )
				{
					S5._Set_Peep = S5._Set_Peep - 1;
					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 10;
						S5.error_count = 0;
						if (S5.Lock_delay >= 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}
				else if( (S5._Peep_Avg_val_int >= (R1._PEEP_Val-4) )  && (S5._Peep_Avg_val_int < (R1._PEEP_Val)) )
				{


					S5.error_count2++;
					if (S5.error_count2 > 3)
					{
						S5.Lock_delay = S5.Lock_delay - 10;
						S5.error_count2 = 0;
						if (S5.Lock_delay <= 1 || S5.Lock_delay > 700)
						{
							S5.Lock_delay = 1;
							S5._Set_Peep = S5._Set_Peep + 1;
						}
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}


				else if( (S5._Peep_Avg_val_int < (R1._PEEP_Val-4) )  && ( (S5._Peep_Avg_val_int !=0)) )
				{


					S5.error_count2++;
					if (S5.error_count2 > 3)
					{
						S5.Lock_delay = S5.Lock_delay - 10;
						S5.error_count2 = 0;
						if (S5.Lock_delay <= 1 || S5.Lock_delay > 700)
						{
							S5.Lock_delay = 1;
							S5._Set_Peep = S5._Set_Peep + 1;
						}
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}





				else if ((S5._Peep_Avg_val_int ==0))
				{


					S5.error_count2++;
					if (S5.error_count2 > 2)
					{
						S5._Set_Peep = S5._Set_Peep + 5;
						S5.Lock_delay = S5.Lock_delay - 30;
						S5.error_count2 = 0;
						if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
							S5.Lock_delay = 1;
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
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

		if (R1.RR >= 30 || R2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		if (S5._Peep_Avg_val_int > (R1._PEEP_Val +5) )
				{

					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 30;
						S5.error_count = 0;
						if (S5.Lock_delay > 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}



				else if( (S5._Peep_Avg_val_int > (R1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (R1._PEEP_Val+5)) )
				{

					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 20;
						S5.error_count = 0;
						if (S5.Lock_delay > 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}


				else if( (S5._Peep_Avg_val_int > (R1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (R1._PEEP_Val+2)) )
				{
					S5._Set_Peep = S5._Set_Peep - 0.5f;
					S5.error_count++;
					if (S5.error_count > 0)
					{
						S5.Lock_delay = S5.Lock_delay + 10;
						S5.error_count = 0;
						if (S5.Lock_delay > 700)
							S5.Lock_delay = 700;
					}
					if (S5._Set_Peep <= 3)
					{
						S5._Set_Peep = 3;
					}
				}
				else if( (S5._Peep_Avg_val_int >= (R1._PEEP_Val-4) )  && (S5._Peep_Avg_val_int < (R1._PEEP_Val)) )
				{


					S5.error_count2++;
					if (S5.error_count2 > 3)
					{
						S5.Lock_delay = S5.Lock_delay - 10;
						S5.error_count2 = 0;
						if (S5.Lock_delay <= 1 || S5.Lock_delay > 700)
						{
							S5._Set_Peep = S5._Set_Peep + 1;
							S5.Lock_delay = 1;
						}
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}

				else if( (S5._Peep_Avg_val_int < (R1._PEEP_Val-4) )  && ((S5._Peep_Avg_val_int !=0)) )
				{


					S5.error_count2++;
					if (S5.error_count2 > 3)
					{
						S5.Lock_delay = S5.Lock_delay - 10;
						S5.error_count2 = 0;
						if (S5.Lock_delay <= 1 || S5.Lock_delay > 700)
						{
							S5._Set_Peep = S5._Set_Peep + 1;
							S5.Lock_delay = 1;
						}
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int ==0))
				{


					S5.error_count2++;
					if (S5.error_count2 > 2)
					{
						S5._Set_Peep = S5._Set_Peep + 5;
						S5.Lock_delay = S5.Lock_delay - 30;
						S5.error_count2 = 0;
						if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
							S5.Lock_delay = 1;
					}
					if (S5._Set_Peep > R1._PEEP_Val + 10)
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

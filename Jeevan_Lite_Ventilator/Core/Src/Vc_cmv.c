/*
 * Vc_cmv.c
 *
 *  Created on: Mar 14, 2022
 *      Author: asus
 */


#include "Vc_cmv.h"


extern uint16_t _60_Seconds;
extern uint8_t now_check_breath;
extern uint8_t Breath;

void Vc_Cmv_Task(void *argument)
{


	while (1)
	{
		        switch (_CurrentComputationState)
				{
						case Compute_I_Wave:
							ExpValve_CLOSE();
							Blower_Signal( V4._DAC_VAL0);
							S5.peep_process_done=0;
							S5.lock=1;
						break;

				      case Compute_E_Wave:
							V4._DAC_VAL0=0;
							Blower_Signal( V4._DAC_VAL0);
									if(S5.peep_process_done==1)
									{
											if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
											{
												Blower_Signal( V4._DAC_VAL0);
											}
											else
											{
												Blower_Signal( V4._DAC_VAL0);
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


void Vc_cmv_PID_Task(void *argument)
{


	while (1)
	{
		if(S1._Mode_Val == 2 )
		{
			if(_CurrentComputationState==Compute_I_Wave)
			{
				if(O2._FIO2_Val==100)
				{
					V4._DAC_VAL0=400;
				}
				else
				{
				  volume_task();
				}

			}
			else if(_CurrentComputationState==Compute_E_Wave)
			{
				if(Ach_vol==1)
				{
					Acheived_Volume=vol.Volume;
					Ach_vol=0;
				}
				V4._DAC_VAL0=0;
			}
		}
		vTaskDelay(V4.PID_task_delay);
	}
}



void VC_CMV_Pulse_I_Parameter()
{
	 now_check_breath=1;
	_Control_Byte &= (uint8_t) (~(0x80));
	vol.Volume = 0;
    V_max=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	_I_TIMER_ACHEIVED = 0;
	Led_Alert();
	               if (V4.F_max > (V1._Flow_Rate +10))
					{
						if (V4.F_max > (V1._Flow_Rate + 20))
						{
							V4.temp_dac = V4.temp_dac_new - 20;
						}
						else if ((V4.F_max <= (V1._Flow_Rate + 20))&& (V4.F_max >= (V1._Flow_Rate + 10)))
						{
							V4.temp_dac = V4.temp_dac_new - 5;
						}
						else if ((V4.F_max < (V1._Flow_Rate + 10)) && (V4.F_max >= (V1._Flow_Rate + 5)))
						{
							V4.temp_dac = V4.temp_dac_new - 1;
						}

						V4.temp_dac_new = V4.temp_dac;
						if (V4.temp_dac_new < 500)
						{
							V4.temp_dac_new = 500;
						}
					}
					else if (V4.F_max < (V1._Flow_Rate- 5))
					{
						if (V3.max_flow_acheived == 0)
						{
							V4.temp_dac = V4.temp_dac_new + 30;
							V4.temp_dac_new = V4.temp_dac;

								if (V4.temp_dac_new >= 3500)
								{
									V4.temp_dac_new = 3500;
								}

						}

					}
					else
					{
						V4.temp_dac = V4.temp_dac_new;
						V4.temp_dac_new = V4.temp_dac;
					}


	               V3.max_flow_acheived = 0;
	               V3.constant_dac_done = 1;
	               V3.Reached_flow_val = 0;
	               V4.F_max = 0;
	               V3.sensordata_done = 1;
	               V3.Volume_acheived = 1;
	               V3.volume_reached = 0;


	               Peep_E_Valve_Lock_delay_Vc_cmv();

	               Ach_vol=1;
	               if(Acheived_Volume>(V1._VT_Val+20))
	               {
						check_count++;
						if (check_count >= 3)
						{
							check_dev = check_dev + 5;
							check_count = 0;
						}
	               	}

	    /***********   when volume not acheived because of high peep this method used***********/
	              /* if( (Acheived_Volume<(V1._VT_Val-10)) )
	               {
						vol_check_count++;
						if (vol_check_count >= 10)
						{
							V1._PEEP_Val = V1._PEEP_Val - 1;
							vol_check_count = 0;
							if(V1._PEEP_Val<5)
								V1._PEEP_Val=5;
						}
	               	}*/

        /****************************************************************************************/

	               S5.P_Max = 0;
	 Alert_I_Time_Parameter();
	 _I_TIMER = V2._I_TIMER_HOLD;
	_CurrentWaveFormState = Generate_E_Wave;
	_CurrentComputationState = Compute_I_Wave ;
	vTaskDelay(V2._I_TIMER_HOLD);

}


void VC_CMV_Pulse_E_Parameter()
{
	_Control_Byte |= (uint8_t) 0x80;
	_E_TIMER_ACHEIVED = 0;
	_E_TIMER = V2._E_TIMER_HOLD;
	 Alert_E_Time_Parameter();
	 adjust_servo();
	_CurrentWaveFormState = Generate_I_Wave;
	_CurrentComputationState = Compute_E_Wave ;
	vTaskDelay(V2._E_TIMER_HOLD);
}

void VC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

			V1._VT_Val = RX_PARAM_CMD_PACKET->_VTI;
			V1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			V1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			V1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			V1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			V1._Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;
			V1.RR=RX_PARAM_CMD_PACKET->_RR;


			V2._I_TIMER_HOLD = (V1.I_Time * (V1.CycleTime / (V1.I_Time +V1.E_Time)));
			V2._E_TIMER_HOLD = (V1.E_Time * (V1.CycleTime / (V1.I_Time + V1.E_Time)));
			V4.PID_task_delay=10;

			S5.Lock_delay=10;
			S5._Set_Peep=V1._PEEP_Val;
			check_dev=0;
			S5.peep_process_done=0;
			S5.lock=1;


			if(V1._Flow_Rate>=60)
				V4.temp_dac_new = 900;
			else if(V1._Flow_Rate<60 && V1._Flow_Rate>=40 )
				V4.temp_dac_new = 800;
			else if(V1._Flow_Rate<40 && V1._Flow_Rate>10 )
				V4.temp_dac_new = 700;

			A.PEEP_VAL=V1._PEEP_Val;
			Alert_Receiving_Parameter();
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._VT_Val=V1._VT_Val;
			O2._Pressure_Base=0;
			O2._Flow_Base=1;
			O2_Parameter();
			P1.Apnea_Mode=0;

			_60_Seconds=60000;				//breath
			 now_check_breath=0;			//breath
			 Breath=0;

			 check_count=0;
			 vol_check_count=0;

			 S5._Peep_Avg_val_int=V1._PEEP_Val;

			vTaskSuspend(pc_mode_Handler);
			vTaskSuspend(Pc_cmv_Pid_Handler);

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

			vTaskResume(Vc_mode_Handler);
			vTaskResume(Vc_cmv_Pid_Handler);
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

			_CurrentMode=VCCMV;
			_CurrentBackupMode = IdleState;
			_CurrentWaveFormState = Generate_E_Wave;
			_CurrentComputationState = Compute_E_Wave ;




}




void volume_task()
{

	        	if(_CurrentComputationState==Compute_I_Wave)
				{

					if(vol.Volume<(V1._VT_Val-check_dev))
					{
					 if(V3.volume_reached==0)
					 {
						//if(V3.constant_dac_done==0)
				        //{
								V4._DAC_VAL0=V4.temp_dac+3;
								V4.temp_dac=V4._DAC_VAL0;
							    if(V4._DAC_VAL0>4094)
							    {
							    	V4._DAC_VAL0=4094;
							    }

				        //}

						  if(Flow_Sensor_cal._Flow_Val>=V1._Flow_Rate)
						  {

							      V3.max_flow_acheived=1;
								  V3.Reached_flow_val=Flow_Sensor_cal._Flow_Val;
								  V3.constant_dac_done=0;
								  if(Flow_Sensor_cal._Flow_Val>V4.F_max)
								  {
									  V4.F_max=Flow_Sensor_cal._Flow_Val;
								  }

						  }
						  if(Flow_Sensor_cal._Flow_Val<V1._Flow_Rate)
						  {

							    if(V3.constant_dac_done==1)
							    {
							    	V4._DAC_VAL0=V4.temp_dac;

							    }

				     	  }
					 }



					}
					else
					{
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						V3.volume_reached=1;
						V4._DAC_VAL0=0;
						V4.temp_dac=V4.temp_dac_new;

					}

				}

}





void Peep_E_Valve_Lock_delay_Vc_cmv()
{
	if (V1.RR < 30 )
	{
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (V1.RR >= 30 || V2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;
		if (S5._Peep_Avg_val_int > (V1._PEEP_Val +6) )
		{

			S5.error_count++;
			if (S5.error_count > 0)
			{
				S5.Lock_delay = S5.Lock_delay + 150;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
					S5.Lock_delay = 700;
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}



		else if( (S5._Peep_Avg_val_int > (V1._PEEP_Val+3))  && (S5._Peep_Avg_val_int <= (V1._PEEP_Val+6)) )
		{

			S5.error_count++;
			if (S5.error_count > 0)
			{
				S5.Lock_delay = S5.Lock_delay + 50;
				S5.error_count = 0;
				if (S5.Lock_delay > 700)
					S5.Lock_delay = 700;
			}
			if (S5._Set_Peep <= 3)
			{
				S5._Set_Peep = 3;
			}
		}


		else if( (S5._Peep_Avg_val_int > (V1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (V1._PEEP_Val+3)) )
		{
			S5._Set_Peep = S5._Set_Peep - 1;
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
		else if ((S5._Peep_Avg_val_int >= (V1._PEEP_Val -4)) && (S5._Peep_Avg_val_int < (V1._PEEP_Val)))
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
			if (S5._Set_Peep > V1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}


		else  if ((S5._Peep_Avg_val_int < (V1._PEEP_Val -4)) && (S5._Peep_Avg_val_int != 0))
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
			if (S5._Set_Peep > V1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}




		else if ((S5._Peep_Avg_val_int == 0))
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{
				S5._Set_Peep = S5._Set_Peep + 5;
				S5.Lock_delay = S5.Lock_delay - 50;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
					S5.Lock_delay = 1;
			}
			if (S5._Set_Peep > V1._PEEP_Val + 10)
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
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (V1.RR >= 30 || V2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;
		if (S5._Peep_Avg_val_int > (V1._PEEP_Val +5) )
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



		else if( (S5._Peep_Avg_val_int > (V1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (V1._PEEP_Val+5)) )
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


		else if( (S5._Peep_Avg_val_int > (V1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (V1._PEEP_Val+2)) )
		{
			S5._Set_Peep = S5._Set_Peep - 1;
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
		else if ((S5._Peep_Avg_val_int >= (V1._PEEP_Val -4)) && (S5._Peep_Avg_val_int < (V1._PEEP_Val)))
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
			if (S5._Set_Peep > V1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}

		else if ((S5._Peep_Avg_val_int < (V1._PEEP_Val -4)) && (S5._Peep_Avg_val_int != 0))
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
			if (S5._Set_Peep > V1._PEEP_Val + 10)
			{
				S5._Set_Peep = S5._Set_Peep;
			}
		}





		else if ((S5._Peep_Avg_val_int == 0))
		{


			S5.error_count2++;
			if (S5.error_count2 > 2)
			{
				S5._Set_Peep = S5._Set_Peep + 5;
				S5.Lock_delay = S5.Lock_delay - 50;
				S5.error_count2 = 0;
				if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
					S5.Lock_delay = 1;
			}
			if (S5._Set_Peep > V1._PEEP_Val + 10)
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





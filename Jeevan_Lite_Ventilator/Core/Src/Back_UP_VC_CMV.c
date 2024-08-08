/*
 * Back_UP_VC_CMV.c
 *
 *  Created on: Mar 28, 2022
 *      Author: asus
 */

#include "Back_UP_VC_CMV.h"



extern int Trigger;
extern int now_update;
extern uint16_t Dac1;
extern uint16_t Dac2;

extern uint8_t kp;
extern uint8_t ki;
extern double kd;
extern int Time;
extern int Speed;
extern int pressure_acheived;
extern int Initial_open_valve;
extern int Alert_Status_count;
extern int Wait_Dac;


extern int Apnea_counter_trigger_Flag;
extern int Alert_error_count;
extern int Dac_control_count;
extern int Alert_Data_send_count;


extern uint8_t now_check_breath;
void Back_Up_VC_CMV_Mode_Task(void *argument)
{

	while(1)
	{

		switch (_CurrentComputationState)
		{
			case Compute_I_Wave:
				ExpValve_CLOSE();
				Blower_Signal( D4._DAC_VAL0);
				S5.peep_process_done=0;
				S5.lock=1;
			break;

		    case Compute_E_Wave:
				 D4._DAC_VAL0=0;
				 Blower_Signal( D4._DAC_VAL0);
				 if(S5.peep_process_done==1)
				 {
					if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
					{
						Blower_Signal( D4._DAC_VAL0);
					}
					else
					{
						Blower_Signal( D4._DAC_VAL0);
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

					if(_E_TIMER <= (P1._TRIG_WINDOW) &&  _E_TIMER != 0)
					{
							if(P1._TRIG_TYPE==1)
							{
								if((Pressure_sensor._Pressure_Val<(P1.simv_trigger_offset-P1._TRIG_LMT)))
								{

									if(_CurrentMode==PSV)
									{
										_Control_Byte &= (uint8_t) (~(0x80));
										A.Alert=1;
										A.Red_Led_Alert=0;
										Led_Alert();


										uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                         //new

										TempSetpoint = (P1.PIP_PSV_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;

										P1._APNEA_COUNTER = (1000*P1._APNEA_TIME) ;
										P1._Apnea_counter_trigger_check_time=3000;
										Trigger=1;
										now_update=2;
										D4._DAC_VAL0=0;
										P1.Apnea_Mode=1;
										_I_TIMER = D2._I_TIMER_HOLD;


										Parkar_valve_Signal(0);
									  O2.O2_process = 0;
									  O2.Servo=45;
									  Servo_Angle(O2.Servo);

										vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
										vTaskResume(Psv_Pid_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
									}

									if(_CurrentMode==cPAP)
									{
										_Control_Byte &= (uint8_t) (~(0x80));
										A.Alert=1;
										A.Red_Led_Alert=0;
										Led_Alert();
										P1._APNEA_COUNTER = (1000*P1._APNEA_TIME) ;
										Trigger=0;
										now_update=0;
										D4._DAC_VAL0=0;
										P1.Apnea_Mode=1;
										P1._Apnea_counter_trigger_check_time = 3000;



										uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                         //new




										Parkar_valve_Signal(0);
									  O2.O2_process = 0;
									  O2.Servo=45;
									  Servo_Angle(O2.Servo);



										TempSetpoint = (P1.PEEP_CPAP_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;

										vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
										vTaskResume(Cpap_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
									}
								    if(_CurrentMode==BiPAP)
									{
								    	_Control_Byte &= (uint8_t) (~(0x80));
								    	A.Alert=1;
								    	A.Red_Led_Alert=0;
								    	Led_Alert();


								    	uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                         //new

										TempSetpoint = (P1.EPAP_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;


										Parkar_valve_Signal(0);
										  O2.O2_process = 0;
										  O2.Servo=45;
										  Servo_Angle(O2.Servo);


										P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
										P1._Apnea_counter_trigger_check_time = 3000;
										Trigger = 0;
										now_update = 0;
										B4._DAC_VAL0 = 0;
										P1.Apnea_Mode=1;
										vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
										vTaskResume(BiPap_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
									}
								}
							}
							else
							{
								if((Flow_Sensor_cal._Flow_Val>(P1.simv_trigger_offset2+P1._TRIG_LMT)))
								{
									if (_CurrentMode == PSV)
									{
										_Control_Byte &= (uint8_t) (~(0x80));
										A.Alert=1;
										A.Red_Led_Alert=0;
										Led_Alert();


										uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                         //new


										TempSetpoint = (P1.PIP_PSV_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;

										P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
										P1._Apnea_counter_trigger_check_time = 3000;
										Trigger = 1;
										now_update = 2;
										D4._DAC_VAL0 = 0;
										P1.Apnea_Mode=1;
										_I_TIMER = D2._I_TIMER_HOLD;



										Parkar_valve_Signal(0);
										  O2.O2_process = 0;
										  O2.Servo=45;
										  Servo_Angle(O2.Servo);

										vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
										vTaskResume(Psv_Pid_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
									}

									if (_CurrentMode == cPAP)
									{
										_Control_Byte &= (uint8_t) (~(0x80));
										A.Alert=1;
										A.Red_Led_Alert=0;
										Led_Alert();
										P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
										Trigger = 0;
										now_update = 0;
										D4._DAC_VAL0 = 0;
										P1.Apnea_Mode=1;

										P1._Apnea_counter_trigger_check_time = 3000;


										uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                        //new

										TempSetpoint = (P1.PEEP_CPAP_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;



										Parkar_valve_Signal(0);
										  O2.O2_process = 0;
										  O2.Servo=45;
										  Servo_Angle(O2.Servo);

										vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
										vTaskResume(Cpap_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);

									}
									if(_CurrentMode==BiPAP)
									{
										_Control_Byte &= (uint8_t) (~(0x80));
										A.Alert=1;
										A.Red_Led_Alert=0;
										Led_Alert();


										uwTick=0;
										TPID.OutputSum=300;
										Speed=40;
										kp=5;
										ki=4;

										Alert_Data_send_count=0;         //new
										Apnea_counter_trigger_Flag=1;    //new
										Alert_Data_send_count=0;         //new
										Alert_error_count=0;             //new
										Dac_control_count=0;             //new
										uwTick=0;                         //new

										TempSetpoint = (P1.EPAP_Val );
										PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
										PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

										PID_SetSampleTime(&TPID, Time);
										PID_SetOutputLimits(&TPID, Dac1, Dac2);

										pressure_acheived=1;
										Alert_Status_count=0;
										Wait_Dac=1;

										P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
										P1._Apnea_counter_trigger_check_time = 3000;
										Trigger = 0;
										now_update = 0;
										B4._DAC_VAL0 = 0;
										P1.Apnea_Mode=1;


										Parkar_valve_Signal(0);
										  O2.O2_process = 0;
										  O2.Servo=45;
										  Servo_Angle(O2.Servo);

										vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);
										vTaskResume(BiPap_Handler);
										vTaskSuspend(One_Time_Handler);
										vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
									}
								  }
								}
							}
							else
							{

									if(Flow_Sensor_cal._Flow_Val==0 || (Flow_Sensor_cal._Flow_Val>=(-8) && Flow_Sensor_cal._Flow_Val<0))
									{
										P1.simv_trigger_offset=Pressure_sensor._Pressure_Val;
										P1.simv_trigger_offset2=Flow_Sensor_cal._Flow_Val;
									}

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



void PID_Back_Up_VC_CMV_Mode_Task(void *argument)
{

	while(1)
	{
		if(S1._Mode_Val == 11 )
		{
			if(_CurrentComputationState==Compute_I_Wave)
			{
				if(O2._FIO2_Val==100)
				{
					D4._DAC_VAL0=400;
				}
				else
				{
					volume_task_BACKUP_SIMV();
				}

			}
			else if(_CurrentComputationState==Compute_E_Wave)
			{
				if(Ach_vol==1)
				{
					Acheived_Volume=vol.Volume;
					Ach_vol=0;
				}
				D4._DAC_VAL0=0;
			}
		}
			vTaskDelay(D4.PID_task_delay);

	}

}





void volume_task_BACKUP_SIMV()
{

	        	if(_CurrentComputationState==Compute_I_Wave)
				{

					if(vol.Volume<(D1._VT_Val-check_dev))
					{


					 if(D3.volume_reached==0)
					 {
						//if(D3.constant_dac_done==0)
				        //{
								D4._DAC_VAL0=D4.temp_dac+3;
								D4.temp_dac=D4._DAC_VAL0;
							    if(D4._DAC_VAL0>4094)
							    {
							    	D4._DAC_VAL0=4094;
							    }
				        //}

						  if(Flow_Sensor_cal._Flow_Val>=D1._Flow_Rate)
						  {

							      D3.max_flow_acheived=1;
								  D3.Reached_flow_val=Flow_Sensor_cal._Flow_Val;
								  D3.constant_dac_done=0;
								  if(Flow_Sensor_cal._Flow_Val>D4.F_max)
								  {
									  D4.F_max=Flow_Sensor_cal._Flow_Val;
								  }





						  }
						  if(Flow_Sensor_cal._Flow_Val<D1._Flow_Rate)
						  {

							    if(D3.constant_dac_done==1)
							    {
							    	D4._DAC_VAL0=D4.temp_dac;

							    }

				     	  }
					 }

					}
					else
					{

						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						D3.volume_reached=1;
						D4._DAC_VAL0=0;
						D4.temp_dac=D4.temp_dac_new;

					}

				}

}


void BACKUP_VC_SIMV_Pulse_I_Parameter()
{
	P1._TOLERANCE_EWAVE = D2._E_TIMER_HOLD - S5.Lock_delay  ;
	P1._TRIG_WINDOW = P1._TOLERANCE_EWAVE * (((float)P1._TRIG_TIME*10.00)/100.00) ;

	now_check_breath=1;
	_Control_Byte &= (uint8_t) (~(0x80));
	vol.Volume = 0;
    V_max=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	_I_TIMER_ACHEIVED = 0;
	Led_Alert();
	               if (D4.F_max > (D1._Flow_Rate +10))
					{
						if (D4.F_max > (D1._Flow_Rate + 20))
						{
							D4.temp_dac = D4.temp_dac_new - 20;
						}
						else if ((D4.F_max <= (D1._Flow_Rate + 20))&& (D4.F_max >= (D1._Flow_Rate + 10)))
						{
							D4.temp_dac = D4.temp_dac_new - 5;
						}
						else if ((D4.F_max < (D1._Flow_Rate + 10)) && (D4.F_max >= (D1._Flow_Rate + 5)))
						{
							D4.temp_dac = D4.temp_dac_new - 1;
						}

						D4.temp_dac_new = D4.temp_dac;
						if (D4.temp_dac_new < 500)
						{
							D4.temp_dac_new = 500;
						}
					}
					else if (D4.F_max < (D1._Flow_Rate- 5))
					{
						if (D3.max_flow_acheived == 0)
						{
							D4.temp_dac = D4.temp_dac_new + 30;
							D4.temp_dac_new = D4.temp_dac;

								if (D4.temp_dac_new > 3500)
								{
									D4.temp_dac_new = 3500;
								}

						}

					}
					else
					{
						D4.temp_dac = D4.temp_dac_new;
						D4.temp_dac_new = D4.temp_dac;
					}


	               D3.max_flow_acheived = 0;
	               D3.constant_dac_done = 1;
	               D3.Reached_flow_val = 0;
	               D4.F_max = 0;
	               D3.sensordata_done = 1;
	               D3.Volume_acheived = 1;
	               D3.volume_reached = 0;


	               Peep_E_Valve_Lock_delay_BACKUP_VC();

	               Ach_vol=1;
	               if(Acheived_Volume>(D1._VT_Val+20))
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
	 _I_TIMER = D2._I_TIMER_HOLD;
	_CurrentWaveFormState = Generate_E_Wave;
	_CurrentComputationState = Compute_I_Wave ;
	vTaskDelay(D2._I_TIMER_HOLD);

}


void BACKUP_VC_SIMV_Pulse_E_Parameter()
{
	_Control_Byte |= (uint8_t) 0x80;
	_E_TIMER_ACHEIVED = 0;
	_E_TIMER = D2._E_TIMER_HOLD;
	Alert_E_Time_Parameter();
	adjust_servo();								//for O2
	_CurrentWaveFormState = Generate_I_Wave;
	_CurrentComputationState = Compute_E_Wave ;
	vTaskDelay(D2._E_TIMER_HOLD);
}



void Peep_E_Valve_Lock_delay_BACKUP_VC()
{
	if (D1.RR <30 )
	{
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (D1.RR >= 30 || D2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		                if (S5._Peep_Avg_val_int >= (D1._PEEP_Val +6) )
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



						else if( (S5._Peep_Avg_val_int > (D1._PEEP_Val+3))  && (S5._Peep_Avg_val_int < (D1._PEEP_Val+6)) )
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


						else if( (S5._Peep_Avg_val_int > (D1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (D1._PEEP_Val+3)) )
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
						else if ( (S5._Peep_Avg_val_int >= (D1._PEEP_Val -4 ))  && (S5._Peep_Avg_val_int < (D1._PEEP_Val)) )
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
							if (S5._Set_Peep > D1._PEEP_Val + 10)
							{
								S5._Set_Peep = S5._Set_Peep;
							}
						}
						else  if ( (S5._Peep_Avg_val_int < (D1._PEEP_Val -4 ))  && (S5._Peep_Avg_val_int != 0) )
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
							if (S5._Set_Peep > D1._PEEP_Val + 10)
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
								S5.Lock_delay = S5.Lock_delay - 30;
								S5.error_count2 = 0;
								if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
									S5.Lock_delay = 1;
							}
							if (S5._Set_Peep > D1._PEEP_Val + 5)
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

		if (D1.RR >= 30 || D2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		                if (S5._Peep_Avg_val_int > (D1._PEEP_Val +5) )
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



						else if( (S5._Peep_Avg_val_int > (D1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (D1._PEEP_Val+5)) )
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


						else if( (S5._Peep_Avg_val_int > (D1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (D1._PEEP_Val+2)) )
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
						else if ( (S5._Peep_Avg_val_int >= (D1._PEEP_Val -4 ))  && (S5._Peep_Avg_val_int < (D1._PEEP_Val)) )
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
							if (S5._Set_Peep > D1._PEEP_Val + 10)
							{
								S5._Set_Peep = S5._Set_Peep ;
							}
						}

						else if ( (S5._Peep_Avg_val_int < (D1._PEEP_Val -4 ))  && (S5._Peep_Avg_val_int != 0) )
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
							if (S5._Set_Peep > D1._PEEP_Val + 10)
							{
								S5._Set_Peep = S5._Set_Peep ;
							}
						}



						else if ((S5._Peep_Avg_val_int == 0))
						{


							S5.error_count2++;
							if (S5.error_count2 > 3)
							{
								S5._Set_Peep = S5._Set_Peep + 5;
								S5.Lock_delay = S5.Lock_delay - 30;
								S5.error_count2 = 0;
								if (S5.Lock_delay < 1 || S5.Lock_delay > 700)
									S5.Lock_delay = 1;
							}
							if (S5._Set_Peep > D1._PEEP_Val + 10)
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



void BACKUP_VC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{


			D1._VT_Val = RX_PARAM_CMD_PACKET->_VTI;
			D1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			D1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			D1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			D1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			D1._Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;
			D1.RR=RX_PARAM_CMD_PACKET->_RR;
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;


			D2._I_TIMER_HOLD = (D1.I_Time * (D1.CycleTime / (D1.I_Time +D1.E_Time)));
			D2._E_TIMER_HOLD = (D1.E_Time * (D1.CycleTime / (D1.I_Time + D1.E_Time)));
			D4.PID_task_delay=10;


			P1._CALC_TRIG_VAL = ((float)E_TIME_TOLERANCE/100.00)*(D2._E_TIMER_HOLD) ;


			S5.Lock_delay=10;
			S5._Set_Peep=D1._PEEP_Val;
			check_dev=0;


			if(D1._Flow_Rate>=60)
				D4.temp_dac_new = 900;
			else if(D1._Flow_Rate<60 && D1._Flow_Rate>=40 )
				D4.temp_dac_new = 800;
			else if(D1._Flow_Rate<40 && D1._Flow_Rate>10 )
				D4.temp_dac_new = 700;



			A.PEEP_VAL=D1._PEEP_Val;
			Alert_Receiving_Parameter();
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._VT_Val=D1._VT_Val;
			O2._Pressure_Base=0;
			O2._Flow_Base=1;
			O2_Parameter();
			P1.Apnea_Mode=1;
			S5.Lock_delay=200;

			Trigger_Count=0;




			vTaskSuspend(Vc_mode_Handler);
			vTaskSuspend(Vc_cmv_Pid_Handler);

			vTaskSuspend(pc_mode_Handler);
			vTaskSuspend(Pc_cmv_Pid_Handler);

			vTaskSuspend(Vc_simv_mode_Handler);
			vTaskSuspend(Vc_simv_Pid_Handler);

			vTaskSuspend(Pc_simv_Mode_Handler);
			vTaskSuspend(Pc_simv_Mode_Pid_Handler);

			vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
			vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);

			vTaskSuspend(Suction_Handler);

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
			Blower_Signal(0);
			ExpValve_OPEN();
			_CurrentBackupMode = VCCMV_BACKUP;

}





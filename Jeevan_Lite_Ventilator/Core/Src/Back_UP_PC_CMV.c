/*
 * Back_UP_PC_CMV.c
 *
 *  Created on: Mar 18, 2022
 *      Author: asus
 */


#include "Back_UP_PC_CMV.h"




extern int Trigger;
extern int now_update;

extern uint16_t Dac1;
extern uint16_t Dac2;

extern uint8_t kp;
extern uint8_t ki;
extern double kd;
extern int Time;
extern int Speed;

BACKUP_PC_SIMV_Mode_DAC_Control B4={400};
extern int pressure_acheived;
extern int Initial_open_valve;
extern int Alert_Status_count;
extern int Wait_Dac;
extern int initial_trigger_check;
extern int Alert_Data_send_count;
extern int Apnea_counter_trigger_Flag;
extern int Alert_error_count;
extern int Dac_control_count;
extern uint8_t now_check_breath;

void Back_Up_PC_CMV_Mode_Task(void *argument)
{

	while(1)
	{

		switch (_CurrentComputationState)
		{
				case Compute_I_Wave:
					ExpValve_CLOSE();
					Blower_Signal( B4._DAC_VAL0);
					S5.peep_process_done=0;
					S5.lock=1;
				break;

		      case Compute_E_Wave:
					B4._DAC_VAL0=0;
					Blower_Signal( B4._DAC_VAL0);
							if(S5.peep_process_done==1)
							{
									if(Pressure_sensor._Pressure_Val<(S5._Set_Peep))
									{
										Blower_Signal( B4._DAC_VAL0);
									}
									else
									{
										Blower_Signal( B4._DAC_VAL0);
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

														       TempSetpoint = (P1.PIP_PSV_Val);
																PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
																PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

																PID_SetSampleTime(&TPID, Time);
																PID_SetOutputLimits(&TPID, Dac1, Dac2);


														P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
														P1._Apnea_counter_trigger_check_time = 2000;
														Trigger = 1;
														now_update = 2;
														B4._DAC_VAL0 = 0;
														P1.Apnea_Mode=1;
														_I_TIMER = B2._I_TIMER_HOLD;

														pressure_acheived=1;
														Alert_Status_count=0;
														Wait_Dac=1;



														Parkar_valve_Signal(0);
													  O2.O2_process = 0;
													  O2.Servo=45;
													  Servo_Angle(O2.Servo);


														vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
														vTaskResume(Psv_Pid_Handler);
														vTaskSuspend(One_Time_Handler);
														vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
													}

													if(_CurrentMode==cPAP)
													{
														_Control_Byte &= (uint8_t) (~(0x80));
														A.Alert=1;
														A.Red_Led_Alert=0;
														Led_Alert();
														P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
														Trigger = 0;
														now_update = 0;
														B4._DAC_VAL0 = 0;
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



														TempSetpoint = P1.PEEP_CPAP_Val;
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

														vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
														vTaskResume(Cpap_Handler);
														vTaskSuspend(One_Time_Handler);
														vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
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

														TempSetpoint = (P1.EPAP_Val);
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
														vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
														vTaskResume(BiPap_Handler);
														vTaskSuspend(One_Time_Handler);
														vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
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

															pressure_acheived=1;
															Alert_Status_count=0;
															Wait_Dac=1;

															PID_SetSampleTime(&TPID, Time);
															PID_SetOutputLimits(&TPID, Dac1, Dac2);


															P1._APNEA_COUNTER = (1000 * P1._APNEA_TIME);
															P1._Apnea_counter_trigger_check_time = 3000;
															Trigger = 1;
															now_update = 2;
															B4._DAC_VAL0 = 0;
															P1.Apnea_Mode=1;


															Parkar_valve_Signal(0);
															  O2.O2_process = 0;
															  O2.Servo=45;
															  Servo_Angle(O2.Servo);

															_I_TIMER = B2._I_TIMER_HOLD;
															vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
															vTaskResume(Psv_Pid_Handler);
															vTaskSuspend(One_Time_Handler);
															vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
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
															B4._DAC_VAL0 = 0;
															P1.Apnea_Mode=1;


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


															pressure_acheived=1;
															Alert_Status_count=0;
															Wait_Dac=1;


															Parkar_valve_Signal(0);
															  O2.O2_process = 0;
															  O2.Servo=45;
															  Servo_Angle(O2.Servo);

															P1._Apnea_counter_trigger_check_time = 3000;

															TempSetpoint = (P1.PEEP_CPAP_Val );
															PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
															PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

															PID_SetSampleTime(&TPID, Time);
															PID_SetOutputLimits(&TPID, Dac1, Dac2);


															vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);
															vTaskResume(Cpap_Handler);
															vTaskSuspend(One_Time_Handler);
															vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
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
															vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
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

void BACKUP_Switch_TASK_I_CYCLE()
{

	vTaskDelete(One_Time_Handler);
	_I_TIMER = 0 ;
	_E_TIMER = 0 ;
	_CurrentWaveFormState = Generate_I_Wave ;
	 xTaskCreate(One_Time_Task, "one-time-task", 256, NULL, 2, &One_Time_Handler);

}

void BACKUP_PC_SIMV_Pulse_I_Parameter()
{
	 P1._TOLERANCE_EWAVE = B2._E_TIMER_HOLD - S5.Lock_delay ;
	 P1._TRIG_WINDOW = P1._TOLERANCE_EWAVE * (((float)P1._TRIG_TIME*10.00)/100.00) ;

	   now_check_breath=1;
	   _Control_Byte &= (uint8_t) (~(0x80));
	    vol.Volume = 0;
	    V_max=0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		_I_TIMER_ACHEIVED = 0;
		S5.peep_process_done = 0;

			B4._DAC_VAL0 = B4.starting_DAC;
			B4.Last_DAC = B4._DAC_VAL0;
			B3.ok = 1;
			B3.cycle_done = 1;
			B4.last_result1 = B4.result1;
			Led_Alert();
			DAC_Value_Correction_BACKUP_PC();
			Ending_Dac_value_correction_BACKUP_PC();
			S5.P_Max = 0;
			B3.Pip_Acheived_Flag = 0;
			S5.now_check = 0;
			S5.P_Min = 60;

		B3.cycle_done = 1;
		Peep_E_Valve_Lock_delay_BACKUP_PC();
		B4.ten_ms = 0;
		B4.Acheived_ten_ms = 0;
		Alert_I_Time_Parameter();
		_I_TIMER = B2._I_TIMER_HOLD;
		_CurrentWaveFormState = Generate_E_Wave;
		_CurrentComputationState = Compute_I_Wave ;
		vTaskDelay(B2._I_TIMER_HOLD);
}



void BACKUP_PC_SIMV_Pulse_E_Parameter()
{
	_Control_Byte |= (uint8_t) 0x80;
	_E_TIMER = B2._E_TIMER_HOLD;
	_E_TIMER_ACHEIVED = 0;
	S5.lock = 1;
	Alert_E_Time_Parameter();
	 adjust_servo();								//for O2
	_CurrentWaveFormState = Generate_I_Wave;
	_CurrentComputationState = Compute_E_Wave ;
	vTaskDelay(B2._E_TIMER_HOLD);

}


void PID_Back_Up_PC_CMV_Mode_Task(void *argument)
{

	while(1)
	{
		B4.ten_ms++;
			if(S1._Mode_Val == 10)
			 {
				if(_CurrentComputationState==Compute_I_Wave)
				{
					if(O2._FIO2_Val==100)
					{
						B4._DAC_VAL0=400;
					}
					else
					{
						pip_value_correction_BACKUP_PC();
					}
				}
			 }
		vTaskDelay(B4.PID_task_delay);

	}

}


void DAC_Value_Correction_BACKUP_PC()
{



		 if(B4.Acheived_ms < (B2.ramp_time_percentage -40))
	     {
			 Pip_Acheived_Early_BACKUP_PC();

	     }


	       else if(B4.Acheived_ms > (B2.ramp_time_percentage +40))
	       {
	    	   Pip_Acheived_Slowly_BACKUP_PC();
	       }

	       else
	       {
	    	   Pip_Acheived_Normally_BACKUP_PC();

	       }

		     Pip_Not_Acheived_BACKUP_PC();

}

void pip_value_correction_BACKUP_PC()
{
	if (Pressure_sensor._Pressure_Val >= (B1._PIP_Val ))
	{
		B3.cycle_done = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		if(B3.ok==1)
		{
			B4.Acheived_ten_ms = B4.ten_ms;
			B4.Acheived_ms=(B4.Acheived_ten_ms*B4.PID_task_delay);
			B3.ok=0;
			B3.Pip_Acheived_Flag=1;
			B4._DAC_VAL0 = B4.Last_DAC;
		}
	}
	if (B3.cycle_done == 1)
	{
		if (B4.Last_DAC >= B4.Ending_Dac)
		{
			B4.Last_DAC = B4.Ending_Dac;
			B3.cycle_done = 0;
		}
		else if (B4.Last_DAC < B4.Ending_Dac)
		{
			B4.Last_DAC = B4._DAC_VAL0;
			B4._DAC_VAL0=B4.Last_DAC + (B4.incrementing_Dac_value_10ms );
		}
	}

}




void BACKUP_PC_CMV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

	//xSemaphoreTake(binarysem,portMAX_DELAY);
			B1._PIP_Val = RX_PARAM_CMD_PACKET->_PIP;
			B1._PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
			B1.CycleTime = 60000 / RX_PARAM_CMD_PACKET->_RR;
			B1.I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E) >> 4;
			B1.E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
			B1.RT_Value =(0xF0 & RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME)>>4;
			B1.RR=RX_PARAM_CMD_PACKET->_RR;
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;


			B2._I_TIMER_HOLD = (B1.I_Time * (B1.CycleTime / (B1.I_Time +B1.E_Time)));
			B2._E_TIMER_HOLD = (B1.E_Time * (B1.CycleTime / (B1.I_Time + B1.E_Time)));
			B2.ramp_time=(B1.RT_Value*10);
			B2.ramp_time_percentage = ((float)B2.ramp_time/100.00)*(B2._I_TIMER_HOLD) ;

			if(B2._I_TIMER_HOLD > 600)
			{
				if(B2.ramp_time_percentage <600)
				{
					B2.ramp_time_percentage =600;
				}
			}

			B4.Ending_Dac=17.1129 * (B1._PIP_Val) + 587.7390+((1/70)*200);
			B4.incrementing_Dac_value_10ms=70;
			B4.Acheived_ms=(B2.ramp_time_percentage*2);

			P1._CALC_TRIG_VAL = ((float)E_TIME_TOLERANCE/100.00)*(B2._E_TIMER_HOLD) ;

			B3.Pip_Acheived_Flag=0;
			B4.nack=1;
			B4.PID_task_delay=10;
			B4.result1_error=0;
			B4.last_result1=0;
			B4.PID_task_delay_lock=0;
			S5._Set_Peep=B1._PEEP_Val/2;

			A.PEEP_VAL=B1._PEEP_Val;
			Alert_Receiving_Parameter();
			O2._FIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
			O2._PIP_Val=B1._PIP_Val;
			O2._Pressure_Base=1;
			O2._Flow_Base=0;
			O2_Parameter();
			P1.Apnea_Mode=1;
			S5.Lock_delay=300;
			Trigger_Count=0;




			vTaskSuspend(Vc_mode_Handler);
			vTaskSuspend(Vc_cmv_Pid_Handler);

			vTaskSuspend(pc_mode_Handler);
			vTaskSuspend(Pc_cmv_Pid_Handler);

			vTaskSuspend(Vc_simv_mode_Handler);
		    vTaskSuspend(Vc_simv_Pid_Handler);

		    vTaskSuspend(Pc_simv_Mode_Handler);
		    vTaskSuspend(Pc_simv_Mode_Pid_Handler);

		    vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
		    vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);

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
			_CurrentBackupMode=PCCMV_BACKUP;
			//xSemaphoreGive(binarysem);

}



void Ending_Dac_value_correction_BACKUP_PC()
{

	if(B1.RR>=30 || B2.ramp_time>80 || (B1.E_Time>4))
    {
		    if(S5.P_Max>(B1._PIP_Val))
			{
		    	B4.pmax_error1++;
		    	B4.pmax_error2=0;
		    	if(B4.pmax_error1>2)
		    	{
		    	B4.Ending_Dac=B4.Ending_Dac-1;
				B4.pmax_error1=0;
					if(B4.Ending_Dac<=500)
						B4.Ending_Dac=500;
		    	}
			}
			else if(S5.P_Max<(B1._PIP_Val))
			{
				B4.pmax_error2++;
				B4.pmax_error1=0;
				if(B4.pmax_error2>2)
				{
				B4.Ending_Dac=B4.Ending_Dac+1;
				B4.pmax_error2=0;
				if(B4.Ending_Dac>=4095)
					 B4.Ending_Dac=4095;
				}
			}
			else
			{
				B4.Ending_Dac=B4.Ending_Dac;
			}
    }

 else if(B1.RR<30 || B2.ramp_time<=80)
 {
	if(S5._Pip_Avg_val_int>B1._PIP_Val)
	{
		B4.Ending_Dac=B4.Ending_Dac-5;
		if(B4.Ending_Dac<=500)
			B4.Ending_Dac=500;
	}
	else if(S5._Pip_Avg_val_int<B1._PIP_Val)
	{
		B4.Ending_Dac=B4.Ending_Dac+5;
		if(B4.Ending_Dac>=4095)
			B4.Ending_Dac=4095;
	}
	else
	{
		B4.Ending_Dac=B4.Ending_Dac;

	}
 }
}


void Pip_Acheived_Early_BACKUP_PC()
{

		     if(B3.Pip_Acheived_Flag==0)
			 {

			 }
			 else
			 {

			    B4.result1=(B4.Acheived_ms/B2.ramp_time_percentage)*100;
			    if(B4.result1<=70)
			    {
			    	B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms-5;
			    	if(B4.incrementing_Dac_value_10ms<=70)
			    	{
			    		B4.incrementing_Dac_value_10ms=70;
			    		B4.nack=0;
			    	}
			    }

			    else if(B4.result1<=80 && B4.result1>70)
			    {
			    	B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms-(0.5);
			    	if(B4.incrementing_Dac_value_10ms<=70)
			    	{
			    		B4.incrementing_Dac_value_10ms=70;
			    		B4.nack=0;
			    	}
			    }
			    else if(B4.result1<=90 && B4.result1>80)
			    {
			    	B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms-(0.1);
			    	if(B4.incrementing_Dac_value_10ms<=70)
			    	{
			    		B4.incrementing_Dac_value_10ms=70;
			    		B4.nack=0;
			    	}
			    }


			    if(B4.nack==0)
			    {

			    	if(S5.P_Max<B1._PIP_Val)
			    	{
			    		 B4.result1_error++;
			    		 if(B4.result1_error>=5)
			    		 {
			    			B4.PID_task_delay=B4.PID_task_delay;
			    			B4.result1_error=6;
			    		    B4.PID_task_delay_lock=1;
			    		 }
			    	}
			    	else if(B4.result1>B4.last_result1)
			    	{
			    		if(B4.PID_task_delay_lock==0)
			    		{
			    		  if(B4.result1<60)
			    		  {
			    			  B4.PID_task_delay=B4.PID_task_delay+10;
			    		  }
			    		  else if(B4.result1<70 && B4.result1>=60)
			    		  {
			    			  B4.PID_task_delay=B4.PID_task_delay+6;
			    		  }
			    		  else if(B4.result1<=80 && B4.result1>=70)
			    		  {
			    			  B4.PID_task_delay=B4.PID_task_delay+4;
			    		  }
			    		  else
			    		  {
			    			  B4.PID_task_delay=B4.PID_task_delay+1;
			    		  }
			    		}

			    	}

			    }
			 }

}



void Pip_Acheived_Slowly_BACKUP_PC()
{
	               B4.result2=(B4.Acheived_ms/B2.ramp_time_percentage)*100;
		    	   B4.last_result2=B4.result2;

		    	   if(B4.result2 >= 130)
		    	   {
		    		    B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms+5;
		    	   	    if(B4.incrementing_Dac_value_10ms>=350)
		    	   	    {
		    	   	    	 B4.incrementing_Dac_value_10ms=350;
		    	   	    	 B4.nack=0;
		    	   	    }
		    	   }


		    	   else if(B4.result2 > 120 && B4.result2 <130)
		    	   {
		    		    B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms+(0.5);
		    	   		if(B4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			  B4.incrementing_Dac_value_10ms=350;
		    	   			  B4.nack=0;
		    	   		}
		    	   	}
		    	   	else if(B4.result2<=120 && B4.result2>=110)
		    	   	{
		    	   		B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms+(0.1);
		    	   		if(B4.incrementing_Dac_value_10ms>=350)
		    	   		{
		    	   			   B4.incrementing_Dac_value_10ms=350;
		    	   			   B4.nack=0;
		    	   		}
		    	   	}


}
void Pip_Acheived_Normally_BACKUP_PC()
{
 	   B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms;
}


void Pip_Not_Acheived_BACKUP_PC()
{
		   if(B3.Pip_Acheived_Flag==0)
		   {
			   B4.incrementing_Dac_value_10ms=B4.incrementing_Dac_value_10ms+1;
			   if(B4.incrementing_Dac_value_10ms>=350)
			   {
				   B4.incrementing_Dac_value_10ms=350;
				   B4.nack=0;
			   }
		   }
}




void Peep_E_Valve_Lock_delay_BACKUP_PC()
{
	if (B1.RR < 30)
	{
		S5._Pip_Avg_val = S5._Pip_Avg / S5._Pip_Avg_count;
		S5._Pip_Avg_val_int = S5._Pip_Avg_val;
		S5._Pip_Avg_count = 0;
		S5._Pip_Avg = 0;

		S5._Peep_Avg_val = S5._Peep_Avg / S5._Peep_Avg_count;
		S5._Peep_Avg_val_int = S5._Peep_Avg_val;

		if (B1.RR >= 30 || B2._E_TIMER_HOLD < 500)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		if (S5._Peep_Avg_val_int > (B1._PEEP_Val +8) )
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



		else if( (S5._Peep_Avg_val_int > (B1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (B1._PEEP_Val+8)) )
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


				else if( (S5._Peep_Avg_val_int > (B1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (B1._PEEP_Val+2)) )
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int < (B1._PEEP_Val -8)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int >= (B1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (B1._PEEP_Val-2)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}


				else if ((S5._Peep_Avg_val_int >= (B1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (B1._PEEP_Val)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
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

		if (B1.RR >= 30 || B2._E_TIMER_HOLD < 700)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}
		S5.peep_max=0;


		if (S5._Peep_Avg_val_int > (B1._PEEP_Val +8) )
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



			else if( (S5._Peep_Avg_val_int > (B1._PEEP_Val+2))  && (S5._Peep_Avg_val_int <= (B1._PEEP_Val+8)) )
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


				else if( (S5._Peep_Avg_val_int > (B1._PEEP_Val))  && (S5._Peep_Avg_val_int <= (B1._PEEP_Val+2)) )
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int < (B1._PEEP_Val -8)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}



				else if ((S5._Peep_Avg_val_int >= (B1._PEEP_Val -8)) && (S5._Peep_Avg_val_int < (B1._PEEP_Val-2)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
					{
						S5._Set_Peep = S5._Set_Peep;
					}
				}


				else if ((S5._Peep_Avg_val_int >= (B1._PEEP_Val -2)) && (S5._Peep_Avg_val_int < (B1._PEEP_Val)))
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
					if (S5._Set_Peep > B1._PEEP_Val + 10)
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


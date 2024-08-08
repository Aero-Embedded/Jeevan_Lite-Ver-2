/*
 * APRV.c
 *
 *  Created on: Apr 1, 2022
 *      Author: asus
 */

#include "APRV.h"



int P_HIGH_FINISHED=0;
int P_LOW_TIME=1;
int P_HIGH_TIME=0;



extern uint16_t Dac1;
extern uint16_t Dac2;

extern uint8_t kp;
extern uint8_t ki;
extern double kd;
extern int Time;
extern int Speed;

extern int Initial_open_valve;
extern int Alert_error_count;
extern int now_open_valve;
extern int Apnea_Patient_circuit_disconnected_Flag;
extern int Alert_Status_count;
extern int Alert_Data_send_count;






float _Peep_Avg;
float _Peep_Avg_count;
float _Peep_Avg_val;
int _Peep_Avg_val_int;

int error_count;
int error_count2;
float _Set_Peep;
uint16_t Lock_delay;
int peep_max;
int lock;


extern uint16_t _60_Seconds;
extern uint8_t now_check_breath;

extern uint8_t Breath;

void APRV_Mode_Task (void *argument)
{

	while(1)
	{

		if(P1.P_HIGH_TIMER>0 )
		{

			PID_Compute(&TPID);
			P1.P_HIGH_DAC_VAL=(int16_t)PIDOut;
			Blower_Signal( P1.P_HIGH_DAC_VAL);

			if( P1._Apnea_counter_trigger_check_time==0)
			{
							if (P1._TRIG_TYPE == 1)
						    {
						        if((Pressure_sensor._Pressure_Val<(P1.simv_trigger_offset-P1._TRIG_LMT)))
						        {
						        	    Trigger=1;
						        	    Trigger_Count++;
						        	    P1._Apnea_counter_trigger_check_time=1000;


						         }
						      }
						      else
						      {
						         if((Flow_Sensor_cal._Flow_Val>(P1.simv_trigger_offset2+P1._TRIG_LMT)))
						         {

						        	 Trigger=1;
						        	 Trigger_Count++;
						        	  P1._Apnea_counter_trigger_check_time=1000;
						         }
						        }



						        	     if (Flow_Sensor_cal._Flow_Val >= (-1) && Flow_Sensor_cal._Flow_Val <= 3)
						        	     {
						        	        P1.simv_trigger_offset = Pressure_sensor._Pressure_Val;
						        	        P1.simv_trigger_offset2=Flow_Sensor_cal._Flow_Val;
						        	     }

					       }
						   else
						   {

						         	if (Flow_Sensor_cal._Flow_Val >= (-6) && Flow_Sensor_cal._Flow_Val <= 3)
						         	{
						         		 P1.simv_trigger_offset = Pressure_sensor._Pressure_Val;
						         		 P1.simv_trigger_offset2=Flow_Sensor_cal._Flow_Val;

						         	}


						    }


			            if(Pressure_sensor._Pressure_Val>( P1.P_HIGH+4))
						{
							if(now_open_valve==1)
							{
								ExpValve_OPEN();
								now_open_valve=0;

							}

						}
						else if(Pressure_sensor._Pressure_Val<=( P1.P_HIGH+4))
						{
							ExpValve_CLOSE();
						}




		}
		else if(P1.P_LOW_TIMER>0)
		{
			Blower_Signal( 0);
				if(Pressure_sensor._Pressure_Val<=(_Set_Peep))
				{

					if(lock==1)
					{
						lock=0;
						vTaskDelay(Lock_delay);
					}

					ExpValve_CLOSE();
					P_HIGH_FINISHED=0;
				}
				else if(P_HIGH_FINISHED==1)
				{

					ExpValve_OPEN();

				}



		}



		Alert_error_count++;

		Patient_Circuit_Disconnected_Alert_Func();
		Main_Supply_or_Battery_Indication();

		if( Alert_error_count >= 500)
		{
			if(Apnea_Patient_circuit_disconnected_Flag==1)
			{

					Alert_Status_count++;
					if(Alert_Status_count>3)
					{
						//A.Alert_Now=1;
						Apnea_Patient_circuit_disconnected_Flag=0;
						Alert_Status_count=0;
					}

			}
			else
			{
				//A.Alert_Now=1;
				CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PATIENT_CIRCUIT);
			}
			now_open_valve=1;
			Alert_error_count=0;
		}

		Alert_Data_send_count++;
		if(Alert_Data_send_count>1500)
		{
			//A.Alert_Now=1;
			Alert_Data_send_count=0;

		}

		vTaskDelay(2);
	}
}


void APRV_Mode_One_Time_Task (void *argument)
{

	while(1)
	{

		if(P_LOW_TIME==1)
		{

			Parkar_valve_Signal(0);
			O2.O2_process = 0;
			O2.Servo=45;
			Servo_Angle(O2.Servo);




			P_LOW_TIME=0;
			P_HIGH_TIME=1;
			P_HIGH_FINISHED=1;
			_Control_Byte |= (uint8_t) 0x80;
			P1.P_HIGH_DAC_VAL=0;

			lock=1;                           //new

			TempSetpoint = P1.P_LOW;
			PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
			PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

			PID_SetSampleTime(&TPID, Time);
			PID_SetOutputLimits(&TPID, Dac1, Dac2);

			P1.P_LOW_TIMER =P1.T_LOW_VAL;
			vTaskDelay(P1.T_LOW_VAL);

		}
		else if(P_HIGH_TIME==1)
		{
			now_check_breath=1;
			P1._Apnea_counter_trigger_check_time=1000;
			ExpValve_CLOSE();
			_Control_Byte &= (uint8_t) (~(0x80));
			P_HIGH_TIME=0;
			P_LOW_TIME=1;
			P1.P_LOW_DAC_VAL=0;

			Peep_E_Valve_Lock_delay_APRV();


			TempSetpoint = P1.P_HIGH;
			PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
			PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

			PID_SetSampleTime(&TPID, Time);
			PID_SetOutputLimits(&TPID, Dac1, Dac2);

			P1.P_HIGH_TIMER =P1.T_HIGH_VAL;
			vol.Volume = 0;
			vTaskDelay(P1.T_HIGH_VAL);

		}

	}

}

void APRV_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{
	     P1.P_HIGH = RX_PARAM_CMD_PACKET->_PIP;
	     P1.P_LOW = RX_PARAM_CMD_PACKET->_PEEP;

		 P1.T_HIGH=(RX_PARAM_CMD_PACKET->_T_HIGH);
		 P1.T_HIGH_VAL=(P1.T_HIGH*100);
		 P1.P_HIGH_TIMER =P1.T_HIGH_VAL;

		 P1.T_LOW=(RX_PARAM_CMD_PACKET->_FlowRate);
		 P1.T_LOW_VAL=(P1.T_LOW*100);

		 P1.P_LOW_TIMER =P1.T_LOW_VAL;
		 P_LOW_TIME=1;
		 P_HIGH_TIME=0;

		 P1._Apnea_counter_trigger_check_time=1000;
		 P1._TRIG_TYPE = (0xF0 & RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT)>>4;
		 P1._TRIG_LMT = 0x0F & (RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT);


		 Initial_open_valve=1;
		 Alert_error_count=0;


		 Dac1=300;
		 Dac2=2000;
		 kp=10;
		 ki=9;
		 kd=1;
		 Time=50;
		 Speed=100;

		 _Peep_Avg_count = 1;
		 _Peep_Avg = 1;

		 _60_Seconds=60000;				//breath
		 now_check_breath=0;			//breath
		 Breath=0;

		 TempSetpoint = P1.P_LOW;
		 PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
		 PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

		 PID_SetSampleTime(&TPID, Time);
		 PID_SetOutputLimits(&TPID, Dac1, Dac2);


		 _Set_Peep=P1.P_LOW/2;

		 lock=1;

		 Trigger_Count=0;
		 ExpValve_CLOSE();

		 vTaskSuspend(pc_mode_Handler);
		 vTaskSuspend(Pc_cmv_Pid_Handler);

		 vTaskSuspend(Pc_simv_Mode_Handler);
		 vTaskSuspend(Pc_simv_Mode_Pid_Handler);

		 vTaskSuspend(Vc_mode_Handler);
		 vTaskSuspend(Vc_cmv_Pid_Handler);

		 vTaskSuspend(Vc_simv_mode_Handler);
		 vTaskSuspend(Vc_simv_Pid_Handler);

		 vTaskSuspend(Psv_Handler);
		 vTaskSuspend(Psv_Pid_Handler);

		 vTaskSuspend(Cpap_Handler);

		 vTaskSuspend(BiPap_Handler);
		 vTaskSuspend(BiPap_Pid_Handler);

		 vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
		 vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);

		 vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
		 vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);

		 vTaskSuspend(One_Time_Handler);

		 vTaskSuspend(Oxygen_Handler);

		 vTaskSuspend(HFNC_Handler);
		 vTaskSuspend(Suction_Handler);

		 vTaskResume(APRV_one_Handler);
		 vTaskResume(APRV_Handler);

		 _CurrentMode = APRV;
		 _CurrentBackupMode = IdleState;

}

void Peep_E_Valve_Lock_delay_APRV()
{


		_Peep_Avg_val = _Peep_Avg / _Peep_Avg_count;
		_Peep_Avg_val_int = _Peep_Avg_val;

		/*if (S1.RR >= 30 || S2._E_TIMER_HOLD < 500)
		{
			S5._Peep_Avg_val_int = S5.peep_max;
		}*/
		//peep_max=0;



		if (_Peep_Avg_val_int > (P1.P_LOW+8) )
		{
			_Set_Peep = _Set_Peep - 1.5f;
		    error_count++;
			if (error_count > 0)
			{
				Lock_delay = Lock_delay + 30;
				error_count = 0;
				if (Lock_delay > 500)
					Lock_delay = 500;
			}
			if (_Set_Peep <= 3)
			{
				_Set_Peep = 3;
			}
		}



		else if( (_Peep_Avg_val_int > (P1.P_LOW+2))  && (_Peep_Avg_val_int <= (P1.P_LOW+8)) )
		{
			_Set_Peep = _Set_Peep - 1;
			error_count++;
			if (error_count > 0)
			{
				Lock_delay = Lock_delay + 20;
				error_count = 0;
				if (Lock_delay > 500)
					Lock_delay = 500;
			}
			if (_Set_Peep <= 3)
			{
				_Set_Peep = 3;
			}
		}


		else if( (_Peep_Avg_val_int > (P1.P_LOW))  && (_Peep_Avg_val_int <= (P1.P_LOW+2)) )
		{
			_Set_Peep = _Set_Peep - 0.5f;
			error_count++;
			if (error_count > 0)
			{
				Lock_delay = Lock_delay + 10;
				error_count = 0;
				if (Lock_delay > 500)
					Lock_delay = 500;
			}
			if (_Set_Peep <= 3)
			{
				_Set_Peep = 3;
			}
		}
		else if (_Peep_Avg_val_int < (P1.P_LOW -8))
		{
			_Set_Peep = _Set_Peep + 1.5;

			error_count2++;
			if (error_count2 > 0)
			{
				Lock_delay = Lock_delay - 30;
				error_count = 0;
				if (Lock_delay < 1 || Lock_delay > 500)
					Lock_delay = 1;
			}
			if (_Set_Peep > (P1.P_LOW + 10))
			{
				_Set_Peep = _Set_Peep;
			}
		}
		else if ((_Peep_Avg_val_int >= (P1.P_LOW -8)) && (_Peep_Avg_val_int < (P1.P_LOW-2)))
		{
			_Set_Peep = _Set_Peep + 1;

			error_count2++;
			if (error_count2 > 0)
			{
				Lock_delay = Lock_delay - 20;
				error_count = 0;
				if (Lock_delay < 1 || Lock_delay > 700)
					Lock_delay = 1;
			}
			if (_Set_Peep > (P1.P_LOW + 10))
			{
				_Set_Peep = _Set_Peep;
			}
		}


		else if ((_Peep_Avg_val_int >= (P1.P_LOW -2)) && (_Peep_Avg_val_int < (P1.P_LOW)))
		{
			_Set_Peep = _Set_Peep + 0.5f;

			error_count2++;
			if (error_count2 > 0)
			{
				Lock_delay = Lock_delay - 10;
				error_count = 0;
				if (Lock_delay < 1 || Lock_delay > 500)
					Lock_delay = 1;
			}
			if (_Set_Peep > (P1.P_LOW + 10))
			{
				_Set_Peep = _Set_Peep;
			}
		}



		else
		{
			_Set_Peep = _Set_Peep;
		}
		_Peep_Avg_count = 1;
		_Peep_Avg = 1;
}



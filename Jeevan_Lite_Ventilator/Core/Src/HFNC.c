/*
 * HFNC.c
 *
 *  Created on: Oct 15, 2022
 *      Author: asus
 */



#include "HFNC.h"

uint16_t HFNC_Dac_Value=0;


extern uint16_t HFNC_Dac1;
extern uint16_t HFNC_Dac2;

extern uint8_t kp;
extern uint8_t ki;
extern double kd;
extern int Time;
extern int Speed;


void HFNC_Task (void *argument)
{
	while (1)
	{


		PID_Compute(&TPID);
		HFNC_Dac_Value=(int16_t)PIDOut;
		Parkar_valve_Signal( HFNC_Dac_Value);

		//Parkar_valve_Signal( HFNC_Dac_Value);     // FOR DEBUG PURPOSE

		O2.Servo=100;
		Servo_Angle(O2.Servo);

		ExpValve_OPEN();

		vTaskDelay(2);

	}

}




void HFNC_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{

	 H1._Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;


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

	 vTaskSuspend(APRV_Handler);
	 vTaskSuspend(APRV_one_Handler);

	 vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
	 vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);

	 vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
	 vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);

	 vTaskSuspend(One_Time_Handler);

	 vTaskSuspend(Oxygen_Handler);

	 vTaskSuspend(alert_Handler);

	 vTaskSuspend(Nebuliser_Handler);


	 Blower_Signal(0);
	 ExpValve_OPEN();
	 _CurrentMode=HFNC;
	 _CurrentBackupMode = IdleState;

	 HFNC_Dac_Value=0;

  	 uwTick=0;

  	 HFNC_Dac1=1800;
  	 HFNC_Dac2=4000;
	 kp=10;
	 ki=9;
	 kd=1;
	 Time=1000;
	 Speed=1000;

	 TempSetpoint = H1._Flow_Rate;
	 PID(&TPID, &Temp_Flow, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
	 PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);

	 PID_SetSampleTime(&TPID, Time);
	 PID_SetOutputLimits(&TPID, HFNC_Dac1, HFNC_Dac2);

	 Parkar_valve_Signal(0);
	 O2.O2_process = 0;
	 O2.Servo=45;
	 Servo_Angle(O2.Servo);

	 vTaskResume(HFNC_Handler);


}


void Suction_Task(void *argument)
{
	while (1)
	{
		Blower_Signal(1300);
		vTaskDelay(1000);
		ExpValve_CLOSE();
	}
}






void SUCTION_PARAMETERS(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{
	 ExpValve_OPEN();
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

	 vTaskSuspend(APRV_Handler);
	 vTaskSuspend(APRV_one_Handler);

	 vTaskSuspend(Back_Up_PC_CMV_Mode_Handler);
	 vTaskSuspend(PID_Back_Up_PC_CMV_Mode_Handler);

	 vTaskSuspend(Back_Up_VC_CMV_Mode_Handler);
	 vTaskSuspend(PID_Back_Up_VC_CMV_Mode_Handler);

	 vTaskSuspend(One_Time_Handler);

	 vTaskSuspend(Oxygen_Handler);

	 vTaskSuspend(alert_Handler);

	 vTaskSuspend(Nebuliser_Handler);

	 vTaskSuspend(HFNC_Handler);

	 Parkar_valve_Signal(0);
	 O2.O2_process = 0;
	 O2.Servo=45;
	 Servo_Angle(O2.Servo);

	 vTaskResume(Suction_Handler);

}

/*
 * Nebuliser.c
 *
 *  Created on: Apr 6, 2022
 *      Author: asus
 */


#include "Nebuliser.h"




void NEBULISER_COMMAND_HANDLER(NEBULISER_RANGE_PACKET * RX_ALERT_RANGE_PACKET)
{

	 N._RANGE_NEBULISER_SYNC_ON_Val= (RX_ALERT_RANGE_PACKET->NEBULISER_SYNC)>>7;
	 N._RANGE_NEBULISER_ON_Val= (RX_ALERT_RANGE_PACKET->NEBULISER_ON)>>7;
	 N._TAB_CHARGER= (RX_ALERT_RANGE_PACKET->TAB_CHARGER)>>7;

	 N._FIND_MY_DEVICE= (RX_ALERT_RANGE_PACKET->FIND_MY_DEVICE)>>7;
	 N._SHUT_DOWN= (RX_ALERT_RANGE_PACKET->SHUT_DOWN)>>7;
	 N._ALERT_SNOOZE= (RX_ALERT_RANGE_PACKET->ALERT_SNOOZE)>>7;
	 //N._INSPIRATORY_HOLD= (RX_ALERT_RANGE_PACKET->INSPIRATORY_HOLD)>>7;
	 //N._EXPIRATORY_HOLD = (RX_ALERT_RANGE_PACKET->EXPIRATORY_HOLD)>>7;
	 N._READ_SENSOR_OFFSET = (RX_ALERT_RANGE_PACKET->READ_SENSOR_OFFSET)>>7;


	 if(N._SHUT_DOWN==1)
	 {
		 vTaskResume(Shutdown_Handler);
	 }
	 if(N._READ_SENSOR_OFFSET == 1)
	 {
	 	Sensor_Voltage_Transmit();
	 }

	 if(N._TAB_CHARGER==1)
	 {
	    Tab_Charger_ON();
	 }
	 else if(N._TAB_CHARGER==0)
	 {
	    Tab_Charger_OFF();
	 }
	 if(N._RANGE_NEBULISER_ON_Val==1)
	 {
		 vTaskResume(Nebuliser_Handler);
	 }
	 else if(N._RANGE_NEBULISER_ON_Val==0)
	 {
		 vTaskResume(Nebuliser_Handler);

	 }

	 if(N._FIND_MY_DEVICE==1)
	 {
		 Buzzer1_ON();
	 }
	 else if(N._FIND_MY_DEVICE==0)
	 {
		 Buzzer1_OFF();
	 }







}



void Shutdown_Task(void *argument)
{
	while(1)
	{
		Call_Shutdown_Func();
	}
}



void Call_Shutdown_Func()
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
		vTaskSuspend(APRV_one_Handler);
		vTaskSuspend(APRV_Handler);
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

  	  vTaskSuspend(Shutdown_Handler);
}

void Nebuliser_Task (void *argument)
{
	while(1)
	{
		if(S1._Mode_Val!=0 && S1._Pause==0)
	    {
			Nebuliser_Func();
	    }

		vTaskDelay(100);
	}
}



void Nebuliser_Func()
{

	        if(N._RANGE_NEBULISER_ON_Val==1)
	        {
	        	if(N._RANGE_NEBULISER_SYNC_ON_Val==0)
	        	{
	        		Nebuliser_ON();
	        	}
	        	else
	        	{
	        		if(_CurrentComputationState==Compute_I_Wave)
	        		{
	        			Nebuliser_ON();
	        		}
	                else if(_CurrentComputationState==Compute_E_Wave)
	               {
	                	Nebuliser_OFF();

	               }
	         	}
	        }
	        else
	        {
	        	Nebuliser_OFF();
	        	vTaskSuspend(Nebuliser_Handler);
	        }

}


uint8_t chksum8_cal_neb(const unsigned char *buff, size_t len)
{
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

void Sensor_Voltage_Transmit()
{

	Pressure_sensor._Pressure_Sensor_Offset_Val = Pressure_sensor._Runtime_Pressure_Val;
	Flow_sensor.AW_Flow_Offset                  = Flow_sensor.AW_flow_milli_volt;

	Sensor_Voltage._header         				 = 0x5058 ;
	Sensor_Voltage._length        				 = sizeof(Sensor_Voltage)-4 ;
	Sensor_Voltage._Dummy_1                      = 0;
	Sensor_Voltage._Pressure_Sensor_Voltage_Val  = Pressure_sensor._Runtime_Pressure_Val;
	Sensor_Voltage._Flow_Sensor_Voltage_Val      = Flow_sensor.AW_flow_milli_volt;
	Sensor_Voltage._O2_Flow_Sensor_Voltage_Val   = 0;
	Sensor_Voltage._Dummy_2                      = 0;
	Sensor_Voltage._CRC8                         = chksum8_cal_neb(&Sensor_Voltage._Dummy_1,Sensor_Voltage._length);



#if UART==6
		  	HAL_UART_Transmit_IT(&huart6,(uint8_t*)&Sensor_Voltage,sizeof(Sensor_Voltage));
#endif
#if UART==5
		  	HAL_UART_Transmit_IT(&huart5,(uint8_t*)&Sensor_Voltage,sizeof(Sensor_Voltage));
#endif
		  	CDC_Transmit_FS((uint8_t*)&Sensor_Voltage,sizeof(Sensor_Voltage));

		  	N._READ_SENSOR_OFFSET =0;

}



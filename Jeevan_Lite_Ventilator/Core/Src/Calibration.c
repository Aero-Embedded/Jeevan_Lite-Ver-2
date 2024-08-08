/*
 * Calibration.c
 *
 *  Created on: Apr 2, 2022
 *      Author: asus
 */


#include "Calibration.h"



float _AVG_CirusO2Sensor_Dummy;
float _AVG_CirusO2Sensor_Total;



void CALIBRATION_COMMAND_HANDLER(REQUEST_CALIBRATION_PACKET_tst *RX_CALIBRATION_RANGE_PACKET)
{

	C.BLOWER= (RX_CALIBRATION_RANGE_PACKET->_BLOWER);
	C.PRESSURE_SENSOR= (RX_CALIBRATION_RANGE_PACKET->_PRESSURE_SENSOR);
	C.FLOW_SENSOR_7002= (RX_CALIBRATION_RANGE_PACKET->_FLOW_SENSOR_7002);
	C.LEAK_VALVE_TEST= (RX_CALIBRATION_RANGE_PACKET->_LEAK_VALVE_TEST);
	C.O2_CHECK= (RX_CALIBRATION_RANGE_PACKET->_O2_CHECK);
	C.ALARAM_TEST= (RX_CALIBRATION_RANGE_PACKET->_ALARAM_TEST);
	C.BATTERY_TEST= (RX_CALIBRATION_RANGE_PACKET->_BATTERY_TEST);

	_RESPOND_CALIBRATION_PACKET._REPORT0=0;
	_RESPOND_CALIBRATION_PACKET._PRESSURE=0;
	_RESPOND_CALIBRATION_PACKET._FLOW=0;
	_RESPOND_CALIBRATION_PACKET._O2FLOW=0;
	_RESPOND_CALIBRATION_PACKET._LEAK=0;
	_RESPOND_CALIBRATION_PACKET._BATTERY=0;


    _AVG_CirusO2Sensor_Total = 0;
    O2._AVG_CirusO2Sensor    = 0;
    _AVG_CirusO2Sensor_Dummy = 0;

    N._READ_SENSOR_OFFSET = 1;

	vTaskResume(Calibration_Handler);

}



uint8_t chksum8_cal(const unsigned char *buff, size_t len)
{
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}


void SEND_REPORT_PACKET()
{

	_RESPOND_CALIBRATION_PACKET._header = 0x5053;
	_RESPOND_CALIBRATION_PACKET._length = 0x08;
	_RESPOND_CALIBRATION_PACKET._CRC8   = chksum8_cal((unsigned char*)&_RESPOND_CALIBRATION_PACKET._REPORT0,_RESPOND_CALIBRATION_PACKET._length);
#if UART==6
		  	HAL_UART_Transmit(&huart6,(uint8_t*)&_RESPOND_CALIBRATION_PACKET,sizeof(_RESPOND_CALIBRATION_PACKET),500);
#endif
#if UART==5
		  	HAL_UART_Transmit(&huart5,(uint8_t*)&_RESPOND_CALIBRATION_PACKET,sizeof(_RESPOND_CALIBRATION_PACKET),300);
#endif
		  	CDC_Transmit_FS((uint8_t*)&_RESPOND_CALIBRATION_PACKET,sizeof(_RESPOND_CALIBRATION_PACKET));
		  	C.Uart_Calib=0;

}


void CALIBRATION_Task(void *argument)
{

	while(1)
	{

		if(C.BLOWER==1)
		{
			CALIBRATION_Blower();

		}
		else if(C.PRESSURE_SENSOR==1)
		{
			CALIBRATION_Pressure_Sensor();
		}
		else if(C.FLOW_SENSOR_7002==1)
		{
			CALIBRATION_Proximal_Flow_Sensor();

		}
		else if(C.LEAK_VALVE_TEST==1)
		{
			CALIBRATION_Exp_valve();
		}
		else if(C.O2_CHECK==1)
		{
			CALIBRATION_Oxygen();
		}
		else if(C.ALARAM_TEST==1)
		{
			CALIBRATION_Led();
		}
		else if(C.BATTERY_TEST==1)
		{
			CALIBRATION_Battery();
		}
		vTaskDelay(1);
	}
}



void CALIBRATION_Blower(void)
{

	if(Pressure_sensor._Pressure_Val<60)
	{
		C.error_count++;
		Blower_ON();
		ExpValve_CLOSE();
		Blower_Signal(1800);
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;
	}
	else
	{
		Blower_Signal(0);
		ExpValve_OPEN();
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x01 ;
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;

		vTaskDelay(2000);

		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.error_count=0;
		C.BLOWER=0;
		vTaskSuspend(Calibration_Handler);
	 }
	 if(C.error_count>4000)
	{
		Blower_Signal(0);
		ExpValve_OPEN();
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00 ;
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;

		vTaskDelay(2000);

		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.error_count=0;
		C.BLOWER=0;
		vTaskSuspend(Calibration_Handler);
	 }
}

void CALIBRATION_Pressure_Sensor(void)
{

	 if(Pressure_sensor._Pressure_Val<60)
	 {
		C.error_count++;
		Blower_ON();
		ExpValve_CLOSE();
		Blower_Signal(1800);
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;
	 }
	 else
	 {
		Blower_Signal(0);
		ExpValve_OPEN();
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x02 ;
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;
		vTaskDelay(2000);
		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.error_count=0;
		C.PRESSURE_SENSOR=0;
		vTaskSuspend(Calibration_Handler);
	 }
	 if(C.error_count>4000)
	 {

		Blower_Signal(0);
		ExpValve_OPEN();
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00 ;
		_RESPOND_CALIBRATION_PACKET._PRESSURE =Pressure_sensor._Pressure_Val ;

		vTaskDelay(2000);
		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.error_count=0;
		C.PRESSURE_SENSOR=0;
		vTaskSuspend(Calibration_Handler);
	 }
}


void CALIBRATION_Proximal_Flow_Sensor()
{

	  if (Flow_Sensor_cal._Flow_Val < 30  )
	  {
	      C.error_count++;
		  Blower_ON();
		  ExpValve_CLOSE();
		  Blower_Signal(1300);
		  _RESPOND_CALIBRATION_PACKET._FLOW = Flow_Sensor_cal._Flow_Val;
	  }
	  else
	  {
		   Blower_Signal(0);
		   ExpValve_OPEN();
		   _RESPOND_CALIBRATION_PACKET._REPORT0 = 0x04;
		   _RESPOND_CALIBRATION_PACKET._FLOW = Flow_Sensor_cal._Flow_Val;
		   vTaskDelay(2000);
		   N._READ_SENSOR_OFFSET = 0;
		   C.Uart_Calib = 1;
		   vTaskDelay(1);
		   C.error_count=0;
		   C.FLOW_SENSOR_7002=0;
		   vTaskSuspend(Calibration_Handler);
	   }
	   if (C.error_count > 4000)
	   {
			Blower_Signal(0);
			ExpValve_OPEN();
			_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00;
			_RESPOND_CALIBRATION_PACKET._FLOW = Flow_Sensor_cal._Flow_Val;

			vTaskDelay(2000);
			N._READ_SENSOR_OFFSET = 0;
			C.Uart_Calib = 1;
			vTaskDelay(1);
			C.error_count = 0;
			C.FLOW_SENSOR_7002=0;
			vTaskSuspend(Calibration_Handler);
		}

}



void CALIBRATION_Exp_valve()
{


	Blower_ON();
	ExpValve_CLOSE();
	Blower_Signal(800);
	vTaskDelay(1000);
	Blower_Signal(0);
	vTaskDelay(1000);
	C.temp_Pressure_Val1=Pressure_sensor._Pressure_Val;
	ExpValve_OPEN();
	vTaskDelay(1000);
	ExpValve_CLOSE();
	Blower_Signal(800);
	vTaskDelay(1000);
	Blower_Signal(0);
	vTaskDelay(1000);
	C.temp_Pressure_Val2=Pressure_sensor._Pressure_Val;
	vTaskDelay(1000);
	C.temp_Pressure_Val2=Pressure_sensor._Pressure_Val;
	ExpValve_OPEN();

	C.total_temp_Pressure_Val=C.temp_Pressure_Val1-C.temp_Pressure_Val2;
	if(C.total_temp_Pressure_Val<=5)
	{
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x08 ;
		_RESPOND_CALIBRATION_PACKET._LEAK = C.total_temp_Pressure_Val ;
		vTaskDelay(2000);
		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.LEAK_VALVE_TEST=0;
		ExpValve_OPEN();
		vTaskSuspend(Calibration_Handler);
	}
	else
	{
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00 ;
		_RESPOND_CALIBRATION_PACKET._LEAK = C.total_temp_Pressure_Val ;
		vTaskDelay(2000);
		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.LEAK_VALVE_TEST=0;
		ExpValve_OPEN();
		vTaskSuspend(Calibration_Handler);
	}

}


void CALIBRATION_Oxygen()
{

	           if(O2.O2_percentage>=18)
			   {

	        	      vTaskDelay(20);
	        	      _RESPOND_CALIBRATION_PACKET._O2FLOW = O2.O2_percentage ;
	        	      vTaskDelay(20);
				     _RESPOND_CALIBRATION_PACKET._REPORT0 = 0x10 ;
				     _RESPOND_CALIBRATION_PACKET._O2FLOW = O2.O2_percentage ;
				     N._READ_SENSOR_OFFSET = 0;
				      C.Uart_Calib=1;
				      vTaskDelay(1);
				      C.O2_CHECK=0;

				      vTaskSuspend(Calibration_Handler);
				}
				else
				{

					  C.error_count++;
					  _AVG_CirusO2Sensor_Dummy = (AdcData[4] * 3300) / 4095;
					  _AVG_CirusO2Sensor_Total = _AVG_CirusO2Sensor_Total + _AVG_CirusO2Sensor_Dummy;



				      if(C.error_count>8000)
				      {

				    	  O2._AVG_CirusO2Sensor    = _AVG_CirusO2Sensor_Total /C.error_count;
						  O2.O2_percentage_float=((float)O2._AVG_CirusO2Sensor /9.2);
						  if(O2.O2_percentage_float>100)
						  	      O2.O2_percentage_float=100;
						  O2.O2_percentage=(int)O2.O2_percentage_float;

				    	     _RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00 ;
				    	     _RESPOND_CALIBRATION_PACKET._O2FLOW = O2.O2_percentage ;

				    	     _AVG_CirusO2Sensor_Total = 0;
				    	     O2._AVG_CirusO2Sensor    = 0;
				    	     _AVG_CirusO2Sensor_Dummy = 0;

				    	     N._READ_SENSOR_OFFSET = 0;
				    	     C.Uart_Calib=1;
				    	     vTaskDelay(1);
				    	     C.error_count=0;
				    	     C.O2_CHECK=0;
				    	     vTaskSuspend(Calibration_Handler);
				    	}

				  }
}



void CALIBRATION_Led()
{

	O2.O2_percentage=21;
		Red_Led_ON();
		Blue_Led_OFF();
		Green_Led_OFF();
	vTaskDelay(400);
	    Red_Led_OFF();
		Blue_Led_ON();
		Green_Led_OFF();
	vTaskDelay(400);
	    Red_Led_OFF();
	 	Blue_Led_OFF();
	 	Green_Led_ON();
	vTaskDelay(400);
	    Buzzer1_ON();
	vTaskDelay(400);
	    Buzzer1_ON();
	vTaskDelay(400);
	    Red_Led_OFF();
		Blue_Led_OFF();
		Green_Led_OFF();
		Buzzer1_OFF();
		Buzzer2_OFF();
	vTaskDelay(400);

	    _RESPOND_CALIBRATION_PACKET._REPORT0 = 0x20 ;
	    N._READ_SENSOR_OFFSET = 0;
	    C.Uart_Calib=1;
	    vTaskDelay(40);
	    C.ALARAM_TEST=0;
	    vTaskSuspend(Calibration_Handler);

}



void CALIBRATION_Battery(void *argument)
{

	if(battery>50)
	{
		vTaskDelay(40);
		_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x40;
		_RESPOND_CALIBRATION_PACKET._BATTERY = battery;
		N._READ_SENSOR_OFFSET = 0;
		C.Uart_Calib=1;
		vTaskDelay(1);
		C.BATTERY_TEST=0;
		vTaskSuspend(Calibration_Handler);
	}
	else
	{

		cal_Battery();
		C.error_count++;
		if(C.error_count>5000)
		{
			C.error_count=0;
			_RESPOND_CALIBRATION_PACKET._REPORT0 = 0x00 ;
			_RESPOND_CALIBRATION_PACKET._BATTERY = battery;
			N._READ_SENSOR_OFFSET = 0;
			C.Uart_Calib=1;
			vTaskDelay(1);
			C.BATTERY_TEST=0;
			vTaskSuspend(Calibration_Handler);

		}

	}


}


void cal_Battery()
{

        battery_raw_value=(AdcData[3]);
		battery1=((battery_raw_value-2250)*100)/1500;
		if(battery1>100)
			battery1=100;

		Bat_Avg+=battery1;
		Bat_Avg_count++;

		Battery_Information();

}

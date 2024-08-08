/*
 * Service.c
 *
 *  Created on: Apr 5, 2022
 *      Author: asus
 */


#include "Service.h"

int fault=0;


void SERVICE_COMMAND_HANDLER(REQUEST_SERVICE_PACKET_tst *RX_SERVICE_RANGE_PACKET)
{


		S.Blower= (RX_SERVICE_RANGE_PACKET->SERVICE_BLOWER);
		S.ADS1115= (RX_SERVICE_RANGE_PACKET->SERVICE_ADS1115);
		S.Pressure_Sensor= (RX_SERVICE_RANGE_PACKET->SERVICE_PRESSURE_SENSOR);
		S.Flow_Sensor= (RX_SERVICE_RANGE_PACKET->SERVICE_FLOW_SENSOR);
		S.Expiratory_Valve= (RX_SERVICE_RANGE_PACKET->SERVICE_EXPIRATORY_VALVE);
		S.E_Valve_use_hand= (RX_SERVICE_RANGE_PACKET->SERVICE_EXPIRATORY_VALVE);
		S.Connect_E_valve= (RX_SERVICE_RANGE_PACKET->SERVICE_EXPIRATORY_VALVE);
		S.Service_Leak= (RX_SERVICE_RANGE_PACKET->SERVICE_LEAK);
		S.Servo= (RX_SERVICE_RANGE_PACKET->SERVICE_SERVO_MOTOR);
		S.O2= (RX_SERVICE_RANGE_PACKET->SERVICE_O2);
		S.Nebuliser= (RX_SERVICE_RANGE_PACKET->SERVICE_NEBULISER);
		S.Remove_Test_Lung= (RX_SERVICE_RANGE_PACKET->REMOVE_TEST_LUNG);
		S.Hand_Lock=(RX_SERVICE_RANGE_PACKET->HAND_LOCK);

		_RESPOND_SERVICE_PACKET._REPORT0_FLAGS=0;
		_RESPOND_SERVICE_PACKET._PRESSURE1=0;
		_RESPOND_SERVICE_PACKET._PRESSURE2=0;
		_RESPOND_SERVICE_PACKET._FLOW=0;
		_RESPOND_SERVICE_PACKET._O2_PERCENTAGE=0;
		_RESPOND_SERVICE_PACKET._RESULT=0;

		S.Step_One=1;
		S.Step_Two=0;
		S.dac_last=500;
		S.Leak_first_Test=1;
		S.servo_step_one=1;
		fault=0;

		vTaskResume(Service_Handler);



}




void SEND_SERVICE_PACKET()
{

	_RESPOND_SERVICE_PACKET._header = 0x5055 ;
	_RESPOND_SERVICE_PACKET._length = 0x08 ;
	_RESPOND_SERVICE_PACKET._CRC8   = chksum8((unsigned char*)&_RESPOND_SERVICE_PACKET._REPORT0_FLAGS,_RESPOND_SERVICE_PACKET._length);
#if UART==6
	HAL_UART_Transmit_IT(&huart6,(uint8_t*)&_RESPOND_SERVICE_PACKET,sizeof(_RESPOND_SERVICE_PACKET));
#endif
#if UART==5
		  	HAL_UART_Transmit_IT(&huart5,(uint8_t*)&_RESPOND_SERVICE_PACKET,sizeof(_RESPOND_SERVICE_PACKET));
#endif
		  	CDC_Transmit_FS((uint8_t*)&_RESPOND_SERVICE_PACKET,sizeof(_RESPOND_SERVICE_PACKET));
		  	S.Uart_Service=0;
}


void SERVICE_Task(void *argument)
{

	while(1)
	{
		if(S.Blower==1)
		{
			SERVICE_Blower();
		}
		else if(S.ADS1115==1)
		{
			SERVICE_ADS1115();
		}
		else if(S.Pressure_Sensor==1)
		{
			SERVICE_Pressure_Sensor();
		}
		else if(S.Flow_Sensor==1)
		{
			SERVICE_Flow_Sensor();
		}
		else if(S.Expiratory_Valve==1)
		{
			SERVICE_Expiratory_Valve();
		}
		else if(S.Service_Leak==1)
		{
			SERVICE_Leak();
		}
		else if(S.Servo==1)
		{
			SERVICE_Servo();
		}
		else if(S.O2==1)
		{
			SERVICE_O2();
		}
		else if(S.Nebuliser==1)
		{
			SERVICE_Nebuliser();
		}
		vTaskDelay(1);
	}
}



void SERVICE_Blower()
{
	if(S.Step_One==1)
	{
		if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6))==0)
		{

		if(fault!=1)
		{
			S.error_count++;
			if(S.error_count>=6000)
			{
				S.error_count=0;
				S.Step_One=0;
				S.Step_Two=1;

			}
		}

		}
		else
		{
		  if(fault==0)
		  {
			S.error_count_2++;
			if(S.error_count_2>=4000)
			{
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x80 ;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.Step_One=1;
				S.error_count_2=0;
				fault=1;
				vTaskSuspend(Service_Handler);

			}
		  }
		}
	}
	else if(S.Step_Two==1)
	{
		   if(Pressure_sensor._Pressure_Val<60)
			{
				S.error_count++;
				Blower_ON();
				ExpValve_CLOSE();
				Blower_Signal(1800);
			}
			else
			{
				Blower_Signal(0);
				ExpValve_OPEN();
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
				_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.error_count=0;
				S.Blower=0;
				S.Step_One=1;
				vTaskSuspend(Service_Handler);
			 }
			 if(C.error_count>4000)
			{
				Blower_Signal(0);
				ExpValve_OPEN();
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x40 ;
				_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.error_count=0;
				S.Blower=0;
				S.Step_One=1;
				vTaskSuspend(Service_Handler);
			 }
	}

}



void SERVICE_ADS1115()
{
	if(HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1,(Flow_sensor.ADSwrite), 3, 10)!=HAL_OK)
	{
	     Flow_sensor.fault=1;
	     _RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x20 ;
	     S.Uart_Service=1;
	     vTaskDelay(20);
	     vTaskSuspend(Service_Handler);
	}
	else
	{
		Flow_sensor.fault=0;
		_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
		S.Uart_Service=1;
		vTaskDelay(20);
		vTaskSuspend(Service_Handler);
	}
}

void SERVICE_Pressure_Sensor()
{
	     if(Pressure_sensor._Pressure_Val<60)
		 {
			S.error_count++;
			Blower_ON();
			ExpValve_CLOSE();
			Blower_Signal(1800);
		 }
		 else
		 {
			Blower_Signal(0);
			ExpValve_OPEN();
			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
			_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
			S.Uart_Service=1;
			vTaskDelay(20);
			S.error_count=0;
			S.Pressure_Sensor=0;
			vTaskSuspend(Service_Handler);
		 }
		 if(C.error_count>4000)
		 {
			Blower_Signal(0);
			ExpValve_OPEN();
			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00 ;
			_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
			S.Uart_Service=1;
			vTaskDelay(20);
			S.error_count=0;
			S.Pressure_Sensor=0;
			vTaskSuspend(Service_Handler);
		 }
}
void SERVICE_Flow_Sensor()
{

	if(S.Remove_Test_Lung==1)
	{

	     if (Flow_Sensor_cal._Flow_Val < 100  )
		  {
		      S.error_count++;
			  Blower_ON();
			  ExpValve_CLOSE();
			  S.dac_last=(S.dac_last+10);
			  Blower_Signal(S.dac_last);
			  vTaskDelay(50);
		  }
		  else
		  {
			   Blower_Signal(0);
			   ExpValve_OPEN();
			   _RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01;
			   _RESPOND_SERVICE_PACKET._FLOW = Flow_Sensor_cal._Flow_Val;
			   S.Uart_Service = 1;
			   vTaskDelay(20);
			   S.error_count=0;
			   S.Flow_Sensor=0;
			   S.Remove_Test_Lung=0;
			   vTaskSuspend(Service_Handler);
		   }
		   if (C.error_count > 4000)
		   {
				Blower_Signal(0);
				ExpValve_OPEN();
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00;
				_RESPOND_SERVICE_PACKET._FLOW = Flow_Sensor_cal._Flow_Val;
				S.Uart_Service = 1;
				vTaskDelay(20);
				S.error_count = 0;
				S.Flow_Sensor=0;
				S.Remove_Test_Lung=0;
				vTaskSuspend(Service_Handler);
			}
	}
}

void SERVICE_Expiratory_Valve()
{
	if(S.Hand_Lock==1)
	{
	    Blower_ON();
		Blower_Signal(1500);
		vTaskDelay(2000);
		Blower_Signal(0);
		vTaskDelay(2000);
		S.temp_Pressure_Val1=Pressure_sensor._Pressure_Val;
		vTaskDelay(1000);
		vTaskSuspend(Service_Handler);
	}
	if(S.Hand_Lock==2)
	{
		ExpValve_CLOSE();
		Blower_Signal(1500);
		vTaskDelay(2000);
		Blower_Signal(0);
		vTaskDelay(2000);
		S.temp_Pressure_Val2=Pressure_sensor._Pressure_Val;
		vTaskDelay(1000);
		S.Hand_Lock=0;
		ExpValve_OPEN();
		vTaskDelay(1000);
	}
	if(S.Hand_Lock==0)
	{

		S.total_temp_Pressure_Val=S.temp_Pressure_Val1-S.temp_Pressure_Val2;
		if(S.total_temp_Pressure_Val<=5)
		{
			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
			_RESPOND_SERVICE_PACKET._PRESSURE1 = S.temp_Pressure_Val1 ;
			_RESPOND_SERVICE_PACKET._PRESSURE2 = S.temp_Pressure_Val2 ;
			_RESPOND_SERVICE_PACKET._RESULT = S.total_temp_Pressure_Val;
			S.Uart_Service=1;
			vTaskDelay(20);
			S.Expiratory_Valve=0;
			ExpValve_OPEN();
			vTaskSuspend(Service_Handler);
		}
		else
		{
			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00 ;
			_RESPOND_SERVICE_PACKET._PRESSURE1 = S.temp_Pressure_Val1 ;
			_RESPOND_SERVICE_PACKET._PRESSURE2 = S.temp_Pressure_Val2 ;
			_RESPOND_SERVICE_PACKET._RESULT = S.total_temp_Pressure_Val;
			S.Uart_Service=1;
			vTaskDelay(20);
			S.Expiratory_Valve=0;
			ExpValve_OPEN();
			vTaskSuspend(Service_Handler);
		}
	}
}

void SERVICE_Leak()
{
	    if(S.Leak_first_Test==1)
		{
	    	ExpValve_CLOSE();
		    Blower_ON();
			Blower_Signal(1500);
			vTaskDelay(2000);
			Blower_Signal(0);
			vTaskDelay(2000);
			S.temp_Pressure_Val1=Pressure_sensor._Pressure_Val;
			ExpValve_OPEN();
			vTaskDelay(1000);
			S.Leak_first_Test=2;
		}
		if(S.Leak_first_Test==2)
		{
			ExpValve_CLOSE();
			Blower_Signal(1500);
			vTaskDelay(2000);
			Blower_Signal(0);
			vTaskDelay(4000);
			S.temp_Pressure_Val2=Pressure_sensor._Pressure_Val;
			vTaskDelay(1000);
			S.Leak_first_Test=0;
			ExpValve_OPEN();
			vTaskDelay(1000);
			S.Leak_first_Test=0;
		}
		if(S.Leak_first_Test==0)
		{

			S.total_temp_Pressure_Val=S.temp_Pressure_Val1-S.temp_Pressure_Val2;
			if(S.total_temp_Pressure_Val<=5)
			{
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
				_RESPOND_SERVICE_PACKET._PRESSURE1 = S.temp_Pressure_Val1 ;
				_RESPOND_SERVICE_PACKET._PRESSURE2 = S.temp_Pressure_Val2 ;
				_RESPOND_SERVICE_PACKET._RESULT = S.total_temp_Pressure_Val;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.Service_Leak=0;
				ExpValve_OPEN();
				vTaskSuspend(Service_Handler);
			}
			else
			{
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00 ;
				_RESPOND_SERVICE_PACKET._PRESSURE1 = S.temp_Pressure_Val1 ;
				_RESPOND_SERVICE_PACKET._PRESSURE2 = S.temp_Pressure_Val2 ;
				_RESPOND_SERVICE_PACKET._RESULT = S.total_temp_Pressure_Val;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.Service_Leak=0;
				ExpValve_OPEN();
				vTaskSuspend(Service_Handler);
			}
		}

}
void SERVICE_Servo()
{
	if(S.servo_step_one==1)
	{
		TIM12->CCR1=90;
		vTaskDelay(2000);
		TIM12->CCR1=65;
		vTaskDelay(2000);
		TIM12->CCR1=45;
		vTaskDelay(2000);
		TIM12->CCR1=90;
		vTaskDelay(2000);
		S.servo_step_one=0;
	}
	else
    {

             if(Pressure_sensor._Pressure_Val<20)
    		 {
    			S.error_count++;
    			Blower_ON();
    			ExpValve_CLOSE();
    			Blower_Signal(1800);
    		 }
    		 else
    		 {
    			TIM12->CCR1=45;
    			Blower_Signal(0);
    			ExpValve_OPEN();
    			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00 ;
    			_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
    			S.Uart_Service=1;
    			vTaskDelay(20);
    			S.error_count=0;
    			S.Servo=0;
    			vTaskSuspend(Service_Handler);
    		 }
    		 if(S.error_count>4000)
    		 {
    			TIM12->CCR1=45;
    			Blower_Signal(0);
    			ExpValve_OPEN();
    			_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
    			_RESPOND_SERVICE_PACKET._PRESSURE1 =Pressure_sensor._Pressure_Val ;
    			S.Uart_Service=1;
    			vTaskDelay(20);
    			S.error_count=0;
    			S.Servo=0;
    			vTaskSuspend(Service_Handler);
    		 }

    }
}

void SERVICE_O2()
{
	TIM12->CCR1=90;
    if(S.O2_Acheived_Count<5)
    {
		O2_Flow_Func();
		if(O2_F.O2_kpa>5)
		{
			Blower_Signal(0);
			Parkar_valve_Signal(0);
			O2_F.O2_kpa=0;
			S.error_count=0;
			S.O2_Acheived_Count++;
			vTaskDelay(1000);
		}
		else
		{
			O2._AVG_CirusO2Sensor = (AdcData[1] * 3300) / 4095;
			O2.O2_percentage_float=(O2._AVG_CirusO2Sensor/7.57);
			O2.O2_percentage=(int)O2.O2_percentage_float;

			Blower_Signal(2000);
			Parkar_valve_Signal(2800);
			S.error_count++;
			if(S.error_count>5000)
			{
				_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x00 ;
			    _RESPOND_SERVICE_PACKET._FLOW =O2_F.O2_kpa ;
			    _RESPOND_SERVICE_PACKET._O2_PERCENTAGE=O2.O2_percentage;
				S.Uart_Service=1;
				vTaskDelay(20);
				S.error_count=0;
				S.O2_Acheived_Count=0;
				TIM12->CCR1=45;
				S.O2=0;
				vTaskSuspend(Service_Handler);
			}

		}
    }
    else if(S.O2_Acheived_Count>=5)
    {
    	TIM12->CCR1=45;
    	_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
    	_RESPOND_SERVICE_PACKET._FLOW =O2_F.O2_kpa ;
    	_RESPOND_SERVICE_PACKET._O2_PERCENTAGE=O2.O2_percentage;
    	S.Uart_Service=1;
    	vTaskDelay(20);
    	S.error_count=0;
    	S.O2_Acheived_Count=0;
    	TIM12->CCR1=45;
    	S.O2=0;
    	vTaskSuspend(Service_Handler);
    }

}

void SERVICE_Nebuliser()
{
	Nebuliser_ON();
	vTaskDelay(1000);
	Nebuliser_OFF();
	vTaskDelay(1000);
	Nebuliser_ON();
	vTaskDelay(1000);
	Nebuliser_OFF();
	vTaskDelay(1000);
	Nebuliser_ON();
	vTaskDelay(1000);
	Nebuliser_OFF();
	vTaskDelay(1000);
	_RESPOND_SERVICE_PACKET._REPORT0_FLAGS = 0x01 ;
	S.Uart_Service=1;
	vTaskDelay(20);
	S.Nebuliser=0;
	vTaskSuspend(Service_Handler);
}

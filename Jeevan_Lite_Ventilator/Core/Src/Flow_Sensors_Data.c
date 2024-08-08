/*
 * Flow_Sensors_Data.c
 *
 *  Created on: Mar 7, 2022
 *      Author: asus
 */

#include "Flow_Sensors_Data.h"


void Get_Volume()
{

	vol.Flow_Volume=(Flow_Sensor_cal.Flow1/60.00)*2;
	vol.Volume=vol.Volume+vol.Flow_Volume;
	vol.Volume_Val=(int)vol.Volume;
	if(V_max<vol.Volume_Val)
	{
		V_max=vol.Volume_Val;
	}


}

float AW_flow_raw_Volt(uint16_t r)
{
	float volt=0;
	volt=(r*6144.00)/32768;
	return volt;
}

uint16_t AW_flow_moving_average(uint16_t value)
{
	    Flow_sensor.AW_flow_raw1_new = value;

	    Flow_sensor.AW_flow_raw1_new = ((Flow_sensor.AW_flow_raw1_new-Flow_sensor.AW_flow_raw_old)/3.00)+Flow_sensor.AW_flow_raw_old;

	    Flow_sensor.AW_flow_raw_old = Flow_sensor.AW_flow_raw1_new;

		return Flow_sensor.AW_flow_raw1_new;
}

uint16_t ADS1115_AW_flow_sensor()
{
		uint16_t  ADCraw=0;


		Flow_sensor.ADSwrite[0] = 0x01;

		Flow_sensor.ADSwrite[1] =0x40;

		Flow_sensor.ADSwrite[2] = 0xE3;

      	if(HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1,(Flow_sensor.ADSwrite), 3, 10)!=HAL_OK)
      	{
      		Flow_sensor.fault=1;
      	}

      	Flow_sensor.ADSwrite[0] = 0x00;


		HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, (Flow_sensor.ADSwrite), 1, 10);

		HAL_I2C_Master_Receive(&hi2c2, (ADS1115_ADDRESS<<1),(Flow_sensor.ADSread),2,10);


		ADCraw = ((Flow_sensor.ADSread[0]) << 8 | (Flow_sensor.ADSread[1]));

		return   ADCraw;
}

void Get_AW_Flow(void)
{

	    Flow_sensor.AW_flow_raw = ADS1115_AW_flow_sensor();

	    Flow_sensor.AW_flow_raw_filtered = AW_flow_moving_average(Flow_sensor.AW_flow_raw);

	 	Flow_sensor.AW_flow_milli_volt = AW_flow_raw_Volt(Flow_sensor.AW_flow_raw_filtered);
}

void Flow_Sensor_7002_offset(void)
{
	for(int i=0;i<1000;i++)
	{
			Get_AW_Flow();
			Flow_sensor.AW_Flow_Offset=Flow_sensor.AW_flow_milli_volt;
			vTaskDelay(2);
	}
	xSemaphoreGive(binarysem);
}





long adj(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void _Flow()
{

	    const int numReadings  = 15;
	    Flow_Sensor_cal.delp_flow1 =(Flow_sensor.AW_flow_milli_volt-Flow_sensor.AW_Flow_Offset)/1000.00;
	    Flow_Sensor_cal.dp=Flow_Sensor_cal.delp_flow1*10;


		if(Flow_Sensor_cal.dp>0)
		{
			Flow_Sensor_cal.Flow1=0.1512*(Flow_Sensor_cal.dp)*(Flow_Sensor_cal.dp)*(Flow_Sensor_cal.dp)-3.3424*(Flow_Sensor_cal.dp)*(Flow_Sensor_cal.dp)+41.657*(Flow_Sensor_cal.dp);
			Flow_Sensor_cal.Flow1=adj(Flow_Sensor_cal.Flow1,0,160,0,185);
		}
		else if(Flow_Sensor_cal.dp<0)
		{
			Flow_Sensor_cal.dp*=-1;
			Flow_Sensor_cal.Flow1=0.1512*Flow_Sensor_cal.dp*Flow_Sensor_cal.dp*Flow_Sensor_cal.dp-3.3424*Flow_Sensor_cal.dp*Flow_Sensor_cal.dp+41.657*Flow_Sensor_cal.dp;
			Flow_Sensor_cal.Flow1=adj(Flow_Sensor_cal.Flow1,0,160,0,185);
			Flow_Sensor_cal.Flow1*=-1;

		}


		      Flow_Sensor_cal.total = Flow_Sensor_cal.total - Flow_Sensor_cal.readings[Flow_Sensor_cal.readIndex];
		      Flow_Sensor_cal.readings[Flow_Sensor_cal.readIndex] = Flow_Sensor_cal.Flow1;
		      Flow_Sensor_cal.total = Flow_Sensor_cal.total + Flow_Sensor_cal.readings[Flow_Sensor_cal.readIndex];
		      Flow_Sensor_cal.readIndex = Flow_Sensor_cal.readIndex + 1;
			  if (Flow_Sensor_cal.readIndex >= numReadings)
			  {
				  Flow_Sensor_cal.readIndex = 0;
			  }
			  Flow_Sensor_cal.average = Flow_Sensor_cal.total / numReadings;
			  Flow_Sensor_cal.Flow1= Flow_Sensor_cal.average;
		if((Flow_Sensor_cal.Flow1<1.2)&&(Flow_Sensor_cal.Flow1>-1.2))
	    {
			Flow_Sensor_cal.Flow1=0;
		}

		    Flow_Sensor_cal._Flow_Val=(int)Flow_Sensor_cal.Flow1;


}

void  Flow_Sensor_Value()
{
	Get_AW_Flow();

	_Flow();

}

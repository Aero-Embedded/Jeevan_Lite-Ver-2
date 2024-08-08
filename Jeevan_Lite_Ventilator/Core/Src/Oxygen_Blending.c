/*
 * Oxygen_Blending.c
 *
 *  Created on: Mar 30, 2022
 *      Author: asus
 */


#include "Oxygen_Blending.h"




O2_PARAMETER O2={0,45,1800,1,0,300,30};



void O2_Func()
{
	  if(_CurrentComputationState==Compute_I_Wave)
	  {
		if(O2.oxygen_Acheived!=1)
		{
			//O2_Flow_Func();
		}

		Servo_Angle(O2.Servo);
		if(O2.FiO2_old!=O2._FIO2_Val)
		{

			O2.O2_DAC=(((9.3452*(O2._FIO2_Val)) + 1586.3095));
			O2.FiO2_old=O2._FIO2_Val;
		}
		if (O2.O2_process == 1)
		{
			    Servo_Angle(O2.Servo);
				if (O2.O2_percentage < (O2._FIO2_Val -2 ))
				{


					O2._FIO2_Val_float=O2._FIO2_Val;
					O2.O2_percentage_float=O2.O2_percentage;
					O2.Fio2_Acheived_Percentage=(O2.O2_percentage_float/O2._FIO2_Val_float)*100;


					if(O2._FIO2_Val>40)
					{
						if(O2.Fio2_Acheived_Percentage2<70)
						{
							O2.O2_DAC = O2.O2_DAC + 80;
						}
						else if(O2.Fio2_Acheived_Percentage2>70)
						{
							O2.O2_DAC = O2.O2_DAC + 10;
						}
					}
					else
					{
						O2.O2_DAC = O2.O2_DAC + 5;
					}

					if(O2.O2_DAC>3300)
					{
						O2.O2_DAC=3300;
					}
					Parkar_valve_Signal(O2.O2_DAC);
					O2.O2_process = 0;
				}
				else if (O2.O2_percentage > (O2._FIO2_Val +2))
				{
					O2._FIO2_Val_float=O2._FIO2_Val;
					O2.O2_percentage_float=O2.O2_percentage;
					O2.Fio2_Acheived_Percentage2=(O2.O2_percentage_float/O2._FIO2_Val_float)*100;
					if(O2._FIO2_Val>40)
					{
						if(O2.Fio2_Acheived_Percentage2>130)
						{
							O2.O2_DAC = O2.O2_DAC - 80;
						}
						else if(O2.Fio2_Acheived_Percentage2<130)
						{
							O2.O2_DAC = O2.O2_DAC - 10;
						}
					}
					else
					{
						O2.O2_DAC = O2.O2_DAC - 5;
					}



					if(O2.O2_DAC<=1800)
					{
						O2.O2_DAC=1800;
					}
					Parkar_valve_Signal(O2.O2_DAC);
					O2.O2_process = 0;

				}
				else
				{
					Parkar_valve_Signal(O2.O2_DAC);
					O2.O2_process = 0;

				}
	  }

			if(O2._Pressure_Base==1)
			{
				if(Pressure_sensor._Pressure_Val>=O2._PIP_Val)
				{

					Parkar_valve_Signal(0);
					O2.oxygen_Acheived=1;

				}
			}
			if(O2._Flow_Base==1)
			{
				if(vol.Volume>=(O2._VT_Val-check_dev))
				{
						Parkar_valve_Signal(0);
						O2.oxygen_Acheived=1;
				}
			}



		}
	     if(_CurrentComputationState==Compute_E_Wave)
	  	 {
	    	 O2.O2_process=1;
			 Parkar_valve_Signal(0);

		 }
}



void Oxygen_Task(void *argument)
{
	while(1)
	{

	  if (O2._FIO2_Val > 21)
	  {
		if(S1._Pause==0 && P1.Apnea_Mode==0)
		{
			if(S1._Mode_Val == 1 || S1._Mode_Val == 2 || S1._Mode_Val == 3 || S1._Mode_Val == 4 || S1._Mode_Val == 10 || S1._Mode_Val == 11)
			{
				O2_Func();
			}
		}

	  }
	  else
	  {

		  Parkar_valve_Signal(0);
		  O2.O2_process = 0;
		  O2.Servo=45;
		  Servo_Angle(O2.Servo);
	  }

	  vTaskDelay(2);

	}
}



void adjust_servo()
{

	                if(O2._FIO2_Val>21)
					{

	                	O2.Result=O2._AVG_CirusO2Sensor_value/O2.count;
	                	O2.O2_percentage_float=(O2.Result/9.2);

	                	if(O2.O2_percentage_float>100)
	                		O2.O2_percentage_float=100;
	                	O2.O2_percentage=(int)O2.O2_percentage_float;

	                	O2.count=10;
	                	O2._AVG_CirusO2Sensor_value=10;


					if(O2.O2_percentage<(O2._FIO2_Val-3))
					{
						O2.fio2_check++;
						if(O2.fio2_check>3)
						{
							O2.Servo=O2.Servo+5;
							O2.fio2_check=0;
						}

						if(O2.Servo>=110)
							O2.Servo=110;
					}
					else if(O2.O2_percentage>(O2._FIO2_Val+3))
					{
						O2.fio2_check2++;
						if(O2.fio2_check2>3)
						{
							O2.Servo=O2.Servo-5;
							O2.fio2_check2=0;
						}
						if(O2.Servo<=45)
							O2.Servo=45;
					}


					}


}

void Get_Oxygen()
{
	if(_CurrentComputationState==Compute_I_Wave)
	{
		if (O2._FIO2_Val > 21)
		{
			O2._AVG_CirusO2Sensor = (AdcData[1] * 3300) / 4095;
			//O2._AVG_CirusO2Sensor = (AdcData[4] * 3300) / 4095;
			O2.count++;
			O2._AVG_CirusO2Sensor_value=O2._AVG_CirusO2Sensor+O2._AVG_CirusO2Sensor_value;
		}
	}
}

void O2_Parameter()
{
			O2.count=10;
			O2._AVG_CirusO2Sensor_value=10;
			O2.O2_DAC=(((9.3452*(O2._FIO2_Val)) + 1586.3095)+100);
			if(O2._FIO2_Val==100)
			{
				O2.Servo=110;
				A.Fio2_Value_Set=6;

			}
			else if(O2._FIO2_Val>=70 && O2._FIO2_Val<95)
			{
				O2.Servo=75;
				A.Fio2_Value_Set=6;
			}
			else if(O2._FIO2_Val>21 && O2._FIO2_Val<70)
			{
				O2.Servo=50;
				A.Fio2_Value_Set=6;
			}
			else if(O2._FIO2_Val<=21)
			{
				O2.Servo=45;
				O2.O2_percentage=21;
				Servo_Angle(O2.Servo);
				Parkar_valve_Signal(1800);
			}



}





void O2_Flow_Func()
{

#if ADC_PIN==0
	O2_F._AVG_O2_Flow_Sensor = (AdcData[0] * 3300) / 4095;
#endif
#if ADC_PIN==4
	O2_F._AVG_O2_Flow_Sensor = (AdcData[4] * 3300) / 4095;
#endif
	O2_F.O2_flow_raw1=O2_F._AVG_O2_Flow_Sensor-O2_F.O2_Flow_Offset;
	O2_F.O2_volt2=O2_F.O2_flow_raw1;
	O2_F.O2_volt_new=O2_F.O2_volt2;
	O2_F.O2_volt_new=((O2_F.O2_volt_new-O2_F.O2_volt_old)/10.00)+O2_F.O2_volt_old;
	O2_F.O2_volt_old=O2_F.O2_volt_new;
	O2_F.O2_p=O2_F.O2_volt_new/90.00;
	O2_F.O2_kpa=(O2_F.O2_p/9.0)*100;
	O2_F.O2_kpa1=(int)O2_F.O2_kpa+20;

}




uint16_t O2_milli_volt1(uint16_t C_value)
{
	O2_F.Raw_O2_Flow=C_value;
	O2_F.O2_milli_volt=(O2_F.Raw_O2_Flow*3300)/4095;
	return O2_F.O2_milli_volt;
}

void O2_Flow_offset(void)
{
	for(int l=0;l<1000;l++)
	{
		#if ADC_PIN==0
			O2_F.O2_Flow_raw1 = O2_milli_volt1(AdcData[0]);
		#endif
		#if ADC_PIN==4
			O2_F.O2_Flow_raw1 = O2_milli_volt1(AdcData[4]);
		#endif
			O2_F.O2_Flow_raw1_new = O2_F.O2_Flow_raw1;
			O2_F.O2_Flow_raw1_new = ((O2_F.O2_Flow_raw1_new-O2_F.O2_Flow_raw1_old)/3.00)+O2_F.O2_Flow_raw1_old;
			O2_F.O2_Flow_raw1_old = O2_F.O2_Flow_raw1_new;
			O2_F.O2_Flow_Offset   = O2_F.O2_Flow_raw1_new;
	}
}

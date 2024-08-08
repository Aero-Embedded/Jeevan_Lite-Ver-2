/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM12_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t Dac1=300;
uint16_t Dac2=1000;


uint16_t HFNC_Dac1=1800;
uint16_t HFNC_Dac2=3000;

uint8_t kp=10;
uint8_t ki=10;
double kd=1;
int Time=50;
extern int Start_check_time;
BackupModes _CurrentBackupMode = IdleState ;
uint16_t _60_Seconds=0;
uint8_t Breath=0;
uint8_t now_check_breath=0;

uint8_t new_Breath=0;

extern float _Peep_Avg;
extern float _Peep_Avg_count;


xTaskHandle Sensor_Offset_Handler;
xTaskHandle Receiver_Handler;
xTaskHandle Two_Milli_Second_Handler;
xTaskHandle Sensor_Read_Handler;
xTaskHandle Uart_Transmit_Handler;





void Offset_Task (void *argument);
void Receiver_Task (void *argument);
void Two_Milli_Second_Task (void *argument);
void Sensor_Data_Read_Task (void *argument);





void Pressure_Min_Max()
{
		if (Pressure_sensor._Pressure_Val < (S5.P_Min))
		{
			if (S5.now_check == 1)
			{
				if(_CurrentComputationState==Compute_I_Wave)
				{

					S5.P_Min = Pressure_sensor._Pressure_Val;

				}
			}
		}
		if(Pressure_sensor._Pressure_Val >= S5.P_Max)
		{
			S5.P_Max = Pressure_sensor._Pressure_Val;
			S5.now_check = 1;
		}
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_TIM12_Init();

  /* USER CODE BEGIN 2 */
         MX_USB_DEVICE_Init();
         vSemaphoreCreateBinary(binarysem);
         Uart_Receive = xQueueCreate(1, sizeof(UART_RX_BUF));
         xTaskCreate(One_Time_Task,"One Time Execute Task", 512, NULL, 2, &One_Time_Handler);
         xTaskCreate(Offset_Task,"Sensor Offset Calculation Task", 512, NULL , 3, &Sensor_Offset_Handler);
         xTaskCreate(Receiver_Task,"BlueTooth Data Receive Task", 512, NULL, 4, &Receiver_Handler);
         xTaskCreate(Two_Milli_Second_Task,"Two millisecond Task", 512, NULL , 3, &Two_Milli_Second_Handler);
         xTaskCreate(Sensor_Data_Read_Task,"Read sensor values Task", 512, NULL, 3, &Sensor_Read_Handler);
         xTaskCreate(Uart_Transmit_Task,"BluetoothData Transmit Task", 512, NULL, 3, &Uart_Transmit_Handler);
         xTaskCreate(PC_CMV_Task,"PC CMV Mode Task", 512, NULL, 2, &pc_mode_Handler);
         xTaskCreate(PC_CMV_PID_Task,"PC CMV Mode PID Task", 512, NULL, 2, &Pc_cmv_Pid_Handler);
         xTaskCreate(Vc_Cmv_Task,"VC CMV Mode Task", 512, NULL, 2, &Vc_mode_Handler);
         xTaskCreate(Vc_cmv_PID_Task,"VC CMV Mode PID Task", 512, NULL, 2, &Vc_cmv_Pid_Handler);
         xTaskCreate(PC_SIMV_Task,"PC SIMV Mode Task", 512, NULL, 2, &Pc_simv_Mode_Handler);
         xTaskCreate(PC_SIMV_PID_Task, "PC SIMV Mode Task", 512, NULL, 2, &Pc_simv_Mode_Pid_Handler);
         xTaskCreate(VC_SIMV_Task, "VC SIMV Mode Task", 512, NULL, 2, &Vc_simv_mode_Handler);
         xTaskCreate(VC_SIMV_PID_Task, "VC SIMV Mode PID Task", 256, NULL, 2, &Vc_simv_Pid_Handler);
         xTaskCreate(PSV_Mode_Task, "PSV Mode Task", 512, NULL, 2, &Psv_Handler);
         xTaskCreate(PSV_PID_Task, "PSV PID Task", 512, NULL, 3, &Psv_Pid_Handler);
         xTaskCreate(Back_Up_PC_CMV_Mode_Task, "Back Up PC CMV Mode Task", 512, NULL, 2, &Back_Up_PC_CMV_Mode_Handler);
         xTaskCreate(PID_Back_Up_PC_CMV_Mode_Task,"PID Back Up PC CMV Mode Task", 512, NULL, 2, &PID_Back_Up_PC_CMV_Mode_Handler);
         xTaskCreate(Back_Up_VC_CMV_Mode_Task, "Back Up VC CMV Mode Task", 512, NULL, 2, &Back_Up_VC_CMV_Mode_Handler);
         xTaskCreate(PID_Back_Up_VC_CMV_Mode_Task, "PID Back Up VC CMV Mode Task", 512, NULL, 2, &PID_Back_Up_VC_CMV_Mode_Handler);
         xTaskCreate(CPAP_Mode_Task, "CPAP Mode Task", 512, NULL, 2, &Cpap_Handler);
         xTaskCreate(BIPAP_Mode_Task, "BIPAP Mode Task", 512, NULL, 2, &BiPap_Handler);
         xTaskCreate(BIPAP_PID_Mode_Task, "BIPAP PID Mode Task", 512, NULL, 3, &BiPap_Pid_Handler);
         xTaskCreate(APRV_Mode_Task, "APRV Mode Task", 512, NULL, 2, &APRV_Handler);
         xTaskCreate(APRV_Mode_One_Time_Task, "APRV Mode Task", 512, NULL, 2, &APRV_one_Handler);
         xTaskCreate(Oxygen_Task, "Oxygen Mode Task", 512, NULL, 2, &Oxygen_Handler);
         xTaskCreate(Alert_Task, "Alert Task", 512, NULL, 2, &alert_Handler);
         xTaskCreate(CALIBRATION_Task, "CALIBRATION Task", 512, NULL, 2, &Calibration_Handler);
         xTaskCreate(SERVICE_Task, "SERVICE Task", 512, NULL, 2, &Service_Handler);
         xTaskCreate(Nebuliser_Task,"Nebulizer-Task", 256, NULL, 2, &Nebuliser_Handler);
         xTaskCreate(HFNC_Task,"HFNC-Task", 256, NULL, 2, &HFNC_Handler);
         xTaskCreate(Suction_Task,"Suction-Task", 256, NULL, 2, &Suction_Handler);
         xTaskCreate(Shutdown_Task,"Shutdown-Task", 256, NULL, 3, &Shutdown_Handler);





         HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);
         Blower_Signal(0);
         HAL_DAC_Start(&hdac,DAC1_CHANNEL_2);
         Parkar_valve_Signal(0);
         HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&AdcData,5);
         HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
         Servo_Power_ON();
         Servo_Angle(45);
         Blower_ON();
         Power_Led_ON();
         Uart_Receive_Debug_Toggle_Led();
         Blue_Led_ON();
         Red_Led_OFF();
         Green_Led_OFF();


               TempSetpoint = 30;
               PID(&TPID, &Temp, &PIDOut, &TempSetpoint, kp, ki, kd, _PID_P_ON_E, _PID_CD_DIRECT);
               PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
               PID_SetSampleTime(&TPID, 50);
               PID_SetOutputLimits(&TPID, Dac1, Dac2);


         vTaskStartScheduler();
  /* USER CODE END 2 */


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1680-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD15 PD1 PD3 PD4
                           PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */






void One_Time_Task (void *argument)
{
	while (1)
	{

		switch (_CurrentWaveFormState)
		{
		case Generate_I_Wave:

			switch (S1._Mode_Val)
			{

			case 1:
				PC_CMV_Pulse_I_Parameter();
				break;
			case 2:
				VC_CMV_Pulse_I_Parameter();
				break;
			case 3:
				PC_SIMV_Pulse_I_Parameter();
			    break;
			case 4:
				VC_SIMV_Pulse_I_Parameter();
		    	break;
			case 10:
				  BACKUP_PC_SIMV_Pulse_I_Parameter();
				break;
			case 11:
				 BACKUP_VC_SIMV_Pulse_I_Parameter();
				break;

			default:
				Main_Supply_or_Battery_Indication();
				vol.Volume = 0;
				A.Alert_Now=1;
				 Flow_sensor.AW_Flow_Offset=Flow_sensor.AW_flow_milli_volt;
				vTaskDelay(2000);
				break;
			}

			break;

		case Generate_E_Wave:
			switch (S1._Mode_Val)
			{
			case 1:
				PC_CMV_Pulse_E_Parameter();
				break;
			case 2:
				VC_CMV_Pulse_E_Parameter();
				break;
			case 3:
				PC_SIMV_Pulse_E_Parameter();
				break;
			case 4:
				VC_SIMV_Pulse_E_Parameter();
				break;
			case 10:
				BACKUP_PC_SIMV_Pulse_E_Parameter();
				break;
			case 11:
				BACKUP_VC_SIMV_Pulse_E_Parameter();
				break;
			default:
				break;
			}
			break;

		case NoWaveFormState:
			break;
		default:
			break;

		}
	}

}

void Offset_Task (void *argument)
{
	while (1)
	{
xSemaphoreTake(binarysem,10);
		vTaskSuspend(pc_mode_Handler);
		vTaskSuspend(Pc_cmv_Pid_Handler);
		vTaskSuspend(Vc_mode_Handler);
	    vTaskSuspend(Vc_cmv_Pid_Handler);
	    vTaskSuspend(Pc_simv_Mode_Handler);
	    vTaskSuspend(Pc_simv_Mode_Handler);
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
	    vTaskSuspend(Oxygen_Handler);
	    vTaskSuspend(alert_Handler);
	    vTaskSuspend(Calibration_Handler);
	    vTaskSuspend(Service_Handler);
	    vTaskSuspend(Nebuliser_Handler);
	    vTaskSuspend(HFNC_Handler);
	    vTaskSuspend(Suction_Handler);
	    vTaskSuspend(Shutdown_Handler);


	    vTaskSuspend(Two_Milli_Second_Handler);
	    vTaskSuspend(Sensor_Read_Handler);
	    vTaskSuspend(Uart_Transmit_Handler);


	    vTaskDelay(1000);
	    battery_old=100;

		Pressure_Sensor_offset();
		Flow_Sensor_7002_offset();
		O2_Flow_offset();
if( (xSemaphoreTake(binarysem,4000)) == pdTRUE)
{
		vTaskResume(Two_Milli_Second_Handler);
		vTaskResume(Sensor_Read_Handler);
		vTaskResume(Uart_Transmit_Handler);
	xSemaphoreGive(binarysem);
}
		vTaskDelete(Sensor_Offset_Handler);

	}
}

void Sensor_Data_Read_Task (void *argument)
{
	while (1)
	{
		//xSemaphoreTake(binarysem,portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		Pressure_Sensor_Value();
		Temp = Pressure_sensor._Pressure_Val;
		Flow_Sensor_Value();
		Temp_Flow=Flow_Sensor_cal._Flow_Val;
		Get_Volume();
		Get_Oxygen();
		Battery_Status();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		//xSemaphoreGive(binarysem);
		vTaskDelay(2);
	}
}



void Two_Milli_Second_Task (void *argument)
{
	while (1)
	{


		if(_E_TIMER>0)
		{

			_E_TIMER=_E_TIMER-1 ;
			_E_TIMER_ACHEIVED=_E_TIMER_ACHEIVED+1;
				if(_E_TIMER<100)
				{
					S5._Peep_Avg+=Pressure_sensor._Pressure_Val;
					S5._Peep_Avg_count++;
						if(S5.peep_max<Pressure_sensor._Pressure_Val)
						{
							S5.peep_max=Pressure_sensor._Pressure_Val;
						}

						if(A.Alert==1)
							A.Red_Led_Alert=1;
						else
							A.Red_Led_Alert=0;
				}



				if(_E_TIMER_ACHEIVED>500)
				{
					S5._Peep_Avg_trigger+=Pressure_sensor._Pressure_Val;
					S5._Peep_Avg_count_trigger++;
				}

		}
		if(_I_TIMER>0)
		{
			_I_TIMER=_I_TIMER-1 ;
			_I_TIMER_ACHEIVED=_I_TIMER_ACHEIVED+1;
			    if(_I_TIMER_ACHEIVED>100)
				{
					 Pressure_Min_Max();
				}
			    if(_I_TIMER<200)
				{
					S5._Pip_Avg+=Pressure_sensor._Pressure_Val;
					S5._Pip_Avg_count++;

				}


		}
		if(P1._APNEA_COUNTER  > 0)
		{
			P1._APNEA_COUNTER=P1._APNEA_COUNTER -1 ;
			if(A.Alert==1)
				A.Red_Led_Alert=1;
			else
				A.Red_Led_Alert=0;

		}

		if(P1._Apnea_counter_trigger_check_time>0)
		{
			 P1._Apnea_counter_trigger_check_time=P1._Apnea_counter_trigger_check_time-1;
		}

		if(P1.P_LOW_TIMER>0)
		{
			P1.P_LOW_TIMER=P1.P_LOW_TIMER-1;
			if(P1.P_LOW_TIMER<100)
			{
			     _Peep_Avg+=Pressure_sensor._Pressure_Val;
			     _Peep_Avg_count++;
			}
		}
		if(P1.P_HIGH_TIMER>0)
		{
			P1.P_HIGH_TIMER=P1.P_HIGH_TIMER-1;
		}

		if(Start_check_time>0)
		{
			Start_check_time=Start_check_time-1;
		}

		if(_60_Seconds>0)
		{
			_60_Seconds=_60_Seconds-1;

			if(_60_Seconds==0)
			{
				new_Breath=Breath;
				_60_Seconds=60000;
				Breath=0;
			}

				if(  ( ! ( _Control_Byte >> 8) & 1)  )
				{
					if(now_check_breath==1)
					{
						Breath++;
						now_check_breath=0;
					}
				}


		}


		vTaskDelay(1);
	}


}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


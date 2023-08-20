/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "crc.h"
#include <string.h>
#include "tempcalc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/////////////////////////////////////////////////////////////

#warning "Should set PROBE_NUM to the number of actual uploaded probe!"
#define PROBE_NUM	20// 1 to 20
#if PROBE_NUM == 0
#warning "PROBE_NUM is 0, nominal NTC resistance will be 10K. Are you sure?"
#endif
/////////////////////////////////////////////////////////////

#define SCOLUMNS 4
#define SROWS 4
#define MAXBUFFER 256
#define ADC_SET_TIME 1
#define RX_BUFFER_SIZE 256
#define KSMOOTH 10

#define NUM_RAW_DATA		((SCOLUMNS) * (SROWS))
#define NUM_TO_COMBINE		(1)
#define NUM_TEMPERATURES	(NUM_RAW_DATA / NUM_TO_COMBINE)
#define TEMP_OFFSET 		(31)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint16_t raw_adc_data[NUM_RAW_DATA];
int16_t calculated_temperatures[NUM_RAW_DATA];
uint16_t adc_smoothed[NUM_RAW_DATA];
uint16_t smooth_raw[KSMOOTH][NUM_RAW_DATA];
uint8_t adc_cal[NUM_RAW_DATA*2];
uint16_t adc_serial=0;

int16_t temps_measured[NUM_RAW_DATA];
uint8_t temps_to_send[NUM_TEMPERATURES];

char tx_buffer[MAXBUFFER];
int rx_available;
char rx_buffer[RX_BUFFER_SIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void prepareTemp(int16_t measured, uint8_t* pdata);	// fn to turn 16 bit measured data to 8 bit prepared-to-send data
void fillTemp();
void fillTempCal();
void sendData();
void sendDataCal();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void select_adc_channel(int channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
//////////    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
    switch (channel)
    {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
              break;
        case 1:
            sConfig.Channel = ADC_CHANNEL_1;
              break;
        case 2:
            sConfig.Channel = ADC_CHANNEL_2;
              break;
        case 3:
            sConfig.Channel = ADC_CHANNEL_3;
              break;
        case 4:
            sConfig.Channel = ADC_CHANNEL_4;
              break;
        default: sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    }
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
}
//----------------------------------------------

//----------------------------------------------

void convert (int column,int row)
{
	int sensor_num = column + SCOLUMNS * row;
	select_adc_channel(column);
	HAL_GPIO_WritePin(PB4_GPIO_Port, PB4_Pin, GPIO_PIN_SET);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_adc_data[sensor_num] = HAL_ADC_GetValue(&hadc1);
//1	calculated_temperatures[sensor_num] = get_temperature_data(raw_adc_data[sensor_num]);
//3	int16_t measured_temp = get_temperature_data(raw_adc_data[sensor_num] );
/*2
	// limit measured temp to avoid wrong pointer
	if (measured_temp < 0)					measured_temp = 0;
	if (measured_temp > CAL_MAX_TEMP_KEY)	measured_temp = CAL_MAX_TEMP_KEY;
	// get correct temperature
	calculated_temperatures[sensor_num] = sensor_correct_values[sensor_num][measured_temp];
*/
//3	calculated_temperatures[sensor_num] = measured_temp;

	// pass probe num and sensor num to get unique nominal resistance for NTC
	// if PROBE_NUM is 0, default 10K will be used
	calculated_temperatures[sensor_num] = get_temperature_data(raw_adc_data[sensor_num], PROBE_NUM, sensor_num);

	HAL_GPIO_WritePin(PB4_GPIO_Port, PB4_Pin, GPIO_PIN_RESET);
	HAL_ADC_Stop(&hadc1);
}

void scan_columns(int row)
	{

		HAL_GPIO_WritePin(COL0_GPIO_Port,COL0_Pin, GPIO_PIN_SET);
		HAL_Delay(ADC_SET_TIME);
		convert(0,row);
		HAL_GPIO_WritePin(COL0_GPIO_Port,COL0_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);

		HAL_GPIO_WritePin(COL1_GPIO_Port,COL1_Pin, GPIO_PIN_SET);
		HAL_Delay(ADC_SET_TIME);
		convert(1,row);
		HAL_GPIO_WritePin(COL1_GPIO_Port,COL1_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);

		HAL_GPIO_WritePin(COL2_GPIO_Port,COL2_Pin, GPIO_PIN_SET);
		HAL_Delay(ADC_SET_TIME);
		convert(2,row);
		HAL_GPIO_WritePin(COL2_GPIO_Port,COL2_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);

		HAL_GPIO_WritePin(COL3_GPIO_Port,COL3_Pin, GPIO_PIN_SET);
		HAL_Delay(ADC_SET_TIME);
		convert(3,row);
		HAL_GPIO_WritePin(COL3_GPIO_Port,COL3_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
/*
		HAL_GPIO_WritePin(COL4_GPIO_Port,COL4_Pin, GPIO_PIN_SET);
		HAL_Delay(5);
		convert(4,row);
		HAL_GPIO_WritePin(COL4_GPIO_Port,COL4_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
*/
	}


void scan_rows()
		{
			HAL_GPIO_WritePin(R0_GPIO_Port,R0_Pin, GPIO_PIN_RESET);
			scan_columns(0);
			HAL_GPIO_WritePin(R0_GPIO_Port,R0_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin, GPIO_PIN_RESET);
			scan_columns(1);
			HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin, GPIO_PIN_RESET);
			scan_columns(2);
			HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin, GPIO_PIN_RESET);
			scan_columns(3);
			HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin, GPIO_PIN_SET);
/*
			HAL_GPIO_WritePin(R4_GPIO_Port,R4_Pin, GPIO_PIN_RESET);
			scan_columns(4);
			HAL_GPIO_WritePin(R4_GPIO_Port,R4_Pin, GPIO_PIN_SET);
*/
		}


void read_all_sensors(void)
{
	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
	scan_rows();
	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
}

void read_all_temperatures(){
	static GPIO_TypeDef* row_gpio[SROWS] = {
		R0_GPIO_Port, R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port
	};
	static uint16_t row_pin[SROWS] = {
			R0_Pin, R1_Pin, R2_Pin, R3_Pin
	};

	static GPIO_TypeDef* col_gpio[SCOLUMNS] = {
			COL0_GPIO_Port, COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port
	};

	static uint16_t col_pin[SCOLUMNS] = {
			COL0_Pin, COL1_Pin, COL2_Pin, COL3_Pin
	};



	// LED on
	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);

	// scan rows
	for (int row = 0; row < SROWS; row++){
		HAL_GPIO_WritePin(row_gpio[row], row_pin[row], GPIO_PIN_RESET);
		// scan columns
		for (int col = 0; col < SCOLUMNS; col++){
			HAL_GPIO_WritePin(col_gpio[row], col_pin[row], GPIO_PIN_SET);

			int sensor_num = col + SCOLUMNS * row;
			int n = 0;
			uint32_t value = 0;

			for (n = 0; n < 3; n++){
				HAL_Delay(ADC_SET_TIME);
				select_adc_channel(col);
				HAL_GPIO_WritePin(PB4_GPIO_Port, PB4_Pin, GPIO_PIN_SET);
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				value += HAL_ADC_GetValue(&hadc1);
				calculated_temperatures[sensor_num] = get_temperature_data(raw_adc_data[sensor_num], PROBE_NUM, sensor_num);
				HAL_GPIO_WritePin(PB4_GPIO_Port, PB4_Pin, GPIO_PIN_RESET);
				HAL_ADC_Stop(&hadc1);
			}

			raw_adc_data[sensor_num] = (uint16_t)round((double)value / n);
			calculated_temperatures[sensor_num] = get_temperature_data(raw_adc_data[sensor_num], PROBE_NUM, sensor_num);

			HAL_GPIO_WritePin(col_gpio[row], col_pin[row], GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(row_gpio[row], row_pin[row], GPIO_PIN_SET);
	}

	// LED off
	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
}

void read_smooth()
{
	int sum;
	for(int k=0;k<KSMOOTH;k++)
	{
		read_all_sensors();
		for(int m=0;m<NUM_RAW_DATA;m++) smooth_raw[k][m]=raw_adc_data[m];

	}
	adc_serial++;
	for(int n=0;n<NUM_RAW_DATA;n++)
	{
		for(int p=1;p<KSMOOTH;p++)
		{
		sum+=smooth_raw[p][n];
		}
	adc_smoothed[n]=(int)((1.0*sum)/KSMOOTH);
	}

}

void eval_comm(char comm)
{
	int s;
	s = sprintf (tx_buffer, "Received %02X\r\n",comm);
	HAL_UART_Transmit (&huart3, (uint8_t *)tx_buffer, s, 10);
	switch(comm)
	{

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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// wait for rising edge
	while (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin));

	read_all_sensors();
/* ez mar mukodott
	for (int i = 0; i < NUM_RAW_DATA; i++){

	    adc_cal[i*2] = raw_adc_data[i] >> 8;

	    adc_cal[i*2 + 1] = raw_adc_data[i] & 0xFF;

	}*/
	for (int i = 0; i < NUM_RAW_DATA; i++){

	    adc_cal[i*2] = calculated_temperatures[i] >> 8;

	    adc_cal[i*2 + 1] = calculated_temperatures[i] & 0xFF;

	}
	//memcpy(adc_cal,raw_adc_data,NUM_RAW_DATA*2);
	// fill temperatures buffer
	//fillTemp();

	// send
	//sendData();
	sendDataCal();
	// wait for falling edge
	while (!HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin));


/*	if(HAL_UART_Receive(&huart3,(uint8_t *)rx_buffer, 1, 1)!= HAL_OK)
		{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
		__HAL_UART_CLEAR_NEFLAG(&huart3);
		__HAL_UART_CLEAR_OREFLAG(&huart3);

		}
	 else eval_comm(rx_buffer[0]);
*/
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV32;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BLUE_Pin|GREEN_Pin|RED_Pin|PC7_Pin
                          |PC7C7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R1_Pin|R2_Pin|R3_Pin|R4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB12_Pin|PB13_Pin|PB15_Pin|PB3_Pin
                          |PB4_Pin|COL4_Pin|COL3_Pin|COL2_Pin
                          |COL1_Pin|COL0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA8_Pin|PA11_Pin|PA12_Pin|PA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PD0_Pin|PD1_Pin|PD2_Pin|PD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BLUE_Pin GREEN_Pin RED_Pin PC7_Pin
                           PC7C7_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin|GREEN_Pin|RED_Pin|PC7_Pin
                          |PC7C7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : START_Pin */
  GPIO_InitStruct.Pin = START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin PA8_Pin PA11_Pin PA12_Pin
                           PA15_Pin */
  GPIO_InitStruct.Pin = R0_Pin|PA8_Pin|PA11_Pin|PA12_Pin
                          |PA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin
                           PB12_Pin PB13_Pin PB15_Pin PB3_Pin
                           PB4_Pin COL4_Pin COL3_Pin COL2_Pin
                           COL1_Pin COL0_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin
                          |PB12_Pin|PB13_Pin|PB15_Pin|PB3_Pin
                          |PB4_Pin|COL4_Pin|COL3_Pin|COL2_Pin
                          |COL1_Pin|COL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0_Pin PD1_Pin PD2_Pin PD3_Pin */
  GPIO_InitStruct.Pin = PD0_Pin|PD1_Pin|PD2_Pin|PD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 *
 *
 * own function definitions
 *
 *
 */

int16_t min(int16_t a, int16_t b){
	if (a > b) return b;
	return a;
}

int16_t max(int16_t a, int16_t b){
	if (a > b) return a;
	return b;
}

void prepareTemp(int16_t measured, uint8_t* pPrepared){
	*pPrepared = min(max((measured + TEMP_OFFSET), 1), 254);
}

void fillTemp(){
	int mltp = (NUM_RAW_DATA / NUM_TEMPERATURES);
	int base = NUM_RAW_DATA / mltp;
	for (int i = 0; i < base; i++){
		int16_t value = 0;
		for (int j = 0; j < mltp; j++){
			value += temps_measured[i * mltp + j];
		}
		value /= mltp;
		prepareTemp(value, &temps_to_send[i]);
	}
}

void sendData(){
	uint8_t packet_len = NUM_TEMPERATURES + 2;
	uint8_t packet[packet_len];
	for (int i = 0; i < NUM_TEMPERATURES; i++){
	    packet[i * 2] = temps_to_send[i] >> 8;
	    packet[i * 2 + 1] = temps_to_send[i] & 0xFF;
	}
	uint16_t crc = calculateCRC16(temps_to_send, NUM_TEMPERATURES);
	packet[packet_len - 2] = crc >> 8;
	packet[packet_len - 1] = crc & 0xFF;
	HAL_UART_Transmit (&huart3, packet, packet_len, 10);
}

void sendDataCal(){
	uint8_t packet_len = NUM_TEMPERATURES*2 + 2;
	uint8_t packet[packet_len];
	memcpy(packet,adc_cal,NUM_TEMPERATURES*2);
	uint16_t crc = calculateCRC16(packet, NUM_TEMPERATURES*2);
	packet[packet_len - 2] = crc >> 8;
	packet[packet_len - 1] = crc & 0xFF;
	HAL_UART_Transmit (&huart3, packet, packet_len, 10);
}

/* USER CODE END 4 */

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

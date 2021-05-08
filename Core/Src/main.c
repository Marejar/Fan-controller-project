/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2_i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE 						1
#define DISABLE 					0

#define ENABLED 					1
#define DISABLED 					0

#define TRUE 						1
#define FALSE 						0

#define VOLTAGE_RISE				0.01 	//STORES VALUE OF VOLTAGE THAT RISE WITH EVERY CELSIUS DEEGREE FOR LM35
#define ADC_REFERENCE_VOLTAGE 		3.3 	//STORES REFERENCE VOLTAGE VALUE OF ADC
#define ADC_RESOLUTION				4096	//STORES ADC RESOLUTION VALUE

#define MAX_THRESHOLD_TEMPERATURE	33		//STORES MAXIMUM THRESHOLD TEMPERATURE
#define MIN_THRESHOLD_TEMPERATURE	19		//STORES MINIMUM THRESHOLD TEMPERATURE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t adc_buff;							//stores ADC readed value
char adc_value_msg[60];						//stores message send via USART2 to HC-05 module
volatile int temperature_threshold = 25; 	//stores limit temperature value at which fan starts
uint8_t fan_state = DISABLED;				//stores information about fan state
int temperature_hysteresis; 				//stores limit value at which fan stops
double voltage;								//stores value of voltage of temperature sensor
double temperature;							//stores value of temperature measured by the sensor
uint16_t systick_cnt = 0;					//stores value of systick inrementations, used in delay function

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void TIM4_Init(void);
void TIM5_Init(void);

void LCD_REFRESH(void);
void START_FAN(void);
void STOP_FAN(void);
double CONV_ADC_TO_TEMPERATURE(int adc_temperature_value);
void DELAY(uint8_t delay);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  TIM4_Init();
  TIM5_Init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim4);

  lcd16x2_i2c_init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		temperature_hysteresis = temperature_threshold-2;

	  if(CONV_ADC_TO_TEMPERATURE((int)adc_buff) > temperature_threshold && fan_state == DISABLED){

		  fan_state = ENABLED;
		  START_FAN();

	  }else if(CONV_ADC_TO_TEMPERATURE((int)adc_buff) <= temperature_hysteresis && fan_state == ENABLED){

		  fan_state = DISABLED;
		  STOP_FAN();

	  }
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @note System clock is 84MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  * @note causes interrupt every 10ms
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 840000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO Pin : PA10 (fan Enable/Disable Pin)*/
   GPIO_InitStruct.Pin = GPIO_PIN_10;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO Pin : PB5 (Increasing threshold temperature)*/
   GPIO_InitStruct.Pin = GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 14, 0);

   /*Configure GPIO Pin : PB4 (Decreasing threshold temperature)*/
   GPIO_InitStruct.Pin = GPIO_PIN_4;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   HAL_NVIC_EnableIRQ(EXTI4_IRQn);
   HAL_NVIC_SetPriority(EXTI4_IRQn, 14, 0);

    /*Configure GPIO pin : LD2_Pin (GPIOA 5) */
   GPIO_InitStruct.Pin = LD2_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  * @note causes interrupt every 500ms
  */
void TIM4_Init(void){


	htim4.Instance = TIM4;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Prescaler = 4999;
	htim4.Init.Period = 8400;

	if(HAL_TIM_Base_Init(&htim4) != HAL_OK){
		Error_Handler();
	}

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  * @note causes interrupt every 3s
  */
void TIM5_Init(void){


	htim5.Instance = TIM5;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Prescaler = 4999;
	htim5.Init.Period = 50400;

	if(HAL_TIM_Base_Init(&htim5) != HAL_OK){
		Error_Handler();
	}

}

/*********************************************************************
 * @fn      		  - CONV_ADC_TO_TEMPERATURE
 *
 * @brief             - This function calculates temperature in celsius degrees from readed ADC value
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  Returns temperature value in celsius degrees
 *
 * @Note              -  LM35DZ sensor has proportional voltage rise to temperature
 * 						 It is rising 1mV for every Celsius degree. 3,3V is refference voltage

 */
double CONV_ADC_TO_TEMPERATURE(int adc_read_value){

	voltage = (ADC_REFERENCE_VOLTAGE*adc_read_value)/ADC_RESOLUTION;
	temperature = voltage/VOLTAGE_RISE;
	return temperature;
}

/*********************************************************************
 * @fn      		  - LCD_REFRESH
 *
 * @brief             - This function refresh informations which are displayed on LCD
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void LCD_REFRESH(void){

			lcd16x2_i2c_1stLine();
		  	lcd16x2_i2c_printf("Temper. value:");
		  	lcd16x2_i2c_2ndLine();
		  	lcd16x2_i2c_printf("%.2f", CONV_ADC_TO_TEMPERATURE((int)adc_buff));
		  	lcd16x2_i2c_setCursor(1, 14);
		  	lcd16x2_i2c_printf("%d", temperature_threshold);

 	if(fan_state == ENABLED){

	  		lcd16x2_i2c_setCursor(1, 8);
		  	lcd16x2_i2c_printf("Fan: EN");

	  	}else if(fan_state == DISABLED){

	  		lcd16x2_i2c_setCursor(1, 8);
	  		lcd16x2_i2c_printf("Fan: DI");
	  	}
}

/*********************************************************************
 * @fn      		  - START_FAN
 *
 * @brief             - This function set GPIOA 10 to high, which turns on the transistor controlling fan
 *
 * @return            -  none
 *
 * @Note              -

 */
void START_FAN(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
}

/*********************************************************************
 * @fn      		  - STOP_FAN
 *
 * @brief             - This function set GPIOA 10 to low, which turns off the transistor controlling fan
 *
 * @return            -  none
 *
 * @Note              -

 */
void STOP_FAN(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

/*********************************************************************
 * @fn      		  - DELAY
 *
 * @brief             - This function creates delay for neutralize button debouncing effect
 *
 * @param1			  - delay value in ms, has to be lesser tan 10000
 *
 * @return            - none
 *
 * @Note              -

 */
void DELAY(uint8_t delay){
	systick_cnt = 0;
	while(systick_cnt != delay);
}

/*********************************************************************
 * @fn      		  - HAL_TIM_PeriodElapsedCallback
 *
 * @brief             - This function is a callback of timer peripherials
 *
 * @return            - none
 *
 * @Note              -	Interrupt from TIM6 enables ADC1 to get voltage drop measurment from temperature sensor
 * 					  - Interrupt from TIM5 sends data to HC-05 module via UART2
					  - Interrupt from TIM4 refresh LCD
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM6){

		HAL_ADC_Start_IT(&hadc1);

	}else if(htim->Instance == TIM5){

		sprintf(adc_value_msg,"Temp: %.2f stop. Fan state: %d \n \r", CONV_ADC_TO_TEMPERATURE((int)adc_buff), fan_state );
		HAL_UART_Transmit(&huart2, (uint8_t*)adc_value_msg, strlen(adc_value_msg), HAL_MAX_DELAY);

	} if(htim->Instance == TIM4){

		LCD_REFRESH();

	}
}

/*********************************************************************
 * @fn      		  - HAL_ADC_ConvCpltCallback
 *
 * @brief             - This function is a callback of ADC
 *
 * @return            - none
 *
 * @Note              -	Interrupt from ADC1 enables ADC to get one voltage drop measurment from temperature sensor, then stops ADC
 *
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_buff = HAL_ADC_GetValue(&hadc1);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_ADC_Stop_IT(&hadc1);

}

/*********************************************************************
 * @fn      		  - HAL_GPIO_EXTI_Callback
 *
 * @brief             - This function is a callback of EXTI peripherial(in this case from external buttons)
 *
 * @return            - none
 *
 * @Note              -	Interrupt from GPIOB 4 will increase temperature threshold by 1
 * 					  - Interrupt from GPIOB 6 will decrease temperature threshold by 1
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_4){

		if(temperature_threshold < MAX_THRESHOLD_TEMPERATURE){
		temperature_threshold++;
		}

	}else if(GPIO_Pin == GPIO_PIN_5){
		if(temperature_threshold > MIN_THRESHOLD_TEMPERATURE){
		temperature_threshold--;
		}
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

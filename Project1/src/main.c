/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "lcd.c"
#include "math.h"

#define THRESHOLD_TEMPERATURE 20
#define THRESHOLD_WIND 90
#define THRESHOLD_NOISE 60

#define MAX_ANGLE 4096
#define MSGYESRAIN "R:YES"
#define MSGNORAIN "R:NO "
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _initialized;
uint8_t _numlines;
uint8_t _currline;

volatile enum ledFSM {ST_GREEN, ST_RED} g_sound_next_state, g_wind_next_state, g_temp_next_state ;
volatile enum measureFSM {ST_SOUND, ST_WIND,ST_TEMP,ST_LIGHT} g_measure_state;
volatile int measures[4]={0,0,0,0}; //mismo orden que ST_ arriba
volatile int rain = 0;
volatile int windsignal[2] = {0,0};
volatile int captura = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
void configChannelADC1(uint32_t channel);
void lcdInit(uint8_t cols, uint8_t lines, uint8_t dotsize);
void lcdPrint(char *str);
void lcdSetRGB(uint8_t r, uint8_t g, uint8_t b);
void lcdHome();
void lcdSetCursor(uint8_t col, uint8_t row);
void lcdClear();
void sound();
void temperature();
void wind();
void light();
void getWindValue();
void lcdUpdateRutine();
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  lcdInit(10,2,1);
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  lcdSetRGB(255,0,255);
  /* USER CODE END 2 */
  g_wind_next_state = ST_GREEN;
  g_temp_next_state = ST_GREEN;
  g_sound_next_state = ST_GREEN;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    sound();
    temperature();
    wind();
    light();
    HAL_Delay(500);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void sound(){
  configChannelADC1(ADC_CHANNEL_8);
  g_measure_state = ST_SOUND;
  HAL_ADC_Start_IT(&hadc1);

  switch(g_sound_next_state){
    case ST_GREEN:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
      if(measures[0] >= THRESHOLD_NOISE){
        g_sound_next_state = ST_RED;
      }
      break;

    case ST_RED:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
      break;
  }
}

void temperature(){
  configChannelADC1(ADC_CHANNEL_1);
  g_measure_state = ST_TEMP;
  HAL_ADC_Start_IT(&hadc1);

  switch(g_temp_next_state){
    case ST_GREEN:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
      if(measures[1] >= THRESHOLD_TEMPERATURE){
        g_temp_next_state = ST_RED;
      }
      break;

    case ST_RED:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
      break;
  }
}

void wind(){
  configChannelADC1(ADC_CHANNEL_0);
  g_measure_state = ST_WIND;
  HAL_ADC_Start_IT(&hadc1);
  getWindValue();

  switch(g_wind_next_state){
    case ST_GREEN:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
      if(measures[2] >= THRESHOLD_WIND){
        g_wind_next_state= ST_RED;
      }
      break;

    case ST_RED:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
      break;
  }
}

void light(){
  configChannelADC1(ADC_CHANNEL_4);
  g_measure_state = ST_LIGHT;
  HAL_ADC_Start_IT(&hadc1);
}

void getWindValue(){
  int valor = 0;
    do{
      HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
      valor = windsignal[1] - windsignal[0];
    }while((windsignal[0] > windsignal[1]) || valor>201 || valor<0); //comprobacion de que la medicion es valida
    measures[2] = valor;
    if(measures[2]==10){
      measures[2] = 0; //el minimo con 1 ms en la señal es 0 como se muestra en el enunciado del proyecto
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  //actualizar pantalla
  if(htim->Instance == htim11.Instance){
    lcdUpdateRutine();
  }
}

void lcdUpdateRutine(){
  char temp[4];
  char wind[4];
  char light[4];
  char sound[4];
  sprintf(light,"%d",measures[3]);
  sprintf(wind,"%d",measures[2]);
  sprintf(temp,"%d",measures[1]);
  sprintf(sound,"%d",measures[0]);

  lcdClear();
  lcdSetCursor(0,0);
  lcdPrint("T:");
  lcdSetCursor(2,0);
  lcdPrint(temp);
  lcdSetCursor(5,0);
  lcdPrint("W:");
  lcdSetCursor(7,0);
  lcdPrint(wind);
  lcdSetCursor(11,0);
  lcdPrint("S:");
  lcdSetCursor(13,0);
  lcdPrint(sound);
  lcdPrint("%");
  lcdSetCursor(0,1);
  lcdPrint("L:");
  lcdPrint(light);
  lcdPrint("%");
  lcdSetCursor(11,1);
  if(rain==1){
    lcdPrint(MSGYESRAIN);
  }else{
    lcdPrint(MSGNORAIN);
  }

}

void HAL_TIM_IC_CaptureCallback(	TIM_HandleTypeDef *htim	){
  if(htim->Instance == htim3.Instance){ //medicion de la señal PWM
    windsignal[captura] = HAL_TIM_ReadCapturedValue( &htim3, TIM_CHANNEL_1 );
    captura++;
    if(captura == 2){
      captura = 0;
    }
  } 
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_5 ){ //boton de reset
     g_sound_next_state = ST_GREEN;
     g_temp_next_state = ST_GREEN;
     g_wind_next_state = ST_GREEN;
  }
  if(GPIO_Pin == GPIO_PIN_4){ //boton touch de lluvia
    rain = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == hadc1.Instance){ 
    int adcValue=HAL_ADC_GetValue(&hadc1);
    switch(g_measure_state){
      case ST_SOUND:
        measures[0] = (adcValue*100)/4096; //se muestra como porcentaje
        break;
      case ST_TEMP:
        measures[1] = adcValue*30/3500; //se realiza una regla de tres para obtener un resultado aproximado
        break;
      case ST_WIND:
        htim2.Instance->CCR1 = 75 + (adcValue*1425/MAX_ANGLE);
        break;
      case ST_LIGHT:
        measures[3] = (adcValue*100)/4095; //se muestra como porcentaje
        break;
    }   
  }
}

void configChannelADC1(uint32_t channel){
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8399;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 50000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

void i2c_send_byte(uint8_t dta){
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, &dta, sizeof(dta) , 0x100);
}

void i2c_send_byteS(uint8_t *dta, uint8_t len){
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, dta, len , 0x100);
}

void lcdCommand(uint8_t value){
    uint8_t dta[2] = {0x80, value};
    i2c_send_byteS(dta, 2);
}

void lcdSetReg(uint8_t addr, uint8_t data){
    uint8_t dta[] = {addr, data};
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, dta, sizeof(dta) , 0x100);
}

void lcdDisplay() {
    _displaycontrol |= LCD_DISPLAYON;
    lcdCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcdClear(){
  lcdSetCursor(0,0);
  lcdPrint("                ");
  lcdSetCursor(0,1);
  lcdPrint("                ");
}

void lcdHome(){
    lcdCommand(LCD_RETURNHOME);        // set cursor position to zero
    HAL_Delay(2);        // this rgb_lcd_command takes a long time!
}

void lcdSetRGB(uint8_t r, uint8_t g, uint8_t b){
    lcdSetReg(REG_RED, r);
    lcdSetReg(REG_GREEN, g);
    lcdSetReg(REG_BLUE, b);
}

void lcdWrite(uint8_t value){
    uint8_t dta[2] = {0x40, value};
    i2c_send_byteS(dta, 2);
}

void lcdPrint(char *str){
    while(*str) {
        lcdWrite(*str);
        str++;
    }
}

void lcdSetCursor(uint8_t col, uint8_t row){
    col = (row == 0 ? col|0x80 : col|0xc0);
    uint8_t dta[2] = {0x80, col};
    i2c_send_byteS(dta, 2);
}

void lcdInit(uint8_t cols, uint8_t lines, uint8_t dotsize){
    if (lines > 1) {
        _displayfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    // for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lines == 1)) {
        _displayfunction |= LCD_5x10DOTS;
    }

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50

    HAL_Delay(50);

    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set rgb_lcd_command sequence
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(5);  // wait more than 4.1ms

    // second try
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);

    // third go
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);


    // finally, set # lines, font size, etc.
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);

    // turn the rgb_lcd_display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcdDisplay();

    // rgb_lcd_clear it off
    lcdClear();

    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    lcdCommand(LCD_ENTRYMODESET | _displaymode);


    // backlight init
    lcdSetReg(0, 0);
    lcdSetReg(1, 0);
    lcdSetReg(0x08, 0xAA);     // all led control by pwm

    lcdSetCursor(0, 0);
    
}


int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart2, &*c, 1, 100);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}

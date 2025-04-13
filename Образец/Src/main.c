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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "n1202.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define KB_NN 0x00
#define KB_UP 0x01
#define	KB_DN 0x02
#define	KB_LF 0x03
#define	KB_RT 0x04
#define	KB_PR 0x05
#define KB_ND 0x06

#define STOP_MODE 0x01
#define RUN_MODE  0x02
#define CV_MODE   0x03
#define CC_MODE   0x04
#define CP_MODE   0x05
#define CS_MODE   0x06

#define VDD_APPLI                      ((uint32_t)3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t)4095)   /* Max value for a full range of 12 bits (4096 values) */
#define VREFINT_CAL_ADDR                0x1FFFF7BA  /* datasheet p. 19 */
#define VREFINT_CAL ((uint16_t*) VREFINT_CAL_ADDR)


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t _LCD_RAM[LCD_X*LCD_String];
uint32_t adc_data[3];
uint16_t rdac;
//
uint32_t mVoltage = 0; 
uint32_t mCurrent = 0; 
uint32_t mPower = 0;
//
uint32_t last_time;
uint32_t pConsumption = 0; 
//
uint8_t  mMode = 1;
uint8_t  mRun = 0;
//
const	char cic[32] = {0x00,0x00,0x80,0xC0,0xE0,0xB0,0x98,0x80,0x80,0x80,0x80,0xBE,0x8A,0x82,0x80,0x00,
	                    0x00,0x00,0x01,0x03,0x07,0x0D,0x19,0x01,0x01,0x01,0x01,0x7D,0x45,0x45,0x01,0x00};
											
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myDisplayTaskHandle;
osThreadId myKBDTaskHandle;
osMessageQId myKBDQueueHandle;
osTimerId myTimer01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartKBDTask(void const * argument);
void TimerCallback(void const * argument);

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//
	HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_SET);
  LCD_Init();
  //
  HAL_ADCEx_Calibration_Start(&hadc);
	//
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  //	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, TimerCallback);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myKBDQueue */
  osMessageQDef(myKBDQueue, 16, uint16_t);
  myKBDQueueHandle = osMessageCreate(osMessageQ(myKBDQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myDisplayTask */
  osThreadDef(myDisplayTask, StartDisplayTask, osPriorityNormal, 0, 128);
  myDisplayTaskHandle = osThreadCreate(osThread(myDisplayTask), NULL);

  /* definition and creation of myKBDTask */
  osThreadDef(myKBDTask, StartKBDTask, osPriorityNormal, 0, 128);
  myKBDTaskHandle = osThreadCreate(osThread(myKBDTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_9BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 480;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 100;
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, LED_CS_Pin|LED_ON_Pin|LED_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_CS_Pin LED_ON_Pin LED_RESET_Pin */
  GPIO_InitStruct.Pin = LED_CS_Pin|LED_ON_Pin|LED_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_P_Pin KBD_D_Pin KBD_L_Pin KBD_R_Pin 
                           KBD_U_Pin */
  GPIO_InitStruct.Pin = KBD_P_Pin|KBD_D_Pin|KBD_L_Pin|KBD_R_Pin 
                          |KBD_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	mVoltage = ((80586 * adc_data[1])/100000)*2.102;
  mCurrent = ((80586 * adc_data[0])/100000)/2.37;
	mPower = (mVoltage * mCurrent) / 1000;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t KBS;
	rdac = 0;
	xTimerStart(myTimer01Handle, 50);
  /* Infinite loop */
  for(;;)
  {
		//
		if (mPower > 1000) TIM2->CCR1 = 800;
		else if (mPower > 750) TIM2->CCR1 = 550;
		else if (mPower > 500) TIM2->CCR1 = 400;
		else if (mPower > 200) TIM2->CCR1 = 300;
		else TIM2->CCR1 = 200;
		osDelay(10);
		// Реагируем на клавиатуру
		if (xQueueReceive(myKBDQueueHandle, &KBS, 0) == pdTRUE)	{
			if (KBS == KB_LF) {
				// Клавиша влево при режиме удержания тока
				if (mMode == 1) {
					if (rdac > 11) rdac = rdac - 10;
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, rdac);
				}
			}				
			if (KBS == KB_RT) {
				// Клавиша вправо в режиме удержания тока
				if (mMode == 1) {
					if (rdac < 4080) rdac = rdac + 10;
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, rdac);
				}
			}				
			if ((KBS == KB_UP)&&(mMode>0)) mMode--;
			if ((KBS == KB_DN)&&(mMode<2)) mMode++;
      if (KBS == KB_PR) {
				if (mRun == 0) { 
					mRun = 1;
					pConsumption = 0;
				}
				else mRun = 0l;
			}				
		}
		// измеряем миллиамерчасы
		if (mRun == 1) {
			uint32_t cicle = HAL_GetTick() - last_time;
      pConsumption = pConsumption + ( mCurrent * 2.7777) * cicle;
			last_time = HAL_GetTick();
		}			
			
	
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the myDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	char c[4];
	char d[16];
	uint8_t s[4];
  /* Infinite loop */
  for(;;)
  {
		// Начинаем обновлять экран
		LCD_Clear();		
    //
		memset(c, 0, 4);
		memset(s, 0, 4);
		sprintf(c, "%d", mVoltage);
		LCD_print(0, 4, 1, "Volt");
		s[0] = 0;
	  if ((c[0] >= 0x30)&&(c[0] <= 0x39)) s[0] = c[0] - 0x30; 
    s[1] = 0;
	  if ((c[1] >= 0x30)&&(c[1] <= 0x39)) s[1] = c[1] - 0x30; 
    s[2] = 0;
    if ((c[2] >= 0x30)&&(c[2] <= 0x39)) s[2] = c[2] - 0x30; 
    s[3] = 0;
	  if ((c[3] >= 0x30)&&(c[3] <= 0x39)) s[3] = c[3] - 0x30;
		if (mVoltage >= 1000) {
			LCD_DrawBitmap(28, 0, mass10x16[s[0]], 10, 16, 1);
		  LCD_DrawBitmap(40, 0, mass10x16[s[1]], 10, 16, 1);
		  LCD_FillRect(51, 14, 2, 2, 1);
		  LCD_DrawBitmap(54, 0, mass10x16[s[2]], 10, 16, 1);
		  LCD_DrawBitmap(66, 0, mass10x16[s[3]], 10, 16, 1);
		}
		else if (mVoltage >= 100) {
			LCD_DrawBitmap(40, 0, mass10x16[s[0]], 10, 16, 1);
		  LCD_FillRect(51, 14, 2, 2, 1);
		  LCD_DrawBitmap(54, 0, mass10x16[s[1]], 10, 16, 1);
		  LCD_DrawBitmap(66, 0, mass10x16[s[2]], 10, 16, 1);
		}
		else if (mVoltage >= 10) {
		  LCD_DrawBitmap(40, 0, mass10x16[0], 10, 16, 1);
		  LCD_FillRect(51, 14, 2, 2, 1);
		  LCD_DrawBitmap(54, 0, mass10x16[s[0]], 10, 16, 1);
		  LCD_DrawBitmap(66, 0, mass10x16[s[1]], 10, 16, 1);
		}
		else if (mVoltage >= 1) {
		  LCD_DrawBitmap(40, 0, mass10x16[0], 10, 16, 1);
		  LCD_FillRect(51, 14, 2, 2, 1);
		  LCD_DrawBitmap(54, 0, mass10x16[0], 10, 16, 1);
		  LCD_DrawBitmap(66, 0, mass10x16[s[0]], 10, 16, 1);
		}
    //
		memset(c, 0, 4);
		memset(s, 0, 4);
		sprintf(c, "%d", mCurrent);
    LCD_print(0, 23, 1, "Curr");
    s[0] = 0;
		if ((c[0] >= 0x30)&&(c[0] <= 0x39)) s[0] = c[0] - 0x30; 
    s[1] = 0;
		if ((c[1] >= 0x30)&&(c[1] <= 0x39)) s[1] = c[1] - 0x30; 
    s[2] = 0;
		if ((c[2] >= 0x30)&&(c[2] <= 0x39)) s[2] = c[2] - 0x30; 
    s[3] = 0;
		if ((c[3] >= 0x30)&&(c[3] <= 0x39)) s[3] = c[3] - 0x30; 
    if (mCurrent >= 1000) {
			LCD_DrawBitmap(28, 19, mass10x16[s[0]], 10, 16, 1);
      LCD_FillRect(39, 33, 2, 2, 1);
		  LCD_DrawBitmap(42, 19, mass10x16[s[1]], 10, 16, 1);
		  LCD_DrawBitmap(54, 19, mass10x16[s[2]], 10, 16, 1);
		  LCD_DrawBitmap(66, 19, mass10x16[s[3]], 10, 16, 1);
		}
		else if (mCurrent >= 100) {
			LCD_DrawBitmap(28, 19, mass10x16[0], 10, 16, 1);
      LCD_FillRect(39, 33, 2, 2, 1);
		  LCD_DrawBitmap(42, 19, mass10x16[s[0]], 10, 16, 1);
		  LCD_DrawBitmap(54, 19, mass10x16[s[1]], 10, 16, 1);
		  LCD_DrawBitmap(66, 19, mass10x16[s[2]], 10, 16, 1);
		}
		else if (mCurrent >= 10) {
			LCD_DrawBitmap(28, 19, mass10x16[0], 10, 16, 1);
      LCD_FillRect(39, 33, 2, 2, 1);
		  LCD_DrawBitmap(42, 19, mass10x16[0], 10, 16, 1);
		  LCD_DrawBitmap(54, 19, mass10x16[s[0]], 10, 16, 1);
		  LCD_DrawBitmap(66, 19, mass10x16[s[1]], 10, 16, 1);
		}
		else if (mCurrent >= 1) {
			LCD_DrawBitmap(28, 19, mass10x16[0], 10, 16, 1);
      LCD_FillRect(39, 33, 2, 2, 1);
		  LCD_DrawBitmap(42, 19, mass10x16[0], 10, 16, 1);
		  LCD_DrawBitmap(54, 19, mass10x16[0], 10, 16, 1);
		  LCD_DrawBitmap(66, 19, mass10x16[s[0]], 10, 16, 1);
		}
    //		
		memset(c, 0, 4);
		memset(s, 0, 4);
		sprintf(c, "%d", mPower);
		LCD_print(0, 42, 1, "Pwr");
    s[0] = 0;
		if ((c[0] >= 0x30)&&(c[0] <= 0x39)) s[0] = c[0] - 0x30; 
    s[1] = 0;
		if ((c[1] >= 0x30)&&(c[1] <= 0x39)) s[1] = c[1] - 0x30; 
    s[2] = 0;
		if ((c[2] >= 0x30)&&(c[2] <= 0x39)) s[2] = c[2] - 0x30; 
    s[3] = 0;
		if ((c[3] >= 0x30)&&(c[3] <= 0x39)) s[3] = c[3] - 0x30; 
    if (mPower >= 1000) {
			LCD_DrawBitmap(28, 38, mass10x16[s[0]], 10, 16, 1);
		  LCD_DrawBitmap(40, 38, mass10x16[s[1]], 10, 16, 1);
		  LCD_FillRect(51, 52, 2, 2, 1);
  		LCD_DrawBitmap(54, 38, mass10x16[s[2]], 10, 16, 1);
	  	LCD_DrawBitmap(66, 38, mass10x16[s[3]], 10, 16, 1);
		}
    else if (mPower >= 100) {
			LCD_DrawBitmap(40, 38, mass10x16[s[0]], 10, 16, 1);
		  LCD_FillRect(51, 52, 2, 2, 1);
  		LCD_DrawBitmap(54, 38, mass10x16[s[1]], 10, 16, 1);
	  	LCD_DrawBitmap(66, 38, mass10x16[s[2]], 10, 16, 1);
		}
    else if (mPower >= 10) {
		  LCD_DrawBitmap(40, 38, mass10x16[0], 10, 16, 1);
		  LCD_FillRect(51, 52, 2, 2, 1);
  		LCD_DrawBitmap(54, 38, mass10x16[s[0]], 10, 16, 1);
	  	LCD_DrawBitmap(66, 38, mass10x16[s[1]], 10, 16, 1);
		}
    else if (mPower >= 1) {
		  LCD_DrawBitmap(40, 38, mass10x16[0], 10, 16, 1);
		  LCD_FillRect(51, 52, 2, 2, 1);
  		LCD_DrawBitmap(54, 38, mass10x16[0], 10, 16, 1);
	  	LCD_DrawBitmap(66, 38, mass10x16[s[0]], 10, 16, 1);
		}
		//
		if (mRun) {
			sprintf(d, "Active %d mah", pConsumption/10000000);
			LCD_print(0, 60, 1, d);
		}
		else LCD_print(30, 60, 1, "Stoped");
	  //
    if (mMode == 0) LCD_DrawBitmap(79, 0, cic, 16, 16, 1);				
    if (mMode == 1) LCD_DrawBitmap(79, 19, cic, 16, 16, 1);				
    if (mMode == 2) LCD_DrawBitmap(79, 38, cic, 16, 16, 1);				
    //
	  LCD_Update();
    //
	  osDelay(250);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartKBDTask */
/**
* @brief Function implementing the myKBDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKBDTask */
void StartKBDTask(void const * argument)
{
  /* USER CODE BEGIN StartKBDTask */
	uint8_t KL = 0, KR = 0, KD = 0, KU = 0, KP = 0;
  uint8_t KB_State;	
  /* Infinite loop */
  for(;;)
  {
		//
		if (HAL_GPIO_ReadPin(KBD_L_GPIO_Port, KBD_L_Pin) == GPIO_PIN_SET) KL = KL << 1 | 0x01; 
		else KL = KL << 1 | 0x00;
		if (HAL_GPIO_ReadPin(KBD_R_GPIO_Port, KBD_R_Pin) == GPIO_PIN_SET) KR = KR << 1 | 0x01;
		else KR = KR << 1 | 0x00;
		if (HAL_GPIO_ReadPin(KBD_U_GPIO_Port, KBD_U_Pin) == GPIO_PIN_SET) KU = KU << 1 | 0x01;
		else KU = KU << 1 | 0x00;
		if (HAL_GPIO_ReadPin(KBD_D_GPIO_Port, KBD_D_Pin) == GPIO_PIN_SET) KD = KD << 1 | 0x01;
		else KD = KD << 1 | 0x00;
		if (HAL_GPIO_ReadPin(KBD_P_GPIO_Port, KBD_P_Pin) == GPIO_PIN_SET) KP = KP << 1 | 0x01;
		else KP = KP << 1 | 0x00;
		//
		KB_State = KB_ND;
		if (KL == 0xFF) { 
			KB_State = KB_LF;
			KL = KL << 1 | 0x00;
		}
		if (KR == 0xFF) { 
			KB_State = KB_RT;
			KR = KR << 1 | 0x00;
		}
		if (KU == 0xFF) { 
			KB_State = KB_UP;
			KU = KU << 1 | 0x00;
		}
		if (KD == 0xFF) { 
			KB_State = KB_DN;
			KD = KD << 1 | 0x00;
		}
		if (KP == 0xFF) { 
			KB_State = KB_PR;
			KP = KP << 1 | 0x00;
		}
    if (!(KL|KR|KU|KP|KD)) KB_State = KB_NN;
		if ((KL|KR|KU|KP|KD)&&(KB_State != KB_NN)) xQueueSendToBack(myKBDQueueHandle, &KB_State, 10);		
		//	
    osDelay(20);
  }
  /* USER CODE END StartKBDTask */
}

/* TimerCallback function */
void TimerCallback(void const * argument)
{
  /* USER CODE BEGIN TimerCallback */
	HAL_ADC_Start_DMA(&hadc, adc_data, 3);
  /* USER CODE END TimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

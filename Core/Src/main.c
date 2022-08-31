/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "5110.h"
#include "keypad4x4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define	tempMax		32
#define tempMin		28
uint8_t RxData;
int _Rx_Index = 0;
uint8_t RxCount = 0;
uint8_t PWM_Enable = 0;
uint8_t PWM_Source = 0;
char PWM_Rate[3] = {0, 0, 0};
char controlMotor[7] = {0, 0, 0, 0, 0, 0, 0};	//controlMotor[0] = Enable; controlMotor[1] = source; controlMotor[2,3,4] = rate
int Rx_Flag = 0;

char t[12];
char t1[2];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void upDateLCD_Enable(char Enable); // cap nhat Enble
void upDateLCD_Source(char Source);//cap nhat Source


//char Read_keypad = 1;



unsigned char key;
char Keypad_Enable = 0;
char Keypad_Source = 0;

void LCD_PWM_Display();

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId uartTaskHandle;
osThreadId ds18b20TaskHandle;
osThreadId keypadTaskHandle;
osThreadId pwmTaskHandle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void StartUartTask(void const * argument);
void StartDs18b20Task(void const * argument);
void StartKeypadTask(void const * argument);
void StartPwmTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	ds18b20_name ds1;
	LCD5110_Time lcd1;
	float temp;
	void motor_control(float temp)
	{
		if(controlMotor[0] == 1)
		{
			float speed = temp;
			htim2.Instance-> CCR1 = speed;		
		}
		else
		{
			htim2.Instance-> CCR1 = 0;	
		}
	}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart->Instance == huart2.Instance)
 {
		if(RxData == '*')
		{
			controlMotor[0] = 1;
			Rx_Flag = 1;
		}
		else if(RxData == '#')
		{
			controlMotor[0] = 0;
			Rx_Flag = 1;
		}
		else if(RxData == 'a'||RxData == 'A')
		{
			controlMotor[1] = 1;
			Rx_Flag = 1;
		}
		else if(RxData == 'b'||RxData == 'B')
		{
			controlMotor[1] = 0;
			Rx_Flag = 1;
		}
		else if(RxData >= '0' && RxData <= '9')
		{
			PWM_Source = 0;
			PWM_Rate[_Rx_Index++] = RxData;		
		}
		else if(_Rx_Index > 0 && RxData == '%')
		{
			_Rx_Index = 0;
			controlMotor[2] = '\0';
			controlMotor[3] = '\0';
			controlMotor[4] = '\0';
			controlMotor[2] = PWM_Rate[0];
			controlMotor[3] = PWM_Rate[1];
			controlMotor[4] = PWM_Rate[2];
			Rx_Flag = 1;
			PWM_Rate[0] = '\0';
			PWM_Rate[1] = '\0';
			PWM_Rate[2] = '\0';
		}
		else
		{
			_Rx_Index = 0;
			RxCount = 0;
			PWM_Enable = 0;
			PWM_Source = 0;
			PWM_Rate[0] = '\0';
			PWM_Rate[1] = '\0';
			PWM_Rate[2] = '\0';
			Rx_Flag = 0;			
		}
    HAL_UART_Receive_DMA(&huart2,&RxData,1);
 }
}

//-------------------------------------------------------------------------------------'
void check_KeyPad(char key)
{
	if(key == 'A')
	{
		osSemaphoreWait(myBinarySem01Handle,500);
		upDateLCD_Source(1);
		controlMotor[1] = 1;
		osSemaphoreRelease(myBinarySem01Handle);
	}
	else if (key == 'B')
	{
		osSemaphoreWait(myBinarySem01Handle,500);
		upDateLCD_Source(0);
		controlMotor[1] = 0;
		osSemaphoreRelease(myBinarySem01Handle);
	}
	else if (key == '*')
	{
		osSemaphoreWait(myBinarySem01Handle,500);
		upDateLCD_Enable(1);
		controlMotor[0] = 1;
		osSemaphoreRelease(myBinarySem01Handle);
	}
	else if (key == '#')
	{
		osSemaphoreWait(myBinarySem01Handle,500);
		upDateLCD_Enable(0);
		controlMotor[0] = 0;
		osSemaphoreRelease(myBinarySem01Handle);
	}
}



void upDateLCD_Enable(char Enable) // cap nhat Enble
{
	controlMotor[0] = Enable;
	LCD5110_set_XY(2,0);
	LCD5110_write_string("Enable:", 0);
	
	if(controlMotor[0] == 1)
	{
		LCD5110_set_XY(2,7);
		LCD5110_write_string("Yes", 0);
	}
	else
	{
		LCD5110_set_XY(2,7);
		LCD5110_write_string("No ", 0);
	}
}
void upDateLCD_Source(char Source)//cap nhat Source
{
	controlMotor[1] = Source;
	LCD5110_set_XY(1,0);
	LCD5110_write_string("Source:", 0);
	if(controlMotor[1] == 1)
	{
		LCD5110_set_XY(1,7);
		LCD5110_write_string("Sensor", 0);
	}
	else
	{
		LCD5110_set_XY(1,7);
		LCD5110_write_string("Uart  ", 0);
	}
}

void LCD_PWM_Display()
{
	LCD5110_clear();
	LCD5110_set_XY(3,0);
	LCD5110_write_string("Duty =  0 %",0);
	char buff[3];
	buff[0] = controlMotor[2];
	buff[1] = controlMotor[3];
	buff[2] = controlMotor[4];	
	LCD5110_set_XY(2,0);
	LCD5110_write_string("Enable:", 0);	
	if(controlMotor[0] == 1)
	{
		LCD5110_set_XY(2,7);
		LCD5110_write_string("Yes", 0);
	}
	else
	{
		LCD5110_set_XY(2,7);
		LCD5110_write_string("No ", 0);
	}	
	LCD5110_set_XY(0,0);
	LCD5110_write_string(t, 0);

	LCD5110_set_XY(1,0);
	LCD5110_write_string("Source:", 0);
	if(controlMotor[1] == 1)
	{
		LCD5110_set_XY(1,7);
		LCD5110_write_string("Sensor", 0);
			LCD5110_set_XY(3,7);
			LCD5110_write_string(t1, 0);
			LCD5110_set_XY(3, 10);
			LCD5110_write_char('%');
			motor_control((100*(temp - tempMin)/(tempMax - tempMin)));
	}
	else
	{
		LCD5110_set_XY(1,7);
		LCD5110_write_string("Uart  ", 0);
		if(atoi(buff) > 100)
		{
			LCD5110_set_XY(3,7);
			LCD5110_write_string("100", 0);
			LCD5110_set_XY(3, 10);
			LCD5110_write_char('%');
			motor_control(100);
		}
		else
		{
			LCD5110_set_XY(3,7);
			LCD5110_write_string(buff, 0);
			LCD5110_set_XY(3, 10);
			LCD5110_write_char('%');
			motor_control(atoi(buff));		
		}
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	ds18b20_init(&ds1, &htim4, ds18b20_GPIO_Port, ds18b20_Pin);
	LCD5110_init(&lcd1, &htim4);
	LCD5110_clear();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_UART_Receive_DMA(&huart2,&RxData,1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of uartTask */
  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of ds18b20Task */
  osThreadDef(ds18b20Task, StartDs18b20Task, osPriorityAboveNormal, 0, 128);
  ds18b20TaskHandle = osThreadCreate(osThread(ds18b20Task), NULL);

  /* definition and creation of keypadTask */
  osThreadDef(keypadTask, StartKeypadTask, osPriorityNormal, 0, 128);
  keypadTaskHandle = osThreadCreate(osThread(keypadTask), NULL);

  /* definition and creation of pwmTask */
  osThreadDef(pwmTask, StartPwmTask, osPriorityNormal, 0, 128);
  pwmTaskHandle = osThreadCreate(osThread(pwmTask), NULL);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim2.Init.Prescaler = 14400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD5110_LED_Pin|LCD5110_SCK_Pin|LCD5110_DIN_Pin|LCD5110_DC_Pin
                          |R1_Pin|R2_Pin|R3_Pin|R4_Pin
                          |ds18b20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD5110_CS_Pin|LCD5110_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD5110_LED_Pin LCD5110_SCK_Pin LCD5110_DIN_Pin LCD5110_DC_Pin
                           ds18b20_Pin */
  GPIO_InitStruct.Pin = LCD5110_LED_Pin|LCD5110_SCK_Pin|LCD5110_DIN_Pin|LCD5110_DC_Pin
                          |ds18b20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD5110_CS_Pin LCD5110_RST_Pin */
  GPIO_InitStruct.Pin = LCD5110_CS_Pin|LCD5110_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C3_Pin C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin|C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		
		if(Rx_Flag == 1)
		{
			osSemaphoreWait(myBinarySem01Handle,1000);
			LCD_PWM_Display();
			osSemaphoreRelease(myBinarySem01Handle);
			Rx_Flag = 0;
		}
	osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDs18b20Task */
/**
* @brief Function implementing the ds18b20Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDs18b20Task */
void StartDs18b20Task(void const * argument)
{
  /* USER CODE BEGIN StartDs18b20Task */
  /* Infinite loop */	

  for(;;)
  {	
		temp = temperature(&ds1);
		if(temp == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t *)"Sensor error!!!", 15, 300);
			HAL_UART_Transmit(&huart2, (uint8_t *)"\n", 1, 300);		
		}
		else
		{
		sprintf(t, "T = %0.2f °C", temp);
		sprintf(t1, "%d", (int)(100*(temp - tempMin)/(tempMax - tempMin)));
		HAL_UART_Transmit(&huart2, (uint8_t *)t, sizeof(t), 300);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\n", 1, 300);
		}
    osDelay(3000);
  }
  /* USER CODE END StartDs18b20Task */
}

/* USER CODE BEGIN Header_StartKeypadTask */
/**
  * @brief  Function implementing the keypadTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartKeypadTask */
void StartKeypadTask(void const * argument)
{
  /* USER CODE BEGIN StartKeypadTask */
  /* Infinite loop */
  for(;;)
  {
		key = read_keypad();
		if(key!= 0x01)
		{
			check_KeyPad(key);		
		}
  }
  /* USER CODE END StartKeypadTask */
}

/* USER CODE BEGIN Header_StartPwmTask */
/**
* @brief Function implementing the pwmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPwmTask */
void StartPwmTask(void const * argument)
{
  /* USER CODE BEGIN StartPwmTask */
  /* Infinite loop */
  for(;;)
  {		
		osSemaphoreWait(myBinarySem01Handle,1000);	
    LCD_PWM_Display();	
		osSemaphoreRelease(myBinarySem01Handle);
		osDelay(300);
  }
  /* USER CODE END StartPwmTask */
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

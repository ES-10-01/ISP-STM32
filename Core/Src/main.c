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
#include "wifi.h"
#include "utils.h"
#include "lcd16x2_i2c.h"
#include <string.h>
//#include "UartRingbuffer_multi.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for init_task */
osThreadId_t init_taskHandle;
const osThreadAttr_t init_task_attributes = {
  .name = "init_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for close_state_tas */
osThreadId_t close_state_tasHandle;
const osThreadAttr_t close_state_tas_attributes = {
  .name = "close_state_tas",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for input_task */
osThreadId_t input_taskHandle;
const osThreadAttr_t input_task_attributes = {
  .name = "input_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for open_task */
osThreadId_t open_taskHandle;
const osThreadAttr_t open_task_attributes = {
  .name = "open_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void InitTask(void *argument);
void CloseStateTask(void *argument);
void InputTask(void *argument);
void OpenTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const char network_name[] = "Mars aliens";
const char network_pasw[]="Zhjcnysqv`l900";
const char serv_ip[]="188.242.89.179";
const char control_id[]="1";

char read_keypad()
{
  HAL_GPIO_WritePin(GPIOB, pin_row_Pin, GPIO_PIN_SET);

  if ((HAL_GPIO_ReadPin(GPIOB, pin_1_Pin))) {
    while ((HAL_GPIO_ReadPin(GPIOB, pin_1_Pin)));
    return '1';
  }

  if ((HAL_GPIO_ReadPin(GPIOB, pin_4_Pin))) {
    while ((HAL_GPIO_ReadPin(GPIOB, pin_4_Pin)));
    return '4';
  }
  if ((HAL_GPIO_ReadPin(GPIOB, pin_2_Pin))) {
    while ((HAL_GPIO_ReadPin(GPIOB, pin_2_Pin)));
    return '2';
  }
  if ((HAL_GPIO_ReadPin(GPIOB, pin_3_Pin))) {
    while ((HAL_GPIO_ReadPin(GPIOB, pin_3_Pin)));
    return '3';
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
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_init(&hi2c1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of init_task */
  init_taskHandle = osThreadNew(InitTask, NULL, &init_task_attributes);

  /* creation of close_state_tas */
  close_state_tasHandle = osThreadNew(CloseStateTask, NULL, &close_state_tas_attributes);

  /* creation of input_task */
  input_taskHandle = osThreadNew(InputTask, NULL, &input_task_attributes);

  /* creation of open_task */
  open_taskHandle = osThreadNew(OpenTask, NULL, &open_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  vTaskSuspend(defaultTaskHandle);
  vTaskSuspend(close_state_tasHandle);
  vTaskSuspend(input_taskHandle);
  vTaskSuspend(open_taskHandle);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  return 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 32000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pin_row_Pin|inter_time_Pin|blue_led_Pin|green_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pin_2_Pin pin_1_Pin pin_4_Pin pin_3_Pin */
  GPIO_InitStruct.Pin = pin_2_Pin|pin_1_Pin|pin_4_Pin|pin_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : pin_row_Pin */
  GPIO_InitStruct.Pin = pin_row_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pin_row_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : inter_time_Pin blue_led_Pin green_led_Pin */
  GPIO_InitStruct.Pin = inter_time_Pin|blue_led_Pin|green_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_InitTask */
/**
* @brief Function implementing the init_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask */
void InitTask(void *argument)
{
  /* USER CODE BEGIN InitTask */
	enum GosServerCommands serv_answer;
	if (RC_FAIL(check_esp_available(&huart1))) {
		return;
	}
	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("ESP OK");
	osDelay(1000);
	if (RC_FAIL(init_esp(&huart1))) {
		return;
	}
	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("ESP READY");

	osDelay(3000);

	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("NETWORK CONNECT");
	if (RC_FAIL(connect_to_network(&huart1, network_name, network_pasw))) {
		lcd16x2_i2c_clear();
		lcd16x2_i2c_printf("CONNECTION FAIL");
		return;
	}
	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("CONNECTED");

	osDelay(5000);

	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("SERVER CONNECT");
	if (RC_FAIL(connect_to_server(&huart1, serv_ip, 9876))) {
		lcd16x2_i2c_clear();
		lcd16x2_i2c_printf("CONNECTION FAIL");
		return;
	}
	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("CONNECTED");

	osDelay(2500);

	lcd16x2_i2c_clear();
	if (RC_FAIL(send_hello(&huart1, control_id))) {
		return;
	}
	osDelay(2000);

	vTaskResume(close_state_tasHandle);

  /* Infinite loop */
  for(;;)
  {
	  serv_answer = recieve_command(&huart1);// ожидание сообщения от сервера
	  if (serv_answer == GOS_CLOSE_CMD){
		 vTaskResume(close_state_tasHandle);
		 serv_answer = UNKNOWN_CMD;
	  }
	  else if (serv_answer == GOS_GET_CMD) {
		 vTaskResume(input_taskHandle);
		 serv_answer = UNKNOWN_CMD;
		 vTaskSuspend(NULL);
	  }
	  else if (serv_answer == GOS_OPEN_CMD){
		 vTaskSuspend(close_state_tasHandle);
		 vTaskSuspend(input_taskHandle);
		 vTaskResume(open_taskHandle);
		 serv_answer = UNKNOWN_CMD;
		 vTaskSuspend(NULL);
	  }

  }
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_CloseStateTask */
/**
* @brief Function implementing the close_state_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CloseStateTask */
void CloseStateTask(void *argument)
{
  /* USER CODE BEGIN CloseStateTask */
  /* Infinite loop */
  for(;;)
  {
	lcd16x2_i2c_clear();
	lcd16x2_i2c_printf("Close");
	send_locked(&huart1);
    osDelay(1);

	vTaskResume(init_taskHandle);
    vTaskSuspend(NULL);
  }
  /* USER CODE END CloseStateTask */
}

/* USER CODE BEGIN Header_InputTask */
/**
* @brief Function implementing the input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputTask */
void InputTask(void *argument)
{
  /* USER CODE BEGIN InputTask */
  /* Infinite loop */
  for (;;) {
		  HAL_TIM_Base_Start_IT(&htim9);
		  HAL_GPIO_WritePin(GPIOB, inter_time_Pin, GPIO_PIN_RESET);
		  int i = 0;
		  char pin[5];
		  char pin_print[10] = "PIN: ";
		  pin[4] = '\0';
		  GPIO_PinState wait = GPIO_PIN_RESET;



		  lcd16x2_i2c_clear();//очистить экран

		  HAL_GPIO_WritePin(GPIOB, blue_led_Pin, GPIO_PIN_SET);
		  lcd16x2_i2c_printf("PIN:");//вывести на экран PIN:
		  while (i < 4 && (wait == GPIO_PIN_RESET)){
			  wait = HAL_GPIO_ReadPin(GPIOB, inter_time_Pin);
			  char a = read_keypad();
			  if (a =='1' || a =='2' || a =='3' || a =='4') {		//проверка на таймают ?
				  pin[i] = a;
				  pin_print[i+5] = a;
				  a = '0';
				  lcd16x2_i2c_clear();
				  lcd16x2_i2c_printf(pin_print);//введеную цифру на экран
				  i++;
			  }
		  }

		  strcpy(pin_print, "PIN: ");

		  if(wait == GPIO_PIN_RESET){
			  send_password(&huart1, pin); //отправить GOS_PASS=pin
		  }
		  vTaskResume(init_taskHandle);
		  HAL_TIM_Base_Stop_IT(&htim9);
		  __HAL_TIM_SET_COUNTER(&htim9, 0x0000);
		  HAL_GPIO_WritePin(GPIOB, inter_time_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(GPIOB, blue_led_Pin, GPIO_PIN_RESET);
		  lcd16x2_i2c_clear();
		  vTaskSuspend(NULL);
  }
  /* USER CODE END InputTask */
}

/* USER CODE BEGIN Header_OpenTask */
/**
* @brief Function implementing the open_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OpenTask */
void OpenTask(void *argument)
{
  /* USER CODE BEGIN OpenTask */
  /* Infinite loop */
  for(;;)
  {

	  	lcd16x2_i2c_clear();
	    lcd16x2_i2c_printf("Open");

		HAL_GPIO_WritePin(GPIOB, green_led_Pin, GPIO_PIN_SET);

		osDelay(10000);
		HAL_GPIO_WritePin(GPIOB, green_led_Pin, GPIO_PIN_RESET);
		vTaskResume(close_state_tasHandle);
		vTaskSuspend(NULL);
  }
  /* USER CODE END OpenTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

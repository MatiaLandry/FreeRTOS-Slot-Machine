/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
/* USER CODE END Includes */




/* Priorities at which the tasks are created. */
#define SenderTASK_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define ReceiverTASK_PRIORITY ( tskIDLE_PRIORITY + 1 )

#define mainQUEUE_LENGTH ( 1 )
static void SenderTask(void * argument);
static void ReceiverTask(void * argument);

/* The rate at which data is sent to the queue.  The times are converted from
milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define mainTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 200UL )
#define mainTIMER_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 2000UL )

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void prvStartDefaultTask(void  * argument);
static void prvStartTask02(void  * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/* A software timer that is started from the tick hook. */
//static TimerHandle_t xTimer = NULL;
/* USER CODE END 0 */
void start_TIM2() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->CR1 |= TIM_CR1_CEN;
}

uint16_t read_TIM2() {
  return TIM2->CNT;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  start_TIM2();

    	/* Create the queue. */
    	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );
      /* USER CODE BEGIN 1 */
    	if( xQueue != NULL )
    		{
  		xTaskCreate(SenderTask,"Sender",configMINIMAL_STACK_SIZE, NULL, SenderTASK_PRIORITY, NULL);
  		xTaskCreate(ReceiverTask, "Receiver", configMINIMAL_STACK_SIZE, NULL, ReceiverTASK_PRIORITY, NULL );

  		vTaskStartScheduler();
  	}
  while (1)
  {
    // Debug is the blue pin
	  HAL_GPIO_TogglePin( GPIOD, BLUE_LED_PIN );
	  HAL_Delay(500);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : red, blue, green, orange */
  GPIO_InitStruct.Pin = RED_LED_PIN | ORANGE_LED_PIN | BLUE_LED_PIN | GREEN_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

static void SenderTask(void  * argument)
{
	uint32_t retVal;
	HAL_GPIO_WritePin(GPIOD,RED_LED_PIN, GPIO_PIN_SET);
  for(;;)
  {
	  if (xQueueReceive(xQueue, &retVal, portMAX_DELAY) == pdTRUE)
	  {
		  if(retVal == 1){
			  HAL_GPIO_WritePin(GPIOD,RED_LED_PIN, GPIO_PIN_RESET);
		  }
		  if(retVal == 2){
			  HAL_GPIO_WritePin(GPIOD,RED_LED_PIN, GPIO_PIN_SET);
		  }
	  }
  }
}

static void ReceiverTask(void  * argument)
{
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN) == GPIO_PIN_SET)
	  {
		  //send message that button has been pressed to fill the queue
		  // Sender task receives the argument and turns the red led off
		  uint32_t msg = 1;
		  uint32_t tickCount = xTaskGetTickCount();
		  uint32_t rawRand = read_TIM2();
		  uint32_t rand = rawRand ^ (tickCount >> 16);
		  xQueueSend(xQueue, &msg, portMAX_DELAY);


		  HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_SET);
		  msg = 2;
		  //scans for 5 seconds
		  vTaskDelay(pdMS_TO_TICKS(500));
		  HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_RESET);
		  //sends second message
		  xQueueSend(xQueue, &msg, portMAX_DELAY);
	  }
	  // checks for button press
	  vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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

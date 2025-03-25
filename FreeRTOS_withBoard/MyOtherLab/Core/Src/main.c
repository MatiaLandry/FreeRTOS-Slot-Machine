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
#include "stdbool.h"
/* USER CODE END Includes */

/* Priorities at which the tasks are created. */
#define SenderTASK_PRIORITY ( tskIDLE_PRIORITY + 3 )
#define ReceiverTASK_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define winTASK_PRIORITY (tskIDLE_PRIORITY + 4)

//queue lengths
#define mainQUEUE_LENGTH ( 4 )
#define winQUEUE_LENGTH ( 1 )
#define transitionQUEUE_LENGTH (1)

#define WIN_EVENT 1
#define TRANSITION true

//start up delay control
bool buttonProcessEnable = false;
#define waitOnStart (50)

static void SenderTask(void * argument);
static void ReceiverTask(void * argument);
static void WinTask(void *argument);

/* The rate at which data is sent to the queue.  The times are converted from
milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define mainTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 200UL )
#define mainTIMER_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 2000UL )

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void prvStartDefaultTask(void  * argument);
static void prvStartTask02(void  * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* The queues that handle the wins and switching between tasks */
static QueueHandle_t xQueue = NULL;
static QueueHandle_t wQueue = NULL;
static QueueHandle_t transQueue = NULL;

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
    	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t) );
    	wQueue = xQueueCreate(winQUEUE_LENGTH, sizeof(uint8_t) );
    	transQueue = xQueueCreate(transitionQUEUE_LENGTH, sizeof(uint8_t) );

    	if( xQueue != NULL ) {
        xTaskCreate(SenderTask,"Sender",configMINIMAL_STACK_SIZE, NULL, SenderTASK_PRIORITY, NULL);
        xTaskCreate(ReceiverTask, "Receiver", configMINIMAL_STACK_SIZE, NULL, ReceiverTASK_PRIORITY, NULL );
        xTaskCreate(WinTask, "Winner", configMINIMAL_STACK_SIZE, NULL, winTASK_PRIORITY, NULL );
        vTaskStartScheduler();
    	}
  while (1) {
    // Debug is the blue pin
	  HAL_GPIO_TogglePin( GPIOD, BLUE_LED_PIN );
	  HAL_Delay(500);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
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
static void beginingAnimation() {
	int i;
	const uint16_t ledPins[] = {GREEN_LED_PIN, ORANGE_LED_PIN, RED_LED_PIN, BLUE_LED_PIN};

	for (int i = 0; i < (sizeof(ledPins) / sizeof(ledPins[0])); i++){
		HAL_GPIO_WritePin(GPIOD, ledPins[i], GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_GPIO_WritePin(GPIOD, ledPins[i], GPIO_PIN_RESET);
	}

  // Blink all LED's
	for(i=0; i<3; i++){
		HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, BLUE_LED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_PIN, GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE_LED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_PIN, GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	 vTaskDelay(pdMS_TO_TICKS(20));
}

// This runs when a loss is detected, it just blinks the given led
static void loss(uint16_t colour){
	for(int i=0;i<3;i++){
		HAL_GPIO_WritePin(GPIOD,colour, GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_GPIO_WritePin(GPIOD,colour, GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// This runs when a win is detected, and plays an animation
static void WinTask(void * argument) {
  uint8_t iswin;
  int transitioning;
  for (;;) {
    // Wait for a transition event; if not received, restart the loop.
    if (xQueueReceive(transQueue, &transitioning, portMAX_DELAY) != pdTRUE) continue;

    // Set transitioning to 8 and verify.
    transitioning = 8;
    if (transitioning != 8) continue;

    // Wait for the win condition message; if not received, restart the loop.
    if (xQueueReceive(wQueue, &iswin, portMAX_DELAY) != pdTRUE) continue;

    // Process win condition
    if (iswin == 0) {
      loss(ORANGE_LED_PIN);
    } else if (iswin == 1) {
      for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOD, BLUE_LED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(10));
        HAL_GPIO_WritePin(GPIOD, BLUE_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, ORANGE_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, RED_LED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GREEN_LED_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(10));
        HAL_GPIO_WritePin(GPIOD, RED_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GREEN_LED_PIN, GPIO_PIN_RESET);
      }
    }
  }
}

static void SenderTask(void  * argument) {
	uint8_t retVal;
	uint8_t transition;

	//ready state == blue pin
	HAL_GPIO_WritePin(GPIOD,BLUE_LED_PIN, GPIO_PIN_SET);

  // Block until a message is recieved on xQueue, then store the message in retVal.
  // Once a message is recieves, turn on the blue led for 1s.
  for(;;) {
	  if (xQueueReceive(xQueue, &retVal, portMAX_DELAY) == pdTRUE) {
		  vTaskDelay(pdMS_TO_TICKS(100));
		  HAL_GPIO_WritePin(GPIOD,BLUE_LED_PIN, GPIO_PIN_RESET);

		  if(retVal == 0) {
			  HAL_GPIO_WritePin(GPIOD,RED_LED_PIN, GPIO_PIN_SET);
		  }

		  if(retVal == 1) {
			  HAL_GPIO_WritePin(GPIOD,GREEN_LED_PIN, GPIO_PIN_SET);
		  }

      // If the rolls are complete, send code 8 to transQueue
		  if(retVal == 2 ) {
			  transition = 8;
			  xQueueSend(transQueue, &transition, portMAX_DELAY  );
		  }

		  vTaskDelay(pdMS_TO_TICKS(50));
		  HAL_GPIO_WritePin(GPIOD,RED_LED_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD,GREEN_LED_PIN, GPIO_PIN_RESET);
	  }
  }
}

// If even, false. If odd, true.
static bool reelHit(uint8_t number){
	return number % 2;
}

// Monitors for a button press. When button is pressed, run LED animation,
// generate a random number, determine hit or miss for each roll, and then send the 
// 4 results onto a queue, then sends overall win/loss to another queue.
static void ReceiverTask(void  * argument) {
	  vTaskDelay(pdMS_TO_TICKS(waitOnStart));
	  buttonProcessEnable = true;
	  int BATCH_END = 2;
    for(;;) {
      HAL_GPIO_WritePin(GPIOD,BLUE_LED_PIN, GPIO_PIN_SET);

      if (buttonProcessEnable && HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN) == GPIO_PIN_SET) {
        beginingAnimation();

        // reading 8 bit uint from the set timer
        // and system timer.
        uint8_t tickCount = xTaskGetTickCount();
        uint8_t rawRand = (read_TIM2() >> 8);

        // XOR the values to scramble the bits
        uint8_t rand = rawRand ^ (tickCount >> 8);

        //partitioning the bits [XX][XX][XX][XX]
        uint8_t val_1 = rand & 0x03;
        uint8_t val_2 = (rand >> 2) & 0x03;
        uint8_t val_3 = (rand >> 4) & 0x03;
        uint8_t val_4 = (rand >> 6) & 0x03;

        //Determine hit or miss
        bool r1 = reelHit(val_1);
        bool r2 = reelHit(val_2);
        bool r3 = reelHit(val_3);
        bool r4 = reelHit(val_4);
        uint8_t iswin;

        // Send each value to the xQueue to flash the individual results.
        const uint16_t winVals[] = {r1, r2, r3, r4};
        for (int i = 0;  i < (sizeof(winVals) / sizeof(winVals[0])); i++){
          xQueueSend(xQueue, &winVals[i], portMAX_DELAY);
        }

        // Send a 2 to the queue, indicating a finished state.
        xQueueSend(xQueue, &BATCH_END, portMAX_DELAY);

        //debugging
        while(uxQueueMessagesWaiting(xQueue) > 0) {
         //HAL_GPIO_WritePin(GPIOD,BLUE_LED_PIN, GPIO_PIN_SET);
        }

        // Determines if all the hits are true and loads the win animations
        // Sends the win flag to the wQueue, indicating an overall win.
        if (r1 && r2 && r3 && r4) {
          iswin = 1;
          xQueueSend(wQueue, &iswin, portMAX_DELAY);
        } else {
          iswin = 0;
          xQueueSend(wQueue, &iswin, portMAX_DELAY);
        }
        while(uxQueueMessagesWaiting(wQueue) > 0) {
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
      // checks for button press
      vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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

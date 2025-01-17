/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <unistd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	EVENT_START_BUTTON_PRESSED,
	EVENT_WATER_LEVEL_REACHED_WASH,
	EVENT_WASH_TIMER_EXPIRED,
	EVENT_DRAIN_COMPLETE,
	EVENT_RINSE_TIMER_EXPIRED,
	EVENT_SPIN_TIMER_EXPIRED,
	EVENT_ERROR_DETECTED,
	EVENT_ERROR_RESOLVED,
	EVENT_FILLING_WATER_COMPLETE,
	EVENT_WATER_LEVEL_REACHED_RINSE,
	EVENT_RINSE_DRAIN_COMPLETE,
	EVENT_WASH_DRAIN_COMPLETE
} washing_machine_event_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for washingTask */
osThreadId_t washingTaskHandle;
const osThreadAttr_t washingTask_attributes = {
  .name = "washingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for userInputTask */
osThreadId_t userInputTaskHandle;
const osThreadAttr_t userInputTask_attributes = {
  .name = "userInputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rinsingTask */
osThreadId_t rinsingTaskHandle;
const osThreadAttr_t rinsingTask_attributes = {
  .name = "rinsingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for washdrainTask */
osThreadId_t washdrainTaskHandle;
const osThreadAttr_t washdrainTask_attributes = {
  .name = "washdrainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fillrinseTask */
osThreadId_t fillrinseTaskHandle;
const osThreadAttr_t fillrinseTask_attributes = {
  .name = "fillrinseTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fillwashTask */
osThreadId_t fillwashTaskHandle;
const osThreadAttr_t fillwashTask_attributes = {
  .name = "fillwashTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rinsedrainTask */
osThreadId_t rinsedrainTaskHandle;
const osThreadAttr_t rinsedrainTask_attributes = {
  .name = "rinsedrainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartWashingTask(void *argument);
void StartUserInputTask(void *argument);
void StartRinsingTask(void *argument);
void StartWashDrainTask(void *argument);
void StartFillRinseTask(void *argument);
void StartFillWashTask(void *argument);
void StartRinseDrainTask(void *argument);
void StarteHearBeatTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------ITM redirect for printf----------------------------------//

int _write(int le, char *ptr, int len) {
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

//-------------------ITM redirect for printf---------------------------------//


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
  /* USER CODE BEGIN 2 */
  printf("I(100):USART2 Initialized = OK\r\n");
  printf("I(101):Scheduler Will take over from Now on\r\n");
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

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (64, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if (myQueue01Handle == NULL) {
    printf("Failed to create message queue.\n");
  } else {
    printf("Message queue created successfully.\n");
  }
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of washingTask */
  washingTaskHandle = osThreadNew(StartWashingTask, NULL, &washingTask_attributes);

  /* creation of userInputTask */
  userInputTaskHandle = osThreadNew(StartUserInputTask, NULL, &userInputTask_attributes);

  /* creation of rinsingTask */
  //rinsingTaskHandle = osThreadNew(StartRinsingTask, NULL, &rinsingTask_attributes);

  /* creation of washdrainTask */
  washdrainTaskHandle = osThreadNew(StartWashDrainTask, NULL, &washdrainTask_attributes);

  /* creation of fillrinseTask */
  //fillrinseTaskHandle = osThreadNew(StartFillRinseTask, NULL, &fillrinseTask_attributes);

  /* creation of fillwashTask */
  fillwashTaskHandle = osThreadNew(StartFillWashTask, NULL, &fillwashTask_attributes);

  /* creation of rinsedrainTask */
  //rinsedrainTaskHandle = osThreadNew(StartRinseDrainTask, NULL, &rinsedrainTask_attributes);

  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(StarteHearBeatTask, NULL, &heartbeatTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
	// Override _write function to redirect printf to UART
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|Motor_ON_OFF_Pin|Drain_Valve_ON_OFF_Pin|Water_Valve_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Motor_ON_OFF_Pin Drain_Valve_ON_OFF_Pin Water_Valve_ON_OFF_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|Motor_ON_OFF_Pin|Drain_Valve_ON_OFF_Pin|Water_Valve_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Start_PB_Pin Water_Level_Input_Pin */
  GPIO_InitStruct.Pin = Start_PB_Pin|Water_Level_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartWashingTask */
/**
 * @brief Function implementing the washingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWashingTask */
void StartWashingTask(void *argument)
{
  /* USER CODE BEGIN StartWashingTask */
/* Infinite loop */
		for (;;) {
			washing_machine_event_t event;
			osStatus_t status = osMessageQueueGet(myQueue01Handle, &event, NULL, osWaitForever);
			if (status == osOK) {

				if (event == EVENT_WATER_LEVEL_REACHED_WASH) {
					// Perform washing operation
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); // Turn on washing motor
					osDelay(5000); // Simulate washing time
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off washing motor
					// Send event to transition to draining
					washing_machine_event_t nextEvent = EVENT_WASH_TIMER_EXPIRED;
					osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);
				}
			osDelay(1);
		}
		}
  /* USER CODE END StartWashingTask */
}

/* USER CODE BEGIN Header_StartUserInputTask */
/**
 * @brief Function implementing the userInputTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUserInputTask */
void StartUserInputTask(void *argument)
{
  /* USER CODE BEGIN StartUserInputTask */
	printf("StartUserInputTask is running.\n");
	  for (;;) {
	    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET) {
	      osDelay(50);
	      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET) {
	        washing_machine_event_t event = EVENT_START_BUTTON_PRESSED;
	        osStatus_t status = osMessageQueuePut(myQueue01Handle, &event, 0, 0);
	        if (status == osOK) {
	          printf("Event EVENT_START_BUTTON_PRESSED sent successfully.\n");
	        } else {
	          printf("Failed to send EVENT_START_BUTTON_PRESSED event. Status: %d\n", status);
	        }
	      }
	    }
	    osDelay(1);
	  }
  /* USER CODE END StartUserInputTask */
}

/* USER CODE BEGIN Header_StartRinsingTask */
/**
 * @brief Function implementing the rinsingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRinsingTask */
void StartRinsingTask(void *argument)
{
  /* USER CODE BEGIN StartRinsingTask */
	/* Infinite loop */
	for (;;) {
		washing_machine_event_t event;
		if (osMessageQueueGet(myQueue01Handle, &event, NULL, osWaitForever) == osOK) {

			if (event == EVENT_WATER_LEVEL_REACHED_RINSE) {
				// Perform rinsing operation
				printf("Rinse Timer Started...\r\n");
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); // Example: Turn on motor
				osDelay(5000); // Simulate rinsing time
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); // Example: Turn off motor
				printf("Rinse Timer Expired...\r\n");
				// Send event to transition to next state
				washing_machine_event_t nextEvent = EVENT_RINSE_TIMER_EXPIRED;
				osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartRinsingTask */
}

/* USER CODE BEGIN Header_StartWashDrainTask */
/**
 * @brief Function implementing the washdrainTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWashDrainTask */
void StartWashDrainTask(void *argument)
{
  /* USER CODE BEGIN StartWashDrainTask */
	/* Infinite loop */
	for (;;) {
		washing_machine_event_t event;
		if (osMessageQueueGet(myQueue01Handle, &event, NULL, osWaitForever) == osOK) {
			if (event == EVENT_WASH_TIMER_EXPIRED) {
				printf("Performing drain operation after wash...\r\n");
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // Open the Drain Valve
				while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) != GPIO_PIN_RESET) {
					osDelay(1); // Wait until water level is reached
				}
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // Turn off water valve
				printf("Drain operation after wash Completed...\r\n");
				washing_machine_event_t nextEvent = EVENT_WASH_DRAIN_COMPLETE;
				osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);
			}

		}
		osDelay(1);
	}
  /* USER CODE END StartWashDrainTask */
}

/* USER CODE BEGIN Header_StartFillRinseTask */
	/**
	 * @brief Function implementing the fillrinseTask thread.
	 * @param argument: Not used
	 * @retval None
	 */
/* USER CODE END Header_StartFillRinseTask */
void StartFillRinseTask(void *argument)
{
  /* USER CODE BEGIN StartFillRinseTask */
		/* Infinite loop */
		for (;;) {
			washing_machine_event_t event;
			if (osMessageQueueGet(myQueue01Handle, &event, NULL, osWaitForever) == osOK) {
				if (event == EVENT_WASH_DRAIN_COMPLETE) {
					// Perform Rinse fill operation
					printf("Perform Rinse fill operation...\r\n");
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // Turn on water valve

					while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)
							!= GPIO_PIN_RESET) {
						osDelay(100); // Wait until water level is reached
					}
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Turn off water valve
					printf("Rinse fill operation Completed...\r\n");
					washing_machine_event_t nextEvent = EVENT_WATER_LEVEL_REACHED_RINSE;
					osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);

				}
				osDelay(1);
			}
		}
  /* USER CODE END StartFillRinseTask */
}

/* USER CODE BEGIN Header_StartFillWashTask */
		/**
		 * @brief Function implementing the fillwashTask thread.
		 * @param argument: Not used
		 * @retval None
		 */
/* USER CODE END Header_StartFillWashTask */
void StartFillWashTask(void *argument)
{
  /* USER CODE BEGIN StartFillWashTask */
	  printf("StartFillWashTask is running.\n");
	  for (;;) {
	    washing_machine_event_t event;
	    osStatus_t status = osMessageQueueGet(myQueue01Handle, &event, NULL, osWaitForever);
	    if (status == osOK) {
	      printf("Received event: %d\n", event);
	      if (event == EVENT_START_BUTTON_PRESSED) {
	        printf("Start button pressed. Starting to fill water...\n");

	        // Perform filling water operation
	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // Turn on water valve

	        while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) != GPIO_PIN_RESET) {
	          osDelay(1); // Wait until water level is reached
	        }

	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Turn off water valve
	        washing_machine_event_t nextEvent = EVENT_WATER_LEVEL_REACHED_WASH;
	        osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);
	      }
	    } else {
	      printf("Failed to get event from queue. Status: %d\n", status);
	    }
	    osDelay(1);
	  }

  /* USER CODE END StartFillWashTask */
}

/* USER CODE BEGIN Header_StartRinseDrainTask */
		/**
		 * @brief Function implementing the rinsedrainTask thread.
		 * @param argument: Not used
		 * @retval None
		 */
/* USER CODE END Header_StartRinseDrainTask */
void StartRinseDrainTask(void *argument)
{
  /* USER CODE BEGIN StartRinseDrainTask */
			/* Infinite loop */
			for (;;) {
				washing_machine_event_t event;
				if (osMessageQueueGet(myQueue01Handle, &event, NULL,
				osWaitForever) == osOK) {

					if (event == EVENT_RINSE_TIMER_EXPIRED) {
						// Perform rinsing operation
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // Open Drain Valve
						osDelay(5000); // Simulate rinsing time
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // Close Drain Valve
						// Send event to transition to next state
						washing_machine_event_t nextEvent =
								EVENT_RINSE_DRAIN_COMPLETE;
						osMessageQueuePut(myQueue01Handle, &nextEvent, 0, 0);
					}
					osDelay(1);
				}
			}
  /* USER CODE END StartRinseDrainTask */
}

/* USER CODE BEGIN Header_StarteHearBeatTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StarteHearBeatTask */
void StarteHearBeatTask(void *argument)
{
  /* USER CODE BEGIN StarteHearBeatTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Open Drain Valve
	  osDelay(100); // On Delay
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Close Drain Valve
	  osDelay(100); // Off Delay

  }
  /* USER CODE END StarteHearBeatTask */
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
				__disable_irq();
				while (1) {
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

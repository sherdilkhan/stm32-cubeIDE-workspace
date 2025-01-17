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

#include "./BME280/bme280.h"
#include "./SSD1306/ssd1306.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WIFI_SSID "Sherdil"
#define WIFI_PASS "03336830763BB"
#define MQTT_CLIENT_ID "OgkyKQYPMSUUIA0lICEeFQI"
#define MQTT_USERNAME "OgkyKQYPMSUUIA0lICEeFQI"
#define MQTT_PASSWORD "QzwKF9IrWL12EE1YAIRR3fui"
#define MQTT_BROKER_URL "mqtt3.thingspeak.com"
#define MQTT_PORT 8883
#define THINGSPEAK_API_KEY "Y32YKHUW1JPAYZ3F"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for bme280Task */
osThreadId_t bme280TaskHandle;
const osThreadAttr_t bme280Task_attributes = {
  .name = "bme280Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mqttTask */
osThreadId_t mqttTaskHandle;
const osThreadAttr_t mqttTask_attributes = {
  .name = "mqttTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bme280Data */
osMessageQueueId_t bme280DataHandle;
const osMessageQueueAttr_t bme280Data_attributes = {
  .name = "bme280Data"
};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void Startbme280Task(void *argument);
void StartlcdTask(void *argument);
void StartmqttTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void sendATCommand(const char *command);
void publishMQTTMessage(const char *topic, const char *payload);
bool waitForResponse(const char *expectedResponse, uint32_t timeout);

//float temperature, humidity, pressure;

typedef struct {
	float temperature, pressure, humidity;
} bme280Data_t;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK)
		return -1;
	if (HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10)
			!= HAL_OK)
		return -1;

	return 0;
}

void user_delay_ms(uint32_t period) {

	HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t *buf;
	buf = malloc(len + 1);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);

	if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*) buf, len + 1,
	HAL_MAX_DELAY) != HAL_OK)
		return -1;

	free(buf);
	return 0;
}

// for printf() redirection to USART2
int _write(int file, char *data, int len) {
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) data, len,
	HAL_MAX_DELAY);
	return (status == HAL_OK ? len : 0);
}



//void publishMQTTMessage(const char *topic, const char *payload) {


//}

void sendATCommand(const char *command) {
	// Implement the function to send AT command via UART to ESP32-C3
	HAL_UART_Transmit(&huart2, (uint8_t*) command, strlen(command),
	HAL_MAX_DELAY);
	//HAL_Delay(100); // Delay to allow response from ESP32-C3
}

bool waitForResponse(const char *expectedResponse, uint32_t timeout) {
	uint32_t start = HAL_GetTick();
	char response[256];
	uint32_t index = 0;

	while ((HAL_GetTick() - start) < timeout) {
		if (HAL_UART_Receive(&huart2, (uint8_t*) &response[index], 1, 100)
				== HAL_OK) {
			if (response[index] == '\n') {
				response[index + 1] = '\0';
				if (strstr(response, expectedResponse) != NULL) {
					return true;
				}
				index = 0;
			} else {
				index++;
			}
		}
	}
	return false;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	/*----------------------------------------BME280 Setup-------------------------------------------------*/
	/* BME280 Initialization */
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev); // Initialize the sensor.
	if (rslt == BME280_OK) {
		printf("BME280 Successfully Initialized\r\n");
	} else {
		printf("Failed to Initialize BME280. Error code: %d\r\n", rslt);
	}

	/* BME280 Configuration */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(
			BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL
					| BME280_FILTER_SEL, &dev);
	//rslt = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &dev); this can also be used in place of above
	if (rslt == BME280_OK) {
		printf("BME280 Successfully Configured\r\n");
	} else {
		printf("Failed to Configure BME280. Error code: %d\r\n", rslt);
	}
	/* BME280 Mode Setting */
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
	dev.delay_ms(40);
	if (rslt == BME280_OK) {
		printf("BME280 Mode Successfully set.\r\n");
	} else {
		printf("Failed to set BME280 Mode. Error code: %d\r\n", rslt);
	}

	/*-------------------------------------SSD1306 Setup-----------------------------------------------------*/
	ssd1306_Init(&hi2c1);

	/*----------------------------------ESP32C3 Wifi Setup---------------------------------------------------*/

	char atCommand[256];

	// Restore ESP32 to factory settings (optional)
	HAL_UART_Transmit(&huart3, (uint8_t*) "Restoring ESP32\r\n",
			strlen("Restoring ESP32\r\n"), HAL_MAX_DELAY);
	sendATCommand("AT+RESTORE\r\n");
	if (!waitForResponse("OK\r\n", 10000)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "Failed to Restore ESP32\r\n",
				strlen("Failed to Restore ESP32\r\n"), HAL_MAX_DELAY);
	}

	HAL_Delay(4000);

	//-------------------------------------------------Wifi Con Setup---------------------------------------------//

	// Test AT communication
	HAL_UART_Transmit(&huart3, (uint8_t*) "Testing AT command\r\n",
			strlen("Testing AT command\r\n"), HAL_MAX_DELAY);
	sendATCommand("AT\r\n");
	if (!waitForResponse("OK\r\n", 2000)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "AT command failed\r\n",
				strlen("AT command failed\r\n"), HAL_MAX_DELAY);
	}

	// Set WiFi mode to station mode
	HAL_UART_Transmit(&huart3,
			(uint8_t*) "Setting WiFi mode to station mode\r\n",
			strlen("Setting WiFi mode to station mode\r\n"), HAL_MAX_DELAY);
	sendATCommand("AT+CWMODE=1\r\n");
	if (!waitForResponse("OK\r\n", 7000)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "Failed to Set Mode ESP32\r\n",
				strlen("Failed to Set Mode ESP32\r\n"), HAL_MAX_DELAY);
	}

	// Connect to WiFi
	HAL_UART_Transmit(&huart3, (uint8_t*) "Connecting to Wifi\r\n",
			strlen("Connecting to Wifi\r\n"), HAL_MAX_DELAY);
	snprintf(atCommand, sizeof(atCommand), "AT+CWJAP=\"%s\",\"%s\"\r\n",
	WIFI_SSID, WIFI_PASS);
	sendATCommand(atCommand);
	if (!waitForResponse("WIFI GOT IP\r\n", 10000)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "WiFi Connection Failed\r\n",
				strlen("WiFi Connection Failed\r\n"), HAL_MAX_DELAY);
	}

	//-------------------------------------------------MQTT Con Setup---------------------------------------------//

	// Set MQTT Username and Password
	HAL_UART_Transmit(&huart3, (uint8_t*) "Configuring MQTT Credentials\r\n",
			strlen("Configuring MQTT Credentials\r\n"), HAL_MAX_DELAY);
	snprintf(atCommand, sizeof(atCommand),
			"AT+MQTTUSERCFG=0,2,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
			MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
	sendATCommand(atCommand);
	if (!waitForResponse("OK\r\n", 1000)) {
		HAL_UART_Transmit(&huart3,
				(uint8_t*) "Failed to Configure MQTT Credentials\r\n",
				strlen("Failed to Configure MQTT Credentials\r\n"),
				HAL_MAX_DELAY);
		//return;
	}

	// Connect to MQTT Broker
	HAL_UART_Transmit(&huart3, (uint8_t*) "Connecting to MQTT Broker\r\n",
			strlen("Connecting to MQTT Broker\r\n"), HAL_MAX_DELAY);
	snprintf(atCommand, sizeof(atCommand), "AT+MQTTCONN=0,\"%s\",%d,1\r\n",
			MQTT_BROKER_URL, MQTT_PORT);
	sendATCommand(atCommand);
	if (!waitForResponse("OK\r\n", 5000)) {
		HAL_UART_Transmit(&huart3,
				(uint8_t*) "Failed to Connect to MQTT Broker\r\n",
				strlen("Failed to Connect to MQTT Broker\r\n"), HAL_MAX_DELAY);
	}

	// Subscribe to a Topic
	HAL_UART_Transmit(&huart3, (uint8_t*) "Subscribing to Topic\r\n",
			strlen("Subscribing to Topic\r\n"), HAL_MAX_DELAY);
	sendATCommand("AT+MQTTSUB=0,\"sensor/bme280\",1\r\n");
	if (!waitForResponse("OK\r\n", 1000)) {
		HAL_UART_Transmit(&huart3,
				(uint8_t*) "Failed to Subscribe to Topic\r\n",
				strlen("Failed to Subscribe to Topic\r\n"), HAL_MAX_DELAY);
		//return;
	}

	// Publish to a Topic
	HAL_UART_Transmit(&huart3, (uint8_t*) "Publishing to Topic\r\n",
			strlen("Publishing to Topic\r\n"), HAL_MAX_DELAY);
	sendATCommand("AT+MQTTPUB=0,\"sensor/bme280\",\"value1\\, value2\",1,0\r\n");
	if (!waitForResponse("OK\r\n", 1000)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "Failed to Publish to Topic\r\n",
				strlen("Failed to Publish to Topic\r\n"), HAL_MAX_DELAY);
		//return;
	}

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

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
  /* creation of bme280Data */
	bme280DataHandle = osMessageQueueNew(16, sizeof(bme280Data_t),
			&bme280Data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of bme280Task */
  bme280TaskHandle = osThreadNew(Startbme280Task, NULL, &bme280Task_attributes);

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartlcdTask, NULL, &lcdTask_attributes);

  /* creation of mqttTask */
  mqttTaskHandle = osThreadNew(StartmqttTask, NULL, &mqttTask_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 175;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|Audio_RST_Pin;
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
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
		osDelay(200);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Startbme280Task */
/**
 * @brief Function implementing the bme280Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Startbme280Task */
void Startbme280Task(void *argument)
{
  /* USER CODE BEGIN Startbme280Task */
	bme280Data_t sensorData;
	/* Infinite loop */
	for (;;) {

		if (osMutexAcquire(i2c1MutexHandle, osWaitForever) == osOK) {
			/* BME280 Data Collection */
			rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
			osMutexRelease(i2c1MutexHandle); // Release the I2C1 mutex
			if (rslt == BME280_OK) {
				sensorData.temperature = comp_data.temperature / 100.0f; //100.0f is to make at least one operand float type
				sensorData.pressure = comp_data.pressure * (0.00750062 / 100); //1hPa = 0.00750062 mmHg
				sensorData.humidity = comp_data.humidity / 1024.0f; //1024.0f is to make at least one operand float type
			}
			// Send data to the queue
			if (osMessageQueuePut(bme280DataHandle, &sensorData, 0,
			osWaitForever) != osOK) {
			}

		}
		osDelay(1);
	}
  /* USER CODE END Startbme280Task */
}

/* USER CODE BEGIN Header_StartlcdTask */
/**
 * @brief Function implementing the lcdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartlcdTask */
void StartlcdTask(void *argument)
{
  /* USER CODE BEGIN StartlcdTask */
	bme280Data_t receivedData;
	char buffer[32];
	/* Infinite loop */
	for (;;) {
		if (osMessageQueueGet(bme280DataHandle, &receivedData, NULL,
		osWaitForever) == osOK) {
		} else {
			// Handle error: Failed to receive data from queue
		}

		// Clear the display
		ssd1306_Fill(Black);

		// Print the received data on the SSD1306 display
		// Write data to local screen buffer
		snprintf(buffer, sizeof(buffer), "Tem:%.2f C",
				receivedData.temperature);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buffer, Font_11x18, White);

		snprintf(buffer, sizeof(buffer), "Hum:%.2f %%", receivedData.humidity);
		ssd1306_SetCursor(0, 18); // Move cursor to the next line
		ssd1306_WriteString(buffer, Font_11x18, White);

		snprintf(buffer, sizeof(buffer), "Prs:%.2f mmHg",
				receivedData.pressure);
		ssd1306_SetCursor(0, 36); // Move cursor to the next line
		ssd1306_WriteString(buffer, Font_11x18, White);

		////////////////////------Mutex------------////////////////////////
		if (osMutexAcquire(i2c1MutexHandle, osWaitForever) == osOK) {
			// Update the SSD1306 display
			ssd1306_UpdateScreen(&hi2c1);
			// Release the mutex after accessing the I2C1 peripheral
			osMutexRelease(i2c1MutexHandle);
		} else {
			// Handle error: Failed to acquire mutex
		}
		// Release the mutex after accessing the I2C1 peripheral
		osMutexRelease(i2c1MutexHandle);
		////////////////////------Mutex------------////////////////////////

		osDelay(1);
	}

  /* USER CODE END StartlcdTask */
}

/* USER CODE BEGIN Header_StartmqttTask */
/**
 * @brief Function implementing the mqttTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartmqttTask */
void StartmqttTask(void *argument)
{
  /* USER CODE BEGIN StartmqttTask */
	  	bme280Data_t sensorData;
	  	char mqttPayload[128];
	  	const char *mqttTopic = "sensor/bme280";
	  	char atCommand[256];
	/* Infinite loop */
	for(;;){
	  		// Get data from the queue
	  		if (osMessageQueueGet(bme280DataHandle, &sensorData, NULL,
	  		osWaitForever) == osOK) {
	  			// Format the data as a JSON string
	  			snprintf(mqttPayload, sizeof(mqttPayload),
	  			         "temperature: %.2f ; pressure: %.2f ; humidity: %.2f",
	  			         sensorData.temperature, sensorData.pressure,
	  			         sensorData.humidity);


	  			// Publish to a Topic
	  			//HAL_UART_Transmit(&huart3,
	  			//		(uint8_t*) "Publishing to Topic in MQTT Task\r\n",
	  			//		strlen("Publishing to Topic\r\n"), HAL_MAX_DELAY);
	  			snprintf(atCommand, sizeof(atCommand),"AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n", mqttTopic,mqttPayload);

	  			sendATCommand(atCommand);

	  			/*
	  			if (!waitForResponse("OK\r\n", 1000)) {
	  				HAL_UART_Transmit(&huart3,
	  						(uint8_t*) "Failed to Publish to Topic\r\n",
	  						strlen("Failed to Publish to Topic\r\n"),
	  						HAL_MAX_DELAY);
	  			} else {
	  				HAL_UART_Transmit(&huart3,
	  						(uint8_t*) "Successfully Published to Topic\r\n",
	  						strlen("Successfully Published to Topic\r\n"),
	  						HAL_MAX_DELAY);
	  			} */
	  		}
	  		osDelay(2000); // Delay for a while before the next publish
	  	}
  /* USER CODE END StartmqttTask */
}

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
volatile int x = 0;

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = H_Sync_Pin|V_Sync_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = vga_green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(vga_green_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == V_Sync_Pin)
	{
		x = 0;
	}

	if (GPIO_Pin == H_Sync_Pin)
	{
		x++;
		if (x<=200 && x>300)
		{
			//HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
		}
		if (x>200 && x<=300)
		{
			//HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);

			//Set the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 14);
			//Reset the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 30);
			//Set the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 14);
			//Reset the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 30);
			//Set the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 14);
			//Reset the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 30);
			//Set the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 14);
			//Reset the 14th bit/pin of Port B
			GPIOB->BSRR = (1U << 30);

			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(vga_green_GPIO_Port, vga_green_Pin, GPIO_PIN_RESET);
		}

	}
}





/* USER CODE END 2 */

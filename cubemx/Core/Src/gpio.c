/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JOINT_DRDY_GPIO_Port, JOINT_DRDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRV8825_M0_Pin|DRV8825_M1_Pin|DRV8825_M2_Pin|AS5600_DIR_Pin
                          |DRV8825_DIR_Pin|DRV8825_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOINT_DELTA_Pin */
  GPIO_InitStruct.Pin = JOINT_DELTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOINT_DELTA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JOINT_DRDY_Pin */
  GPIO_InitStruct.Pin = JOINT_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JOINT_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV8825_M0_Pin DRV8825_M1_Pin DRV8825_M2_Pin AS5600_DIR_Pin
                           DRV8825_DIR_Pin DRV8825_EN_Pin */
  GPIO_InitStruct.Pin = DRV8825_M0_Pin|DRV8825_M1_Pin|DRV8825_M2_Pin|AS5600_DIR_Pin
                          |DRV8825_DIR_Pin|DRV8825_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

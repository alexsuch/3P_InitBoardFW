/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_GPIO_1_Pin GPIO_PIN_4
#define TEST_GPIO_1_GPIO_Port GPIOA
#define EX_LED_OUT_Pin GPIO_PIN_5
#define EX_LED_OUT_GPIO_Port GPIOA
#define LED_YELLOW_OUT_Pin GPIO_PIN_1
#define LED_YELLOW_OUT_GPIO_Port GPIOB
#define LED_GREEN_OUT_Pin GPIO_PIN_2
#define LED_GREEN_OUT_GPIO_Port GPIOB
#define BOOM_LOW_SIDE_OUT_2_Pin GPIO_PIN_12
#define BOOM_LOW_SIDE_OUT_2_GPIO_Port GPIOB
#define BOOM_LOW_SIDE_OUT_1_Pin GPIO_PIN_13
#define BOOM_LOW_SIDE_OUT_1_GPIO_Port GPIOB
#define CHARGE_EN_OUT_Pin GPIO_PIN_10
#define CHARGE_EN_OUT_GPIO_Port GPIOA
#define TEST_2_Pin GPIO_PIN_11
#define TEST_2_GPIO_Port GPIOA
#define FUSE_IN_Pin GPIO_PIN_12
#define FUSE_IN_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA
#define ACC_INT_2_Pin GPIO_PIN_6
#define ACC_INT_2_GPIO_Port GPIOB
#define ACC_IN_1_Pin GPIO_PIN_7
#define ACC_IN_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

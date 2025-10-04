/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
#define EM_BUZZ_Pin GPIO_PIN_13
#define EM_BUZZ_GPIO_Port GPIOC
#define FUSE_IN_Pin GPIO_PIN_15
#define FUSE_IN_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define ACC_INT_Pin GPIO_PIN_0
#define ACC_INT_GPIO_Port GPIOB
#define HIGH_SIDE_OUT_Pin GPIO_PIN_1
#define HIGH_SIDE_OUT_GPIO_Port GPIOB
#define VBAT_IN_Pin GPIO_PIN_10
#define VBAT_IN_GPIO_Port GPIOB
#define FC_PWM_GPIO_IN_Pin GPIO_PIN_14
#define FC_PWM_GPIO_IN_GPIO_Port GPIOB
#define FC_PWM_GPIO_IN_EXTI_IRQn EXTI4_15_IRQn
#define CHARGE_EN_OUT_Pin GPIO_PIN_15
#define CHARGE_EN_OUT_GPIO_Port GPIOA
#define BOOM_LOW_SIDE_OUT_1_Pin GPIO_PIN_2
#define BOOM_LOW_SIDE_OUT_1_GPIO_Port GPIOD
#define BOOM_LOW_SIDE_OUT_2_Pin GPIO_PIN_4
#define BOOM_LOW_SIDE_OUT_2_GPIO_Port GPIOB
#define PUMP_2_Pin GPIO_PIN_6
#define PUMP_2_GPIO_Port GPIOB
#define LED_GREEN_OUT_Pin GPIO_PIN_7
#define LED_GREEN_OUT_GPIO_Port GPIOB
#define PUMP_1_Pin GPIO_PIN_8
#define PUMP_1_GPIO_Port GPIOB
#define LED_YELLOW_OUT_Pin GPIO_PIN_9
#define LED_YELLOW_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

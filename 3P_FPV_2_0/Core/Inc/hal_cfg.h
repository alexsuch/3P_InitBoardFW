/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_cfg.h
  * @brief          : HAL configuration definitions and macros
  ******************************************************************************
  * @attention
  *
  * This file contains hardware abstraction layer configuration definitions
  * that can be easily modified to adapt to different hardware configurations
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __HAL_CFG_H
#define __HAL_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Timer Definitions ---------------------------------------------------------*/
#define SYS_TICK_TIMER_BASE         TIM3        /* System tick timer instance */
#define DETON_HIGH_SIDE_SWITH_TIMER_BASE    TIM14       /* Detonation high-side switch PWM timer */
#define PWM_PUMP_TIMER_BASE         TIM16       /* Pump PWM timer */
#define PWM_STICK_COUNTER_TIMER     TIM17       /* Stick counter timer (10us) */

/* Timer Channels ------------------------------------------------------------*/
#define DETON_HIGH_SIDE_SWITH_CHANNEL       TIM_CHANNEL_1   /* Detonation high-side switch PWM channel */
#define PWM_PUMP_CHANNEL            TIM_CHANNEL_1   /* Pump PWM channel */

/* SPI Configuration ---------------------------------------------------------*/
#define ACC_SPI_INSTANCE            SPI1        /* Accelerometer SPI instance */

/* UART Configuration --------------------------------------------------------*/
#define MAIN_UART_INSTANCE          UART1       /* Main UART for communication */
#define VUSA_UART_INSTANCE          UART2       /* VUSA UART instance */

/* ADC Configuration ---------------------------------------------------------*/
#define MAIN_ADC_INSTANCE           ADC1        /* Main ADC instance */
#define ADC_CHANNELS_COUNT          2           /* Number of ADC channels */

/* GPIO Pin Definitions ------------------------------------------------------*/
/* SPI Chip Select */
#define SPI_CS_PORT                 SPI_CS_GPIO_Port
#define SPI_CS_PIN                  SPI_CS_Pin

/* LEDs */
#define LED_ERROR_PORT              LED_YELLOW_OUT_GPIO_Port
#define LED_ERROR_PIN               LED_YELLOW_OUT_Pin
#define LED_STATUS_PORT             LED_GREEN_OUT_GPIO_Port
#define LED_STATUS_PIN              LED_GREEN_OUT_Pin

/* Boom Control */
#define DETON_LOW_SIDE_SWITCH_OUT_1_PORT    BOOM_LOW_SIDE_OUT_1_GPIO_Port
#define DETON_LOW_SIDE_SWITCH_OUT_1_PIN     BOOM_LOW_SIDE_OUT_1_Pin
#define DETON_LOW_SIDE_SWITCH_OUT_2_PORT    BOOM_LOW_SIDE_OUT_2_GPIO_Port
#define DETON_LOW_SIDE_SWITCH_OUT_2_PIN     BOOM_LOW_SIDE_OUT_2_Pin

/* Input Pins */
#define FUSE_INPUT_PORT             FUSE_IN_GPIO_Port
#define FUSE_INPUT_PIN              FUSE_IN_Pin
#define ACC_INT_PORT                ACC_INT_GPIO_Port
#define ACC_INT_PIN                 ACC_INT_Pin

/* PWM Input Pins (for PWM control mode) */
#define PWM1_INPUT_PORT             GPIOA
#define PWM1_INPUT_PIN              GPIO_PIN_10
#define PWM1_INPUT_EXTI_IRQn        EXTI4_15_IRQn

/* PWM2 Input Pins (for PWM control mode) */
#define PWM2_INPUT_PORT             FC_PWM_GPIO_IN_GPIO_Port
#define PWM2_INPUT_PIN              FC_PWM_GPIO_IN_Pin
#define PWM2_INPUT_EXTI_IRQn        EXTI4_15_IRQn

/* UART Communication Pins */
#define COMM_UART_TX_PORT           GPIOA
#define COMM_UART_TX_PIN            GPIO_PIN_9      /* PA9 - USART1 TX */
#define COMM_UART_RX_PORT           GPIOA
#define COMM_UART_RX_PIN            GPIO_PIN_10     /* PA10 - USART1 RX */

/* Buzzer */
#define BUZZER_PORT                 EM_BUZZ_GPIO_Port
#define BUZZER_PIN                  EM_BUZZ_Pin

/* Charging Control */
#define CHARGE_EN_PORT              CHARGE_EN_OUT_GPIO_Port
#define CHARGE_EN_PIN               CHARGE_EN_OUT_Pin

/* External Handle References ------------------------------------------------*/
extern TIM_HandleTypeDef htim3;     /* System tick timer handle */
extern TIM_HandleTypeDef htim14;    /* Detonation high-side switch PWM timer handle */
extern TIM_HandleTypeDef htim16;    /* Pump PWM timer handle */
extern TIM_HandleTypeDef htim17;    /* Stick counter timer handle */
extern SPI_HandleTypeDef hspi1;    /* Accelerometer SPI handle */
extern UART_HandleTypeDef huart1;  /* Main UART handle */
extern UART_HandleTypeDef huart2;  /* VUSA UART handle */
extern ADC_HandleTypeDef hadc1;    /* Main ADC handle */

/* Macro Definitions ---------------------------------------------------------*/
#define SYS_TICK_TIMER_HANDLE       htim3
#define DETON_HIGH_SIDE_SWITH_TIMER_HANDLE  htim14
#define PWM_PUMP_TIMER_HANDLE       htim16
#define PWM_STICK_COUNTER_HANDLE    htim17
#define ACC_SPI_HANDLE              hspi1
#define MAIN_UART_HANDLE            huart1
#define VUSA_UART_HANDLE            huart2
#define MAIN_ADC_HANDLE             hadc1

/* Timeout Definitions -------------------------------------------------------*/
#define SPI_TIMEOUT_MS              10          /* SPI communication timeout */
#define UART_TIMEOUT_MS             100         /* UART communication timeout */

/* Buffer Size Definitions ---------------------------------------------------*/
#define SPI_WR_BUFFER_SIZE          7           /* SPI write buffer size */
#define UART_RX_TEMP_BUFFER_SIZE    10          /* UART RX temporary buffer size */
#define VUSA_PACKET_LENGTH          4           /* VUSA packet length */

/* Hardware Feature Flags ----------------------------------------------------*/
#ifndef BUZZER_DISABLE
#define BUZZER_DISABLE              0           /* Set to 1 to disable buzzer */
#endif

/* ADC Resolution ------------------------------------------------------------*/
#define ADC_RESOLUTION              ADC_RESOLUTION_12B  /* ADC resolution setting */
#define ADC_MAX_VALUE               4095        /* Maximum ADC value for 12-bit */

#ifdef __cplusplus
}
#endif

#endif /* __HAL_CFG_H */

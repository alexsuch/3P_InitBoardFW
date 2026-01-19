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
#define SYS_TICK_TIMER_BASE TIM2               /* System tick timer instance */
#define DETON_HIGH_SIDE_SWITH_TIMER_BASE TIM15 /* Detonation high-side switch PWM timer */
#define PWM_PUMP_TIMER_BASE TIM1               /* Pump PWM timer */
#define LOGGER_TIMESTAMP_TIMER_BASE TIM3       /* Logger global timestamp counter (slave to TIM6 @ 100kHz) */

/* Timer Channels ------------------------------------------------------------*/
#define DETON_HIGH_SIDE_SWITH_CHANNEL TIM_CHANNEL_2 /* Detonation high-side switch PWM channel */
#define PWM_PUMP_CHANNEL TIM_CHANNEL_2              /* Pump PWM channel */

/* PWM GPIO Configuration ----------------------------------------------------*/
/* Detonation High-Side Switch PWM (TIM15_CH2) */
#define DETON_PWM_PORT GPIOB
#define DETON_PWM_PIN GPIO_PIN_15
#define DETON_PWM_AF GPIO_AF1_TIM15

/* Pump PWM (TIM1_CH2 and TIM1_CH2N) */
#define PUMP_PWM_MAIN_PORT GPIOA
#define PUMP_PWM_MAIN_PIN GPIO_PIN_9
#define PUMP_PWM_COMPL_PORT GPIOB
#define PUMP_PWM_COMPL_PIN GPIO_PIN_14
#define PUMP_PWM_AF GPIO_AF6_TIM1

/* SPI Configuration ---------------------------------------------------------*/
#define ACC_SPI_INSTANCE SPI1 /* Accelerometer SPI instance */

/* SPI GPIO Configuration */
#define ACC_SPI_SCK_PORT GPIOB
#define ACC_SPI_SCK_PIN GPIO_PIN_3
#define ACC_SPI_MISO_PORT GPIOB
#define ACC_SPI_MISO_PIN GPIO_PIN_4
#define ACC_SPI_MOSI_PORT GPIOB
#define ACC_SPI_MOSI_PIN GPIO_PIN_5

/* UART Configuration --------------------------------------------------------*/
#define MAIN_UART_INSTANCE USART2 /* Main UART for communication */
#define VUSA_UART_INSTANCE USART3 /* VUSA UART instance */

/* Main UART GPIO Configuration */
#define MAIN_UART_TX_PORT GPIOA
#define MAIN_UART_TX_PIN GPIO_PIN_2
#define MAIN_UART_RX_PORT GPIOA
#define MAIN_UART_RX_PIN GPIO_PIN_3
#define MAIN_UART_GPIO_AF GPIO_AF7_USART2
#define MAIN_UART_IRQn USART2_IRQn

/* VUSA UART GPIO Configuration */
#define VUSA_UART_TX_PORT GPIOB
#define VUSA_UART_TX_PIN GPIO_PIN_10
#define VUSA_UART_RX_PORT GPIOB
#define VUSA_UART_RX_PIN GPIO_PIN_11
#define VUSA_UART_GPIO_AF GPIO_AF7_USART3

/* VUSA UART DMA Configuration */
// VUSA UART TX now uses DMA1_Channel4 to avoid conflict with ADC2 and SPI1 RX
#define VUSA_UART_DMA_INSTANCE DMA1_Channel4
#define VUSA_UART_DMA_REQUEST DMA_REQUEST_USART3_TX
#define VUSA_UART_DMA_IRQ DMA1_Channel4_IRQn
#define VUSA_UART_IRQn USART3_IRQn

/* ADC2 GPIO Configuration */
#define ADC2_IN_PORT GPIOA
#define ADC2_IN_PIN GPIO_PIN_7

/* GPIO Pin Definitions ------------------------------------------------------*/
/* Test GPIO Outputs */
#define TEST_1_PORT GPIOA
#define TEST_1_PIN GPIO_PIN_8
#define TEST_2_PORT GPIOA
#define TEST_2_PIN GPIO_PIN_11

/* LED Outputs */
#define LED_ERROR_OUT_PORT GPIOB
#define LED_ERROR_OUT_PIN GPIO_PIN_1
#define LED_STATUS_OUT_PORT GPIOB
#define LED_STATUS_OUT_PIN GPIO_PIN_0
#define EX_LED_OUT_PORT GPIOA
#define EX_LED_OUT_PIN GPIO_PIN_5

/* LED Aliases */
#define LED_ERROR_PORT LED_ERROR_OUT_PORT
#define LED_ERROR_PIN LED_ERROR_OUT_PIN
#define LED_STATUS_PORT LED_STATUS_OUT_PORT
#define LED_STATUS_PIN LED_STATUS_OUT_PIN

/* Boom Control Outputs */
#define BOOM_LOW_SIDE_OUT_1_PORT GPIOB
#define BOOM_LOW_SIDE_OUT_1_PIN GPIO_PIN_13
#define BOOM_LOW_SIDE_OUT_2_PORT GPIOB
#define BOOM_LOW_SIDE_OUT_2_PIN GPIO_PIN_12

/* Boom Aliases */
#define DETON_LOW_SIDE_SWITCH_OUT_1_PORT BOOM_LOW_SIDE_OUT_1_PORT
#define DETON_LOW_SIDE_SWITCH_OUT_1_PIN BOOM_LOW_SIDE_OUT_1_PIN
#define DETON_LOW_SIDE_SWITCH_OUT_2_PORT BOOM_LOW_SIDE_OUT_2_PORT
#define DETON_LOW_SIDE_SWITCH_OUT_2_PIN BOOM_LOW_SIDE_OUT_2_PIN

/* Charging Control Output */
#define CHARGE_EN_OUT_PORT GPIOA
#define CHARGE_EN_OUT_PIN GPIO_PIN_10

/* Charging Alias */
#define CHARGE_EN_PORT CHARGE_EN_OUT_PORT
#define CHARGE_EN_PIN CHARGE_EN_OUT_PIN

/* Input Pins */
#define FUSE_IN_PORT GPIOA
#define FUSE_IN_PIN GPIO_PIN_12

/* Fuse Input Alias */
#define FUSE_INPUT_PORT FUSE_IN_PORT
#define FUSE_INPUT_PIN FUSE_IN_PIN

/* Accelerometer SPI CS and interrupt pins */
#define ACC_CS_PORT GPIOA
#define ACC_CS_PIN GPIO_PIN_15

#define ACC_INT1_PORT GPIOB
#define ACC_INT1_PIN GPIO_PIN_7

#define ACC_INT2_PORT GPIOB
#define ACC_INT2_PIN GPIO_PIN_6

/* Legacy definitions for compatibility */
#define ACC_INT_PORT ACC_INT1_PORT
#define ACC_INT_PIN ACC_INT1_PIN

/* External Handle References ------------------------------------------------*/
extern TIM_HandleTypeDef htim1;          /* Pump PWM timer handle */
extern TIM_HandleTypeDef htim2;          /* System tick timer handle */
extern TIM_HandleTypeDef htim3;          /* Logger timestamp timer handle (slave to TIM6 @ 100 kHz) */
extern TIM_HandleTypeDef htim6;          /* ADC clock timer handle */
extern TIM_HandleTypeDef htim15;         /* Detonation high-side switch PWM timer handle */
extern SPI_HandleTypeDef hspi1;          /* Accelerometer SPI handle */
extern SPI_HandleTypeDef hspi2;          /* Logger SPI2 slave handle */
extern UART_HandleTypeDef huart2;        /* Main UART handle */
extern UART_HandleTypeDef huart3;        /* VUSA UART handle */
extern DMA_HandleTypeDef hdma_usart3_tx; /* VUSA UART DMA TX handle */
extern OPAMP_HandleTypeDef hopamp2;      /* OPAMP2 handle (CubeMX-generated in main.c) */
extern ADC_HandleTypeDef hadc2;

/* DAC Configuration (Test Signal Generation) -------------------------------*/
#define SYS_TICK_TIMER_HANDLE htim2
#define ADC_TICK_TIMER_HANDLE htim6
#define DETON_HIGH_SIDE_SWITH_TIMER_HANDLE htim15
#define PWM_PUMP_TIMER_HANDLE htim1
#define ACC_SPI_HANDLE hspi1
#define MAIN_UART_HANDLE huart2
#define VUSA_UART_HANDLE huart3
#define OPAMP_HANDLE hopamp2
#define ADC_PIEZO_HANDLE hadc2

/* Logger SPI Slave Configuration */
#define LOGGER_SPI_HANDLE hspi2             /* Logger SPI2 slave handle */
#define LOGGER_SPI_DATA_RDY_PORT GPIOB      /* Data ready signal GxPIO port */
#define LOGGER_SPI_DATA_RDY_PIN GPIO_PIN_11 /* Data ready signal GPIO pin (B11) - used only when SPI_LOGGER_ENABLE */

/* Timeout Definitions -------------------------------------------------------*/
#define SPI_TIMEOUT_MS 10   /* SPI communication timeout */
#define UART_TIMEOUT_MS 100 /* UART communication timeout */

/* Buffer Size Definitions ---------------------------------------------------*/
#define SPI_WR_BUFFER_SIZE 13       /* SPI write buffer size (13 for LSM6DS3 gyro+accel) */
#define UART_RX_TEMP_BUFFER_SIZE 10 /* UART RX temporary buffer size */
#define VUSA_PACKET_LENGTH 4        /* VUSA packet length */

/* ADC Resolution ------------------------------------------------------------*/

/* OPAMP Configuration ------------------------------------------------------*/
/* OPAMP instance used in the design (OPAMP1/OPAMP2/etc.) */
#define OPAMP_INSTANCE OPAMP2

/* DAC Configuration (Test Signal Generation) -------------------------------*/
#define TEST_DAC_INSTANCE DAC1
#define TEST_DAC_CHANNEL DAC_CHANNEL_1
#define TEST_DAC_GPIO_PORT GPIOA
#define TEST_DAC_GPIO_PIN GPIO_PIN_4
#define TEST_DAC_DMA_INSTANCE DMA1_Channel1
#define TEST_DAC_DMA_REQUEST DMA_REQUEST_DAC1_CHANNEL1
#define TEST_DAC_DMA_IRQ DMA1_Channel1_IRQn
#define TEST_DAC_TRIGGER DAC_TRIGGER_T6_TRGO

/* DAC Test Signal Parameters */
#define DAC_MAX_VALUE 4095 /* 12-bit DAC maximum value */
#define DAC_SAMPLES 151    /* Number of samples in DAC waveform buffer */

/* External Handle References ------------------------------------------------*/
extern TIM_HandleTypeDef htim1;          /* Pump PWM timer handle */
extern TIM_HandleTypeDef htim2;          /* System tick timer handle */
extern TIM_HandleTypeDef htim3;          /* Logger timestamp timer handle (slave to TIM6 @ 100 kHz) */
extern TIM_HandleTypeDef htim6;          /* ADC clock timer handle */
extern TIM_HandleTypeDef htim15;         /* Detonation high-side switch PWM timer handle */
extern SPI_HandleTypeDef hspi1;          /* Accelerometer SPI handle */
extern SPI_HandleTypeDef hspi2;          /* Logger SPI2 slave handle */
extern UART_HandleTypeDef huart2;        /* Main UART handle */
extern UART_HandleTypeDef huart3;        /* VUSA UART handle */
extern DMA_HandleTypeDef hdma_usart3_tx; /* VUSA UART DMA TX handle */
extern OPAMP_HandleTypeDef hopamp2;      /* OPAMP2 handle (CubeMX-generated in main.c) */
extern ADC_HandleTypeDef hadc2;

#if (TEST_DAC_ENABLE == 1u)
extern uint32_t dac_test_buffer[DAC_SAMPLES]; /* Test DAC waveform buffer */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HAL_CFG_H */

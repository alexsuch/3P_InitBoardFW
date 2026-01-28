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
#define SYS_TICK_TIMER_IRQ TIM2_IRQn           /* System tick timer interrupt */
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
#define ACC_SPI_GPIO_AF GPIO_AF5_SPI1

/* SPI DMA Configuration */
#define ACC_SPI_DMA_RX_INSTANCE DMA1_Channel5
#define ACC_SPI_DMA_TX_INSTANCE DMA1_Channel3
#define ACC_SPI_DMA_RX_REQUEST DMA_REQUEST_SPI1_RX
#define ACC_SPI_DMA_TX_REQUEST DMA_REQUEST_SPI1_TX
#define ACC_SPI_DMA_RX_IRQ DMA1_Channel5_IRQn
#define ACC_SPI_DMA_TX_IRQ DMA1_Channel3_IRQn

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

/* ADC Piezo Configuration --------------------------------------------------*/
#define ADC_PIEZO_INSTANCE ADC2
#define ADC_PIEZO_IN_PORT GPIOA
#define ADC_PIEZO_IN_PIN GPIO_PIN_7
#define ADC_PIEZO_IRQ ADC1_2_IRQn

/* ADC Piezo DMA Configuration */
#define ADC_PIEZO_DMA_INSTANCE DMA1_Channel2
#define ADC_PIEZO_DMA_REQUEST DMA_REQUEST_ADC2
#define ADC_PIEZO_DMA_IRQ DMA1_Channel2_IRQn

/* OPAMP Configuration ------------------------------------------------------*/
/* OPAMP instance used in the design (OPAMP1/OPAMP2/etc.) */
#define OPAMP_INSTANCE OPAMP2

/* DAC Configuration (Test Signal Generation) -------------------------------*/
#define TEST_DAC_INSTANCE DAC1
#define TEST_DAC_CHANNEL DAC_CHANNEL_1
#define TEST_DAC_OUT_PORT GPIOA
#define TEST_DAC_OUT_PIN GPIO_PIN_4
#define TEST_DAC_DMA_INSTANCE DMA1_Channel1
#define TEST_DAC_DMA_REQUEST DMA_REQUEST_DAC1_CHANNEL1
#define TEST_DAC_DMA_IRQ DMA1_Channel1_IRQn
#define TEST_DAC_TRIGGER DAC_TRIGGER_T6_TRGO

/* ADC Tick Timer Trigger Configuration */
#define ADC_PIEZO_TRIGGER ADC_EXTERNALTRIG_T6_TRGO

/* Logger SPI Slave Configuration */
#define LOGGER_SPI_HANDLE hspi2            /* Logger SPI2 slave handle */
#define LOGGER_SPI_IRQ SPI2_IRQn           /* Logger SPI2 interrupt */
#define LOGGER_SPI_DATA_RDY_PORT GPIOB     /* Data ready signal GxPIO port */
/*
 * Logger SPI "DATA_RDY/INT" signal from STM32 -> ESP32.
 *
 * Note: This codebase can be reused across boards with different pin routing.
 * For the logger configuration and the dedicated logger board wiring, this signal is on PB11.
 */
#define LOGGER_SPI_DATA_RDY_PIN GPIO_PIN_11 /* Data ready signal GPIO pin (B11) - used only when SPI_LOGGER_ENABLE */

/* COMP1 Hit Detection Configuration (DAC1 threshold) -----------------------*/
#define COMP1_INSTANCE COMP1
#define COMP1_IRQn COMP1_2_3_IRQn
#define COMP1_INP_PORT GPIOA
#define COMP1_INP_PIN GPIO_PIN_1

/* ADC Tick Timer Configuration (TIM6) ----------------------------------------*/
#define ADC_TICK_TIMER_INSTANCE TIM6
#define ADC_TICK_TIMER_IRQ TIM6_DAC_IRQn
#define ADC_TICK_TIMER_RCC_CLK_ENABLE __HAL_RCC_TIM6_CLK_ENABLE

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
extern DMA_HandleTypeDef hdma_spi1_rx;   /* SPI1 RX DMA handle */
extern DMA_HandleTypeDef hdma_spi1_tx;   /* SPI1 TX DMA handle */
extern SPI_HandleTypeDef hspi2;          /* Logger SPI2 slave handle */
extern UART_HandleTypeDef huart2;        /* Main UART handle */
extern UART_HandleTypeDef huart3;        /* VUSA UART handle */
extern DMA_HandleTypeDef hdma_usart3_tx; /* VUSA UART DMA TX handle */
extern OPAMP_HandleTypeDef hopamp2;      /* OPAMP2 handle (CubeMX-generated in main.c) */
extern ADC_HandleTypeDef hadc2;          /* ADC2 Piezo handle */
extern DMA_HandleTypeDef hdma_adc2;      /* ADC2 DMA handle */
extern DAC_HandleTypeDef hdac1;          /* DAC1 handle */
extern DMA_HandleTypeDef hdma_dac1_ch1;  /* DAC1 DMA handle */
extern COMP_HandleTypeDef hcomp1;        /* COMP1 handle */

/* Handle Macros -------------------------------------------------------------*/
#define SYS_TICK_TIMER_HANDLE htim2
#define ADC_TICK_TIMER_HANDLE htim6
#define DETON_HIGH_SIDE_SWITH_TIMER_HANDLE htim15
#define PWM_PUMP_TIMER_HANDLE htim1
#define ACC_SPI_HANDLE hspi1
#define ACC_SPI_DMA_RX_HANDLE hdma_spi1_rx
#define ACC_SPI_DMA_TX_HANDLE hdma_spi1_tx
#define MAIN_UART_HANDLE huart2
#define VUSA_UART_HANDLE huart3
#define OPAMP_HANDLE hopamp2
#define ADC_PIEZO_HANDLE hadc2
#define ADC_PIEZO_DMA_HANDLE hdma_adc2
#define TEST_DAC_HANDLE hdac1
#define TEST_DAC_DMA_HANDLE hdma_dac1_ch1
#define COMP_HIT_HANDLE hcomp1

/**
 * @brief  Configure HAL peripherals (replaces MX_xxx_Init functions)
 * @note   Call this function before Solution_HalInit()
 * @retval None
 */
void Solution_HalConfigure(void);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_CFG_H */

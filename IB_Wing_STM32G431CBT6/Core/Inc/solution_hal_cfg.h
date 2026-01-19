/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : solution_hal_cfg.h
 * @brief          : Solution-specific HAL configuration header
 ******************************************************************************
 * @attention
 *
 * This file contains custom HAL initialization functions that replace
 * CubeMX-generated code for easier project portability between chips
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __SOLUTION_HAL_CFG_H
#define __SOLUTION_HAL_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_cfg.h"
#include "prj_config.h"
#include "stm32g4xx_hal.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Configure HAL peripherals (replaces MX_xxx_Init functions)
 * @note   Call this function before Solution_HalInit()
 * @retval None
 */
void Solution_HalConfigure(void);

/**
 * @brief  Start DAC DMA after TIM6 is running
 * @note   Call this function from Solution_HalInit() after timer start
 * @retval HAL status
 */
HAL_StatusTypeDef Solution_DacStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __SOLUTION_HAL_CFG_H */

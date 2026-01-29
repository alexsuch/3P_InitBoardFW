/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : solution_hal_cfg.c
 * @brief          : Solution-specific HAL configuration implementation
 ******************************************************************************
 * @attention
 *
 * This file contains custom HAL initialization functions that replace
 * CubeMX-generated code for easier project portability between chips
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "hal_cfg.h"
#include "main.h"
#include "prj_config.h"
#include "solution_wrapper.h"

/* Private variables ---------------------------------------------------------*/
/* Timer handles */
TIM_HandleTypeDef htim1;  /* Pump PWM timer handle */
TIM_HandleTypeDef htim2;  /* System tick timer handle */
TIM_HandleTypeDef htim7;  /* Logger timestamp timer handle (free-running) */
TIM_HandleTypeDef htim15; /* Detonation high-side switch PWM timer handle */

/* UART handles */
UART_HandleTypeDef huart2; /* Main UART handle */
UART_HandleTypeDef huart3; /* VUSA UART handle */

/* DMA handles */
DMA_HandleTypeDef hdma_usart3_tx; /* VUSA UART DMA TX handle */
DAC_HandleTypeDef hdac1;          /* Test DAC handle */
DMA_HandleTypeDef hdma_dac1_ch1;  /* Test DAC DMA handle */
SPI_HandleTypeDef hspi1;          /* Accelerometer SPI handle */
DMA_HandleTypeDef hdma_spi1_rx;   /* SPI1 RX DMA handle */
DMA_HandleTypeDef hdma_spi1_tx;   /* SPI1 TX DMA handle */
COMP_HandleTypeDef hcomp1;

/* Private function prototypes -----------------------------------------------*/

static HAL_StatusTypeDef HalConfigure_Gpio_Init(void);
static HAL_StatusTypeDef HalConfigure_SysTickTimer_Init(void);
#if (SPI_LOGGER_ENABLE == 0)
static HAL_StatusTypeDef HalConfigure_HighSidePwmTimer_Init(void);
static HAL_StatusTypeDef HalConfigure_PumpPwmTimer_Init(void);
static HAL_StatusTypeDef HalConfigure_VusaUart_Init(void);
#endif /* SPI_LOGGER_ENABLE == 0 */

#if (SPI_LOGGER_ENABLE == 1u)
static HAL_StatusTypeDef HalConfigure_TimestampTimer_Init(void);
#endif

static HAL_StatusTypeDef HalConfigure_MainUart_Init(void);
static HAL_StatusTypeDef HalConfigure_AccSpi_Init(void);
static HAL_StatusTypeDef HalConfigure_Opamp_Init(void);
static HAL_StatusTypeDef HalConfigure_Adc2_Init(void);
static HAL_StatusTypeDef HalConfigure_AdcTickTimer_Init(void);
#if (TEST_DAC_ENABLE == 1u)
static HAL_StatusTypeDef HalConfigure_Dac1_Init(void);
#endif
#if (COMP_HIT_DETECTION_ENABLE == 1u)
static HAL_StatusTypeDef HalConfigure_Comp1_Dac1_Init(void);
#endif
static HAL_StatusTypeDef HalConfigure_DMA_Init(void);

/**
 * @brief  Configure HAL peripherals (replaces MX_xxx_Init functions)
 * @note   Call this function before Solution_HalInit()
 * @retval None
 */
void Solution_HalConfigure(void) {
    /* Initialize GPIO pins */
    if (HalConfigure_Gpio_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize DMA controller */
    if (HalConfigure_DMA_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize System Tick Timer */
    if (HalConfigure_SysTickTimer_Init() != HAL_OK) {
        Error_Handler();
    }

#if (SPI_LOGGER_ENABLE == 0)
    /* Initialize Detonation PWM Timer */
    if (HalConfigure_HighSidePwmTimer_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize Pump PWM Timer */
    if (HalConfigure_PumpPwmTimer_Init() != HAL_OK) {
        Error_Handler();
    }
#endif
    /* Initialize Main UART */
    if (HalConfigure_MainUart_Init() != HAL_OK) {
        Error_Handler();
    }

#if (SPI_LOGGER_ENABLE == 0)
    /* Initialize VUSA UART (only when not using Logger mode) */
    if (HalConfigure_VusaUart_Init() != HAL_OK) {
        Error_Handler();
    }
#endif

    /* Initialize ADC2 */
    if (HalConfigure_Adc2_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize ADC Tick Timer */
    if (HalConfigure_AdcTickTimer_Init() != HAL_OK) {
        Error_Handler();
    }

#if (TEST_DAC_ENABLE == 1u)
/* Initialize DAC1 (Test Signal Generator) */
#if (SPI_LOGGER_ENABLE == 1u)
    /* Initialize hardware timestamp timer */
    if (HalConfigure_TimestampTimer_Init() != HAL_OK) {
        Error_Handler();
    }
#endif

    /* Initialize DAC1 (Test Signal Generator) */
    if (HalConfigure_Dac1_Init() != HAL_OK) {
        Error_Handler();
    }
#endif

#if (COMP_HIT_DETECTION_ENABLE == 1u)
    /* Initialize COMP1 with DAC1 threshold for hit detection */
    if (HalConfigure_Comp1_Dac1_Init() != HAL_OK) {
        Error_Handler();
    }
#endif

    /* Initialize Accelerometer SPI */
    if (HalConfigure_AccSpi_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize OPAMP (moved from CubeMX) */
    if (HalConfigure_Opamp_Init() != HAL_OK) {
        Error_Handler();
    }
}
/**
 * @brief  Initialize DMA controller clocks (global DMA setup)
 * @note   Call before any peripheral using DMA
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_DMA_Init(void) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    return HAL_OK;
}

/**
 * @brief  Initialize GPIO pins for LEDs, test outputs, boom control, charging, and inputs
 * @note   This replaces MX_GPIO_Init()
 *         Includes: Clock enable, pin configuration
 *         Independent from CubeMX configuration
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_Gpio_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ========== Clock Configuration ========== */
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ========== Set Initial Output Levels ========== */
    /* GPIOA outputs: TEST_1, EX_LED_OUT, CHARGE_EN_OUT, TEST_2 - all LOW */
    HAL_GPIO_WritePin(GPIOA, TEST_1_PIN | EX_LED_OUT_PIN | CHARGE_EN_OUT_PIN | TEST_2_PIN, GPIO_PIN_RESET);

#if (SPI_LOGGER_ENABLE == 0)
    /* GPIOB outputs: LED_ERROR, LED_STATUS, BOOM_LOW_SIDE_OUT_1, BOOM_LOW_SIDE_OUT_2 - all LOW */
    HAL_GPIO_WritePin(GPIOB, LED_ERROR_OUT_PIN | LED_STATUS_OUT_PIN | BOOM_LOW_SIDE_OUT_1_PIN | BOOM_LOW_SIDE_OUT_2_PIN, GPIO_PIN_RESET);
#else
    /* GPIOB outputs: LOGGER_SPI_DATA_RDY - initially LOW (no data ready) */
    HAL_GPIO_WritePin(LOGGER_SPI_DATA_RDY_PORT, LOGGER_SPI_DATA_RDY_PIN, GPIO_PIN_RESET);
#endif

    /* ========== Configure GPIOA Output Pins ========== */
    GPIO_InitStruct.Pin = TEST_1_PIN | EX_LED_OUT_PIN | CHARGE_EN_OUT_PIN | TEST_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if (SPI_LOGGER_ENABLE == 0)
    /* ========== Configure GPIOB Output Pins (standard detonation mode) ========== */
    /* Configure LED_ERROR_OUT (PB1), LED_STATUS_OUT (PB2), BOOM_LOW_SIDE_OUT_2 (PB12), BOOM_LOW_SIDE_OUT_1 (PB13) */
    GPIO_InitStruct.Pin = LED_ERROR_OUT_PIN | LED_STATUS_OUT_PIN | BOOM_LOW_SIDE_OUT_2_PIN | BOOM_LOW_SIDE_OUT_1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
    /* ========== Configure GPIOB Output Pins (logger mode) ========== */
    /*
     * Logger mode (SPI_LOGGER_ENABLE):
     * - Some boards route VUSA UART RX to PB11 (see HalConfigure_VusaUart_Init()).
     * - The dedicated logger board uses PB11 for LOGGER_SPI_DATA_RDY (STM32 -> ESP32 INT/READY),
     *   as defined by LOGGER_SPI_DATA_RDY_PIN in hal_cfg.h.
     */
    GPIO_InitStruct.Pin = LOGGER_SPI_DATA_RDY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LOGGER_SPI_DATA_RDY_PORT, &GPIO_InitStruct);
#endif

    /* ========== Configure Input Pins ========== */
    /* Configure FUSE_IN (PA12) as input */
    GPIO_InitStruct.Pin = FUSE_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FUSE_IN_PORT, &GPIO_InitStruct);

    return HAL_OK;
}

/**
 * @brief  Initialize system tick timer (TIM2) for 10ms interrupts
 * @note   Timer clock: APB1 x2 = 168 MHz (when APB1 prescaler != 1)
 *         Prescaler: 1680 (1679+1)
 *         Period: 1000 (999+1)
 *         Interrupt frequency: 168MHz / (1680 * 1000) = 100 Hz (10ms)
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_SysTickTimer_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Enable TIM2 peripheral clock */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* ========== Configure timer base ========== */
    SYS_TICK_TIMER_HANDLE.Instance = SYS_TICK_TIMER_BASE;
    SYS_TICK_TIMER_HANDLE.Init.Prescaler = SYS_TICK_TIMER_PRESCALER; /* 1680 divider */
    SYS_TICK_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    SYS_TICK_TIMER_HANDLE.Init.Period = SYS_TICK_TIMER_PERIOD; /* 1000 counts */
    SYS_TICK_TIMER_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    SYS_TICK_TIMER_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&SYS_TICK_TIMER_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure clock source */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    status = HAL_TIM_ConfigClockSource(&SYS_TICK_TIMER_HANDLE, &sClockSourceConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure master mode */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    status = HAL_TIMEx_MasterConfigSynchronization(&SYS_TICK_TIMER_HANDLE, &sMasterConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* ========== NVIC Configuration ========== */
    /* Configure TIM2 interrupt */
    HAL_NVIC_SetPriority(SYS_TICK_TIMER_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(SYS_TICK_TIMER_IRQ);

    return HAL_OK;
}

#if (SPI_LOGGER_ENABLE == 0)
/**
 * @brief  Initialize detonation high-side switch PWM timer (TIM15) for 138kHz PWM
 * @note   Timer clock: APB2 x2 = 168 MHz (when APB2 prescaler != 1)
 *         Prescaler: 1 (0+1)
 *         Period: 1217 (1216+1)
 *         PWM frequency: 168MHz / (1 * 1217) = 138.05 kHz
 *         Duty cycle: 50% (608/1217)
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_HighSidePwmTimer_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Enable TIM15 and GPIOB peripheral clocks */
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ========== Configure timer base ========== */
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Instance = DETON_HIGH_SIDE_SWITH_TIMER_BASE;
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.Prescaler = DETON_PWM_TIMER_PRESCALER; /* No prescaler for high resolution */
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.Period = DETON_PWM_TIMER_PERIOD; /* 168MHz / 1217 = 138kHz */
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.RepetitionCounter = 0;
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure clock source */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    status = HAL_TIM_ConfigClockSource(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, &sClockSourceConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* Initialize PWM mode */
    status = HAL_TIM_PWM_Init(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure master mode */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    status = HAL_TIMEx_MasterConfigSynchronization(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, &sMasterConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure PWM channel */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = DETON_PWM_PULSE; /* 50% duty cycle (1217/2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    status = HAL_TIM_PWM_ConfigChannel(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, &sConfigOC, DETON_HIGH_SIDE_SWITH_CHANNEL);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure break and dead time */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    status = HAL_TIMEx_ConfigBreakDeadTime(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, &sBreakDeadTimeConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* ========== GPIO Configuration ========== */
    /* Configure PB15 for TIM15_CH2 PWM output */
    GPIO_InitStruct.Pin = DETON_PWM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = DETON_PWM_AF;
    HAL_GPIO_Init(DETON_PWM_PORT, &GPIO_InitStruct);

    return HAL_OK;
}

/**
 * @brief  Initialize pump PWM timer (TIM1) for 40kHz PWM
 * @note   Timer clock: APB2 x2 = 168 MHz (when APB2 prescaler != 1)
 *         Prescaler: 1 (0+1)
 *         Period: 2100 (2099+1)
 *         Counter Mode: Center-aligned (counts up and down)
 *         PWM frequency: 168MHz / (1 * 2100 * 2) = 40 kHz
 *         Duty cycle: 50% (1050/2100)
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_PumpPwmTimer_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Enable TIM1, GPIOA and GPIOB peripheral clocks */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ========== Configure timer base ========== */
    PWM_PUMP_TIMER_HANDLE.Instance = PWM_PUMP_TIMER_BASE;
    PWM_PUMP_TIMER_HANDLE.Init.Prescaler = PUMP_PWM_TIMER_PRESCALER; /* No prescaler */
    PWM_PUMP_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    PWM_PUMP_TIMER_HANDLE.Init.Period = PUMP_PWM_TIMER_PERIOD; /* 168MHz / (1 * 2100 * 2) = 40kHz */
    PWM_PUMP_TIMER_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    PWM_PUMP_TIMER_HANDLE.Init.RepetitionCounter = 0;
    PWM_PUMP_TIMER_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_PWM_Init(&PWM_PUMP_TIMER_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure master mode */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    status = HAL_TIMEx_MasterConfigSynchronization(&PWM_PUMP_TIMER_HANDLE, &sMasterConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure PWM channel */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PUMP_PWM_PULSE; /* 50% duty cycle (2100/2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    status = HAL_TIM_PWM_ConfigChannel(&PWM_PUMP_TIMER_HANDLE, &sConfigOC, PWM_PUMP_CHANNEL);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure break and dead time */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    status = HAL_TIMEx_ConfigBreakDeadTime(&PWM_PUMP_TIMER_HANDLE, &sBreakDeadTimeConfig);
    if (status != HAL_OK) {
        return status;
    }

    /* ========== GPIO Configuration ========== */
    /* Configure PB14 for TIM1_CH2N (complementary output) */
    GPIO_InitStruct.Pin = PUMP_PWM_COMPL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = PUMP_PWM_AF;
    HAL_GPIO_Init(PUMP_PWM_COMPL_PORT, &GPIO_InitStruct);

    /* Configure PA9 for TIM1_CH2 (main output) */
    GPIO_InitStruct.Pin = PUMP_PWM_MAIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = PUMP_PWM_AF;
    HAL_GPIO_Init(PUMP_PWM_MAIN_PORT, &GPIO_InitStruct);

    return HAL_OK;
}
#endif /* SPI_LOGGER_ENABLE == 0 */

/**
 * @brief  Initialize Main UART (USART2) with full configuration
 * @note   This replaces MX_USART2_UART_Init() and HAL_UART_MspInit()
 *         Includes: Clock, GPIO, NVIC configuration
 *         Independent from CubeMX configuration
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_MainUart_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Configure peripheral clock source */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Enable peripheral clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* ========== GPIO Configuration ========== */
    /* Configure TX pin (PA2) - No pull resistor */
    GPIO_InitStruct.Pin = MAIN_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = MAIN_UART_GPIO_AF;
    HAL_GPIO_Init(MAIN_UART_TX_PORT, &GPIO_InitStruct);

    /* Configure RX pin (PA3) - Pull-up resistor */
    GPIO_InitStruct.Pin = MAIN_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = MAIN_UART_GPIO_AF;
    HAL_GPIO_Init(MAIN_UART_RX_PORT, &GPIO_InitStruct);

    /* ========== NVIC Configuration ========== */
    /* Configure USART2 interrupt */
    HAL_NVIC_SetPriority(MAIN_UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(MAIN_UART_IRQn);

    /* ========== UART Configuration ========== */
    /* Configure UART parameters:
     * - Baud Rate: 9600
     * - Word Length: 8 bits
     * - Stop Bits: 1
     * - Parity: None
     * - Flow Control: None
     */
    MAIN_UART_HANDLE.Instance = MAIN_UART_INSTANCE;
    MAIN_UART_HANDLE.Init.BaudRate = MAIN_UART_BAUD_RATE;
    MAIN_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
    MAIN_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
    MAIN_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
    MAIN_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
    MAIN_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    MAIN_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
    MAIN_UART_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    MAIN_UART_HANDLE.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    MAIN_UART_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status = HAL_UART_Init(&MAIN_UART_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure FIFO thresholds */
    status = HAL_UARTEx_SetTxFifoThreshold(&MAIN_UART_HANDLE, UART_TXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UARTEx_SetRxFifoThreshold(&MAIN_UART_HANDLE, UART_RXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK) {
        return status;
    }

    /* Disable FIFO mode */
    status = HAL_UARTEx_DisableFifoMode(&MAIN_UART_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

#if (SPI_LOGGER_ENABLE == 0)
/**
 * @brief  Initialize VUSA UART (USART3) with full configuration
 * @note   This replaces MX_USART3_UART_Init() and HAL_UART_MspInit()
 *         Includes: Clock, GPIO, DMA, NVIC configuration
 *         Independent from CubeMX configuration
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_VusaUart_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Configure peripheral clock source */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Enable peripheral clocks */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ========== GPIO Configuration ========== */
    /* Configure TX pin (PB10) - No pull resistor */
    GPIO_InitStruct.Pin = VUSA_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = VUSA_UART_GPIO_AF;
    HAL_GPIO_Init(VUSA_UART_TX_PORT, &GPIO_InitStruct);

    /* Configure RX pin (PB11) - Pull-up resistor */
    GPIO_InitStruct.Pin = VUSA_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = VUSA_UART_GPIO_AF;
    HAL_GPIO_Init(VUSA_UART_RX_PORT, &GPIO_InitStruct);

    /* ========== DMA Configuration ========== */
    /* Configure DMA for USART3_TX using macros */
    hdma_usart3_tx.Instance = VUSA_UART_DMA_INSTANCE;
    hdma_usart3_tx.Init.Request = VUSA_UART_DMA_REQUEST;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_usart3_tx);
    if (status != HAL_OK) {
        return status;
    }

    /* Link DMA handle to UART */
    __HAL_LINKDMA(&VUSA_UART_HANDLE, hdmatx, hdma_usart3_tx);

    /* ========== NVIC Configuration ========== */
    /* Configure DMA interrupt for USART3_TX using macro */
    HAL_NVIC_SetPriority(VUSA_UART_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(VUSA_UART_DMA_IRQ);

    /* Configure USART3 interrupt */
    HAL_NVIC_SetPriority(VUSA_UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(VUSA_UART_IRQn);

    /* ========== UART Configuration ========== */
    /* Configure UART parameters:
     * - Baud Rate: 115200
     * - Word Length: 8 bits
     * - Stop Bits: 1
     * - Parity: None
     * - Flow Control: None
     */
    VUSA_UART_HANDLE.Instance = VUSA_UART_INSTANCE;
    VUSA_UART_HANDLE.Init.BaudRate = VUSA_UART_BAUD_RATE;
    VUSA_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
    VUSA_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
    VUSA_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
    VUSA_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
    VUSA_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    VUSA_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
    VUSA_UART_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    VUSA_UART_HANDLE.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    VUSA_UART_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status = HAL_UART_Init(&VUSA_UART_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure FIFO thresholds */
    status = HAL_UARTEx_SetTxFifoThreshold(&VUSA_UART_HANDLE, UART_TXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UARTEx_SetRxFifoThreshold(&VUSA_UART_HANDLE, UART_RXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK) {
        return status;
    }

    /* Disable FIFO mode */
    status = HAL_UARTEx_DisableFifoMode(&VUSA_UART_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}
#endif /* (SPI_LOGGER_ENABLE == 0) */

/**
 * @brief  Initialize Accelerometer SPI (SPI1) with DMA
 * @note   This replaces MX_SPI1_Init() and HAL_SPI_MspInit()
 *         Includes: Clock, GPIO, DMA, NVIC configuration
 *         SPI Mode 3 (CPOL=1, CPHA=1) for LSM6DS3/LIS2DH12
 *         Uses DMA for RX and TX
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_AccSpi_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_StatusTypeDef status;

    /* ========== Clock Configuration ========== */
    /* Enable peripheral clocks */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ========== GPIO Configuration ========== */
    /* Configure SPI1 GPIO pins using definitions from hal_cfg.h */

    /* MISO pin */
    GPIO_InitStruct.Pin = ACC_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = ACC_SPI_GPIO_AF;
    HAL_GPIO_Init(ACC_SPI_MISO_PORT, &GPIO_InitStruct);

    /* SCK pin */
    GPIO_InitStruct.Pin = ACC_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = ACC_SPI_GPIO_AF;
    HAL_GPIO_Init(ACC_SPI_SCK_PORT, &GPIO_InitStruct);

    /* MOSI pin */
    GPIO_InitStruct.Pin = ACC_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = ACC_SPI_GPIO_AF;
    HAL_GPIO_Init(ACC_SPI_MOSI_PORT, &GPIO_InitStruct);

    /* ========== DMA Configuration ========== */
    /* SPI1_RX DMA Init */
    ACC_SPI_DMA_RX_HANDLE.Instance = ACC_SPI_DMA_RX_INSTANCE;
    ACC_SPI_DMA_RX_HANDLE.Init.Request = ACC_SPI_DMA_RX_REQUEST;
    ACC_SPI_DMA_RX_HANDLE.Init.Direction = DMA_PERIPH_TO_MEMORY;
    ACC_SPI_DMA_RX_HANDLE.Init.PeriphInc = DMA_PINC_DISABLE;
    ACC_SPI_DMA_RX_HANDLE.Init.MemInc = DMA_MINC_ENABLE;
    ACC_SPI_DMA_RX_HANDLE.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ACC_SPI_DMA_RX_HANDLE.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    ACC_SPI_DMA_RX_HANDLE.Init.Mode = DMA_NORMAL;
    ACC_SPI_DMA_RX_HANDLE.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&ACC_SPI_DMA_RX_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    __HAL_LINKDMA(&ACC_SPI_HANDLE, hdmarx, ACC_SPI_DMA_RX_HANDLE);

    /* SPI1_TX DMA Init */
    ACC_SPI_DMA_TX_HANDLE.Instance = ACC_SPI_DMA_TX_INSTANCE;
    ACC_SPI_DMA_TX_HANDLE.Init.Request = ACC_SPI_DMA_TX_REQUEST;
    ACC_SPI_DMA_TX_HANDLE.Init.Direction = DMA_MEMORY_TO_PERIPH;
    ACC_SPI_DMA_TX_HANDLE.Init.PeriphInc = DMA_PINC_DISABLE;
    ACC_SPI_DMA_TX_HANDLE.Init.MemInc = DMA_MINC_ENABLE;
    ACC_SPI_DMA_TX_HANDLE.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ACC_SPI_DMA_TX_HANDLE.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    ACC_SPI_DMA_TX_HANDLE.Init.Mode = DMA_NORMAL;
    ACC_SPI_DMA_TX_HANDLE.Init.Priority = DMA_PRIORITY_LOW;

    status = HAL_DMA_Init(&ACC_SPI_DMA_TX_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    __HAL_LINKDMA(&ACC_SPI_HANDLE, hdmatx, ACC_SPI_DMA_TX_HANDLE);

    /* ========== SPI Configuration ========== */
    /* Configure SPI1 parameters:
     * - Mode: Master
     * - Direction: Full-Duplex (2 lines)
     * - Data Size: 8-bit
     * - Clock Polarity: HIGH (CPOL=1) for SPI Mode 3
     * - Clock Phase: 2nd Edge (CPHA=1) for SPI Mode 3
     * - NSS: Software control
     * - Baud Rate Prescaler: 16 (APB2=84MHz / 16 = 5.25 MHz)
     * - Bit Order: MSB First
     */
    ACC_SPI_HANDLE.Instance = ACC_SPI_INSTANCE;
    ACC_SPI_HANDLE.Init.Mode = SPI_MODE_MASTER;
    ACC_SPI_HANDLE.Init.Direction = SPI_DIRECTION_2LINES;
    ACC_SPI_HANDLE.Init.DataSize = SPI_DATASIZE_8BIT;
    ACC_SPI_HANDLE.Init.CLKPolarity = SPI_POLARITY_HIGH; /* CPOL=1 */
    ACC_SPI_HANDLE.Init.CLKPhase = SPI_PHASE_2EDGE;      /* CPHA=1 */
    ACC_SPI_HANDLE.Init.NSS = SPI_NSS_SOFT;
    ACC_SPI_HANDLE.Init.BaudRatePrescaler = ACC_SPI_BAUD_RATE_PRESCALER; /* APB2=84MHz / 16 = 5.25 MHz */
    ACC_SPI_HANDLE.Init.FirstBit = SPI_FIRSTBIT_MSB;
    ACC_SPI_HANDLE.Init.TIMode = SPI_TIMODE_DISABLE;
    ACC_SPI_HANDLE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    ACC_SPI_HANDLE.Init.CRCPolynomial = 7;
    ACC_SPI_HANDLE.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    ACC_SPI_HANDLE.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    status = HAL_SPI_Init(&ACC_SPI_HANDLE);
    if (status != HAL_OK) {
        return status;
    }

    /* ========== CS and INT Pins Configuration ========== */
    /* Configure PC11 (ACC_CS) as output push-pull, initially HIGH (inactive) */
    GPIO_InitStruct.Pin = ACC_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACC_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET); /* CS HIGH = inactive */

    /* Configure PB7 (ACC_INT1) and PB6 (ACC_INT2) as inputs with pull-down */
    GPIO_InitStruct.Pin = ACC_INT1_PIN | ACC_INT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ========== NVIC Configuration ========== */
    /* Configure DMA interrupts for SPI1 RX/TX */
    HAL_NVIC_SetPriority(ACC_SPI_DMA_RX_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ACC_SPI_DMA_RX_IRQ);
    HAL_NVIC_SetPriority(ACC_SPI_DMA_TX_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ACC_SPI_DMA_TX_IRQ);

    return HAL_OK;
}

#if 1
/**
 * @brief  Initialize OPAMP2 (mirror of CubeMX MX_OPAMP2_Init)
 * @note   Uses the CubeMX-generated OPAMP handle `hopamp2` defined in main.c
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_Opamp_Init(void) {
    /* Configure OPAMP2 parameters as in CubeMX */
    OPAMP_HANDLE.Instance = OPAMP_INSTANCE;
    OPAMP_HANDLE.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
    OPAMP_HANDLE.Init.Mode = OPAMP_FOLLOWER_MODE;
    OPAMP_HANDLE.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    OPAMP_HANDLE.Init.InternalOutput = DISABLE;
    OPAMP_HANDLE.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    OPAMP_HANDLE.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

    if (HAL_OPAMP_Init(&OPAMP_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}
#endif

/* --- Modular peripheral initializers for ADC2, TIM6, DAC1 --- */

static HAL_StatusTypeDef HalConfigure_Adc2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_StatusTypeDef status;
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Enable clocks */
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure ADC2 GPIO */
    GPIO_InitStruct.Pin = ADC_PIEZO_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_PIEZO_IN_PORT, &GPIO_InitStruct);

    /* Configure DMA for ADC2 */
    ADC_PIEZO_DMA_HANDLE.Instance = ADC_PIEZO_DMA_INSTANCE;
    ADC_PIEZO_DMA_HANDLE.Init.Request = ADC_PIEZO_DMA_REQUEST;
    ADC_PIEZO_DMA_HANDLE.Init.Direction = DMA_PERIPH_TO_MEMORY;
    ADC_PIEZO_DMA_HANDLE.Init.PeriphInc = DMA_PINC_DISABLE;
    ADC_PIEZO_DMA_HANDLE.Init.MemInc = DMA_MINC_ENABLE;
    ADC_PIEZO_DMA_HANDLE.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    ADC_PIEZO_DMA_HANDLE.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    ADC_PIEZO_DMA_HANDLE.Init.Mode = DMA_CIRCULAR;
    ADC_PIEZO_DMA_HANDLE.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&ADC_PIEZO_DMA_HANDLE);
    if (status != HAL_OK) return status;

    /* Link DMA to ADC2 handle */
    __HAL_LINKDMA(&ADC_PIEZO_HANDLE, DMA_Handle, ADC_PIEZO_DMA_HANDLE);

    /* DMA1_Channel2_IRQn interrupt configuration for ADC2 */
    HAL_NVIC_SetPriority(ADC_PIEZO_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_PIEZO_DMA_IRQ);

    /* NVIC configuration for ADC2 (DMA interrupt already configured in MX_DMA_Init) */
    HAL_NVIC_SetPriority(ADC_PIEZO_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_PIEZO_IRQ);

    /* ADC2 configuration */
    ADC_PIEZO_HANDLE.Instance = ADC_PIEZO_INSTANCE;
    ADC_PIEZO_HANDLE.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_PIEZO_HANDLE.Init.Resolution = ADC_RESOLUTION_12B;
    ADC_PIEZO_HANDLE.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_PIEZO_HANDLE.Init.GainCompensation = 0;
    ADC_PIEZO_HANDLE.Init.ScanConvMode = ADC_SCAN_DISABLE;
    ADC_PIEZO_HANDLE.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    ADC_PIEZO_HANDLE.Init.LowPowerAutoWait = DISABLE;
    ADC_PIEZO_HANDLE.Init.ContinuousConvMode = DISABLE;
    ADC_PIEZO_HANDLE.Init.NbrOfConversion = 1;
    ADC_PIEZO_HANDLE.Init.DiscontinuousConvMode = DISABLE;
    ADC_PIEZO_HANDLE.Init.ExternalTrigConv = ADC_PIEZO_TRIGGER;
    ADC_PIEZO_HANDLE.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    ADC_PIEZO_HANDLE.Init.DMAContinuousRequests = ENABLE;
    ADC_PIEZO_HANDLE.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    ADC_PIEZO_HANDLE.Init.OversamplingMode = DISABLE;
    status = HAL_ADC_Init(&ADC_PIEZO_HANDLE);
    if (status != HAL_OK) return status;

    /* Configure ADC2 regular channel (match CubeMX) */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    status = HAL_ADC_ConfigChannel(&ADC_PIEZO_HANDLE, &sConfig);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief  Initialize ADC Tick Timer for ADC sampling clock
 * @note   This timer provides the trigger signal for ADC2 conversions
 *         Timer clock: APB1 x2 = 168 MHz (when APB1 prescaler != 1)
 *         Prescaler: 0 (no division)
 *         Period calculated based on ADC_SAMPLING_FREQ_KHZ configuration
 *         For 168MHz clock:
 *         - 100 kHz: Period = 1680 - 1 = 1679
 *         - 50 kHz:  Period = 3360 - 1 = 3359
 *         - 200 kHz: Period = 840 - 1 = 839
 * @retval HAL status
 */
static HAL_StatusTypeDef HalConfigure_AdcTickTimer_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* ========== Clock Configuration ========== */
    /* Enable ADC Tick Timer peripheral clock */
    ADC_TICK_TIMER_RCC_CLK_ENABLE();

    /* ========== Timer Period Calculation ========== */
    /* Calculate timer period based on ADC sampling frequency
     * Period = SystemClockFreq / (ADC_SAMPLING_FREQ_KHZ * 1000) - 1
     */
    uint32_t sysclk_freq = HAL_RCC_GetSysClockFreq();
    uint32_t adc_tick_timer_period = (sysclk_freq / (ADC_SAMPLING_FREQ_KHZ * 1000)) - 1;

    /* ========== Configure Timer Base ========== */
    ADC_TICK_TIMER_HANDLE.Instance = ADC_TICK_TIMER_INSTANCE;
    ADC_TICK_TIMER_HANDLE.Init.Prescaler = 0;
    ADC_TICK_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    ADC_TICK_TIMER_HANDLE.Init.Period = adc_tick_timer_period;
    ADC_TICK_TIMER_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&ADC_TICK_TIMER_HANDLE) != HAL_OK) return HAL_ERROR;

    /* ========== Configure Master Mode ========== */
    /* Configure as master trigger for ADC */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&ADC_TICK_TIMER_HANDLE, &sMasterConfig) != HAL_OK) return HAL_ERROR;

    /* ========== NVIC Configuration ========== */
    /* ADC Tick Timer interrupt configuration (optional - currently disabled) */
    // HAL_NVIC_SetPriority(ADC_TICK_TIMER_IRQ, 0, 0);
    // HAL_NVIC_EnableIRQ(ADC_TICK_TIMER_IRQ);

    return HAL_OK;
}

#if (SPI_LOGGER_ENABLE == 1u)
static HAL_StatusTypeDef HalConfigure_TimestampTimer_Init(void) {
    /* Enable TIM7 clock */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* Configure TIM7 as a free-running 16-bit counter ticking at ADC sampling rate (e.g. 100 kHz). */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const bool apb1_div1 = ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1);
    uint32_t timclk = apb1_div1 ? pclk1 : (pclk1 * 2u);
    uint32_t tick_hz = (uint32_t)ADC_SAMPLING_FREQ_KHZ * 1000u;
    if (tick_hz == 0) {
        return HAL_ERROR;
    }
    uint32_t prescaler = (timclk / tick_hz);
    if (prescaler == 0) {
        return HAL_ERROR;
    }

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = (uint16_t)(prescaler - 1u);
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 0xFFFFu;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}
#endif

#if (TEST_DAC_ENABLE == 1u)
static HAL_StatusTypeDef HalConfigure_Dac1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    DAC_ChannelConfTypeDef sConfig = {0};

    /* Enable DAC1, GPIO, DMA clocks */
    __HAL_RCC_DAC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure DAC GPIO */
    GPIO_InitStruct.Pin = TEST_DAC_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TEST_DAC_OUT_PORT, &GPIO_InitStruct);

    /* DAC1 DMA Init */
    /* DAC1_CH1 Init */
    TEST_DAC_DMA_HANDLE.Instance = TEST_DAC_DMA_INSTANCE;
    TEST_DAC_DMA_HANDLE.Init.Request = TEST_DAC_DMA_REQUEST;
    hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    TEST_DAC_DMA_HANDLE.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    TEST_DAC_DMA_HANDLE.Init.Mode = DMA_CIRCULAR;
    TEST_DAC_DMA_HANDLE.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&TEST_DAC_DMA_HANDLE) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&TEST_DAC_HANDLE, DMA_Handle1, TEST_DAC_DMA_HANDLE);

    /** DAC Initialization
     */
    TEST_DAC_HANDLE.Instance = TEST_DAC_INSTANCE;
    if (HAL_DAC_Init(&TEST_DAC_HANDLE) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = TEST_DAC_TRIGGER;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&TEST_DAC_HANDLE, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /* DMA1_Channel1_IRQn interrupt configuration for DAC DMA */
    HAL_NVIC_SetPriority(TEST_DAC_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(TEST_DAC_DMA_IRQ);

    /* Note: DAC DMA start moved to Solution_HalInit() to ensure TIM6 is running first */
    /* DAC1 interrupt Init */
    // HAL_NVIC_SetPriority(ADC_TICK_TIMER_IRQ, 0, 0);
    // HAL_NVIC_EnableIRQ(ADC_TICK_TIMER_IRQ);
    /* USER CODE BEGIN DAC1_MspInit 1 */

    return HAL_OK;
}

#endif /* TEST_DAC_ENABLE */

#if (COMP_HIT_DETECTION_ENABLE == 1u)
/**
 * @brief  Configure COMP1 with DAC1 threshold for hit detection
 * @note   COMP1 compares PA1 (positive input) vs DAC1 channel 1 (negative input)
 *         Interrupt generated when PA1 exceeds DAC1 threshold voltage
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
static HAL_StatusTypeDef HalConfigure_Comp1_Dac1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    DAC_ChannelConfTypeDef sConfig = {0};

    /* Enable clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DAC1_CLK_ENABLE();

    /* Configure PA1 as analog input for COMP1 positive input */
    GPIO_InitStruct.Pin = COMP1_INP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(COMP1_INP_PORT, &GPIO_InitStruct);

    /* Initialize DAC1 */
    TEST_DAC_HANDLE.Instance = TEST_DAC_INSTANCE;
    if (HAL_DAC_Init(&TEST_DAC_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_RCC_DAC1_CLK_ENABLE();
    /* DAC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_TICK_TIMER_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_TICK_TIMER_IRQ);

    /* Configure DAC1 Channel 1 for internal connection to COMP1 */
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE; /* No trigger, static threshold */
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL; /* Connect to COMP1 */
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&TEST_DAC_HANDLE, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Initialize COMP1 */
    COMP_HIT_HANDLE.Instance = COMP1_INSTANCE;
    COMP_HIT_HANDLE.Init.InputPlus = COMP_INPUT_PLUS_IO1;        /* PA1 signal input */
    COMP_HIT_HANDLE.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1; /* DAC1 threshold */
    COMP_HIT_HANDLE.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    COMP_HIT_HANDLE.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM; /* Medium hysteresis for noise immunity */
    COMP_HIT_HANDLE.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    COMP_HIT_HANDLE.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING; /* Interrupt on rising edge (PA1 > threshold) */
    if (HAL_COMP_Init(&COMP_HIT_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Configure NVIC for COMP1 interrupt */
    HAL_NVIC_SetPriority(COMP1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(COMP1_IRQn);

    return HAL_OK;
}

#endif /* COMP_HIT_DETECTION_ENABLE */

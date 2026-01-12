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
#include "solution_hal_cfg.h"

#include "hal_cfg.h"
#include "main.h"
#include "prj_config.h"

/* Private variables ---------------------------------------------------------*/
/* Timer handles */
TIM_HandleTypeDef htim1;  /* Pump PWM timer handle */
TIM_HandleTypeDef htim2;  /* System tick timer handle */
TIM_HandleTypeDef htim15; /* Detonation high-side switch PWM timer handle */

/* UART handles */
UART_HandleTypeDef huart2; /* Main UART handle */
UART_HandleTypeDef huart3; /* VUSA UART handle */

/* DMA handles */
DMA_HandleTypeDef hdma_usart3_tx; /* VUSA UART DMA TX handle */

/* SPI handles */
SPI_HandleTypeDef hspi1;        /* Accelerometer SPI handle */
DMA_HandleTypeDef hdma_spi1_rx; /* SPI1 RX DMA handle */
DMA_HandleTypeDef hdma_spi1_tx; /* SPI1 TX DMA handle */

/* Private function prototypes -----------------------------------------------*/

static HAL_StatusTypeDef HalConfigure_Gpio_Init(void);
static HAL_StatusTypeDef HalConfigure_SysTickTimer_Init(void);
#if (SPI_LOGGER_ENABLE == 0)
static HAL_StatusTypeDef HalConfigure_HighSidePwmTimer_Init(void);
static HAL_StatusTypeDef HalConfigure_PumpPwmTimer_Init(void);
#endif /* SPI_LOGGER_ENABLE == 0 */
static HAL_StatusTypeDef HalConfigure_MainUart_Init(void);
static HAL_StatusTypeDef HalConfigure_VusaUart_Init(void);
static HAL_StatusTypeDef HalConfigure_AccSpi_Init(void);
static HAL_StatusTypeDef HalConfigure_Opamp_Init(void);
static HAL_StatusTypeDef HalConfigure_Adc2_Init(void);
static HAL_StatusTypeDef HalConfigure_Tim6_Init(void);
static HAL_StatusTypeDef HalConfigure_Dac1_Init(void);
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

    /* Initialize TIM6 */
    if (HalConfigure_Tim6_Init() != HAL_OK) {
        Error_Handler();
    }

    /* Initialize DAC1 */
    if (HalConfigure_Dac1_Init() != HAL_OK) {
        Error_Handler();
    }

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
    /* Optionally, configure NVIC priorities for global DMA interrupts here */
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
    /* When SPI_LOGGER_ENABLE: PB11 is used for VUSA UART RX (configured in HalConfigure_VusaUart_Init)
       Configure LOGGER_SPI_DATA_RDY (PB9) as output push-pull for Logger ready signal */
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
    SYS_TICK_TIMER_HANDLE.Init.Prescaler = 1679; /* 1680 divider */
    SYS_TICK_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    SYS_TICK_TIMER_HANDLE.Init.Period = 999; /* 1000 counts */
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
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

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
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.Prescaler = 0; /* No prescaler for high resolution */
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    DETON_HIGH_SIDE_SWITH_TIMER_HANDLE.Init.Period = 1216; /* 168MHz / 1217 = 138kHz */
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
    sConfigOC.Pulse = 608; /* 50% duty cycle (1217/2) */
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
    PWM_PUMP_TIMER_HANDLE.Init.Prescaler = 0; /* No prescaler */
    PWM_PUMP_TIMER_HANDLE.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    PWM_PUMP_TIMER_HANDLE.Init.Period = 2099; /* 168MHz / (1 * 2100 * 2) = 40kHz */
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
    sConfigOC.Pulse = 1050; /* 50% duty cycle (2100/2) */
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
    MAIN_UART_HANDLE.Init.BaudRate = 9600;
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
    VUSA_UART_HANDLE.Init.BaudRate = 115200;
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
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(ACC_SPI_MISO_PORT, &GPIO_InitStruct);

    /* SCK pin */
    GPIO_InitStruct.Pin = ACC_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(ACC_SPI_SCK_PORT, &GPIO_InitStruct);

    /* MOSI pin */
    GPIO_InitStruct.Pin = ACC_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(ACC_SPI_MOSI_PORT, &GPIO_InitStruct);

    /* ========== DMA Configuration ========== */
    /* SPI1_RX DMA Init: використовує DMA1_Channel5 замість DMA1_Channel2 */
    hdma_spi1_rx.Instance = DMA1_Channel5;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_spi1_rx);
    if (status != HAL_OK) {
        return status;
    }

    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

    /* SPI1_TX DMA Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;

    status = HAL_DMA_Init(&hdma_spi1_tx);
    if (status != HAL_OK) {
        return status;
    }

    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

    /* ========== SPI Configuration ========== */
    /* Configure SPI1 parameters:
     * - Mode: Master
     * - Direction: Full-Duplex (2 lines)
     * - Data Size: 8-bit
     * - Clock Polarity: HIGH (CPOL=1) for SPI Mode 3
     * - Clock Phase: 2nd Edge (CPHA=1) for SPI Mode 3
     * - NSS: Software control
     * - Baud Rate Prescaler: 8 (APB2=84MHz / 8 = 10.5 MHz)
     * - Bit Order: MSB First
     */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH; /* CPOL=1 */
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;      /* CPHA=1 */
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* APB2=84MHz / 16 = 5.25 MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    status = HAL_SPI_Init(&hspi1);
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
    /* Configure DMA1 Channel 5 interrupt for SPI1_RX */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
    hopamp2.Instance = OPAMP_INSTANCE;
    hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
    hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
    hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp2.Init.InternalOutput = DISABLE;
    hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

    if (HAL_OPAMP_Init(&hopamp2) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}
#endif

/* --- Modular peripheral initializers for ADC2, TIM6, DAC1 --- */

static HAL_StatusTypeDef HalConfigure_Adc2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    extern DMA_HandleTypeDef hdma_adc2;
    extern ADC_HandleTypeDef hadc2;
    HAL_StatusTypeDef status;
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Enable clocks */
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure ADC2 GPIO */
    GPIO_InitStruct.Pin = ADC2_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC2_IN_PORT, &GPIO_InitStruct);

    /* Configure DMA for ADC2 */
    hdma_adc2.Instance = DMA1_Channel2;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_adc2);
    if (status != HAL_OK) return status;

    /* Link DMA to ADC2 handle */
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

    /* NVIC configuration for DMA */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* NVIC configuration for ADC2 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    /* ADC2 configuration */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode = DISABLE;
    status = HAL_ADC_Init(&hadc2);
    if (status != HAL_OK) return status;

    /* Configure ADC2 regular channel (match CubeMX) */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    status = HAL_ADC_ConfigChannel(&hadc2, &sConfig);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

static HAL_StatusTypeDef HalConfigure_Tim6_Init(void) {
    extern TIM_HandleTypeDef htim6;
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* Enable TIM6 clock */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Calculate TIM6 period based on ADC sampling frequency
     * TIM6 is driven by system clock with Prescaler=0
     * Period = SystemClockFreq / (ADC_SAMPLING_FREQ_KHZ * 1000) - 1
     * For SYSCLK=168MHz:
     * - 100 kHz: (168000000 / 100000) - 1 = 1680 - 1 = 1679
     * - 50 kHz:  (168000000 / 50000) - 1 = 3360 - 1 = 3359
     * - 200 kHz: (168000000 / 200000) - 1 = 840 - 1 = 839
     */
    uint32_t sysclk_freq = HAL_RCC_GetSysClockFreq();
    uint32_t tim6_period = (sysclk_freq / (ADC_SAMPLING_FREQ_KHZ * 1000)) - 1;

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = tim6_period;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) return HAL_ERROR;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) return HAL_ERROR;

    /* NVIC configuration for TIM6 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    return HAL_OK;
}

static HAL_StatusTypeDef HalConfigure_Dac1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    extern DMA_HandleTypeDef hdma_dac1_ch1;
    extern DAC_HandleTypeDef hdac1;
    DAC_ChannelConfTypeDef sConfig = {0};
    HAL_StatusTypeDef status;

    /* Enable DAC1, GPIO, DMA clocks */
    __HAL_RCC_DAC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure DAC1 GPIO */
    GPIO_InitStruct.Pin = DAC1_OUT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DAC1_OUT1_PORT, &GPIO_InitStruct);

    /* DMA for DAC1_CH1 (CubeMX) */
    hdma_dac1_ch1.Instance = DMA1_Channel1;
    hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CHANNEL1;
    hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_dac1_ch1);
    if (status != HAL_OK) return status;

    /* Link DMA to DAC1 handle */
    __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

    /* NVIC configuration for DMA1_Channel1 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* DAC1 configuration */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) return HAL_ERROR;

    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) return HAL_ERROR;

    /* NVIC configuration for DAC */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    return HAL_OK;
}

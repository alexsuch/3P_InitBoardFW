/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : solution_wrapper.c
 * @brief          : Hardware abstraction layer - GPIO, UART, SPI, ADC control
 *
 * This module provides hardware initialization and control functions for:
 *   - GPIO management (LEDs, test pins, detonation, pump, fuse)
 *   - UART communication (main and VUSA)
 *   - SPI accelerometer (LIS2DH12 / LSM6DS3) data acquisition
 *   - ADC2 sampling for logging (100 kHz DMA)
 *   - Timer callbacks for system tick and logger timestamp
 *
 * When SPI_LOGGER_ENABLE is defined, additional logger SPI slave support is enabled:
 *   - Logger frame assembly and SPI transmission to ESP32 master
 *   - Timestamp counter, ADC/IMU buffering, frame queuing
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "solution_wrapper.h"

#include <string.h>

#include "LIS2DH12.h"
#include "LSM6DS3.h"
#include "app.h"
#include "hal_cfg.h"
#include "init_brd.h"
#include "logger.h"
#if PIEZO_DETECTION_ENABLE
#include "piezo_proc.h"
#endif /* PIEZO_DETECTION_ENABLE */
#include "main.h"
#include "prj_config.h"
#include "solution_hal_cfg.h"
#include "timer.h"
#include "uart_configurator.h"
#if (CONTROL_MODE == PWM_CTRL_SUPP)
#include "stick_ctrl.h"
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
#include "mavlink_uart.h"
#endif

// ============================================================================
// ADC2 DMA buffer (declared at module level for early access)
// ============================================================================

/* ADC2 DMA circular buffer configuration
 * Size depends on build mode:
 *   - Logger mode (SPI_LOGGER_ENABLE=1): LOGGER_ADC_DMA_BUFFER_SIZE samples (typically 512)
 *   - Piezo mode (SPI_LOGGER_ENABLE=0): PIEZO_ADC_DMA_BLOCK samples (typically 16)
 * Used for configurable frequency synchronized sampling with DMA callbacks
 */
#if (SPI_LOGGER_ENABLE == 1u)
uint16_t adc2_dma_buffer[LOGGER_ADC_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
#else
uint16_t adc2_dma_buffer[PIEZO_ADC_DMA_BLOCK] __attribute__((aligned(4)));
#endif

/**
 * @brief Initialize HAL and core system peripherals
 *
 * Sets up basic hardware:
 *   - Resets accelerometer CS pin (inactive high)
 *   - Starts system tick timer (10ms periodic interrupts)
 *   - Enables OPAMP if configured
 *   - Starts MAIN UART receiving in IT mode
 *
 * Called once at application startup before app initialization
 */
void Solution_HalInit(void) {
    /* Reset all GPIOs to the correct state */
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);

    /* Run 10ms main timer tick */
    if (HAL_TIM_Base_Start_IT(&SYS_TICK_TIMER_HANDLE) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
    }

    /* Start OPAMP if available (configured by CubeMX) */
    if (HAL_OPAMP_Start(&OPAMP_HANDLE) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
    }

    /* Start MAIN UART receiving in interrupt mode */
    UartGetOneByteRx();

#if (TEST_DAC_ENABLE == 1u)
    /* Start DAC DMA (after TIM6 is configured but before it starts) */
    if (Solution_DacStart() != HAL_OK) {
        Error_Handler();
    }
#endif

#if ((SPI_LOGGER_ENABLE == 0u) && (PIEZO_DETECTION_ENABLE == 1u))
    /* Start ADC tick timer (100 kHz) */
    if (HAL_TIM_Base_Start_IT(&ADC_TICK_TIMER_HANDLE) != HAL_OK) {
        Error_Handler();
    }

    /* 1. Calibrate ADC2 for best SNR */
    if (HAL_ADCEx_Calibration_Start(&ADC_PIEZO_HANDLE, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    /* 2. Start ADC2 DMA circular buffer for synchronized configurable kHz sampling */
    if (HAL_ADC_Start_DMA(&ADC_PIEZO_HANDLE, (uint32_t*)adc2_dma_buffer, PIEZO_ADC_DMA_BLOCK) != HAL_OK) {
        Error_Handler();
    }
#endif /* ((SPI_LOGGER_ENABLE == 0u) && (PIEZO_DETECTION_ENABLE == 1u)) */
}

// ---------------------- SYSTEM TIMER CALLBACKS ------------------------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == SYS_TICK_TIMER_BASE) {
        /* Call global timer tick app handler */
        Timer_TickCbk();
    }
#if SPI_LOGGER_ENABLE
    else if (htim->Instance == TIM6) {
        /* Increment 32-bit software timestamp counter @ 100 kHz */
        Logger_TIM6_UpdateCallback();
    }
#endif /* SPI_LOGGER_ENABLE */
}

// ---------------------- FUSE READ -----------------------------------

bool ReadFuseGpio(void) { return (HAL_GPIO_ReadPin(FUSE_INPUT_PORT, FUSE_INPUT_PIN) == GPIO_PIN_SET) ? true : false; }

bool IsFuseRemoved(void) { return (ReadFuseGpio() == GPIO_PIN_RESET) ? true : false; }

// ---------------------- DETONATION CONTROL -------------------------------

void DetonHighSideSwithSet(bool state) {
    if (state) {
        HAL_TIM_PWM_Start(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, DETON_HIGH_SIDE_SWITH_CHANNEL);
    } else {
        HAL_TIM_PWM_Stop(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, DETON_HIGH_SIDE_SWITH_CHANNEL);
    }
}

void delay_us(uint16_t us)  // TODO OSAV rework due to proper mcu clock
{
    // Калібровано для 24MHz
    // Кожна ітерація займає приблизно 3-4 такти
    for (volatile uint16_t i = 0; i < (us * 4); i++) {
        __NOP();
    }
}

void DetonLowSideSwitchSet(bool state) {
    HAL_GPIO_WritePin(DETON_LOW_SIDE_SWITCH_OUT_2_PORT, DETON_LOW_SIDE_SWITCH_OUT_2_PIN, state);
    delay_us(2);
    HAL_GPIO_WritePin(DETON_LOW_SIDE_SWITCH_OUT_1_PORT, DETON_LOW_SIDE_SWITCH_OUT_1_PIN, state);
}

// ---------------------- CHARGING CONTROL ---------------------------------

void ChargingEnable(void) {
    /* Opens charge MOSFET and closes Discharge MOSFET */
    HAL_GPIO_WritePin(CHARGE_EN_PORT, CHARGE_EN_PIN, false);
}

void ChargingDisable(void) {
    /* Closes charge MOSFET and opens Discharge MOSFET */
    HAL_GPIO_WritePin(CHARGE_EN_PORT, CHARGE_EN_PIN, true);
}

// ---------------------- LED INDICATION -----------------------------------

void LedErrorSet(bool state) { HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, state); }

void LedStatusSet(bool state) { HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, state); }

// ------------------------ TEST GPIOs ---------------------------------
void Test1Set(bool state) { HAL_GPIO_WritePin(TEST_1_PORT, TEST_1_PIN, state); }

void Test2Set(bool state) { HAL_GPIO_WritePin(TEST_2_PORT, TEST_2_PIN, state); }

void Test1Toggle(void) { HAL_GPIO_TogglePin(TEST_1_PORT, TEST_1_PIN); }

void Test2Toggle(void) { HAL_GPIO_TogglePin(TEST_2_PORT, TEST_2_PIN); }

// ---------------------- PUMP CONTROL -------------------------------------

void PumpEnable(void) {
    HAL_TIM_PWM_Start(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
}

void PumpDisable(void) {
    HAL_TIM_PWM_Stop(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
}

// ---------------------- UART COMMUNICATION -------------------------------

// UART Communication Variables
volatile uint8_t uartRxTempByte;
uint8_t uartRxTempBuff[UART_RX_TEMP_BUFFER_SIZE];

bool UartSendData(uint8_t* wrBuff, uint8_t len) { return (HAL_UART_Transmit_IT(&MAIN_UART_HANDLE, wrBuff, len) == HAL_OK) ? true : false; }

bool UartGetOneByteRx(void) {
    /* Clear flags if received byte is already in the buffer */
    __HAL_UART_CLEAR_OREFLAG(&MAIN_UART_HANDLE);
    __HAL_UART_CLEAR_FLAG(&MAIN_UART_HANDLE, UART_FLAG_RXNE);

    return (HAL_UART_Receive_IT(&MAIN_UART_HANDLE, (uint8_t*)&uartRxTempByte, 1) == HAL_OK) ? true : false;
}

// ---------------------- VUSA BLOCK ---------------------------------

/**
 * VUSA (external power/fuse link) communication protocol
 * Single packet format: {0xA5, 0x5A, 0x02, 0x3E}
 */
volatile uint8_t vusaUartRxTempByte;  // Temporary byte for IT-based RX
app_cbk_fn vusa_cbk = NULL;           // Callback on valid packet reception

#define SYNC_BYTE 0xA5                                                     // Start-of-packet marker
const uint8_t vusa_packet[VUSA_PACKET_LENGTH] = {0xA5, 0x5A, 0x02, 0x3E};  // Expected packet
static uint8_t vusa_i, vusa_eq;                                            // Loop counter and equality flag
volatile uint8_t vusa_rx_buf[VUSA_PACKET_LENGTH];                          // RX buffer
volatile uint8_t vusa_rx_idx = 0;                                          // Current RX index

bool VusaUartGetOneByteRx(void) {
    /* Clear flags if received byte is already in the buffer */
    __HAL_UART_CLEAR_OREFLAG(&VUSA_UART_HANDLE);
    __HAL_UART_CLEAR_FLAG(&VUSA_UART_HANDLE, UART_FLAG_RXNE);

    return (HAL_UART_Receive_IT(&VUSA_UART_HANDLE, (uint8_t*)&vusaUartRxTempByte, 1) == HAL_OK) ? true : false;
}

void VusaStart(app_cbk_fn cbk) {
    /* Save callback from Vusa input */
    vusa_cbk = cbk;

    /* Start DMA Transmission */
    HAL_UART_Transmit_DMA(&VUSA_UART_HANDLE, (uint8_t*)vusa_packet, VUSA_PACKET_LENGTH);
    /* Start VUSA receiving */
    VusaUartGetOneByteRx();
}

void VusaUart_IRQ_Cbk(uint8_t vusa_byte) {
    // Handle RX interrupt
    if (vusa_rx_idx == 0) {
        if (vusa_byte == SYNC_BYTE) {
            vusa_rx_buf[0] = vusa_byte;
            vusa_rx_idx = 1;
        }
    } else {
        vusa_rx_buf[vusa_rx_idx++] = vusa_byte;

        if (vusa_rx_idx == VUSA_PACKET_LENGTH) {
            vusa_rx_idx = 0;
            vusa_eq = 1;

            for (vusa_i = 0; vusa_i < VUSA_PACKET_LENGTH; vusa_i++) {
                if (vusa_rx_buf[vusa_i] != vusa_packet[vusa_i]) {
                    vusa_eq = 0;
                    break;
                }
            }

            vusa_cbk(SYSTEM_EVT_READY, vusa_eq);  // packet received
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (&VUSA_UART_HANDLE == huart) {
        /* Handle Vusa received byte */
        VusaUart_IRQ_Cbk(vusaUartRxTempByte);
        VusaUartGetOneByteRx();
    } else if (&MAIN_UART_HANDLE == huart) {
#if UART_ENABLE
        /* Handle Configurator received byte */
        UartConfig_ByteReceived(uartRxTempByte);
#endif /* UART_ENABLE */
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
        /* Handle Mavlink comands */
        Mavlink_UartRxByte(uartRxTempByte);
#endif /* MAVLINK_V2_CTRL_SUPP */
        /* Get another byte */
        UartGetOneByteRx();
    }
}

// ---------------------- SPI COMMUNICATION --------------------------------

/**
 * SPI communication with accelerometer (LIS2DH12 or LSM6DS3)
 * TX buffer: DMA-aligned write buffer (address byte + padding)
 * RX buffer: Received register data (variable size per sensor)
 */
#if LIS2DH12_ACC_ENABLE
__attribute__((aligned(4))) uint8_t spi_wr_buff[SPI_WR_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#elif LSM6DS3_ACC_ENABLE
__attribute__((aligned(4))) uint8_t spi_wr_buff[SPI_WR_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#else
__attribute__((aligned(4))) uint8_t spi_wr_buff[SPI_WR_BUFFER_SIZE] = {0x00};
#endif
uint8_t readCommand = 0xF2;     // Default read command (unused in current implementation)
app_cbk_fn acc_cbk = NULL;      // Callback on accelerometer SPI transaction complete
app_cbk_fn ext_pwr_cbk = NULL;  // Callback on external power events

static void Acc_ReportStatus(system_evt_t evt) {
    if (acc_cbk != NULL) {
        acc_cbk(evt, 0u);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
    Acc_ReportStatus(SYSTEM_EVT_ERROR);
}

void HAL_SPI_DMAErrorCallback(SPI_HandleTypeDef* hspi) {
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
    Acc_ReportStatus(SYSTEM_EVT_ERROR);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
    Acc_ReportStatus(SYSTEM_EVT_READY);
}

/* SPI DMA callbacks not needed in blocking mode */

acc_read_status_t SpiGetAccData(uint8_t* rd_data_ptr, app_cbk_fn cbk) {
    acc_cbk = cbk;

#if LIS2DH12_ACC_ENABLE
    uint8_t reg_addr = LIS2DH12_ACC_DATA_ZERO_BYTE;
    uint8_t data_size = LIS2DH12_ACC_DATA_READ_SIZE - 1; /* -1 for address byte in TransmitReceive */
#elif LSM6DS3_ACC_ENABLE
    uint8_t reg_addr = LSM6DS3_GYRO_DATA_ZERO_BYTE;
    uint8_t data_size = LSM6DS3_ACC_DATA_READ_SIZE - 1; /* 12 data bytes (6 gyro + 6 accel) */
#else
#error "No accelerometer type defined!"
#endif

    /* Ensure previous transaction is complete */
    if (ACC_SPI_HANDLE.State != HAL_SPI_STATE_READY) {
        return ACC_READ_BUSY; /* SPI module is busy with previous transaction */
    }

    /* Prepare transmit buffer: address byte followed by dummy bytes */
    spi_wr_buff[0] = reg_addr;
    /* Dummy bytes already initialized to 0x00 in global buffer */

    /* Pull CS LOW to start transaction */
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);

    /* Start DMA-based TransmitReceive */
    /* Takes ~10us @ 5MHz for 13 bytes */
    if (HAL_SPI_TransmitReceive_DMA(&ACC_SPI_HANDLE, spi_wr_buff, rd_data_ptr, data_size + 1) != HAL_OK) {
        HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
        return ACC_READ_FAIL; /* DMA start failed */
    }

    return ACC_READ_OK; /* SPI read initiated successfully */
}

bool ReadAccIntGpio(void) { return (HAL_GPIO_ReadPin(ACC_INT_PORT, ACC_INT_PIN) == GPIO_PIN_SET) ? true : false; }

bool SpiWriteSingleRegister(uint8_t address, uint8_t value) {
    uint8_t wrBuff[2] = {address, value};

    // Setting R/W = 0, i.e.: Write Mode
    address &= ~(0x80);

    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&ACC_SPI_HANDLE, wrBuff, 2, SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);

    return true;
}

bool SpiReadRegister(uint8_t address, uint8_t* value, uint8_t num) {
    // Multiple Byte Read Settings
    if (num > 1)
        address |= 0x40;
    else
        address &= ~(0x40);

    // Setting R/W = 1, i.e.: Read Mode
    address |= (0x80);

    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&ACC_SPI_HANDLE, &address, 1, SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
        return false;
    }
    if (HAL_SPI_Receive(&ACC_SPI_HANDLE, value, num, SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);

    return true;
}

// ---------------------- ADC OPERATIONS -----------------------------------

/**
 * @brief ADC2 DMA half-complete callback (first LOGGER_ADC_DMA_HALF_SIZE samples ready)
 *
 * Called when ADC2 DMA has filled first half of circular buffer (LOGGER_ADC_DMA_HALF_SIZE samples).
 * Triggers ADC data processing via Logger_AdcBuffer_OnComplete (when SPI_LOGGER_ENABLE).
 *
 * This allows frame assembly to begin while second half is still being acquired,
 * enabling continuous 100 kHz ADC sampling without gaps.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc_ptr) {
    if (hadc_ptr == &ADC_PIEZO_HANDLE) {
#if SPI_LOGGER_ENABLE
        /* Phase 2: Copy first LOGGER_ADC_DMA_HALF_SIZE samples from DMA buffer */
        Logger_AdcBuffer_OnComplete(adc2_dma_buffer, LOGGER_ADC_DMA_HALF_SIZE);
#endif /* SPI_LOGGER_ENABLE */
        Test1Toggle();
        Test1Toggle();
#if PIEZO_DETECTION_ENABLE
        /* Piezo processing: first half of DMA buffer */
        Piezo_OnDmaHalf();
#endif /* PIEZO_DETECTION_ENABLE */
    }
}

/**
 * @brief ADC2 DMA complete callback (second LOGGER_ADC_DMA_HALF_SIZE samples ready)
 *
 * Called when ADC2 DMA circular buffer wraps and fills second half (samples LOGGER_ADC_DMA_HALF_SIZE..LOGGER_ADC_DMA_BUFFER_SIZE).
 * Triggers ADC data processing via Logger_AdcBuffer_OnComplete (when SPI_LOGGER_ENABLE).
 *
 * Completes the frame assembly cycle, allowing Logger_Task() to build and queue frame
 * while first half continues to acquire new samples.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc_ptr) {
    if (hadc_ptr == &ADC_PIEZO_HANDLE) {
#if SPI_LOGGER_ENABLE
        /* Phase 2: Copy second LOGGER_ADC_DMA_HALF_SIZE samples from DMA buffer */
        Logger_AdcBuffer_OnComplete(&adc2_dma_buffer[LOGGER_ADC_DMA_HALF_SIZE], LOGGER_ADC_DMA_HALF_SIZE);
#endif /* SPI_LOGGER_ENABLE */

#if PIEZO_DETECTION_ENABLE
        /* Piezo processing: second half of DMA buffer */
        Piezo_OnDmaFull();
#endif /* PIEZO_DETECTION_ENABLE */
    }
}

// ============================================================================
// Logger Subsystem: Frame Assembly, ADC Buffering, SPI Slave Communication
// ============================================================================

/**
 * Global RX buffer – receives 5-byte command from master (cmd + 4 padding bytes).
 * Master sends command 42 (LOGGER_SPI_CMD_CONFIG) to request configuration transmission.
 * Persistent buffer for SPI DMA reception (not stack-allocated due to asynchronous DMA access).
 */
static uint8_t logger_spi_rx_cmd_buffer[5] = {0};

/* Logger SPI Slave Support - Enabled when SPI_LOGGER_ENABLE is defined */

#if (SPI_LOGGER_ENABLE == 1u)

/**
 * @brief Start ADC data acquisition and initialize logging subsystem
 *
 * Called once during logger initialization (after configuration phase)
 * or from Logger_DrainQueue() after first SPI config transmission completes.
 *
 * Initializes:
 *   - Frame builder state machine (FRAME_STATE_IDLE)
 *   - ADC2 linear buffer (256-sample accumulator)
 *   - IMU ring buffer (0-50 sample FIFO)
 *   - ADC tick timer (100 kHz, enables ADC2 DMA sampling)
 *   - ADC2 calibration and DMA circular buffer startup
 *
 * Returns: void (calls Error_Handler on HAL errors)
 */
void Solution_LoggingStart(void) {
    /* Initialize logger frame builder (Phase 4) */
    Logger_FrameBuilder_Init();

    /* Start LSM6DS3 IMU data polling */
    Lsm6ds3_StartReadData();

    /* Start ADC tick timer (100 kHz) */
    if (HAL_TIM_Base_Start_IT(&ADC_TICK_TIMER_HANDLE) != HAL_OK) {
        Error_Handler();
    }

    /* 1. Calibrate ADC2 for best SNR */
    if (HAL_ADCEx_Calibration_Start(&ADC_PIEZO_HANDLE, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    /* 2. Start ADC2 DMA circular buffer for synchronized configurable kHz sampling */
    if (HAL_ADC_Start_DMA(&ADC_PIEZO_HANDLE, (uint32_t*)adc2_dma_buffer, LOGGER_ADC_DMA_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }
}

// ============================================================================
// Software 32-bit timestamp counter (TIM6 @ 100 kHz)
// ============================================================================
/*
 * TIM6 increments this counter in interrupt every 10 µs @ 100 kHz
 * Provides 32-bit microsecond timestamp (overcomes 16-bit TIM3 hardware limit)
 * Overflows after ~12 hours (acceptable for logging sessions)
 */
static volatile uint32_t g_timestamp = 0;

/* Called from TIM6_DAC_IRQHandler in stm32g4xx_it.c
 * Increments software timestamp counter (one increment = 10 µs)
 */
void Logger_TIM6_UpdateCallback(void) { g_timestamp++; }

/* Returns current 32-bit software timestamp value
 * Called by Logger_FrameBuilder to timestamp ADC samples
 */
uint32_t Logger_GetTimestamp(void) { return g_timestamp; }

/**
 * @brief Control SPI_DATA_RDY GPIO pin (ready signal to master)
 * @param ready: true to set GPIO high (data ready), false to set GPIO low
 */
void Logger_GPIO_SetReady(bool ready) {
    uint8_t pin_state = ready ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LOGGER_SPI_DATA_RDY_PORT, LOGGER_SPI_DATA_RDY_PIN, pin_state);
}

/**
 * @brief Transmit data via SPI2 slave using DMA transfer
 * @param buffer: Pointer to data buffer to transmit
 * @param size: Number of bytes to transmit (e.g., 32 for config, 640 for frame)
 *
 * Universal transmission function that handles both config and frame data.
 * Uses DMA transfer for efficient data transmission.
 * Automatically manages GPIO ready signal.
 * RX will be re-armed in HAL_SPI_TxCpltCallback after TX completes.
 */
void Logger_SPI_Transmit(const uint8_t* buffer, uint16_t size) {
    if (!buffer || size == 0) {
        return;
    }

    // Use DMA transfer for efficient transmission
    HAL_SPI_Transmit_DMA(&LOGGER_SPI_HANDLE, (uint8_t*)buffer, size);
}

/**
 * @brief Initialize Logger SPI slave with NVIC interrupt
 */
void Logger_SPI_Init(void) {
    // Enable SPI2 interrupt in NVIC
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);  // TODO OSAV priority
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

/**
 * @brief Arm SPI slave to listen for incoming command bytes from master
 *
 * Call this once during initialization to start the SPI slave.
 * Master will later assert CS and send 5-byte command to trigger transmission.
 */
void Logger_SPI_StartListening(void) {
    // Arm RX for LOGGER_SPI_RX_COMMAND_SIZE bytes from master (command + padding bytes)
    HAL_SPI_Receive_IT(&LOGGER_SPI_HANDLE, logger_spi_rx_cmd_buffer, LOGGER_SPI_RX_COMMAND_SIZE);
}

#endif /* SPI_LOGGER_ENABLE */

/**
 * @brief SPI2 RX Complete callback – master sent command byte
 *
 * Called when master sends 5-byte command to trigger frame transmission.
 * When SPI_LOGGER_ENABLE is enabled, forwards to Logger_SPI_RxCallback.
 * Otherwise, this is a no-op callback for HAL SPI RX interrupt.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
#if (SPI_LOGGER_ENABLE == 1u)
    if (hspi != &LOGGER_SPI_HANDLE) {
        return;
    }

    // Master sent command bytes – handle the request with buffer pointer
    Logger_SPI_RxCallback(logger_spi_rx_cmd_buffer);
#endif
}

/**
 * @brief SPI2 TX Complete callback – HAL wrapper
 *
 * Called by STM32 HAL when DMA transmission completes.
 * When SPI_LOGGER_ENABLE is enabled, forwards to Logger_SPI_TxCallback.
 * Otherwise, this is a no-op callback for HAL SPI TX interrupt.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
#if (SPI_LOGGER_ENABLE == 1u)
    if (hspi != &LOGGER_SPI_HANDLE) {
        return;
    }

    Logger_SPI_TxCallback();
#endif
}
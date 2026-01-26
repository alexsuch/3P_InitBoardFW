/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : solution.h
 * @brief          : Hardware abstraction layer functions
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __SOLUTION_H
#define __SOLUTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "hal_cfg.h"
#include "init_brd.h"
#include "prj_config.h"
#include "stm32g4xx_hal.h"

#define LIS2DH12_ACC_DATA_READ_SIZE (7u)
#define LIS2DH12_ACC_DATA_ZERO_BYTE (0xE8u)
#define LSM6DS3_ACC_DATA_READ_SIZE (13u)   /* 1 dummy + 6 gyro + 6 accel bytes */
#define LSM6DS3_ACC_DATA_ZERO_BYTE (0xA2u) /* OUTX_L_G (0x22) with READ bit (0x80) - start from gyro */

#define ADC_SELF_PWR_CHANNEL_IDX (ADC_CHANNEL_VREFINT)
#define ADC_VBAT_CHANNEL_IDX (ADC_CHANNEL_11)

#define UART_BUFFER_SIZE (64u)

/* Timeout Definitions -------------------------------------------------------*/
#define SPI_TIMEOUT_MS 10   /* SPI communication timeout */
#define UART_TIMEOUT_MS 100 /* UART communication timeout */

/* System 10ms Tick Timer Configuration (TIM2) ------------------------------------*/
#define SYS_TICK_TIMER_PRESCALER 1679 /* 1680 divider: APB1 clock / 1680 */
#define SYS_TICK_TIMER_PERIOD 999     /* 1000 counts: results in 10ms interrupt @ 168MHz */
/* Detonation High-Side Switch PWM (TIM15) -----------------------------------*/
#define DETON_PWM_TIMER_PRESCALER 0 /* No prescaler for high resolution */
#define DETON_PWM_TIMER_PERIOD 1216 /* 168MHz / 1217 = 138.05 kHz PWM frequency */
#define DETON_PWM_PULSE 608         /* 50% duty cycle: 1217/2 */
/* Pump PWM (TIM1) -----------------------------------------------------------*/
#define PUMP_PWM_TIMER_PRESCALER 0 /* No prescaler */
#define PUMP_PWM_TIMER_PERIOD 2099 /* 168MHz / (1 * 2100 * 2) = 40 kHz PWM frequency (center-aligned) */
#define PUMP_PWM_PULSE 1050        /* 50% duty cycle: 2100/2 */

/* UART Baud Rate Configuration ----------------------------------------------*/
#define MAIN_UART_BAUD_RATE 9600   /* Main UART baud rate */
#define VUSA_UART_BAUD_RATE 115200 /* VUSA UART baud rate */

/* SPI Configuration ---------------------------------------------------------*/
#define ACC_SPI_BAUD_RATE_PRESCALER SPI_BAUDRATEPRESCALER_16 /* APB2=84MHz / 16 = 5.25 MHz */

/* Buffer Size Definitions ---------------------------------------------------*/
#define UART_RX_TEMP_BUFFER_SIZE 10  /* UART RX temporary buffer size */
#define VUSA_PACKET_LENGTH 4         /* VUSA packet length */
#define LOGGER_SPI_RX_COMMAND_SIZE 5 /* Logger SPI RX command buffer size (1 cmd + 4 padding bytes) */

/* DAC Test Signal Parameters */
#define DAC_MAX_VALUE 4095 /* 12-bit DAC maximum value */
#define DAC_SAMPLES 151    /* Number of samples in DAC waveform buffer */

/* COMP1 DAC Threshold Configuration -----------------------------------------*/
#define COMP_DAC_VREF_MV 3300u
#define COMP_DAC_THRESHOLD_VALUE ((COMP_DAC_THRESHOLD_MV * 4095UL) / COMP_DAC_VREF_MV)

#define SET_ERROR_LED(status) LedErrorSet(status)
#define SET_STATUS_LED(status) LedStatusSet(status)

/* SPI Accelerometer Read Status Enum ----------------------------------------*/
typedef enum {
    ACC_READ_OK = 0, /**< SPI read successful */
    ACC_READ_BUSY,   /**< SPI module busy - transaction in progress */
    ACC_READ_FAIL    /**< SPI read failed - DMA or transmission error */
} acc_read_status_t;

/* Exported functions prototypes ---------------------------------------------*/
void Solution_HalInit(void);
void delay_us(uint16_t us);
bool SpiReadRegister(uint8_t address, uint8_t* value, uint8_t num);
bool SpiWriteSingleRegister(uint8_t address, uint8_t value);
bool ReadAccIntGpio(void);
acc_read_status_t SpiGetAccData(uint8_t* rd_data_ptr, app_cbk_fn cbk);
bool AdcGetVoltage(app_cbk_fn cbk);
void TestLedSet(bool state);
void Test2LedToggle(void);
void BuzzerEnable(void);
void BuzzerDisable(void);
void DetonHighSideSwithSet(bool state);
void DetonLowSideSwitchSet(bool state);
bool ReadFuseGpio(void);
bool IsFuseRemoved(void);
bool ReadVusaGpio(void);
bool ReadExtPwrGpio(void);
void Test1Set(bool state);
void Test2Set(bool state);
void Test1Toggle(void);
void Test2Toggle(void);
void VusaStart(app_cbk_fn cbk);
void FusePwrSet(bool state);
void LedErrorSet(bool state);
void LedStatusSet(bool state);
void DischargeEnable(void);
void DischargeDisable(void);
void ChargingEnable(void);
void ChargingDisable(void);
void PumpEnable(void);
void PumpDisable(void);
void ADC_PathEnable(void);
void ADC_PathDisable(void);
void Set_VbatAdcChannel(void);
void Set_TemptAdcChannel(void);
void Set_VcapAdcChannel(void);
void FcPwmSetCbk(app_cbk_fn pwm_cbk1);
uint32_t ReadStickCounter10Us(void);
void TestLedToggle();
void Test2LedSet(bool state);
void Test3LedSet(bool state);
void Test3LedToggle();
void PWM_IN_GPIO_Init(void);
bool UartSendData(uint8_t* wrBuff, uint8_t len);
bool UartGetOneByteRx(void);
void VusaUart_IRQ_Cbk(uint8_t vusa_byte);
bool VusaUartGetOneByteRx(void);
void GpioIntHandler(uint16_t GPIO_Pin, bool is_rise_edge);
bool ReadFcPwm1Gpio(void);
bool ReadFcPwm2Gpio(void);
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_DMAErrorCallback(SPI_HandleTypeDef* hspi);
bool Logger_SPI_Poll_CS(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef* hcomp);
void PiezoComp_Init(app_ext_cbk_fn system_cbk, uint16_t threshold_mV);

#ifdef __cplusplus
}
#endif

#endif /* __SOLUTION_H */

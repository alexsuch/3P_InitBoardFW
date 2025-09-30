/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : solution.c
  * @brief          : Hardware abstraction layer implementation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "solution_wrapper.h"
#include "main.h"
#include "hal_cfg.h"

#include "app.h"
#include "timer.h"
#include "init_brd.h"
#include "LIS2DH12.h"
#include "uart_configurator.h"
#if (CONTROL_MODE == PWM_CTRL_SUPP)
#include "stick_ctrl.h"
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
#include "mavlink_uart.h"
#endif

// -----------------------HAL Init -------------------------------------------------
void Solution_HalInit (void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Reset all GPIOs to the correct state */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin, GPIO_PIN_SET);

  /* Run 10ms main timer tick */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

#if (CONTROL_MODE == PWM_CTRL_SUPP)
  /* Enable Stick tracking timer */
  if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
#endif
}

// ---------------------- SYSTEM TIMER CALLBACKS ------------------------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == SYS_TICK_TIMER_BASE)
    {
        /* Call global timer tick app handler */
		Timer_TickCbk();
    }
}

// ---------------------- FUSE READ -----------------------------------

bool ReadFuseGpio (void)
{
	return (HAL_GPIO_ReadPin(FUSE_INPUT_PORT, FUSE_INPUT_PIN) == GPIO_PIN_SET) ? true : false;
}

bool IsFuseRemoved (void)
{
	return (ReadFuseGpio() == GPIO_PIN_RESET) ? true : false;
}

// ---------------------- DETONATION CONTROL -------------------------------

void DetonHighSideSwithSet (bool state)
{
	if (state)
	{
		HAL_TIM_PWM_Start(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, DETON_HIGH_SIDE_SWITH_CHANNEL);
	}
	else
	{
	    HAL_TIM_PWM_Stop(&DETON_HIGH_SIDE_SWITH_TIMER_HANDLE, DETON_HIGH_SIDE_SWITH_CHANNEL);
	}
}

void DetonLowSideSwitchSet (bool state)
{
	HAL_GPIO_WritePin(DETON_LOW_SIDE_SWITCH_OUT_1_PORT, DETON_LOW_SIDE_SWITCH_OUT_1_PIN, state);
	HAL_GPIO_WritePin(DETON_LOW_SIDE_SWITCH_OUT_2_PORT, DETON_LOW_SIDE_SWITCH_OUT_2_PIN, state);
}

// ---------------------- LED INDICATION -----------------------------------

void LedErrorSet (bool state)
{
	HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, state);
}

void LedStatusSet (bool state)
{
	HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, state);
}

// ---------------------- PWM CONTROL MODE ---------------------------------

#if (CONTROL_MODE == PWM_CTRL_SUPP)

// PWM Control Variables
app_cbk_fn fc_pwm_cbk = NULL;
bool pwm1_rise_edge = false;
bool pwm2_rise_edge = false;

uint32_t ReadStickCounter10Us (void)
{
	return __HAL_TIM_GetCounter(&PWM_STICK_COUNTER_HANDLE);
}

void PWM_IN_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

#if (TESTER_FW == 0u)
  /* Deinit UART */
  HAL_UART_DeInit(&MAIN_UART_HANDLE);
  /* Wait for some time */
  HAL_Delay(10);

  /*Configure GPIO pin : PWM2_IN_Pin */
  GPIO_InitStruct.Pin = PWM2_INPUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PWM2_INPUT_PORT, &GPIO_InitStruct);
#endif

  /*Configure GPIO pin : PWM1_IN_Pin */
  GPIO_InitStruct.Pin = PWM1_INPUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PWM1_INPUT_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* Wait for some time */
  HAL_Delay(10);
}

bool ReadFcPwm1Gpio (void)
{
	return (HAL_GPIO_ReadPin(PWM1_INPUT_PORT, PWM1_INPUT_PIN) == GPIO_PIN_SET) ? true : false;
}

bool ReadFcPwm2Gpio (void)
{
	return (HAL_GPIO_ReadPin(PWM2_INPUT_PORT, PWM2_INPUT_PIN) == GPIO_PIN_SET) ? true : false;
}

void FcPwmSetCbk(app_cbk_fn cbk)
{
	/* Save callback from FC PWM input */
	fc_pwm_cbk = cbk;
}

void GpioIntHandler (uint16_t GPIO_Pin, bool is_rise_edge)
{
	if (fc_pwm_cbk != NULL)
	{
		if (GPIO_Pin == PWM1_INPUT_PIN)
		{
			//TestLedToggle();
			/* Handle Interrupt from FC PWM1 */
			if ((is_rise_edge) && (ReadFcPwm1Gpio() == true) && (pwm1_rise_edge == false))
			{
				pwm1_rise_edge = true;
				fc_pwm_cbk(PWM_CH_1, true);
			}
			else if ((is_rise_edge == false) && (ReadFcPwm1Gpio() == false) && (pwm1_rise_edge == true))
			{
				pwm1_rise_edge = false;
				fc_pwm_cbk(PWM_CH_1, false);
			}

		}

		if (GPIO_Pin == PWM2_INPUT_PIN)
		{
			//TestLedToggle();
			/* Handle Interrupt from FC PWM2 */
			if ((is_rise_edge) && (ReadFcPwm2Gpio() == true) && (pwm2_rise_edge == false))
			{
				pwm2_rise_edge = true;
				fc_pwm_cbk(PWM_CH_2, true);
			}
			else if ((is_rise_edge == false) && (ReadFcPwm2Gpio() == false) && (pwm2_rise_edge == true))
			{
				pwm2_rise_edge = false;
				fc_pwm_cbk(PWM_CH_2, false);
			}
		}
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	GpioIntHandler(GPIO_Pin, true);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	GpioIntHandler(GPIO_Pin, false);
}
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)

#endif /* CONTROL_MODE == PWM_CTRL_SUPP */

// ---------------------- BUZZER CONTROL -----------------------------------

void BuzzerEnable(void)
{
#if (!BUZZER_DISABLE)
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, true);
#endif
}

void BuzzerDisable(void)
{
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, false);
}

// ---------------------- CHARGING CONTROL ---------------------------------

void ChargingEnable(void)
{
	/* Opens charge MOSFET and closes Discharge MOSFET */
	HAL_GPIO_WritePin(CHARGE_EN_PORT, CHARGE_EN_PIN, false);
}

void ChargingDisable(void)
{
	/* Closes charge MOSFET and opens Discharge MOSFET */
	HAL_GPIO_WritePin(CHARGE_EN_PORT, CHARGE_EN_PIN, true);
}

// ---------------------- PUMP CONTROL -------------------------------------

void PumpEnable(void)
{
	HAL_TIM_PWM_Start(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
	HAL_TIMEx_PWMN_Start(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
}

void PumpDisable(void)
{
	HAL_TIM_PWM_Stop(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
	HAL_TIMEx_PWMN_Stop(&PWM_PUMP_TIMER_HANDLE, PWM_PUMP_CHANNEL);
}

// ---------------------- UART COMMUNICATION -------------------------------

// UART Communication Variables
uint8_t uartRxTempByte;
uint8_t uartRxTempBuff[UART_RX_TEMP_BUFFER_SIZE];

bool UartSendData(uint8_t* wrBuff, uint8_t len)
{
	return (HAL_UART_Transmit_IT(&MAIN_UART_HANDLE, wrBuff, len) == HAL_OK) ? true : false;
}

bool UartGetOneByteRx(void)
{
	/* Clear flags if received byte is already in the buffer */
	__HAL_UART_CLEAR_OREFLAG(&MAIN_UART_HANDLE);
	__HAL_UART_CLEAR_FLAG(&MAIN_UART_HANDLE, UART_FLAG_RXNE);

	return (HAL_UART_Receive_IT(&MAIN_UART_HANDLE, &uartRxTempByte, 1) == HAL_OK) ? true : false;
}

// ---------------------- VUSA BLOCK ---------------------------------

// VUSA Communication Variables
uint8_t vusaUartRxTempByte;
app_cbk_fn vusa_cbk = NULL;

#define SYNC_BYTE 0xA5
const uint8_t vusa_packet[VUSA_PACKET_LENGTH] = {0xA5, 0x5A, 0x02, 0x3E};
static uint8_t vusa_i, vusa_eq;
volatile uint8_t vusa_rx_buf[VUSA_PACKET_LENGTH];
volatile uint8_t vusa_rx_idx = 0;


bool VusaUartGetOneByteRx(void)
{
	/* Clear flags if received byte is already in the buffer */
	__HAL_UART_CLEAR_OREFLAG(&VUSA_UART_HANDLE);
	__HAL_UART_CLEAR_FLAG(&VUSA_UART_HANDLE, UART_FLAG_RXNE);

	return (HAL_UART_Receive_IT(&VUSA_UART_HANDLE, &vusaUartRxTempByte, 1) == HAL_OK) ? true : false;
}

void VusaStart(app_cbk_fn cbk)
{
	/* Save callback from Vusa input */
	vusa_cbk = cbk;

    /* Start DMA Transmission */
    HAL_UART_Transmit_DMA(&VUSA_UART_HANDLE, (uint8_t*)vusa_packet, VUSA_PACKET_LENGTH);
    /* Start VUSA receiving */
    VusaUartGetOneByteRx();
}

void VusaUart_IRQ_Cbk (uint8_t vusa_byte)
{
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

			vusa_cbk(SYSTEM_EVT_READY, vusa_eq);   // packet received
		}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (&VUSA_UART_HANDLE == huart)
	{
		/* Handle Vusa received byte */
		VusaUart_IRQ_Cbk(vusaUartRxTempByte);
		VusaUartGetOneByteRx();
	}
	else if (&MAIN_UART_HANDLE == huart)
	{
		/* Handle Configurator received byte */
		UartConfig_ByteReceived(uartRxTempByte);
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
		/* Handle Mavlink comands */
		Mavlink_UartRxByte(uartRxTempByte);
#endif /* MAVLINK_V2_CTRL_SUPP */
		/* Get another byte */
		UartGetOneByteRx();
	}
}

// ---------------------- SPI COMMUNICATION --------------------------------

// SPI Communication Variables
uint8_t spi_wr_buff[SPI_WR_BUFFER_SIZE] = {LIS2DH12_ACC_DATA_ZERO_BYTE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t readCommand = 0xF2;
app_cbk_fn acc_cbk = NULL;
app_cbk_fn ext_pwr_cbk = NULL;

static void Acc_ReportStatus (system_evt_t evt)
{
	if (acc_cbk != NULL)
	{
		acc_cbk(evt, 0u);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
	Acc_ReportStatus(SYSTEM_EVT_READY);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
	Acc_ReportStatus(SYSTEM_EVT_ERROR);
}

void HAL_SPI_DMAErrorCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
	Acc_ReportStatus(SYSTEM_EVT_ERROR);
}

bool SpiGetAccData (uint8_t *rd_data_ptr, app_cbk_fn cbk)
{
	acc_cbk = cbk;
	spi_wr_buff[0] = LIS2DH12_ACC_DATA_ZERO_BYTE;

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive_DMA(&ACC_SPI_HANDLE, spi_wr_buff, rd_data_ptr, LIS2DH12_ACC_DATA_READ_SIZE) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
		return false;
	}

	return true;
}

bool ReadAccIntGpio (void)
{
	return (HAL_GPIO_ReadPin(ACC_INT_PORT, ACC_INT_PIN) == GPIO_PIN_SET) ? true : false;
}

bool SpiWriteSingleRegister(uint8_t address, uint8_t value)
{
	uint8_t wrBuff[2] = {address, value};

	// Setting R/W = 0, i.e.: Write Mode
    address &= ~(0x80);

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&ACC_SPI_HANDLE, wrBuff,2, SPI_TIMEOUT_MS) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
		return false;
	}

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);

	return true;
}

bool SpiReadRegister(uint8_t address, uint8_t* value, uint8_t num)
{
	// Multiple Byte Read Settings
	if (num > 1)
		address |= 0x40;
	else
		address &= ~(0x40);

	// Setting R/W = 1, i.e.: Read Mode
    address |= (0x80);

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&ACC_SPI_HANDLE, &address, 1, SPI_TIMEOUT_MS) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
		return false;
	}
	if (HAL_SPI_Receive(&ACC_SPI_HANDLE, value, num, SPI_TIMEOUT_MS) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
		return false;
	}

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);

	return true;
}

// ---------------------- ADC OPERATIONS -----------------------------------

// ADC Variables
app_cbk_fn adc_cbk = NULL;
uint8_t adc_idx = 0u;
uint16_t adc_buff[ADC_CHANNELS_COUNT];
uint16_t cpuVoltagemV, actualMillivolts;

static uint16_t adcResultToMillivolts(uint16_t adcValue)
{
    return (uint16_t)((adcValue * cpuVoltagemV) / ADC_MAX_VALUE);
}

bool AdcGetVoltage (app_cbk_fn cbk)
{
	bool ret = false;

	if (cbk != NULL)
	{
		/* Save a callback */
		adc_cbk = cbk;
		/* Run ADC measurement */
		if (HAL_ADC_Start_DMA(&MAIN_ADC_HANDLE, (uint32_t*)adc_buff, ADC_CHANNELS_COUNT) == HAL_OK)
		{
		    ret = true;
		}
	}

	return ret;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc_ptr) //TODO add calibration
{
	uint32_t report_val = 0u;

	if (hadc_ptr == &MAIN_ADC_HANDLE)
	{
		// calculate mcu voltage
		cpuVoltagemV = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_buff[0], ADC_RESOLUTION);
		// calculate battery voltage
		actualMillivolts = adcResultToMillivolts(adc_buff[1]);
		// store 2 + 2 bytes
		report_val = cpuVoltagemV | (actualMillivolts << 16);

		if (adc_cbk != NULL)
		{
			adc_cbk (SYSTEM_EVT_READY, (uint32_t)report_val);
		}
	}
}
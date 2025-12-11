
#include "prj_config.h"

#if (LSM6DS3_ACC_ENABLE == 1u)

#include <LSM6DS3.h>
#include <stdlib.h>
#include <stdio.h>
#include "init_brd.h"
#include "timer.h"
#include "solution_wrapper.h"
#include "acc_proc.h"

#if SPI_LOGGER_ENABLE
#include "logger.h"
#endif /* SPI_LOGGER_ENABLE */

static lsm6ds3_Status_t lsm6ds3_Stat;
static app_cbk_fn  lsm6ds3_sys_cbk = NULL;
static volatile uint16_t timeoutCnt = 0u;

/* ============================================================================
 * REGISTER DECODING HELPER FUNCTIONS (for SPI_LOGGER_ENABLE)
 * ============================================================================
 */

#if SPI_LOGGER_ENABLE

/**
 * @brief Extract Output Data Rate from LSM6DS3 control register bits [7:4]
 * 
 * Maps ODR code to frequency in Hz:
 *   0x0 = Off (0 Hz)
 *   0x3 = 52 Hz
 *   0x4 = 104 Hz
 *   0x8 = 1.66 kHz (1660 Hz)
 *   0x9 = 3.33 kHz (3330 Hz)
 *   0xA = 6.66 kHz (6660 Hz)
 *
 * @param reg_value: Control register value (e.g., CTRL1_XL or CTRL2_G)
 * @return uint16_t: Output data rate in Hz (0 if off)
 */
static uint16_t LSM6DS3_ExtractOdrHz(uint8_t reg_value)
{
    uint8_t odr_code = (reg_value >> 4) & 0x0F;
    
    switch (odr_code) {
        case 0x0: return 0;      // Off
        case 0x3: return 52;
        case 0x4: return 104;
        case 0x8: return 1660;   // 1.66 kHz
        case 0x9: return 3330;   // 3.33 kHz
        case 0xA: return 6660;   // 6.66 kHz
        default:  return 0;      // Off/invalid
    }
}

/**
 * @brief Extract accelerometer full-scale range from CTRL1_XL bits [3:2]
 *
 * Maps FS code to range in G:
 *   0x0 = ±2 G
 *   0x1 = ±16 G
 *   0x2 = ±4 G
 *   0x3 = ±8 G
 *
 * @param ctrl1_xl: CTRL1_XL register value
 * @return uint8_t: Full-scale range in G (2, 4, 8, or 16)
 */
static uint8_t LSM6DS3_ExtractAccelRangeG(uint8_t ctrl1_xl)
{
    uint8_t fs_code = (ctrl1_xl >> 2) & 0x03;
    
    switch (fs_code) {
        case 0x0: return 2;
        case 0x1: return 16;
        case 0x2: return 4;
        case 0x3: return 8;
        default:  return 2;     // Default to 2G if invalid
    }
}

/**
 * @brief Extract gyroscope full-scale range from CTRL2_G bits [3:2]
 *
 * Maps FS code to range in DPS:
 *   0x0 = 245 DPS
 *   0x1 = 500 DPS
 *   0x2 = 1000 DPS
 *   0x3 = 2000 DPS
 *
 * @param ctrl2_g: CTRL2_G register value
 * @return uint16_t: Full-scale range in DPS (245, 500, 1000, or 2000)
 * @note Returns uint16_t because 2000 DPS exceeds uint8_t max (255)
 */
static uint16_t LSM6DS3_ExtractGyroRangeDps(uint8_t ctrl2_g)
{
    uint8_t fs_code = (ctrl2_g >> 2) & 0x03;
    
    switch (fs_code) {
        case 0x0: return 245;
        case 0x1: return 500;
        case 0x2: return 1000;
        case 0x3: return 2000;
        default:  return 245;   // Default to 245 DPS if invalid
    }
}

#endif /* SPI_LOGGER_ENABLE */

static void Lsm6ds3_ErrHandler (void);

static void Lsm6ds3_ResetTimeout (void)
{
	timeoutCnt = 0;
}

void Lsm6ds3_Reset (app_cbk_fn sys_cbk, int16_t** x_axis, int16_t** y_axis, int16_t** z_axis)
{
	memset(&lsm6ds3_Stat, 0u, sizeof(lsm6ds3_Stat));

	lsm6ds3_sys_cbk = sys_cbk;

	/* Set pointers to the x/y/z accelerometer values */
	*x_axis = &lsm6ds3_Stat.x_axis;
	*y_axis = &lsm6ds3_Stat.y_axis;
	*z_axis = &lsm6ds3_Stat.z_axis;

	/* Set initial state */
	lsm6ds3_Stat.state = LSM6DS3_STATE_CHECK_ID;
}

void Lsm6ds3_GetGyroData(int16_t** gyro_x, int16_t** gyro_y, int16_t** gyro_z)
{
	/* Set pointers to the gyroscope values */
	*gyro_x = &lsm6ds3_Stat.gyro_x;
	*gyro_y = &lsm6ds3_Stat.gyro_y;
	*gyro_z = &lsm6ds3_Stat.gyro_z;
}

void Lsm6ds3_GotoHitMode (void)
{
	/* Reset the flag for new initialization */
	lsm6ds3_Stat.reset_done = false;
	/* Set hit detection parameters */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_HIT_PARAMS;
}

void Lsm6ds3_GotoMoveMode (void)
{
	/* Reset the flag for new initialization */
	lsm6ds3_Stat.reset_done = false;
	/* Set move detection parameters */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_MOVE_PARAMS;
}

#if ACC_SHAKE_DETECTION_ENABLE
void Lsm6ds3_GotoShakeMode (void)
{
	/* Set shake detection parameters - same as hit detection */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_SHAKE_PARAMS;
}
#endif /* ACC_SHAKE_DETECTION_ENABLE */

void Lsm6ds3_Deinit (void)
{
	memset(&lsm6ds3_Stat, 0u, sizeof(lsm6ds3_Status_t));
	Timer_Stop (ACC_TIMEOUT_TMR);
}

static void Lsm6ds3_PauseCbk (uint8_t timer_id)
{
    (void)timer_id;

    lsm6ds3_Stat.wait_flag = false;
}

static void Lsm6ds3_ErrHandler (void)
{
	/* Handle potential error */
	if (lsm6ds3_Stat.retry_cnt++ > LSM6DS3_MAX_RETRY_CNT)
	{
		/* Report Error */
		if (lsm6ds3_sys_cbk != NULL)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_ERROR, 0u);
		}
		lsm6ds3_Stat.wait_flag = true;
		lsm6ds3_Stat.retry_cnt = 0u;
		lsm6ds3_Stat.state = LSM6DS3_STATE_ERROR;
	}
	else
	{
		lsm6ds3_Stat.wait_flag = true;
		/* Start retry timer */
		Timer_Start (ACC_TIMEOUT_TMR, ACC_TIMEOUT_TMR_RETRY_PERIOD_MS, Lsm6ds3_PauseCbk);
	}
}

static void Lsm6ds3_CheckID (void)
{
	uint8_t rdVal = 0xFF;  /* Initialize with invalid value for debugging */
	bool stat = false;
	volatile bool spi_result = false;  /* DEBUG: check SPI function result */

	/* Read device id */
	spi_result = SpiReadRegister(LSM6DS3_WHO_AM_I, &rdVal, 1);
	
	if (spi_result != false)
	{
		/* DEBUG: Set breakpoint here and check rdVal */
		if (rdVal == LSM6DS3_ID)
		{
			/* Init is done */
			if (lsm6ds3_sys_cbk != NULL)
			{
				/* move to idle if response is correct */
				lsm6ds3_Stat.state = LSM6DS3_STATE_IDLE;
				lsm6ds3_Stat.retry_cnt = 0u;
				lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_INIT_OK);
				stat = true;
#if SPI_LOGGER_ENABLE
				/* Initialize hit mode for logging */
				Lsm6ds3_GotoHitMode();
#endif /* SPI_LOGGER_ENABLE */
			}
		}
	}
	else
	{
		/* DEBUG: SPI read failed completely */
		LedErrorSet(true);  /* Turn on error LED */
	}

	if (stat == false)
	{
		/* handle the error */
		Lsm6ds3_ErrHandler();
	}
}

static void Lsm6ds3_SetHitParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;
#if 1
	/* Software reset first (only once) */
	if (!lsm6ds3_Stat.reset_done)
	{
		if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_RESET_VAL) != false)
		{
			lsm6ds3_Stat.reset_done = true;
			/* Wait for reset to complete */
			Timer_Start(ACC_TIMEOUT_TMR, 60, Lsm6ds3_PauseCbk);
			lsm6ds3_Stat.wait_flag = true;
			return;
		}
		else
		{
			/* Reset failed */
			Lsm6ds3_ErrHandler();
			return;
		}
	}
#endif

	/* Set Gyroscope FIRST: Frequency configured in prj_config.h (1666/3332/6664 Hz), 2000 dps */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL2_G, LSM6DS3_CTRL2_G_HIT_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Accelerometer: Frequency configured in prj_config.h (1666/3332/6664 Hz), ±16g */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL1_XL, LSM6DS3_CTRL1_XL_HIT_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LSM6DS3_CTRL1_XL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL1_XL_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set common control register: BDU=1 (Block Data Update), IF_INC=1 (auto-increment) */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Enable accelerometer axes (CTRL9_XL = 0x18) */
	if (SpiWriteSingleRegister(0x18, 0x38) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x18, &rdVal, 1) != false)
		{
			if (rdVal == 0x38)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set CTRL7_G for gyroscope high-performance mode (0x16 = 0x00) */
	if (SpiWriteSingleRegister(0x16, 0x00) != false)
	{
		if (SpiReadRegister(0x16, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Ensure CTRL4_C DRDY_MASK=0 (0x13 = 0x00) */
	if (SpiWriteSingleRegister(0x13, 0x00) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x13, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Disable interrupts on INT1 (0x0D = 0x00) */
	if (SpiWriteSingleRegister(LSM6DS3_INT1_CTRL, LSM6DS3_INT1_CTRL_DRDY_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LSM6DS3_INT1_CTRL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_INT1_CTRL_DRDY_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}

	/* Wait 20ms for sensor to stabilize after configuration */
	lsm6ds3_Stat.wait_flag = true;
	Timer_Start(ACC_TIMEOUT_TMR, 20, Lsm6ds3_PauseCbk);
	
	/* DEBUG: Read back CTRL2_G to verify gyroscope is enabled */
	uint8_t ctrl2g_readback = 0;
	SpiReadRegister(LSM6DS3_CTRL2_G, &ctrl2g_readback, 1);
	
	/* Read STATUS_REG (0x1E) to check data ready flags */
	uint8_t statusReg = 0;
	SpiReadRegister(0x1E, &statusReg, 1);

	/* Init is done - send appropriate callback based on current state */
	if (lsm6ds3_sys_cbk != NULL)
	{
#if ACC_SHAKE_DETECTION_ENABLE
		if (lsm6ds3_Stat.state == LSM6DS3_STATE_SET_SHAKE_PARAMS)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_SHAKE_INIT_OK);
		}
		else
#endif /* ACC_SHAKE_DETECTION_ENABLE */
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_HIT_INIT_OK);
#if SPI_LOGGER_ENABLE
			/* Build IMU configuration structure with decoded register values */
			imu_config_t imu_cfg = {
				.accel_present = 0,    // Will be set based on ODR
				.gyro_present = 0,     // Will be set based on ODR
				.chip_id = 0x69,       // LSM6DS3 WHO_AM_I value
				.reserved_pad = 0,
				
				/* Register snapshots (will be used for reference by master) */
				.reserved0 = LSM6DS3_CTRL1_XL_HIT_VAL,  // CTRL1_XL snapshot
				.reserved1 = LSM6DS3_CTRL2_G_HIT_VAL,   // CTRL2_G snapshot
				.reserved2 = LSM6DS3_CTRL3_C_VAL,       // CTRL3_C snapshot
				.reserved3 = 0x00,                      // CTRL7_G snapshot
				.reserved4 = 0x00,                      // CTRL4_C snapshot
				
				.reserved5 = 0,
				.reserved6 = 0,
				.reserved7 = 0,
				.reserved8 = 0,
				.reserved9 = 0,
				.reserved10 = 0,
				.reserved11 = 0,
			};
			
			/* Decode ODR from register bits [7:4] */
			imu_cfg.accel_odr_hz = LSM6DS3_ExtractOdrHz(LSM6DS3_CTRL1_XL_HIT_VAL);
			imu_cfg.gyro_odr_hz = LSM6DS3_ExtractOdrHz(LSM6DS3_CTRL2_G_HIT_VAL);
			
			/* Set presence flags based on ODR > 0 */
			imu_cfg.accel_present = (imu_cfg.accel_odr_hz > 0) ? 1 : 0;
			imu_cfg.gyro_present = (imu_cfg.gyro_odr_hz > 0) ? 1 : 0;
			
			/* Decode full-scale ranges from register bits [3:2] */
			imu_cfg.accel_range_g = LSM6DS3_ExtractAccelRangeG(LSM6DS3_CTRL1_XL_HIT_VAL);
			imu_cfg.reserved_align = 0;  // Alignment byte
			imu_cfg.gyro_range_dps = LSM6DS3_ExtractGyroRangeDps(LSM6DS3_CTRL2_G_HIT_VAL);
			
			/* Notify logger with fully populated structure */
			Logger_OnAccelerometerReady(&imu_cfg);
#endif /* SPI_LOGGER_ENABLE */
		}
	}

	lsm6ds3_Stat.retry_cnt = 0u;
	lsm6ds3_Stat.state = LSM6DS3_STATE_GET_DATA;
}

static void Lsm6ds3_SetMoveParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;
#if 1
	/* Software reset first (only once) */
	if (!lsm6ds3_Stat.reset_done)
	{
		if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_RESET_VAL) != false)
		{
			lsm6ds3_Stat.reset_done = true;
			/* Wait 50ms for reset to complete */
			Timer_Start(ACC_TIMEOUT_TMR, 50, Lsm6ds3_PauseCbk);
			lsm6ds3_Stat.wait_flag = true;
			return;
		}
		else
		{
			/* Reset failed */
			Lsm6ds3_ErrHandler();
			return;
		}
	}
#endif

	/* Set common control register: BDU=1, IF_INC=1 */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL3_C, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL3_C_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Accelerometer: 104 Hz, ±2g */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL1_XL, LSM6DS3_CTRL1_XL_MOVE_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL1_XL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL1_XL_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Gyroscope: 104 Hz, 245 dps */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL2_G, LSM6DS3_CTRL2_G_MOVE_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Enable accelerometer axes (CTRL9_XL = 0x18) */
	if (SpiWriteSingleRegister(0x18, 0x38) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x18, &rdVal, 1) != false)
		{
			if (rdVal == 0x38)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set CTRL7_G for gyroscope high-performance mode (0x16 = 0x00) */
	if (SpiWriteSingleRegister(0x16, 0x00) != false)
	{
		if (SpiReadRegister(0x16, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Ensure CTRL4_C DRDY_MASK=0 (0x13 = 0x00) */
	if (SpiWriteSingleRegister(0x13, 0x00) != false)
	{
		if (SpiReadRegister(0x13, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Disable interrupts on INT1 (0x0D = 0x00) */
	if (SpiWriteSingleRegister(LSM6DS3_INT1_CTRL, LSM6DS3_INT1_CTRL_DRDY_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_INT1_CTRL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_INT1_CTRL_DRDY_VAL)
			{
			    stat = true;
			}
		}
	}

	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}

	/* Wait 20ms for sensor to stabilize after configuration */
	lsm6ds3_Stat.wait_flag = true;
	Timer_Start(ACC_TIMEOUT_TMR, 20, Lsm6ds3_PauseCbk);
	
	/* Read STATUS_REG (0x1E) to clear any pending interrupts */
	uint8_t statusReg = 0;
	SpiReadRegister(0x1E, &statusReg, 1);

	/* Init is done */
	lsm6ds3_Stat.retry_cnt = 0u;
	lsm6ds3_Stat.state = LSM6DS3_STATE_GET_DATA;
	if (lsm6ds3_sys_cbk != NULL)
	{
		lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_MOVE_INIT_OK);
	}
}

static void Lsm6ds3_GetDataCbk (system_evt_t evt, uint32_t usr_data)
{
	/* Stop timeout timer */
	//Timer_Stop (ACC_TIMEOUT_TMR);
	
	if (evt == SYSTEM_EVT_READY)
	{
		/* We read starting from 0x22 (OUTX_L_G) with auto-increment via IF_INC, 13 bytes total */
		/* Read address is 0xA2 = 0x22 | 0x80 (READ bit, auto-increment via CTRL3_C.IF_INC) */
		/* Data format: 1 dummy byte + 6 gyro bytes + 6 accel bytes */
		
		/* Convert gyroscope data: signed 16bit */
		/* Data format: OUTX_L_G, OUTX_H_G, OUTY_L_G, OUTY_H_G, OUTZ_L_G, OUTZ_H_G */
		/* Note: With HAL_SPI_TransmitReceive, first byte [0] is dummy, data starts at [1] */
		lsm6ds3_Stat.gyro_x = (int16_t)(lsm6ds3_Stat.rd_buff[1] | (lsm6ds3_Stat.rd_buff[2] << 8));
		lsm6ds3_Stat.gyro_y = (int16_t)(lsm6ds3_Stat.rd_buff[3] | (lsm6ds3_Stat.rd_buff[4] << 8));
		lsm6ds3_Stat.gyro_z = (int16_t)(lsm6ds3_Stat.rd_buff[5] | (lsm6ds3_Stat.rd_buff[6] << 8));
		
		/* Convert accelerometer data: signed 16bit */
		/* Data format: OUTX_L_XL, OUTX_H_XL, OUTY_L_XL, OUTY_H_XL, OUTZ_L_XL, OUTZ_H_XL */
		lsm6ds3_Stat.x_axis = (int16_t)(lsm6ds3_Stat.rd_buff[7] | (lsm6ds3_Stat.rd_buff[8] << 8));
		lsm6ds3_Stat.y_axis = (int16_t)(lsm6ds3_Stat.rd_buff[9] | (lsm6ds3_Stat.rd_buff[10] << 8));
		lsm6ds3_Stat.z_axis = (int16_t)(lsm6ds3_Stat.rd_buff[11] | (lsm6ds3_Stat.rd_buff[12] << 8));
		
		/* Phase 3: Capture synchronized timestamp and notify logger */
#if (SPI_LOGGER_ENABLE == 1u)
		Logger_ImuOnNewSample(&lsm6ds3_Stat.rd_buff[1]);  // Skip dummy byte, pass 12 raw bytes directly
#else
		
		/* Report data is ready */
		if (lsm6ds3_sys_cbk != NULL)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_DATA_READY);
		}
#endif /* SPI_LOGGER_ENABLE */
	}
	else
	{
		/* handle the error */
		Lsm6ds3_ErrHandler();
	}
}

static void Lsm6ds3_GetData (void)
{
	/* Read accelerometer and gyroscope data in one burst read
	 * LSM6DS3 supports auto-increment, so we can read all data at once
	 * Starting from OUTX_L_XL (0x28) to OUTZ_H_G (0x27) 
	 * Total: 6 bytes accel + 6 bytes gyro = 12 bytes
	 */
	acc_read_status_t read_status = SpiGetAccData(lsm6ds3_Stat.rd_buff, Lsm6ds3_GetDataCbk);
	if (read_status == ACC_READ_OK)
	{
		lsm6ds3_Stat.retry_cnt = 0u;
		/* Start timeout counter */
		Lsm6ds3_ResetTimeout();
	}
	else if (read_status == ACC_READ_FAIL)
	{
		/* handle the error (both ACC_READ_BUSY and ACC_READ_FAIL) */
		Lsm6ds3_ErrHandler();
	}
}

void Lsm6ds3_Task (void)
{
	if (lsm6ds3_Stat.wait_flag == false)
	{
#if 0
		if (timeoutCnt++ > LSM6DS3_MAX_TIMEOUT_CNT)
		{
			Lsm6ds3_ResetTimeout();
		}
#endif
		switch (lsm6ds3_Stat.state)
		{
		    case LSM6DS3_STATE_CHECK_ID:
		    	Lsm6ds3_CheckID();
		    	break;
#if ACC_SHAKE_DETECTION_ENABLE
		    case LSM6DS3_STATE_SET_SHAKE_PARAMS:
#endif /* ACC_SHAKE_DETECTION_ENABLE */
		    case LSM6DS3_STATE_SET_HIT_PARAMS:
		    	Lsm6ds3_SetHitParams();
		    	break;
		    case LSM6DS3_STATE_SET_MOVE_PARAMS:
		    	Lsm6ds3_SetMoveParams();
		    	break;
		    case LSM6DS3_STATE_GET_DATA:
		    	if (ReadAccIntGpio () != false)
		    	{

		    	    Lsm6ds3_GetData();
		    	}
		    	break;
		    default:
		    	break;
		}
	}
}

#endif /* LSM6DS3_ACC_ENABLE */


#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "init_brd.h"
#include "solution_wrapper.h"
#include "app_config.h"

static FLASH_EraseInitTypeDef EraseInitStruct = {0};
static config_data_t config_data_table;

static bool AppGonfig_IsArrMatch(uint8_t* ptr1, uint8_t* ptr2, uint16_t len)
{
    bool  ret = true;
    uint16_t idx;

    for (idx = 0; idx < len; idx++)
    {
        if (ptr1[idx] != ptr2[idx])
        {
            ret = false;
            break;
        }
    }

    return ret;
}

bool AppGonfig_IsAllElements(uint8_t* ptr, uint16_t len, uint8_t val)
{
    bool  ret = true;
    uint16_t idx;

    for (idx = 0; idx < len; idx++)
    {
        if (ptr[idx] != val)
        {
            ret = false;
            break;
        }
    }

    return ret;
}

static uint8_t AppConfig_CalcCRC (uint8_t* data, uint8_t len)
{
    uint16_t idx, checksum = 1;

    for(idx = 0; idx < len; idx++)
    {
        checksum ^= data[idx];
    }

    return checksum;
}

void AppConfig_SetDefaultConfig (config_data_t* cfg_data)
{
	/* Set default values */
	cfg_data->safeTimeoutSec               		    = SAFE_TIMEOUT_SEC_DEFAULT;
	cfg_data->ignitionDelayMiliSec           	    = IGNITION_DELAY_MILISEC_DEFAULT;
	cfg_data->miningMode                            = MINING_MODE_ENABLE_DEFAULT;
	cfg_data->accEnable                     	    = ACC_ENABLE_DEFAULT;
	cfg_data->selfDestroyTimeoutMin                 = SELF_DESTROY_TIMEOUT_MIN_DEFAULT;
	cfg_data->miningAutoActivationMin               = MINING_SELF_DESTROY_EN_TIMEOUT_MIN_DEFAULT;
	cfg_data->miningEnableDelaySec                  = MINING_ENABLE_DELAY_SEC;
	cfg_data->dev_move_threshold                    = MOVEMENT_THRESHOLD_DEFAULT;
	cfg_data->device_type                           = DEFAULT_DEVICE_TYPE;
	cfg_data->customer_info                         = DEFAULT_CUSTOMER_INFO;
}

static bool AppGonfig_ReadFlashConfig(uint32_t addr, config_data_t* cfg_data)
{
	bool status = false;
	config_save_data_t* CfgDataPtr = (config_save_data_t*)addr;
	uint8_t calc_crc;

	if (IS_FLASH_MAIN_MEM_ADDRESS(addr) != 0u)
	{
        /* Check if config is empty */
        if (
               (AppGonfig_IsAllElements((uint8_t*)&addr, sizeof(config_save_data_t), 0x00)) ||
               (AppGonfig_IsAllElements((uint8_t*)&addr, sizeof(config_save_data_t), 0xFF))
            )
        {
            return status;
        }

		/* Calculate CRC */
		calc_crc = AppConfig_CalcCRC((uint8_t*)(&CfgDataPtr->config_data), (sizeof(config_data_t)));

		/* Check if CRC are matching */
		if (calc_crc == CfgDataPtr->crc)
		{
			status = true;
		}
	}

	if (status != false)
	{
		/* Configuration is correct - copy data to the config structure */
		memcpy((uint8_t*)cfg_data, (uint8_t*)addr, sizeof(config_data_t));
	}

	return status;
}

bool AppConfig_ApplyDefaultConfig (void)
{
	config_data_t TmpCfgData;

	/* Erase temporary configuration structure  */
	memset(&TmpCfgData, 0u, sizeof(config_data_t));

	/* Get default configuration */
	AppConfig_SetDefaultConfig(&TmpCfgData);

	/* Save default configuration */
	return AppConfig_SaveConfiguration (&TmpCfgData);
}

const config_data_t* AppConfig_GetConfiguration(void)
{
	/* Try to read config from address 0 */
	if (AppGonfig_ReadFlashConfig(CONFIG_ADDR_0, &config_data_table) != false)
	{
		//do nothing
	}
	/* Try to read config from address 1 */
	else if (AppGonfig_ReadFlashConfig(CONFIG_ADDR_1, &config_data_table) != false)
	{
		//do nothing
	}
	else
	{
		/* If both configurations read failed then use default configuration */
		AppConfig_SetDefaultConfig(&config_data_table);
	}

	return &config_data_table;
}


bool AppConfig_SaveConfiguration (config_data_t* config)
{
	bool status = false;
	uint8_t size;
	uint32_t addr, PageError = 0;
	uint64_t value;
	config_save_data_t TmpCfgData;
	uint8_t* data_ptr = (uint8_t*)&TmpCfgData;

	if (config == NULL) return false;

	/* Erase temporary configuration structure  */
	memset(&TmpCfgData, 0u, sizeof(config_save_data_t));

	/* Copy configuration data to temp buffer */
	memcpy(&TmpCfgData.config_data, config, sizeof(config_data_t));
	/* Calculate CRC */
	TmpCfgData.crc = AppConfig_CalcCRC((uint8_t*)(config), (sizeof(config_data_t)));

	/* Unlock flash */
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page        = USR_PAGE;
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		/*
		  Error occurred while page erase.
		  User can add here some code to deal with this error.
		  PageError will contain the faulty page and then to know the code error on this page,
		  user can call function 'HAL_FLASH_GetError()'
		*/
	      HAL_FLASH_Lock();
	      return false;
	}

	//https://community.st.com/t5/stm32-mcus-products/writing-to-internal-flash-stm32l422kbt6/td-p/127419 //INFO

	// elf has a load segment with rwx permissions WARNING
	// https://community.st.com/t5/stm32-mcus-products/quot-warning-lt-elffile-gt-has-a-load-segment-with-rwx/td-p/83148  (Adding "-Wl,--no-warn-rwx-segment" disables that warning)

	/* Write config to flash address 0 */
	size = (sizeof(config_save_data_t) + 7) & ~7; // Round to 64-bit words
	size /= sizeof(uint64_t); // Byte count to 64-bit word count
	data_ptr = (uint8_t*)&TmpCfgData;
	addr = CONFIG_ADDR_0;

	/* Write data to flash */
	while (size--)
	{
		memcpy(&value, data_ptr, sizeof(uint64_t)); // alignment safe copy of byte array into 64-bit value

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, value) != HAL_OK)
		{
			  HAL_FLASH_Lock();
			  return false;
		}

	    addr += sizeof(uint64_t);     // Advance address
	    data_ptr += sizeof(uint64_t); // Advance byte buffer
	}

	/* Write config to flash address 1 */
	size = (sizeof(config_save_data_t) + 7) & ~7; // Round to 64-bit words
	size /= sizeof(uint64_t); // Byte count to 64-bit word count
	data_ptr = (uint8_t*)&TmpCfgData;
	addr = CONFIG_ADDR_1;

	/* Write data to flash */
	while (size--)
	{
		memcpy(&value, data_ptr, sizeof(uint64_t)); // alignment safe copy of byte array into 64-bit value

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, value) != HAL_OK)
		{
			  HAL_FLASH_Lock();
			  return false;
		}

	    addr += sizeof(uint64_t);     // Advance address
	    data_ptr += sizeof(uint64_t); // Advance byte buffer
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) */
	HAL_FLASH_Lock();

	/* Compare written configuration with desired one for address 0 */
	if (AppGonfig_IsArrMatch((uint8_t*)CONFIG_ADDR_0, (uint8_t*)&TmpCfgData, sizeof(config_save_data_t)) != false)
	{
		status = true;
	}

	/* Compare written configuration with desired one for address 1 */
	if (AppGonfig_IsArrMatch((uint8_t*)CONFIG_ADDR_1, (uint8_t*)&TmpCfgData, sizeof(config_save_data_t)) != false)
	{
		status = true;
	}

	/* Copy config data locally */
	if (status != false)
	{
		memcpy(&config_data_table, config, sizeof(config_data_t));
	}

	return status;
}

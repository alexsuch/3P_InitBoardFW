
/* Define to prevent#define RELEASE_CONFIGURATION_ENABLE                     (1u)recursive inclusion -------------------------------------*/
#ifndef __PRJ_CONFIG_H
#define __PRJ_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*-------------------------- APP VERSION & DEVICE TYPE-----------------------------------------*/
#define APP_MINOR_VERSION                             (0u)
#define APP_MAJOR_VERSION                             (2u)
#define APP_VERSION                                   ((APP_MAJOR_VERSION << 4) | (APP_MINOR_VERSION))
#define DEFAULT_DEVICE_TYPE                           (DEVICE_TYPE_FOR_REGULAR_FPV)
#define DEFAULT_CUSTOMER_INFO                         (0u)

#define NO_CTRL                                       (0u)
#define PWM_CTRL_SUPP                                 (1u)
#define MAVLINK_V2_CTRL_SUPP                          (2u)

#define LIS2DH12_ACC_ENABLE                           (0u)
#define LSM6DS3_ACC_ENABLE                            (1u)

/* LSM6DS3 Sampling Frequency Configuration (Hz)
 * Options: 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
 * Note: These frequencies are used for both accelerometer and gyroscope
 * ODR bits [7:4] in CTRL1_XL and CTRL2_G registers
 */
#define LSM6DS3_SAMPLING_FREQ_HZ                      (3332)  // Valid: 26, 52, 104, 208, 416, 833, 1666, 3332, 6664

/* LSM6DS3 ODR Register Values (from datasheet CTRL1_XL[7:4] and CTRL2_G[7:4])
 * 0000 = Off
 * 0001 = 12.5 Hz
 * 0010 = 26 Hz
 * 0011 = 52 Hz
 * 0100 = 104 Hz
 * 0101 = 208 Hz
 * 0110 = 416 Hz
 * 0111 = 833 Hz
 * 1000 = 1.66 kHz (1666 Hz)
 * 1001 = 3.33 kHz (3332 Hz)
 * 1010 = 6.66 kHz (6664 Hz)
 */
#if (LSM6DS3_SAMPLING_FREQ_HZ == 26)
    #define LSM6DS3_ODR_VALUE                         (0x20)  // 26 Hz (0010 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 52)
    #define LSM6DS3_ODR_VALUE                         (0x30)  // 52 Hz (0011 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 104)
    #define LSM6DS3_ODR_VALUE                         (0x40)  // 104 Hz (0100 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 208)
    #define LSM6DS3_ODR_VALUE                         (0x50)  // 208 Hz (0101 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 416)
    #define LSM6DS3_ODR_VALUE                         (0x60)  // 416 Hz (0110 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 833)
    #define LSM6DS3_ODR_VALUE                         (0x70)  // 833 Hz (0111 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 1666)
    #define LSM6DS3_ODR_VALUE                         (0x80)  // 1.66 kHz (1000 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 3332)
    #define LSM6DS3_ODR_VALUE                         (0x90)  // 3.33 kHz (1001 binary)
#elif (LSM6DS3_SAMPLING_FREQ_HZ == 6664)
    #define LSM6DS3_ODR_VALUE                         (0xA0)  // 6.66 kHz (1010 binary)
#else
    #error "Invalid LSM6DS3_SAMPLING_FREQ_HZ. Choose from: 26, 52, 104, 208, 416, 833, 1666, 3332, 6664"
#endif

/*-------------------------- ADC Configuration -----------------------------------------*/

#define ADC_SAMPLING_FREQ_KHZ                         (100u)   // ADC sampling frequency in kHz
#define ADC_DMA_BUFFER_SIZE                           (512u)   // Total DMA buffer size in samples
#define ADC_DMA_HALF_SIZE                             (ADC_DMA_BUFFER_SIZE / 2)  // Half buffer for callbacks (256 samples @ 512 total)
#define LOGGER_ADC_BLOCK_SIZE                         (ADC_DMA_HALF_SIZE)  // Samples per ADC block


#define RELEASE_CONFIGURATION_ENABLE                  (0u)


#if (RELEASE_CONFIGURATION_ENABLE == 0u) //TEST CONFIGURATION

/*-------------------------- LOGGER MACROS -----------------------------------------*/
#define TEST_DAC_ENABLE                               (1u) // RELEASE VALUE - 0u

/*-------------------------- BUILD MACROS -----------------------------------------*/
#define CONTROL_MODE                                  (MAVLINK_V2_CTRL_SUPP)
#define VBAT_MEASURE_FEATURE                          (0u)
#define ACC_SUPPORTED_ENABLE                          (1u)
#define START_UP_DELAY_ENABLE                         (1u)
#define UART_ENABLE                                   (0u)
#define SPI_LOGGER_ENABLE                             (1u)
/*--------------------------- VUSA MACROS -----------------------------------------*/
#define VUSA_ENABLE                                   (1u) // RELEASE VALUE - 1u
#define VUSA_ENABLE_DEFAULT                           (1u) // RELEASE VALUE - 1u      

/*--------------------------- OSD MACROS -----------------------------------------*/
#define OSD_ENABLE                                    (1u) // RELEASE VALUE - 1u
#define OSD_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/*----------------------------PWM 2 MACROS -----------------------------------------*/
#define PWM2_INPUT_ENABLE_DEFAULT                     (0u) // RELEASE VALUE - 1u

/*-------------------------- Main Configuration parameters -----------------------*/
/* Safe Timer Timeout Value is seconds */
#define SAFE_TIMEOUT_SEC_DEFAULT                      (5u) // RELEASE VALUE - 120u

/* Ignition delay in milliseconds */
#define IGNITION_DELAY_MILISEC_DEFAULT                (0u) // RELEASE VALUE - 0u

/* Enable/disable accelerometer functionality */
#define ACC_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/* Self destruction related timings */
#define SELF_DESTROY_TIMEOUT_MIN_DEFAULT              (1u)     //RELEASE VALUE - 30u

/* Magic number chosen by experimental way */
#define MOVEMENT_THRESHOLD_DEFAULT                    (72089LL) //RELEASE VALUE - 72089LL

/*------------------------ Mining Mode Configuration -----------------------*/

#define MINING_MODE_SUPP                              (1u)

/* Enable/disable mining mode  */
#define MINING_MODE_ENABLE_DEFAULT                    MINING_MODE_AUTO //MINING_MODE_MANUAL//MINING_MODE_AUTO //RELEASE VALUE - MINING_MODE_NONE
#define MINING_SELF_DESTROY_EN_TIMEOUT_MIN_DEFAULT    (1u)  //RELEASE VALUE - 30u
#define MINING_ENABLE_DELAY_SEC                       (5u)  //RELEASE VALUE - 30u

/*------------------------ Accelerometer Parameters -----------------------------------*/
#define ACC_HIT_THRESHOLD_MG                          (12000u)  // units in mili G //RELEASE VALUE - 12G - 12000u

#define ACC_BUFF_SIZE                                 (6u) //RELEASE VALUE - WINDOW SIZE = 6u

#define ACC_NO_DIVIDE_ENABLE                          (0u)

#define NET_DETECTION_ENABLE                          (0u)

#define ACC_NET_BUFF_SIZE                             (100u) //RELEASE VALUE - WINDOW SIZE = 100u

#define ACC_HIT_NET_DIFF_THRESHOLD_MG                 (4500u)  // units in mili G //RELEASE VALUE - 4G - 4500u

#define ACC_NET_DECIMATION_RATE                       (5u)

/*------------------------ Battery Configuration parameters -----------------------*/

// Battery level calculation thresholds for AAA battery
#define BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS      (1450u)  // 100% - 1.45V
#define BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS        (1000u)   // 0% - 0.9V

#define BATTERY_VOLTAGE_VERY_LOW_SD_LOW_THRESHOLD_MILIVOLTS  (1100u) //RELEASE VALUE - 1100 mV
#define BATTERY_VOLTAGE_LOW_NO_SOUND_THRESHOLD_MILIVOLTS     (1200u) //RELEASE VALUE - 1200 mV
#define BATTERY_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS       (1300u) //RELEASE VALUE - 1300 mV 
#define SELF_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS          (2500u) //RELEASE VALUE - 2400 mV

#define BATTERY_THRESHOLD_ADJUSTMENT_MV                      (100u) //RELEASE VALUE - 100 mV

/*------------------------ Application Configuration parameters -----------------------*/

/* Capacitor charging timeout in milliseconds */
#define CHARGING_TMR_TIMEOUT_MS  		     	      (7000u) //RELEASE VALUE - 10000u

/* Ignition time in milliseconds */
#define IGNITION_OFF_TMR_PERIOD_MS  		 		  (4000u) //RELEASE VALUE - 4000u

/* Disarm -> Ignition timeout */
#define DISARM_IGNITION_BLOCK_TIMEOUT_MS              (3000) // RELEASE VALUE - 5000u

/* Internal temperature sensor offset */
#define TEMP_OFFSET                                   (8u)

/* Disable Buzzer */
#define BUZZER_DISABLE                                (1u)


/*----------------------------- Mavlink parameters -----------------------*/
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
// Timing Configuration
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS       (750u)
#define MAVLINK_CONNECTION_TIMEOUT_MS                 (3000u)
#endif /* MAVLINK_V2_CTRL_SUPP */

/*----------------------------- Test parameters -----------------------*/
#define SELF_DESTROY_DISABLE                          (0u) //RELEASE VALUE - 0u

#define TEST_SELF_DESTROY_ONLY_MODE                   (0u) //RELEASE VALUE - 0u
#define TEST_SELF_DESTROY_MINING_MODE                 (0u) //RELEASE VALUE - 0u

#define SELF_DESTROY_INDICATE_LAST_SECONDS            (15u) //RELEASE VALUE - 15

#define ACC_HIT_DETECTED_STICKY_LED_FEATURE           (0u) //RELEASE VALUE - 0u
#define MOVE_DETECTED_STICKY_LED_FEATURE              (0u) //RELEASE VALUE - 0u


/*----------------------------- Mavlink parameters -----------------------*/
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
// Timing Configuration
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS       (750u)
#define MAVLINK_CONNECTION_TIMEOUT_MS                 (3000u)

#define FLIGHT_STABLE_PARAMETERS_TIMEOUT_SEC          (4u)  // 7 seconds speed and altitude must be higher than minimum thresholds
#define FLIGHT_SPEED_MINIMUM_THRESHOLD_M_S            (17u) // 17 m/s
#define FLIGHT_ALTITUDE_MINIMUM_THRESHOLD_M           (100u)  // 100 meters
#endif /* MAVLINK_V2_CTRL_SUPP */
/*----------------------------- Test parameters -----------------------*/


#define TEST_SELF_DESTROY_ONLY_MODE                   (0u) //RELEASE VALUE - 0u
#define TEST_SELF_DESTROY_MINING_MODE                 (0u) //RELEASE VALUE - 0u

#define ACC_HIT_DETECTED_STICKY_LED_FEATURE           (0u) //RELEASE VALUE - 0u
#define MOVE_DETECTED_STICKY_LED_FEATURE              (0u) //RELEASE VALUE - 0u

#else //RELEASE CONFIGURATION !!!!!!!!!
/*-------------------------- BUILD MACROS -----------------------------------------*/
#define CONTROL_MODE                                  (PWM_CTRL_SUPP)
#define VBAT_MEASURE_FEATURE                          (1u)
#define START_UP_DELAY_ENABLE                         (1u)
#define UART_ENABLE                                   (1u)

/*-------------------------- ACCELEROMETER MACROS --------------------------------*/
#define LIS2DH12_ACC_ENABLE                           (1u)
#define LSM6DS3_ACC_ENABLE                            (0u)
#define ACC_NO_DIVIDE_ENABLE                          (1u)

#define ACC_HIT_THRESHOLD_MG                          (12000u)  // units in mili G //RELEASE VALUE - 12G - 12000u

#if ACC_SHAKE_DETECTION_ENABLE
#define ACC_SHAKE_THRESHOLD_MG                        (5000u)   // units in mili G - Shake detection threshold
#endif /* ACC_SHAKE_DETECTION_ENABLE */

#define ACC_BUFF_SIZE                                 (6u) //RELEASE VALUE - WINDOW SIZE = 6u

#define NET_DETECTION_ENABLE                          (0u)

#define ACC_SHAKE_DETECTION_ENABLE                    (0u)   // Enable shake detection - RELEASE: disabled

#define ACC_NET_BUFF_SIZE                             (100u)
#define ACC_HIT_NET_DIFF_THRESHOLD_MG                 (5000u)
#define ACC_NET_DECIMATION_RATE                       (5u)

/*-------------------------- Main Configuration parameters -----------------------*/
/* Safe Timer Timeout Value is seconds */
#define SAFE_TIMEOUT_SEC_DEFAULT                      (120u) // RELEASE VALUE - 120u

/* Ignition delay in milliseconds */
#define IGNITION_DELAY_MILISEC_DEFAULT                (0u) // RELEASE VALUE - 0u

/* Enable/disable accelerometer functionality */
#define ACC_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/*------------------------ Mining Mode Configuration -----------------------*/
#define MINING_MODE_SUPP                              (1u)  // Production Enable - 1u

/* Enable/disable mining mode  */
#define MINING_MODE_ENABLE_DEFAULT                    MINING_MODE_NONE //MINING_MODE_MANUAL//MINING_MODE_AUTO //RELEASE VALUE - MINING_MODE_NONE
#define MINING_SELF_DESTROY_EN_TIMEOUT_MIN_DEFAULT    (30u)  //RELEASE VALUE - 30u
#define MINING_ENABLE_DELAY_SEC                       (30u)  //RELEASE VALUE - 30u

/* Self destruction related timings */
#define SELF_DESTROY_TIMEOUT_MIN_DEFAULT              (30u)     //RELEASE VALUE - 30u

/* Magic number chosen by experimental way */
#define MOVEMENT_THRESHOLD_DEFAULT                    (72089LL) //RELEASE VALUE - 72089LL

/*------------------------ Application Configuration parameters -----------------------*/

#define SELF_DESTROY_INDICATE_LAST_SECONDS            (15u) //RELEASE VALUE - 15

/* Capacitor charging timeout in milliseconds */
#define CHARGING_TMR_TIMEOUT_MS  		     	      (9000u) //RELEASE VALUE - 9000u

/* Ignition time in milliseconds */
#define IGNITION_OFF_TMR_PERIOD_MS  		 		  (1000u) //RELEASE VALUE - 1000u

/* Disarm -> Ignition timeout */
#define DISARM_IGNITION_BLOCK_TIMEOUT_MS              (5000u) // RELEASE VALUE - 5000u

#define BUZZER_DISABLE                                (0u)

/*------------------------ Battery Configuration parameters -----------------------*/

// Battery level calculation thresholds for AAA battery
#define BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS      (1450u)  // 100% - 1.45V
#define BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS        (900u)   // 0% - 0.9V

#define BATTERY_VOLTAGE_VERY_LOW_SD_LOW_THRESHOLD_MILIVOLTS  (1000u) //RELEASE VALUE - 1000 mV
#define BATTERY_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS       (1200u) //RELEASE VALUE - 1200 mV
#define SELF_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS          (2500u) //RELEASE VALUE - 2400 mV

#define BATTERY_THRESHOLD_ADJUSTMENT_MV                      (100u) //RELEASE VALUE - 100 mV

/* Internal temperature sensor offset */
#define TEMP_OFFSET                                          (8u)
/*----------------------------- Mavlink parameters -----------------------*/
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
// Timing Configuration
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS       (750u)
#define MAVLINK_CONNECTION_TIMEOUT_MS                 (3000u)
#endif /* MAVLINK_V2_CTRL_SUPP */

/*----------------------------- Test parameters -----------------------*/
#define TEST_SELF_DESTROY_ONLY_MODE                   (0u) //RELEASE VALUE - 0u
#define TEST_SELF_DESTROY_MINING_MODE                 (0u) //RELEASE VALUE - 0u

#define SELF_DESTROY_DISABLE                          (0u) //RELEASE VALUE - 0u

#define ACC_HIT_DETECTED_STICKY_LED_FEATURE           (0u) //RELEASE VALUE - 0u
#define MOVE_DETECTED_STICKY_LED_FEATURE              (0u) //RELEASE VALUE - 0u

#endif /* RELEASE_CONFIGURATION_ENABLE */

/*------------------------ Flash Configuration parameters -----------------------*/

#define CONFIG_1_OFFSET_BYTES                         (128u)
#define USR_PAGE                                      (FLASH_PAGE_NB - 1)
#define CONFIG_ADDR_0                                 (FLASH_BASE + (USR_PAGE * FLASH_PAGE_SIZE))
#define CONFIG_ADDR_1                                 (FLASH_BASE + CONFIG_1_OFFSET_BYTES + (USR_PAGE * FLASH_PAGE_SIZE))


#ifdef __cplusplus
}
#endif

#endif /* __PRJ_CONFIG_H */

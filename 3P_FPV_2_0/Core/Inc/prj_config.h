
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



#define RELEASE_CONFIGURATION_ENABLE                  (0u)

#if (RELEASE_CONFIGURATION_ENABLE == 0u) //TEST CONFIGURATION
/*-------------------------- BUILD MACROS -----------------------------------------*/
#define CONTROL_MODE                                  (PWM_CTRL_SUPP)
#define VBAT_MEASURE_FEATURE                          (1u)
#define ACC_SUPPORTED_ENABLE                          (1u)
#define LIS2DH12_ACC_ENABLE                           (1u)
#define START_UP_DELAY_ENABLE                         (1u)
#define UART_ENABLE                                   (1u)

/*--------------------------- VUSA MACROS -----------------------------------------*/
#define VUSA_ENABLE                                   (0u) // RELEASE VALUE - 1u
#define VUSA_ENABLE_DEFAULT                           (1u) // RELEASE VALUE - 1u      

/*--------------------------- OSD MACROS -----------------------------------------*/
#define OSD_ENABLE                                    (1u) // RELEASE VALUE - 1u
#define OSD_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/*----------------------------PWM 2 MACROS -----------------------------------------*/
#if (OSD_ENABLE == 1u)
#define PWM2_INPUT_ENABLE                             (0u) // RELEASE VALUE - 1u
#define PWM2_INPUT_ENABLE_DEFAULT                     (0u) // RELEASE VALUE - 1u
#else
#define PWM2_INPUT_ENABLE                             (1u) // RELEASE VALUE - 0u
#define PWM2_INPUT_ENABLE_DEFAULT                     (1u) // RELEASE VALUE - 0u
#endif /* OSD_ENABLE == 0u */

/*-------------------------- Main Configuration parameters -----------------------*/
/* Safe Timer Timeout Value is seconds */
#define SAFE_TIMEOUT_SEC_DEFAULT                      (10u) // RELEASE VALUE - 120u

/* Ignition delay in milliseconds */
#define IGNITION_DELAY_MILISEC_DEFAULT                (0u) // RELEASE VALUE - 0u

/* Enable/disable accelerometer functionality */
#define ACC_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/* Self destruction related timings */
#define SELF_DESTROY_TIMEOUT_MIN_DEFAULT              (0u)     //RELEASE VALUE - 30u

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

#define ACC_NO_DIVIDE_ENABLE                          (1u)

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

#else //RELEASE CONFIGURATION !!!!!!!!!
/*-------------------------- BUILD MACROS -----------------------------------------*/
#define CONTROL_MODE                                  (PWM_CTRL_SUPP)
#define VBAT_MEASURE_FEATURE                          (1u)
#define START_UP_DELAY_ENABLE                         (1u)
#define UART_ENABLE                                   (1u)

/*--------------------------- OSD MACROS -----------------------------------------*/
#define OSD_ENABLE                                    (1u) // RELEASE VALUE - 1u
#define OSD_ENABLE_DEFAULT                            (1u) // RELEASE VALUE - 1u

/*----------------------------PWM 2 MACROS -----------------------------------------*/
#if (OSD_ENABLE == 1u)
#define PWM2_INPUT_ENABLE                             (0u) // RELEASE VALUE - 1u
#define PWM2_INPUT_ENABLE_DEFAULT                     (0u) // RELEASE VALUE - 1u
#else
#define PWM2_INPUT_ENABLE                             (1u) // RELEASE VALUE - 0u
#define PWM2_INPUT_ENABLE_DEFAULT                     (1u) // RELEASE VALUE - 0u
#endif /* OSD_ENABLE == 0u */
/*--------------------------- VUSA MACROS -----------------------------------------*/
#define VUSA_ENABLE                                   (1u) // RELEASE VALUE - 1u
#define VUSA_ENABLE_DEFAULT                           (1u) // RELEASE VALUE - 1u      
/*-------------------------- ACCELEROMETER MACROS --------------------------------*/
#define LIS2DH12_ACC_ENABLE                           (1u)
#define ACC_NO_DIVIDE_ENABLE                          (1u)

#define ACC_HIT_THRESHOLD_MG                          (12000u)  // units in mili G //RELEASE VALUE - 12G - 12000u
#define ACC_BUFF_SIZE                                 (6u) //RELEASE VALUE - WINDOW SIZE = 6u

#define NET_DETECTION_ENABLE                          (0u)
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
#define IGNITION_OFF_TMR_PERIOD_MS  		 		  (2500u) //RELEASE VALUE - 2500u

/* Disarm -> Ignition timeout */
#define DISARM_IGNITION_BLOCK_TIMEOUT_MS              (4000u) // RELEASE VALUE - 4000u

#define BUZZER_DISABLE                                (0u)

/*------------------------ Battery Configuration parameters -----------------------*/

// Battery level calculation thresholds for AAA battery
#define BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS      (1450u)  // 100% - 1.45V
#define BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS        (1000u)   // 0% - 0.9V

#define BATTERY_VOLTAGE_VERY_LOW_SD_LOW_THRESHOLD_MILIVOLTS  (1100u) //RELEASE VALUE - 1100 mV
#define BATTERY_VOLTAGE_LOW_NO_SOUND_THRESHOLD_MILIVOLTS     (1200u) //RELEASE VALUE - 1200 mV
#define BATTERY_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS       (1300u) //RELEASE VALUE - 1300 mV 
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

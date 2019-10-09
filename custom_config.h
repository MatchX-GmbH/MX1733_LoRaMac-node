/**
\addtogroup BSP
\{
\addtogroup CONFIG
\{
\addtogroup CUSTOM
\{
*/

/**
****************************************************************************************
*
* @file custom_config_qspi.h
*
* @brief Board Support Package. User Configuration file for cached QSPI mode.
*
* Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
* program includes Confidential, Proprietary Information and is a Trade Secret of
* Dialog Semiconductor Ltd. All use, disclosure, and/or reproduction is prohibited
* unless authorized in writing. All Rights Reserved.
*
* <black.orca.support@diasemi.com> and contributors.
*
****************************************************************************************
*/

#ifndef CUSTOM_CONFIG_QSPI_H_
#define CUSTOM_CONFIG_QSPI_H_

#define REGION_EU868

#define dg_configDISABLE_BACKGROUND_FLASH_OPS   (1)

#include "bsp_definitions.h"

#define dg_configUSE_LP_CLK                     LP_CLK_32768
#define dg_configEXEC_MODE                      MODE_IS_CACHED
#define dg_configCODE_LOCATION                  NON_VOLATILE_IS_FLASH
#define dg_configEXT_CRYSTAL_FREQ               EXT_CRYSTAL_IS_16M

/*
 * If the PRODUCTION_MODE is used the debugger is disabled from the firmware.
 * So once a firmware is flashed with PRODUCTION_MODE the chip will not
 * respond to JTAG again and "cannot open gdb interface" error will be seen.
 * To overcome this you need to hold the reset button and release as soon as
 * JTAG is trying to reach the chip.
 * */
#define dg_configIMAGE_SETUP                    PRODUCTION_MODE
//#define dg_configIMAGE_SETUP                    DEVELOPMENT_MODE

#if dg_configIMAGE_SETUP == PRODUCTION_MODE
#define dg_configSKIP_MAGIC_CHECK_AT_START      (1)
#define dg_configENABLE_DEBUGGER                (0)
#endif

#define dg_configEMULATE_OTP_COPY               (0)

#define dg_configIMAGE_FLASH_OFFSET             0x20000
#define dg_configSUOTA_SUPPORT                  (1)

#define dg_configUSE_WDOG                       (1)
#define dg_configWDOG_MAX_TASKS_CNT             (8)

#define dg_configUSE_DCDC                       (1)

#define dg_configFLASH_CONNECTED_TO             (FLASH_CONNECTED_TO_1V8)
#define dg_configFLASH_POWER_DOWN               (1)

#define dg_configPOWER_1V8_ACTIVE               (1)
#define dg_configPOWER_1V8_SLEEP                (1)

#define dg_configBATTERY_TYPE                   (BATTERY_TYPE_LIMN2O4)
#define dg_configBATTERY_CHARGE_CURRENT         11      // 300mA
#define dg_configBATTERY_PRECHARGE_CURRENT      29      // 15.3mA
#define dg_configBATTERY_CHARGE_NTC             1       // disabled
#define dg_configPRECHARGING_THRESHOLD          2462    // 3.006V
#define dg_configCHARGING_THRESHOLD             2498    // 3.05V
#define dg_configPRECHARGING_TIMEOUT    (2 * 60 * 60 * 100)  // 2 hours
#define dg_configCHARGING_CC_TIMEOUT    (9 * 60 * 60 * 100)  // 9 hours
#define dg_configCHARGING_CV_TIMEOUT    (8 * 60 * 60 * 100)  // 8 hours

#define dg_configUSE_USB_CHARGER                1
#define dg_configALLOW_CHARGING_NOT_ENUM        1
#if 0
#define DEBUG_USB_CHARGER                       (1)
#define DEBUG_USB_CHARGER_FSM                   (1)
#define DEBUG_USB_CHARGER_PRINT                 (1)
#define USB_CHARGER_TIMING_DEBUG                (1)
#endif

#define dg_configUSE_ProDK                      (1)

#define dg_configUSE_SW_CURSOR                  (1)


/*************************************************************************************************\
 * FreeRTOS specific config
 */
#define OS_FREERTOS                              /* Define this to use FreeRTOS */
#define configTOTAL_HEAP_SIZE                    14584   /* This is the FreeRTOS Total Heap Size */
//#define OS_BAREMETAL

/*************************************************************************************************\
 * Memory specific config
 */
#define dg_configQSPI_CACHED_OPTIMAL_RETRAM_0_SIZE_AE   ( 64 * 1024)
#define dg_configQSPI_CACHED_RAM_SIZE_AE                ( 32 * 1024)
#define dg_configQSPI_CACHED_RETRAM_0_SIZE_AE           ( 96 * 1024)

/*************************************************************************************************\
 * Peripheral specific config
 */
#define dg_configUSE_HW_I2C                     (1)
#define dg_configUSE_HW_IRGEN                   (1)
#define dg_configUSE_HW_QUAD                    (1)
#define dg_configUSE_HW_RF                      (1)
#define dg_configUSE_HW_SPI                     (1)
#define dg_configUSE_HW_TRNG                    (1)

#define dg_configUSE_HW_TIMER1                  (1)
#define dg_configUSE_HW_TIMER2                  (1)

#define dg_configGPADC_ADAPTER                  (1)
#define dg_configI2C_ADAPTER                    (1)
#define dg_configNVPARAM_ADAPTER                (1)
#define dg_configSPI_ADAPTER                    (1)
#define dg_configUART_ADAPTER                   (1)
#define dg_configRF_ADAPTER                     (1)
#define dg_configTEMPSENS_ADAPTER               (1)

#define dg_configCACHEABLE_QSPI_AREA_LEN        (NVMS_PARAM_PART_start - MEMORY_QSPIF_BASE)

#define CONFIG_USE_BLE
#define SUOTA_VERSION                           SUOTA_VERSION_1_2
#define SUOTA_PSM                               0x81
#define USE_PARTITION_TABLE_1MB_WITH_SUOTA

#define HW_UART_ENABLE_USER_ISR

#define CONFIG_CUSTOM_PRINT
//#define CONFIG_NO_PRINT
//#define CONFIG_RTT

#ifdef CONFIG_CUSTOM_PRINT
#define __HEAP_SIZE                             0x600
#endif

#define dg_configOPTIMAL_RETRAM                 (1)

#if (dg_configOPTIMAL_RETRAM == 1)
        #if (dg_configBLACK_ORCA_IC_REV == BLACK_ORCA_IC_REV_A)
                #define dg_configMEM_RETENTION_MODE             (0x1B)
                #define dg_configSHUFFLING_MODE                 (0x0)
        #else
                #define dg_configMEM_RETENTION_MODE             (0x07)
                #define dg_configSHUFFLING_MODE                 (0x0)
        #endif
#endif

/* Include bsp default values */
#include "bsp_defaults.h"
/* Include memory layout */
#include "bsp_memory_layout.h"

#endif /* CUSTOM_CONFIG_QSPI_H_ */

/**
\}
\}
\}
*/

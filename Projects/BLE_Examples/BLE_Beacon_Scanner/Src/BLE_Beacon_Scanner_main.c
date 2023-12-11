/******************** (C) COPYRIGHT 2022 STMicroelectronics ********************
* File Name          : BLE_Beacon_Scanner_main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : 23-August-2022
* Description        : Main file for beacon device
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP Beacon Scanner demo \see BLE_Beacon_Scanner_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */ 
 
/* Includes-----------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "OTA_btl.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_stack.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "osal.h"
#include "scanner.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "beacon_scanner_config.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private typedef ------------------------------ */
/* Private define ------------------------------ */
#define BLE_BEACON_SCANNER_VERSION_STRING "1.0"

/* Private macro ------------------------------ */
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);
/* Private variables ------------------------------ */
/* Private function prototypes ------------------------------ */
/* Private functions ------------------------------ */


void ModulesInit(void)
{
  uint8_t ret;

  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);
  BLECNTR_InitGlobal();

  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  BLEPLAT_Init();  

  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
    while(1);
  }

  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
    while(1);
  }

  /* Init the AES block */
  AESMGR_Init();

  /* BlueNRG-LPS stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }

}

void ModulesTick(void)
{

  /* Timer tick */
  HAL_VTIMER_Tick();

  /* Bluetooth stack tick */
  BLE_STACK_Tick();

  /* NVM manager tick */
  NVMDB_Tick();

}

int main(void)
{
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for power save modes */
  BSP_IO_Init();


  /* Init the UART peripheral */
  BSP_COM_Init(NULL);

  /* Configure application LED */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  BSP_LED_Off(BSP_LED1);
  BSP_LED_Off(BSP_LED2);
  BSP_LED_Off(BSP_LED3);

  /* Init BLE stack, HAL virtual timer and NVM modules */
  ModulesInit(); 
  
  PRINTF("BlueNRG-X Bluetooth LE Beacon Scanner Application (version: %s): ", BLE_BEACON_SCANNER_VERSION_STRING);
#if defined(CTE_LOCATOR)
  PRINTF("AoA Locator\r\n");
#elif defined(PERIODIC_ADV_SYNC)
  PRINTF("periodic advertising receiver\r\n");
#else
  PRINTF("beacon receiver\r\n");
#endif 
  
  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;
  wakeupIO.LPU_enable = 0;
  
  /* Init the Bluetooth LE stack layers */
  device_initialization();
  
  while(1) {
    /* Bluetooth LE stack tick */
    ModulesTick();

    /* Application Tick */
    APP_Tick();

    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);

  }
}

/****************** BlueNRG-x Power Management Callback ********************************/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}


/**
 * @brief  Hardware Error event
 * @param  uint8_t Hardware_Code.
 * @retval void.
*/
void hci_hardware_error_event(uint8_t Hardware_Code)
{

  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }

}


#ifdef USE_FULL_ASSERT


/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file	pointer to the source file name.
 * @param  line	assert_param error line source number.
 * @retval None.
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/** \endcond 
*/

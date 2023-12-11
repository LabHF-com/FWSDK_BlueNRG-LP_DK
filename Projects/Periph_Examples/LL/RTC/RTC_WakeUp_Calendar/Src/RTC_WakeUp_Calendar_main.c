/**
  ******************************************************************************
  * @file    RTC/RTC_WakeUp_Calendar/Src/RTC_WakeUp_Calendar_main.c
  * @author  RF Application Team
  * @brief   this example shows the configuration of the calendar and the 
  *          configuration of the RTC to wake up from Standby mode using the 
  *          RTC Wakeup timer.
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_rtc.h"
#include "rf_driver_ll_utils.h"
#include "hal_miscutil.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_device_it.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#include "bluenrg_lp_evb_config.h"
#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_io.h"
#include "bluenrg_lp_evb_led.h"
#endif


#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* the WUTF flag is set every (WUT[15:0] + 1) ck_wut cycles */
#define WAKEUP_TIMEOUT       4         /* 5 s */ 

/* ck_apre = Freq/(ASYNC prediv + 1) with 32 kHz  */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre = ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

#define RTC_ERROR_NONE    0
#define RTC_ERROR_TIMEOUT 1

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/**
  * @brief BSP_LED2 
  */

#ifdef STEVAL_IDB011V1
#define LED2_PIN                                LL_GPIO_PIN_8
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define LED2_PIN                                LL_GPIO_PIN_4
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */



/* Define used to indicate date/time updated */
#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F2)

/** @addtogroup StdPeriph_Examples Peripheral Examples
* @{
*/


/** @addtogroup RTC Examples
* @{
*/

/** @addtogroup RTC_WakeUp_Calendar Example
* @{
*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/  
/* Buffers used for displaying Time and Date */
uint8_t aShowTime[] = "hh:ms:ss";
uint8_t aShowDate[] = "mm:dd:aaaa";

/* Private function prototypes -----------------------------------------------*/
void PrintNegitatedLevel(uint8_t stopLevel);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
void PrintWakeupSource(uint32_t wakeupSources);
#if defined(STEVAL_IDB011V1)
void TogglePinsOutputDeepstop(void);
#endif /* STEVAL_IDB011V1 */
#endif


uint32_t Enter_RTC_InitMode(void);
uint32_t Exit_RTC_InitMode(void);
uint32_t WaitForSynchro_RTC(void);
void Show_RTC_Calendar(void);

void LED_On(void);
void LED_Blinking(uint32_t Period);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Display the Stop Level negotiated.
* @param  stopLevel negotiated Stop level
* @retval None
*/
void PrintNegitatedLevel(uint8_t stopLevel)
{
  printf("Power save level negotiated: ");
  switch (stopLevel)
  { 
  case POWER_SAVE_LEVEL_RUNNING:
    printf ("RUNNING\r\n");
    break;
  case POWER_SAVE_LEVEL_CPU_HALT:
    printf ("CPU_HALT\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_WITH_TIMER:
    printf ("STOP_WITH_TIMER\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_NOTIMER:
    printf ("STOP_NOTIMER\r\n");
    break;
  }
}

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
/**
* @brief  Display the Wakeup Source.
* @param  wakeupSource Wakeup Sources
* @retval None
*/
void PrintWakeupSource(uint32_t wakeupSources)
{
  printf("Wakeup Source : ");
  switch (wakeupSources)
  {
  case WAKEUP_RTC:
    printf("WAKEUP_RTC ");
    break;
  case WAKEUP_PA10:
    printf("WAKEUP_PA10 ");
    break;
  case WAKEUP_PA8:
    printf("WAKEUP_PA8 ");
    break;
  case WAKEUP_PB0:
    printf("WAKEUP_PB0 ");
    break;
  default:
    printf("(default) WAKEUP source 0x%08x ", wakeupSources);
  }
  printf("\r\n");
}
#endif


void RTC_WakeupInit(void)
{
  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};

  /* Enable Peripheral Clock */
  if (LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR1) != RTC_BKP_DATE_TIME_UPDTATED)
  {
    /* Enable Peripheral Clock */
    LL_APB0_EnableClock(LL_APB0_PERIPH_RTC);
    
    /* RTC peripheral reset */
    LL_RCC_ClearFlag_RTCRSTREL();
    LL_APB0_ForceReset(LL_APB0_PERIPH_RTC);
    LL_APB0_ReleaseReset(LL_APB0_PERIPH_RTC);
    while(LL_RCC_IsActiveFlag_RTCRSTREL() == 0);
    LL_RCC_ClearFlag_RTCRSTREL();
    
    /* Disable the write protection for RTC registers */
    LL_RTC_DisableWriteProtection(RTC);
  
    /* Enter in initialization mode */
    if (Enter_RTC_InitMode() != RTC_ERROR_NONE)
    {
      /* Initialization Error */
      LED_Blinking(LED_BLINK_ERROR);
    }
    RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
    RTC_InitStruct.AsynchPrescaler = RTC_ASYNCH_PREDIV;
    RTC_InitStruct.SynchPrescaler = RTC_SYNCH_PREDIV;
    LL_RTC_Init(RTC, &RTC_InitStruct);
    
    RTC_TimeStruct.TimeFormat = LL_RTC_TIME_FORMAT_AM_OR_24;
    RTC_TimeStruct.Hours = 0x23;
    RTC_TimeStruct.Minutes = 0x59;
    RTC_TimeStruct.Seconds = 0x50;
    LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
    RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_FRIDAY;
    RTC_DateStruct.Month = LL_RTC_MONTH_DECEMBER;
    RTC_DateStruct.Day = 0x31;
    RTC_DateStruct.Year = 0x19;
    
    LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);
    
    /* Output disabled */
    LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
    
    /* Output polarity */
    LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
    
    /* Exit of initialization mode */
    if (Exit_RTC_InitMode() != RTC_ERROR_NONE)
    {
      /* Initialization Error */
      LED_Blinking(LED_BLINK_ERROR);
    }
    
    /* Enable RTC registers write protection */
    LL_RTC_EnableWriteProtection(RTC);
    
    /* Writes a data in a RTC Backup data Register1 */
    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR1, RTC_BKP_DATE_TIME_UPDTATED);
  }
}

void SetRTC_WakeupTimeout(uint32_t time)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Disable Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* In case of interrupt mode is used, the interrupt source must disabled */
  LL_RTC_DisableIT_WUT(RTC);
  
  /* Wait till RTC WUTWF flag is set  */
  while(LL_RTC_IsActiveFlag_WUTW(RTC) == 0);

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* Clear PWR wake up Flag */
  LL_PWR_ClearWakeupSource(LL_PWR_EWS_INT);
#endif

  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  /* Configure the Wake-up Timer counter */
  LL_RTC_WAKEUP_SetAutoReload(RTC, time);
  
  /* Configure the clock source */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Configure the Interrupt in the RTC_CR register */
  LL_RTC_EnableIT_WUT(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* Configure NVIC for RTC */
  NVIC_SetPriority(RTC_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(RTC_IRQn);    
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
}

void DisableRTC_WakeupTimeout(void)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
  uint8_t ret_val;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  uint32_t wakeupSources;
#endif

  
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  printf("** Application started **\n\r");
  
  /* Init LED2 */
  BSP_LED_Init(BSP_LED2);
  
  /* Init BUTTON 1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* RTC Wakeup Peripheral Init */
  RTC_WakeupInit();
  
  while(1) {
    /* Display the updated Time and Date */
    Show_RTC_Calendar();
    
#if defined(STEVAL_IDB011V1) 
    TogglePinsOutputDeepstop();
#endif /* STEVAL_IDB011V1 */
    
    /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART (PA8)/timeout 5 sec, (RTC)/button PUSH1 (PA10) */
    printf("Enable Power Save Request : STOP_WITH_TIMER (RTC)\r\n");
    /* Display time Format : hh:ms:ss */
    printf(" Time : ");
    printf("%.2d:%.2d:%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
    printf("\n\r");
    /* Display date Format : mm-dd-yy */
    printf(" Date : ");
    printf("%.2d-%.2d-%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
           2000 + __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
    printf("\n\r");
    
    while(BSP_COM_UARTBusy());
    wakeupIO.RTC_enable = 1;
    wakeupIO.LPU_enable = 0;
#if defined(STEVAL_IDB011V1)
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
#endif
#if defined(STEVAL_IDB012V1)
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PB0;
#endif
    SetRTC_WakeupTimeout(WAKEUP_TIMEOUT);
    ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
    if (ret_val != SUCCESS)
      printf("Error during clock config 0x%2x\r\n", ret_val);
    PrintNegitatedLevel(stopLevel);
    if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
      wakeupSources = HAL_PWR_MNGR_WakeupSource();
      PrintWakeupSource(wakeupSources);
    }
#endif

  }
}

/**
* @brief  Enter in initialization mode
* @note In this mode, the calendar counter is stopped and its value can be updated
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Enter_RTC_InitMode(void)
{
  /* Set Initialization mode */
  LL_RTC_EnableInitMode(RTC);
    
  /* Check if the Initialization mode is set */
  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
  {
  }
  return RTC_ERROR_NONE;
}

/**
* @brief  Exit Initialization mode
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Exit_RTC_InitMode(void)
{
  LL_RTC_DisableInitMode(RTC);
  
  /* Wait for synchro */
  /* Note: Needed only if Shadow registers is enabled           */
  /*       LL_RTC_IsShadowRegBypassEnabled function can be used */
  return (WaitForSynchro_RTC());
}

/**
* @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
*         synchronized with RTC APB clock.
* @param  None
* @retval RTC_ERROR_NONE if no error (RTC_ERROR_TIMEOUT will occur if RTC is
*         not synchronized)
*/
uint32_t WaitForSynchro_RTC(void)
{
  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTC);

  /* Wait the registers to be synchronised */
  while (LL_RTC_IsActiveFlag_RS(RTC) != 1)
  {
  }
  return RTC_ERROR_NONE;
}

/**
* @brief  Display the current time and date.
* @param  None
* @retval None
*/
void Show_RTC_Calendar(void)
{
  /* Note: need to convert in decimal value in using __LL_RTC_CONVERT_BCD2BIN helper macro */
  /* Display time Format : hh:mm:ss */
  sprintf((char *)aShowTime, "%.2d:%.2d:%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
  /* Display date Format : mm-dd-yy */
  sprintf((char *)aShowDate, "%.2d-%.2d-%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
          2000 + __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
}


#if defined(STEVAL_IDB011V1)
/**
* @brief  Toggle output signa on PA11 every time that wake on.
* @param  None
* @retval None
*/
void TogglePinsOutputDeepstop(void)
{
  uint32_t tmp = LL_PWR_GetPA11OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA11OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA11OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA4OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA4OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA4OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA6OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA7OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA7OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA7OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
}
#endif /* STEVAL_IDB011V1 */

/**
* @brief  Turn-on LED2.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  LED_Blinking(LED_BLINK_ERROR);
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/




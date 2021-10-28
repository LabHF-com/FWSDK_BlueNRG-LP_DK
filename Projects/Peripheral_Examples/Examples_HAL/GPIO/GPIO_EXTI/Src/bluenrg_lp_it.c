/**
******************************************************************************
* @file    GPIO/GPIO_EXTI/Src/bluenrg_lp_it.c
* @author  RF Application Team
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the 
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "GPIO_EXTI_main.h"
#include "bluenrg_lp_it.h"


extern EXTI_HandleTypeDef HEXTI_InitStructure;
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/


/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_IRQHandler(void)
{
  while (1)
  {
  }
}



#define DEBOUNCE_CNT  350
volatile uint32_t debounce_count = 0;

/**
* @brief This function handles System tick timer.
*/
void SysTick_IRQHandler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  debounce_count++;
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
* @brief  This function handles external line 10 interrupt request.
* @param  None
* @retval None
*/
void GPIOA_IRQHandler(void)
{  
  static uint32_t debounce_last = 0;
  
  if(HAL_EXTI_GetPending( &HEXTI_InitStructure )){
    if ( (debounce_count - debounce_last) >= DEBOUNCE_CNT )
    { 
      /* Add the SW no bounce */
      debounce_last = debounce_count;
      
      /* Handle user button press in dedicated function */
      HAL_EXTI_IRQHandler( &HEXTI_InitStructure );
    }
    LL_EXTI_ClearInterrupt(HEXTI_InitStructure.Line);
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
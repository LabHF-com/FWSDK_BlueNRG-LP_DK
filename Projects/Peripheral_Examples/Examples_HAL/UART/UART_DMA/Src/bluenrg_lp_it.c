/**
  ******************************************************************************
  * @file    UART/UART_DMA/Src/bluenrg_lp_it.c
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "UART_DMA_main.h"
#include "bluenrg_lp_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles System tick timer.
  */
void SysTick_IRQHandler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 and channel2 global interrupt.
  */
void DMA_IRQHandler(void)
{
  if( __HAL_DMA_GET_FLAG(DMA1, DMA_FLAG_TC1 ) ){
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
  }
  
  if( __HAL_DMA_GET_FLAG(DMA1, DMA_FLAG_TC2 ) ){
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  }
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file    TIM/TIM_OnePulse/Inc/TIM_OnePulse_main.h
  * @author  RF Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_hal.h"
#include "bluenrg_lp_evb_config.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Definition for TIMx clock resources */

  
#define TIMx                       TIM1
#define EnableClock_TIMx()         __HAL_RCC_TIM1_CLK_ENABLE()  
#define DisableClock_TIMx()        __HAL_RCC_TIM1_CLK_DISABLE()    
  
  
  /**TIM1 GPIO Configuration  
  PA4 / AF4   ------> TIM1_CH1 
  */
#define TIMx_CH1_PIN    GPIO_PIN_4
#define TIMx_CH1_AF     GPIO_AF4_TIM1
#define TIMx_CH1_PORT   GPIOA

  /**TIM1 GPIO Configuration  
  PA5 / AF4   ------> TIM1_CH2
  */
#define TIMx_CH2_PIN    GPIO_PIN_5
#define TIMx_CH2_AF     GPIO_AF4_TIM1
#define TIMx_CH2_PORT   GPIOA

  /**TIM1 GPIO Configuration  
  PB2/AF3     ------> TIM1_CH3
  */
#define TIMx_CH3_PIN    GPIO_PIN_2
#define TIMx_CH3_AF     GPIO_AF3_TIM1
#define TIMx_CH3_PORT   GPIOB

  /**TIM1 GPIO Configuration  
  PA1/AF4     ------> TIM1_CH4 
  */
#define TIMx_CH4_PIN    GPIO_PIN_1
#define TIMx_CH4_AF     GPIO_AF4_TIM1
#define TIMx_CH4_PORT   GPIOA

#define EXTERNAL_TRIGGER_SIGNAL_PIN             GPIO_PIN_8          
#define EXTERNAL_TRIGGER_SIGNAL_PORT            GPIOB




    /**
  * @brief Key push-button
  */
#if defined(STEVAL_S38681V1) 
#define USER_BUTTON_PIN                         GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         __HAL_RCC_SYSCFG_CLK_ENABLE()
#define USER_BUTTON_EXTI_LINE                   EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler
#endif
#define USER_BUTTON_PIN                         GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE() 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         __HAL_RCC_SYSCFG_CLK_ENABLE()   
#define USER_BUTTON_EXTI_LINE                   EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler

/* Exported functions ------------------------------------------------------- */
void Example_EXTI_Callback(uint32_t Line);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

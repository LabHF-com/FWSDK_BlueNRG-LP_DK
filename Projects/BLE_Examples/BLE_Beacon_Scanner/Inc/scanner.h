/*
  ******************************************************************************
  * @file    scanner.h 
  * @author  RF Application Team
  * @date    22 - 08 - 2022
  * @brief   Application Header functions
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2022 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */ 
 
#ifndef _SCANNER_H_
#define _SCANNER_H_

/**
  * @brief  This function initializes the BLE stack to be ready for following activities.
  * @param  None
  * @retval None
  */
void device_initialization(void);

/**
  * @brief  User Application tick 
  * @param  None
  * @retval None
  */
void APP_Tick(void);



#endif /* _SCANNER_H_ */
/** \endcond 
*/

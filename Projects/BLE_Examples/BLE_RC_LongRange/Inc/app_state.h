/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : app.h
* Author             : AMS - AAS, RF Application Team
* Version            : V1.0.0
* Date               : 14-March-2014
* Description        : Header file wich contains variable used for application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Exported variables ------------------------------------------------------- */  
     
/** 
  * @brief  Variable which contains some flags useful for application
  */ 
extern volatile int app_flags;

/**
 * @name Flags for application
 * @{
 */
#define CONNECTED               0x01
#define SET_CONNECTABLE         0x02
/**
 * @}
 */



/* Exported macros -----------------------------------------------------------*/
#define APP_FLAG(flag) (app_flags & flag)

#define APP_FLAG_SET(flag) (app_flags |= flag)
#define APP_FLAG_CLEAR(flag) (app_flags &= ~flag)


#ifdef __cplusplus
}
#endif

#endif /*__APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

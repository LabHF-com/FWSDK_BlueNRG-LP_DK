/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : Reset_Manager.c 
* Author             : AMS - RF Application team
* Version            : V1.0.0
* Date               : 23-June-2020
* Description        : Reset manager main file. It jumps to the application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP BLE Static Stack \see Reset_Manager.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
#include "Reset_Manager_Config.h"
#include "system_bluenrg_lp.h"

int main(void)
{
  if(*(uint32_t *)BLUE_FLAG_FLASH_BASE_ADDRESS == BLUE_FLAG_TAG){
    
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) APP_ADDR);
    entryPoint();
  }  
  while(1);
}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
/** \endcond
 */

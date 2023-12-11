/**
  ******************************************************************************
  * @file    LL/GPIO/FLASH_BasicDemo/Src/FLASH_BasicDemo_main.c
  * @author  RF Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the GPIO LL API.
  *          Peripheral initialization done using LL unitary services functions.
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
#include "FLASH_BasicDemo_main.h"

/** @addtogroup RF_DRIVER_LL_Examples
  * @{
  */

/** @addtogroup FLASH_BasicDemo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR    (LL_FLASH_END_ADDR - LL_FLASH_PAGE_SIZE + 1)  
#define FLASH_USER_END_ADDR      (LL_FLASH_END_ADDR + 1)
#define DATA_32                  ((uint32_t)0xAABBCCDD)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t firstPageNumber = 0;
uint32_t lastPageNumber = 0;
uint32_t numberOfPages = 0;
uint32_t address = 0;
uint32_t addressCheck = 0;
__IO uint32_t data32 = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock); 
  
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  printf("** Application started **\n\r");
  
  address = FLASH_USER_START_ADDR;
  
  if(!IS_LL_ADDR_ALIGNED_32BITS(address))
  {
    Error_Handler();
  }
  
  firstPageNumber = (FLASH_USER_START_ADDR - LL_FLASH_START_ADDR) / LL_FLASH_PAGE_SIZE;
  lastPageNumber = (FLASH_USER_END_ADDR - LL_FLASH_START_ADDR) / LL_FLASH_PAGE_SIZE;
  numberOfPages = lastPageNumber - firstPageNumber;

  if(!IS_LL_FLASH_PAGE(firstPageNumber))
  {
    Error_Handler();
  }
  if(!IS_LL_FLASH_PAGE(lastPageNumber))
  {
    Error_Handler();
  }
  
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, firstPageNumber, numberOfPages);
  
  while (address < FLASH_USER_END_ADDR)
  {
    LL_FLASH_Program(FLASH, address, DATA_32);
    address = address + 4;
  }
  
  address = FLASH_USER_START_ADDR;
  
  while (address < FLASH_USER_END_ADDR)
  {
    data32 = *(__IO uint32_t *)address;
    
    if (data32 != DATA_32)
    {
      addressCheck++;
    }
    address = address + 4;
  }
  
  /*Check if there is an issue to program data*/
  if (addressCheck == 0)
  {
    /* No error detected 2*/
    printf("No error detected.\n\r");
    printf("** Test successfully. ** \n\r\n\r");
  }
  else
  {
    /* Error detected. */
    printf("Error detected.\n\r");
    Error_Handler();
  }
    
  /* Infinite loop */
  while (1)
  {
  }
}


/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the LL error return state */
  while(1);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



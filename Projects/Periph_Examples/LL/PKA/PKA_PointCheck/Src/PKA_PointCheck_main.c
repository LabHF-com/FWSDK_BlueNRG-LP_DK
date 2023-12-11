
/******************** (C) COPYRIGHT 2022 STMicroelectronics ********************
* File Name          : PKA_PointCheck_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 21-September-2015
* Description        : Code demonstrating the PKA functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  PKA_PointCheck/PKA_PointCheck_main.c
 * @brief This example demonstrates how to use the low-layer PKA API to generate an ECC signature
 *


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
 <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_PointCheck\\MDK-ARM\\{STEVAL-IDB012V1}\\PKA_PointCheck.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
   <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_PointCheck\\EWARM\\{STEVAL-IDB012V1}\\PKA_PointCheck.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_PointCheck\\WiSE-Studio\\{STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB012V1
- \c STEVAL-IDB013V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  | STEVAL-IDB012V1 |
-----------------------------------
|     A1     |       Not Used      |
|     A11    |       Not Used      |
|     A12    |         N.A.        |
|     A13    |         N.A.        |
|     A14    |         N.A.        |
|     A15    |         N.A.        |
|     A4     |         N.A.        |
|     A5     |         N.A.        |
|     A6     |         N.A.        |
|     A7     |         N.A.        |
|     A8     |       Not Used      |
|     A9     |       Not Used      |
|     B0     |       Not Used      |
|     B14    |       Not Used      |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |         DL2         |
|     B5     |       Not Used      |
|     B7     |       Not Used      |
|     B8     |         N.A.        |
|     B9     |         N.A.        |
|     A0     |       Not Used      |
|     A10    |       Not Used      |
|     B1     |       Not Used      |
|     B6     |       Not Used      |
|     B15    |       Not Used      |
|     GND    |       Not Used      |
|     RST    |       Not Used      |
|    VBAT    |       Not Used      |
@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |             STEVAL-IDB012V1            |             STEVAL-IDB013V1            |
-------------------------------------------------------------------------------------------------
|     DL1    |                Not Used                |                Not Used                |
|     DL2    |   On: success; Slowly blinking: error  |   On: success; Slowly blinking: error  |
|     DL3    |                Not Used                |                Not Used                |
|     DL4    |                Not Used                |                Not Used                |
|     U5     |                Not Used                |                Not Used                |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |    STEVAL-IDB012V1   |    STEVAL-IDB013V1   |
-----------------------------------------------------------------
|      PUSH1     |       Not Used       |       Not Used       |
|      PUSH2     |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LPS  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

How to use the PKA peripheral to determine if a point is on a curve. This allows validating an external public key.

For this example, 2 test vector have been extracted from National Institute of Standards and Technology (NIST) - Cryptographic Algorithm Validation Program (CAVP) in order to demonstrate the usage of the hal.
Those reference files can be found under:
"http://csrc.nist.gov/groups/STM/cavp/documents/dss/186-3ecdsatestvectors.zip (ZIP PKV.rsp)"

Those two test vectors have been choosen to demonstrate the behavior in a case where the input point validates the curve equation and in a case where the input point does not verify the curve equation. Their definition is included in PKV.c. You can refer to this file for more informations.

The selected curve for this example is P-256 (ECDSA-256) published by NIST in Federal Information Processing Standards Publication FIPS PUB 186-4. The description
of this curve is present in file Src/prime256v1.c.

In case of success, the LED2 is ON.
In case of any error, the LED2 is toggling slowly.

BlueNRG_LP-EVB Set-up
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin to PC COM port TX signal
- Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 

**/
   
/* Includes ------------------------------------------------------------------*/
#include "PKA_PointCheck_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint32_t endOfProcess = 0;
uint8_t RBuffer[32] = {0};
uint8_t SBuffer[32] = {0};

int state = 0x0;
uint32_t errorCode[] = {0,0};
static uint32_t PKAStartPoint[16] = { 0xd898c296U, 0xf4a13945U, 0x2deb33a0U, 0x77037d81U, 0x63a440f2U, 0xf8bce6e5U, 0xe12c4247U, 0x6b17d1f2U, 
0x37bf51f5U, 0xcbb64068U, 0x6b315eceU, 0x2bce3357U, 0x7c0f9e16U, 0x8ee7eb4aU, 0xfe1a7f9bU, 0x4fe342e2U};

static const uint32_t P256_number_bits_mod[2] =
{
  256,
  0
};

static const uint32_t P256_coefficient_a_sign[2] =
{
  1,
  0
};

static const uint32_t P256_gfp[8] =
{
  0xFFFFFFFF, /* LSB */
  0xFFFFFFFF,
  0xFFFFFFFF,
  0x00000000,
  0x00000000,
  0x00000000,
  0x00000001,
  0xFFFFFFFF,
};


static const uint32_t P256_a[8] =
{
  0x00000003, /* LSB */
  0x00000000,
  0x00000000,
  0x00000000,
  0x00000000,
  0x00000000,
  0x00000000,
  0x00000000,
};

static const uint32_t P256_b[8] =
{
  0x27D2604B, /* LSB */
  0x3BCE3C3E,
  0xCC53B0F6,
  0x651D06B0,
  0x769886BC,
  0xB3EBBD55,
  0xAA3A93E7,
  0x5AC635D8,
};

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_PKA_Init(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
void printInfo(uint32_t indexPKARAM, uint32_t dim);

/* Private user code ---------------------------------------------------------*/

/**
* @brief The application entry point.
* @retval int
*/
int main(void)
{ 
  /* System initialization function */
  if (SystemInit(SYSCLK_16M, RADIO_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  LL_Init1msTick(SystemCoreClock);
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  printf("** Application started **\n\r");
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_PKA_Init();
  
  LL_PKA_SetMode(PKA, LL_PKA_MODE_POINT_CHECK );
  
#ifdef CONFIG_DEVICE_BLUENRG_LPS
  
  /* Set the muber of bits of p */
  LL_PKA_WriteSingleInput( PKA_POINT_CHECK_IN_MOD_NB_BITS, P256_number_bits_mod[0] );
  printInfo(PKA_POINT_CHECK_IN_MOD_NB_BITS, 1);
  
  /* Set the coefficient a sign */
  LL_PKA_WriteSingleInput( PKA_POINT_CHECK_IN_A_COEFF_SIGN, P256_coefficient_a_sign[0] );
  printInfo(PKA_POINT_CHECK_IN_A_COEFF_SIGN, 1);
  
  /* Set the coefficient |a| */
  LL_PKA_WriteOperand( PKA_POINT_CHECK_IN_A_COEFF, 8, P256_a );
  printInfo(PKA_POINT_CHECK_IN_A_COEFF, 9);
  
  /* Set the coefficient |b| */
  LL_PKA_WriteOperand( PKA_POINT_CHECK_IN_B_COEFF, 8, P256_b );
  printInfo(PKA_POINT_CHECK_IN_B_COEFF, 9);
  
  /* Set the modulus value p */
  LL_PKA_WriteOperand( PKA_POINT_CHECK_IN_MOD_GF, 8, P256_gfp );
  printInfo(PKA_POINT_CHECK_IN_MOD_GF, 9);
  
  /* Set the point coordinate x */
  LL_PKA_WriteOperand( PKA_POINT_CHECK_IN_INITIAL_POINT_X, 8, (uint32_t*)&PKAStartPoint[0] );
  /* Set the point coordinate y */
  LL_PKA_WriteOperand( PKA_POINT_CHECK_IN_INITIAL_POINT_Y, 8, (uint32_t*)&PKAStartPoint[8] );
  printInfo(PKA_POINT_CHECK_IN_INITIAL_POINT_X, 9);
  printInfo(PKA_POINT_CHECK_IN_INITIAL_POINT_Y, 9);
  
#endif 
  
  while(LL_PKA_IsActiveFlag_BUSY(PKA));
  
  /* Launch the computation in interrupt mode */
  LL_PKA_Start(PKA);
  
  /* Wait for the interrupt callback */
  while(endOfProcess != 1);
  
  /* Retreive the result and output buffer */
  
#ifdef CONFIG_DEVICE_BLUENRG_LPS
  errorCode[0] = LL_PKA_ReadSingleOutput(PKA_POINT_CHECK_OUT_ERROR);
  printInfo(PKA_POINT_CHECK_OUT_ERROR, 1);
  if( errorCode[0] == 0x0)
  {
    /* Error is equal to 0 if the point satisfies the curve equation. */
    printf("The point satisfies the curve equation.\n\r");
    LED_On();
    printf("** Test successfully. ** \n\r\n\r");
  }
  else
  {
    printf("** Test fail **\n\r");
  }
#endif
  /* Error is equal to 1 if the point is not on the curve. */
  /* The location storing Error is set equal to 2 at the beginning of the computation, 
  so if Error is equal to 2 at the end of the operation then it means that a fault has altered the execution of the operation. */
  
  /* Infinite loop */
  while (1)
  {
  }
}

void printInfo(uint32_t indexPKARAM, uint32_t dim)
{
  uint32_t ramAddress = 0;
  for(int i=indexPKARAM;i<(indexPKARAM+dim);i++)
  {
    ramAddress = PKA_RAM_BASE + ( i << 2);
    printf("0x%08X PKA_RAM[%d] = 0x%08X \n\r", ramAddress, i, PKA_RAM->RAM[i]);
  }
  printf("\n\r");
}

/**
* @brief PKA Initialization Function
* @param None
* @retval None
*/
static void MX_PKA_Init(void)
{
  /* Peripheral clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA);
  
  /* Configure NVIC for PKA interrupts */
  /* Set priority for PKA_IRQn */
  /* Enable PKA_IRQn */
  NVIC_SetPriority(PKA_IRQn, IRQ_MED_PRIORITY);
  NVIC_EnableIRQ(PKA_IRQn);
  
  LL_PKA_Enable(PKA);
  LL_PKA_EnableIT_ADDRERR(PKA);
  LL_PKA_EnableIT_RAMERR(PKA);
  LL_PKA_EnableIT_PROCEND(PKA);
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LED2_GPIO_CLK_ENABLE();
  
  /* Set several pins to low level on dedicated gpio port */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  
  /* Configure GPIO for LED */
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
}

void PKA_ERROR_callback(void)
{
  state++;
  LED_Blinking(LED_BLINK_ERROR);
}

void PKA_PROCEND_callback(void)
{
  LL_PKA_Disable(PKA);
  endOfProcess = 1;
}



/**
* @brief Turn-on LED2.
* @param None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param Period : Period of time (in ms) between each toggling of LED
* This parameter can be user defined values. Pre-defined values used in that example are :
* @arg LED_BLINK_FAST : Fast Blinking
* @arg LED_BLINK_SLOW : Slow Blinking
* @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}

/**
* @brief This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1);
}

#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : ADC_TemperatureSensor_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the ADC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  ADC_TemperatureSensor/ADC_TemperatureSensor_main.c
 * @brief This example shows how to sample the temperature using the internal temperature sensor.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\ADC\\ ADC_TemperatureSensor\\MDK-ARM\\{STEVAL-IDB011V1}\\ADC_TemperatureSensor.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\ADC\\ADC_TemperatureSensor\\EWARM\\{STEVAL-IDB011V1}\\ADC_TemperatureSensor.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\ADC\\ADC_TemperatureSensor\\WiSE-Studio\\{STEVAL-IDB011V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1



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
|  PIN name  |   STEVAL-IDB011V1  |
-----------------------------------
|     A1     |      Not Used      |
|     A11    |      Not Used      |
|     A12    |      Not Used      |
|     A13    |      Not Used      |
|     A14    |      Not Used      |
|     A15    |      Not Used      |
|     A4     |      Not Used      |
|     A5     |      Not Used      |
|     A6     |         U5         |
|     A7     |      Not Used      |
|     A8     |      USART TX      |
|     A9     |      USART RX      |
|     B0     |      Not Used      |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |        DL2         |
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |
-----------------------------------
|     DL1    |      Not Used      |
|     DL2    |   Overflow detect  |
|     DL3    |      Not Used      |
|     DL4    |      Not Used      |
|     U5     |    Activity LED    |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |
---------------------------------------
|      PUSH1     |      Not Used      |
|      PUSH2     |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage


This example shows how to sample the temperature using the internal temperature sensor.
The data is sampled and then printed out by using the USART peripheral with a rate of 1 sample each 100 ms.
This example is driven by polling the End of conversion from Down Sampler.
The overrun from Down Sampler event is also monitored.
This example is based on the BLUENRG_LP ADC LL API.
The peripheral initialization is done using LL unitary service functions for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample the output of the internal temperature sensor.
The USART baudrate is set to 115200 bps.

Example execution:
The user must just load and then run the application.
The user can get the output data through a serial terminal program.

Connection needed:
None.

Other peripherals used:
  1 USART for output the data acquired


**/
   
/* Includes ------------------------------------------------------------------*/
#include "rf_driver_ll_adc.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_dma.h"
#include "bluenrg_lpx.h"
#include "rf_driver_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */

#include "bluenrg_lp_evb_config.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define USER_SAMPLERATE       (LL_ADC_SAMPLE_RATE_28)

#define USER_DATAWIDTH        (LL_ADC_DS_DATA_WIDTH_16_BIT)
#define USER_RATIO            (LL_ADC_DS_RATIO_128)

/* If USE_CALIBRATION is 1 then the internal calibration
   points are used to compensate the ADC acquired sample */
#define USE_CALIBRATION       (1)

/* Private macro -------------------------------------------------------------*/
#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void APP_ADC_Init(void);
static void BSP_Init(void);

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t nTemperatureRawVal = 0;
  float adcValue = 0.0;

  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
 
  /* BSP Init */
  BSP_Init();

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);

  /* Initialize all configured peripherals */
  APP_ADC_Init();
  
  /* Start ADC conversion */
  LL_ADC_StartConversion(ADC);

  /* Infinite loop */
  while (1) {

    /* Check the ADC flag End Of Down Sampler conversion */
    if( LL_ADC_IsActiveFlag_EODS(ADC) == 1) {

      /* Get the temperature raw value from the Down Sampler */
      nTemperatureRawVal = LL_ADC_DSGetOutputData(ADC);
      adcValue = (float)LL_ADC_GetADCConvertedValueTemp(ADC, nTemperatureRawVal, USER_DATAWIDTH)/100.0;

      /* Printout the output value */
      printf("Temperature %d.%02d %cC\r\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue), 248);

      /* Clear the ADC flag End Of Down Sampler conversion */
      LL_ADC_ClearFlag_EODS(ADC);

      /* Toggle the conversion/activity LED */
      BSP_LED_Toggle(BSP_LED1);
      
      /* Add 100 ms of delay between each measure */
      LL_mDelay(100);
      
      /* Restart ADC conversion */
      LL_ADC_StartConversion(ADC);
    }
   
    /* Check the ADC flag overrun of Down Sampler */
    if( LL_ADC_IsActiveFlag_OVRDS(ADC) == 1) {
      
      /* Clear the ADC flag overrun of Down Sampler */
      LL_ADC_ClearFlag_OVRDS(ADC);

      /* Turn on the LED2 is overrun occurs */
      BSP_LED_On(BSP_LED2);
    }

  }
}


static void BSP_Init(void)
{
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);  

  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
}


/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void APP_ADC_Init(void)
{
  /* Peripheral clock enable */
  LL_APB1_EnableClock(LL_APB1_PERIPH_ADCDIG | LL_APB1_PERIPH_ADCANA);

  /* This function must not be called on QFN32 package */
  LL_ADC_LDOEnable(ADC);

  /* Enable the ADC */
  LL_ADC_Enable(ADC);

  /* Configure the sample rate */
  LL_ADC_SetSampleRate(ADC, USER_SAMPLERATE);
  
  /* Configure the Down Sampler data width and ratio */
  LL_ADC_ConfigureDSDataOutput(ADC, USER_DATAWIDTH, USER_RATIO);

  /* Configure the operation mode as ADC mode (static/low frequency signal) */
  LL_ADC_SetADCMode(ADC, LL_ADC_OP_MODE_ADC);

  /* Set the first (and only in this example) input of the conversion sequence as temperature sensor */
  LL_ADC_SetChannelSeq0(ADC, LL_ADC_CH_TEMPERATURE_SENSOR);

  /* Set the length of the conversion sequence as 1 */
  LL_ADC_SetSequenceLength(ADC, LL_ADC_SEQ_LEN_01);

  /* Enable the temperature sensor */
  LL_PWR_EnableTempSens();
  
  /* Set the GAIN */
#if (USE_CALIBRATION==1)
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() != 0xFFF) {
    LL_ADC_SetCalibPoint1Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() );
  }
  else {
    LL_ADC_SetCalibPoint1Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_1V2);
  }
#else
  LL_ADC_SetCalibPoint1Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_1V2);
#endif
}




/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
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
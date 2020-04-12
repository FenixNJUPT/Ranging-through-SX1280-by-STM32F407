/*!
 * \file      main-RangingDemo.c
 *
 * \brief     Main Program body of the Ranging demo.
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Semtech
 */


/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "radio.h"
#include "sx1280.h"
#include "radio.h"
#include "utilities.h"

#include "demoRanging.h"
#include "RangingDisplay.h"

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/****************************************************************************\
 * CONSTANT and MACRO definition
\****************************************************************************/

#define DEMO_SETTING_ENTITY                          MASTER

#define PROXBUFFERSIZE                                  20



#define LED_RX_PIN          GPIO_PIN_0
#define LED_RX_PORT         GPIOC

#define LED_TX_PIN          GPIO_PIN_1
#define LED_TX_PORT         GPIOC



/***************************************************************************\
 * RAM data
\***************************************************************************/

/***************************************************************************\
 * Local function Prototype
\***************************************************************************/

/***************************************************************************\
 * Ranging Demo Main function
\***************************************************************************/

int main( void )
{
	  uint16_t FwVersion=0;
	  HAL_Init();
	  SystemClock_Config();
	  MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    HwInit( );
	
	  GpioWrite(LED_TX_PORT, LED_TX_PIN, 1);
    GpioWrite(LED_RX_PORT, LED_RX_PIN, 1);


    /* 1- Initialize the Ranging Application */
    RangingDemoInitApplication( DEMO_SETTING_ENTITY );

    RangingDemoSetRangingParameters( 40u, DEMO_RNG_ADDR_1, DEMO_RNG_ANT_1, DEMO_RNG_UNIT_SEL_M );
    RangingDemoSetRadioParameters( LORA_SF6, LORA_BW_1600, LORA_CR_4_5, DEMO_CENTRAL_FREQ_PRESET2, DEMO_POWER_TX_MAX );
	
	  Radio.Reset();
	  FwVersion = Radio.GetFirmwareVersion();
		
	  if(DEMO_SETTING_ENTITY == MASTER)
	  {
		    printf("Ranging Demo as Master , firmware %d \n\r",FwVersion);
	  }
	  else
	  {
		    printf("Ranging Demo as Slave , firmware %d  \n\r",FwVersion);
	  }
	
	  GpioWrite(LED_TX_PORT, LED_TX_PIN, 0);
    GpioWrite(LED_RX_PORT, LED_RX_PIN, 0);

    /* Infinite loop */
    while(1)
		{
        RangingDemoStatus_t demoStatus;
		    GpioWrite(LED_TX_PORT, LED_TX_PIN, 1);
        // Run the ranging demo.
        do{
            demoStatus = RangingDemoRun( );
        }while( demoStatus == DEMO_RANGING_RUNNING );

		    GpioWrite(LED_TX_PORT, LED_TX_PIN, 0);
        // If master, display the ranging result.
        if( DEMO_SETTING_ENTITY == MASTER )
				{
            RangingDisplayUartOutputData( );
            RangingDisplayUartOutputDistance( );
			      HAL_Delay(1000);
        }

        if( demoStatus != DEMO_RANGING_TERMINATED )
				{
            RangingDemoReset( );
        }

    }
}

/***************************************************************************\
 * Local Function Definition
\***************************************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


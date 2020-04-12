/*!
 * \file      hw-spi.c
 *
 * \brief     SPI driver implementation
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
#include "spi.h"
#include "hw.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef RadioSpiHandle;
volatile bool blockingDmaFlag;

/* SPI1 init function */
void SpiInit(void)
{
	MX_SPI1_Init();
	RadioSpiHandle = hspi1;
}

void SpiDeInit( void )
{
    HAL_SPI_DeInit( &RadioSpiHandle );
}

#define WAIT_FOR_BLOCKING_FLAG         while( blockingDmaFlag ) { }
/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
	#ifdef STM32L4XX_FAMILY
    	HAL_SPIEx_FlushRxFifo( &RadioSpiHandle ); // Comment For STM32L0XX and STM32L1XX Intégration, uncomment for STM32L4XX Intégration 
	#endif
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_TransmitReceive_DMA( &SpiHandle, txBuffer, rxBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_TransmitReceive( &RadioSpiHandle, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
    #endif
}

void SpiIn( uint8_t *txBuffer, uint16_t size )
{
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_Transmit_DMA( &SpiHandle, txBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_Transmit( &RadioSpiHandle, txBuffer, size, HAL_MAX_DELAY );
    #endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

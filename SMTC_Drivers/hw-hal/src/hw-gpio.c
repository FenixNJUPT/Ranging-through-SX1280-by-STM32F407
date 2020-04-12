/*!
 * \file      hw-gpio.c
 *
 * \brief     GPIO driver implementation
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
#include "gpio.h"


#define ANT_SW_PIN          GPIO_PIN_0
#define ANT_SW_PORT         GPIOB

#define LED_RX_PIN          GPIO_PIN_0
#define LED_RX_PORT         GPIOC

#define LED_TX_PIN          GPIO_PIN_1
#define LED_TX_PORT         GPIOC


/***************************************************************************\
 * RAM data
\***************************************************************************/
static GpioIrqHandler *gpioIrq[16] = { NULL };

/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

/*!
 * \brief Initializes and configure the GPIO.
 */
void GpioInit(void)
{
    MX_GPIO_Init();
}

/*!
 * \brief DeInitializes the GPIO.
 */
void GpioDeInit( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* DeInit GPIOs which were not set by buses because they are DeInit by the Bus DeInit Function */
	
	/*Configure GPIO pins : LED_RX_Pin */
    GPIO_InitStruct.Pin = LED_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LED_RX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : LED_TX_PIN */
    GPIO_InitStruct.Pin = LED_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LED_TX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : nRESET_Pin */
    GPIO_InitStruct.Pin = RADIO_nRESET_PIN ;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_nRESET_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : RADIO_NSS_Pin */
    GPIO_InitStruct.Pin = RADIO_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_NSS_PORT, &GPIO_InitStruct );
	
	/*Configure GPIO pin : ANT_SW_Pin */
    GPIO_InitStruct.Pin = ANT_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ANT_SW_PORT, &GPIO_InitStruct );
	
	/*Configure GPIO pin : BUSY_Pin */
    GPIO_InitStruct.Pin = RADIO_BUSY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( RADIO_BUSY_PORT, &GPIO_InitStruct );

    /*Configure GPIO pin : DIOx_Pin */
    GPIO_InitStruct.Pin = RADIO_DIOx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( RADIO_DIOx_PORT, &GPIO_InitStruct);

      /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE( );
    __HAL_RCC_GPIOB_CLK_DISABLE( );
    __HAL_RCC_GPIOC_CLK_DISABLE( );
    __HAL_RCC_GPIOD_CLK_DISABLE( );
    __HAL_RCC_GPIOE_CLK_DISABLE( );
    __HAL_RCC_GPIOF_CLK_DISABLE( );
    __HAL_RCC_GPIOG_CLK_DISABLE( );
    __HAL_RCC_GPIOH_CLK_DISABLE( );
}

/*!
 * @brief Records the interrupt handler for the GPIO  object
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] prio       NVIC priority (0 is highest)
 * @param [IN] irqHandler  points to the  function to execute
 * @retval none
 */
void GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
{
    IRQn_Type   IRQnb;
    uint32_t    BitPos = GpioGetBitPos( GPIO_Pin ) ;

    if ( irqHandler != NULL )
    {
		gpioIrq[BitPos] = irqHandler;

        IRQnb = MSP_GetIRQn( GPIO_Pin );

        HAL_NVIC_SetPriority( IRQnb , prio, 0u );

        HAL_NVIC_EnableIRQ( IRQnb );
    }
}

/*!
 * @brief Execute the interrupt from the object
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
void GpioLaunchIrqHandler( uint16_t GPIO_Pin )
{
    uint32_t BitPos = GpioGetBitPos( GPIO_Pin );

    if ( gpioIrq[BitPos]  != NULL )
    {
        gpioIrq[BitPos]( );
    }
}


/*!
 * @brief Get the position of the bit set in the GPIO_Pin
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval the position of the bit
 */
uint8_t GpioGetBitPos( uint16_t GPIO_Pin )
{
    uint8_t pinPos = 0u;

    if ( ( GPIO_Pin & 0xFF00u ) != 0u ){
        pinPos |= 0x8u;
    }
    if ( ( GPIO_Pin & 0xF0F0u ) != 0u ){
        pinPos |= 0x4u;
    }
    if ( ( GPIO_Pin & 0xCCCCu ) != 0u ){
        pinPos |= 0x2u;
    }
    if ( ( GPIO_Pin & 0xAAAAu ) != 0u ){
        pinPos |= 0x1u;
    }

    return pinPos;
}


/*!
 * @brief Writes the given value to the GPIO output
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] value New GPIO output value
 * @retval none
 */
void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t value )
{
    HAL_GPIO_WritePin( GPIOx, GPIO_Pin , ( GPIO_PinState ) value );
}

/*!
 * @brief Reads the current GPIO input value
 *
 * @param  GPIOx: where x can be (A..E and H)
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval value   Current GPIO input value
 */
uint32_t GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    return HAL_GPIO_ReadPin( GPIOx, GPIO_Pin );
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  GpioLaunchIrqHandler( GPIO_Pin );
}

/**
  * @brief  Gets IRQ number as a finction of the GPIO_Pin.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval IRQ number
  */
IRQn_Type MSP_GetIRQn( uint16_t GPIO_Pin )
{
  switch( GPIO_Pin )
  {
    case GPIO_PIN_0:  return EXTI0_IRQn;
    case GPIO_PIN_1:  return EXTI1_IRQn;
    case GPIO_PIN_2:  return EXTI2_IRQn;
    case GPIO_PIN_3:  return EXTI3_IRQn;
    case GPIO_PIN_4:  return EXTI4_IRQn;
    case GPIO_PIN_5:  
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:  return EXTI9_5_IRQn;
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15: 
    default: return EXTI15_10_IRQn;
  }
}

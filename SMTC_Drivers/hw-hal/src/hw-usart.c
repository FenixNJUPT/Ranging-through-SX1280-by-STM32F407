/*!
 * \file      hw-usart.c
 *
 * \brief     UART driver implementation
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

/* Includes -----------------------------------------------------------------*/
#include "usart.h"
#include "hw.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar( int ch )
#else
#define PUTCHAR_PROTOTYPE int fputc( int ch, FILE *f )
#endif /* __GNUC__ */

/****************************************************************************\
 * CONSTANT and MACRO definition
\****************************************************************************/

// Defines the mode of the UART driver, using Control Flow or not.
#define UART_CONTROLFLOW_MODE                                           0
#define UART_NOCONTROLFLOW_MODE                                         1
#define UART_MODE                                   UART_NOCONTROLFLOW_MODE

// Defines the command start character
#define UART_RECEIVE_START_CHARACTER                                0x23u
// Defines the size of the UART buffer.
#define UART_BUFFER_SIZE                                              100

#define UART_RX_START_RECEIVE_LEN                                       1

typedef enum
{
    UART_STATE_RESET,
    UART_STATE_IDLE,
    UART_STATE_SENDING,
    UART_STATE_READYTORECEIVE,
    UART_STATE_RECEIVING_GETLEN,
    UART_STATE_RECEIVING_GETOPCODE,
    UART_STATE_RECEIVING_GETPARAM
}UartState_t;

/***************************************************************************\
 * RAM data
\***************************************************************************/
// Global UART handle, status and callback.
UART_HandleTypeDef  gUartHandle;
uint8_t             gUartState;
UartCallback        gUartCallback;

// Transmit and receive information.
uint16_t            txUartBufferIndex;
uint8_t             rxUartBufferIndex;
uint8_t             rxUartCommandLen;
// The UART buffer used for transmitting and receiving data.
uint8_t             uartBuffer[UART_BUFFER_SIZE];

/***************************************************************************\
 * Local function Prototype
\***************************************************************************/

static void UartReceiveComplete( void );

/***************************************************************************\
 * External Function Prototype
\***************************************************************************/

/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

/*!
 * \brief Initializes the UART variables and peripheral
 *
 * \param [in] uartCallback  The UART Callback.
 *
 * \retval None
 */
void UartInit( UartCallback uartCallback ){

    MX_USART1_UART_Init();
    txUartBufferIndex = 0u;
    gUartCallback = uartCallback;
	gUartState = UART_STATE_IDLE;
}

/*!
 * \brief De-initializes the UART peripheral.
 *
 * \retval None
 */
void UartDeInit( void ){

	HAL_UART_DeInit( &huart1 );
    gUartState = UART_STATE_RESET;
}

/*!
 * \brief Enable or Disable UART flow control
 *
 * \param [in] uartCallback  The UART Callback.
 * \param [in] enable  Enable or Disable Flow Control.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartEnableFlowControl( UartCallback uartCallback, bool enable ){

	if (enable == UART_CONTROLFLOW_MODE )
	{
		huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
	}
	else //(enable == UART_NOCONTROLFLOW_MODE )
	{
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	}
	
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		return FAIL;
	}
	return SUCCESS;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit( &huart1, ( uint8_t * )&ch, 1, 0xFFFF );

  return ch;
}

/*!
 * \brief Sends the amount of data given in parameter in the data buffer.
 *          Once the data transmitted the UARTEVENT_TXCOMPLETE event is sent
 *           on the callback.
 *
 * \param [in] uartTxBuffer     A pointer on the buffer to transmit.
 * \param [in] uartTxBufferLen  The length of the buffer.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartSend( uint8_t *uartTxBuffer, uint16_t uartTxBufferLen ){

    // Check the UART state
    if( (gUartState == UART_STATE_IDLE) || (gUartState == UART_STATE_READYTORECEIVE) ){
        // Transmit the data in parameter
        if( HAL_UART_Transmit_IT(&huart1, uartTxBuffer, uartTxBufferLen) != HAL_OK ){
            return FAIL;
        }
        gUartState = UART_STATE_SENDING;
    }else if( gUartState == UART_STATE_SENDING ){
        // Check the buffer can store the data to transmit.
        if( (txUartBufferIndex + uartTxBufferLen) <= UART_BUFFER_SIZE ){
            // Save the data and transmit once the previous data are sent
            memcpy( &uartBuffer[txUartBufferIndex], uartTxBuffer, uartTxBufferLen );
            txUartBufferIndex += uartTxBufferLen;
        }else{
            return FAIL;
        }
    }else{
		 return FAIL;
    }
    return SUCCESS;
}

#ifdef STM32L4XX_FAMILY
/*!
 * \brief Stop the sending operation. The operation is done when the
 *          UARTEVENT_TXCOMPLETE event is received on the callback.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartStopSending( void ){

    //If UART state is in sending state, abort transmitting operation.
    if( gUartState == UART_STATE_SENDING ){
        if( HAL_UART_AbortTransmit_IT(&huart1) != HAL_OK ){
            return FAIL;
        }
    }
    return SUCCESS;
}
#endif

/*!
 * \brief Receives data on UART and call the callback function to
 *          handle the data received along the UARTEVENT_RXCOMPLETE event.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartReceive( void ){

    if( gUartState == UART_STATE_IDLE ){
        // Ready to receive UART Command.
        rxUartBufferIndex = 0u;
        if( HAL_UART_Receive_IT(&huart1, &uartBuffer[0], UART_RX_START_RECEIVE_LEN) != HAL_OK ){
            return FAIL;
        }
        gUartState = UART_STATE_READYTORECEIVE;
    }else if( gUartState == UART_STATE_READYTORECEIVE ){
        return SUCCESS;
    }else{
        return FAIL;
    }
    return SUCCESS;
}

#ifdef STM32L4XX_FAMILY
/*!
 * \brief Stops the receiving operation. The operation is done when the
 *          UARTEVENT_RXCOMPLETE event is received on the callback.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartStopReceiving( void ){

    //If UART state is in sending state, abort transmitting operation.
    if( (gUartState == UART_STATE_READYTORECEIVE) ||
 #if (UART_MODE == UART_CONTROLFLOW_MODE )
        (gUartState == UART_STATE_RECEIVING_GETLEN) ||
        (gUartState == UART_STATE_RECEIVING_GETOPCODE) ||
 #endif //(UART_MODE == UART_CONTROLFLOW_MODE )
        (gUartState == UART_STATE_RECEIVING_GETPARAM) ){

        if( HAL_UART_AbortReceive_IT(&huart1) != HAL_OK ){
            return FAIL;
        }
    }
    return SUCCESS;
}
#endif
/***************************************************************************\
 * External Function Definition
\***************************************************************************/

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uartHandle)
{
    /* Check if there are more data to send */
    if( txUartBufferIndex != 0 ){

        if( HAL_UART_Transmit_IT(uartHandle, &uartBuffer[0], txUartBufferIndex) != HAL_OK ){
            //Stop sending if failed.
            gUartState = UART_STATE_IDLE;
            gUartCallback( UARTEVENT_TXCOMPLETE, FAIL, 0 );
        }
        txUartBufferIndex = 0u;
    }else{
        gUartState = UART_STATE_IDLE;
        gUartCallback( UARTEVENT_TXCOMPLETE, SUCCESS, 0 );
    }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
    UartReceiveComplete( );
}

#ifdef STM32L4XX_FAMILY
/**
  * @brief  UART Abort Complete callback.
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *uartHandle){

    gUartState = UART_STATE_IDLE;
    // The transmitting operation has been aborted, return TX complete with
    // fail status.
    gUartCallback( UARTEVENT_TXCOMPLETE, FAIL, 0 );

}

/**
  * @brief  UART Abort Receive Complete callback.
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *uartHandle){

    gUartState = UART_STATE_IDLE;
    //The receiving operation has been aborted, return RX complete with
    // fail status.
    gUartCallback( UARTEVENT_RXCOMPLETE, FAIL, 0 );

}
#endif
/***************************************************************************\
 * Local Function Definition
\***************************************************************************/

static void UartReceiveComplete( void ){

    bool        continueReceiving = true;
    uint16_t    uartRxLen = 1u;

    switch( gUartState ){
        case UART_STATE_READYTORECEIVE:
        {
            if( uartBuffer[0] == UART_RECEIVE_START_CHARACTER ){  // byte_0 == UART_RECEIVE_START_CHARACTER
                rxUartBufferIndex++;
                gUartState = UART_STATE_RECEIVING_GETLEN;
            }
        }
        break;

        case UART_STATE_RECEIVING_GETLEN:
        {
            rxUartCommandLen = uartBuffer[rxUartBufferIndex];
            //Check the Command Length is not longer than the size of the buffer.
            if( rxUartCommandLen < (UART_BUFFER_SIZE - 2u) ){
                rxUartBufferIndex++;
                gUartState = UART_STATE_RECEIVING_GETOPCODE;
            }else{
                // Not enougth memory to receive the command
                // Stop receiving.
                gUartState = UART_STATE_IDLE;
                continueReceiving = false;
                gUartCallback( UARTEVENT_RXCOMPLETE, FAIL, 0 );
            }
        }
        break;

        case UART_STATE_RECEIVING_GETOPCODE:
        {
            rxUartBufferIndex++;
            if( rxUartCommandLen == 0u ){
                //No Command parameter, Command fully received.
                UartData uartParameter;

                gUartState = UART_STATE_IDLE;
                continueReceiving = false;
                uartParameter.ptrData = &uartBuffer[0];
                uartParameter.dataLen = rxUartBufferIndex;
                gUartCallback( UARTEVENT_RXCOMPLETE, SUCCESS, &uartParameter );
            }else{
                // Continue to receive the Command parameter.
                gUartState = UART_STATE_RECEIVING_GETPARAM;
                uartRxLen = rxUartCommandLen;
            }
        }
        break;

        case UART_STATE_RECEIVING_GETPARAM:
        {
            //Stop receiving and add request to handle the command
            UartData uartParameter;

            rxUartBufferIndex += rxUartCommandLen;
            gUartState = UART_STATE_IDLE;
            continueReceiving = false;
            uartParameter.ptrData = &uartBuffer[0];
            uartParameter.dataLen = rxUartBufferIndex;
            gUartCallback( UARTEVENT_RXCOMPLETE, SUCCESS, &uartParameter );

        }
        break;

        default:
        {
            continueReceiving = false;
        }
        break;
    }

    // Continue to receive data if needed.
    if( continueReceiving == true ){
        if( HAL_UART_Receive_IT(&huart1, &uartBuffer[rxUartBufferIndex], uartRxLen) != HAL_OK ){
            gUartState = UART_STATE_IDLE;
        }
    }
}

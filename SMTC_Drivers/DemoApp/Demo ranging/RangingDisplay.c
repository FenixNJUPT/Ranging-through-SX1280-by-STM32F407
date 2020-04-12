/*!
 * \file      RangingDisplay.c
 *
 * \brief     Ranging output display implementation.
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
#include "demoRanging.h"
#include "RangingDisplay.h"

/****************************************************************************\
 * CONSTANT and MACRO definition
\****************************************************************************/

/*!
 * \brief Defines the distance threshold for LED display
 */
#define DISTANCE_THRESHOLD_1                           2.0
#define DISTANCE_THRESHOLD_2                           4.0
#define DISTANCE_THRESHOLD_3                           6.0
#define DISTANCE_THRESHOLD_4                           8.0
#define DISTANCE_THRESHOLD_5                          10.0

/*!
 * \brief Defines the size of the uart buffer
 */
#define UART_STRING_LEN                                100

/*!
 * \brief Defines the display states
 */
#define RANGING_DISPLAY_STATE_IDLE                       0
#define RANGING_DISPLAY_STATE_CONFIG                     1
#define RANGING_DISPLAY_STATE_RESULT_HEADER              2
#define RANGING_DISPLAY_STATE_RESULT_RAW                 3
#define RANGING_DISPLAY_STATE_RESULT_RANGING             4

/***************************************************************************\
 * RAM data
\***************************************************************************/

static uint8_t          resultDisplaying;
static uint8_t          rangingResultIndex;
static DemoResult_t    *demoResult;

/***************************************************************************\
 * Local function Prototype
\***************************************************************************/

static char *RangingDisplayGetSpreadingFactor( uint8_t spreadingFactor );

static char *RangingDisplayGetBandwidth( uint8_t bandwidth );

static void RangingDisplayUartOutputResultHeader( void );

static void RangingDisplayUartOutputResultRaw( void );

static void RangingDisplayUartOutputStart( void );

static void RangingDisplayUartOutputContinue( void );

static uint32_t RangingDisplayGetRangingResult( double rangingResult );

/***************************************************************************\
 * External Function Definition
\***************************************************************************/



/*!
 * \brief Display the Ranging Demo results over UART.
 */
void RangingDisplayUartOutputData( void )
{
    RangingDisplayUartOutputStart( );

    while( resultDisplaying != RANGING_DISPLAY_STATE_IDLE ){

        RangingDisplayUartOutputContinue();
    }

}

/*!
 * \brief Display the Ranging Demo distance and RSSI over UART.
 */
void RangingDisplayUartOutputDistance( void )
{
    demoResult = RangingDemoGetResult();
    if( demoResult->RngResultIndex != 0u ){
		printf("Distance Ranging = %f, RSSI = %d \r\n", demoResult->RngDistance,demoResult->RssiValue );
        //printf("Distance Ranging = %d, RSSI = %d \r\n", RangingDisplayGetRangingResult(demoResult->RngDistance),demoResult->RssiValue );
    }
}


/***************************************************************************\
 * Local Function Definition
\***************************************************************************/

/*!
 * \brief Display the Ranging setting informations
 */
static void RangingDisplayUartOutputStart( void )
{
    DemoSettings_t *demoSettings;

    demoSettings = RangingDemoGetConfiguration();

	printf("{ \"Ranging\":{ \r\n" );
	//HAL_Delay(2);
    // Display the Spreading Factor (Length : 24 < 100 (Buffer Len))
    printf("\t%s,\r\n", RangingDisplayGetSpreadingFactor( demoSettings->SpreadingFactor ) );
	//HAL_Delay(2);
    // Display the Bandwidth (Length : 38 < 100 (Buffer Len))
    printf("\t%s,\r\n", RangingDisplayGetBandwidth( demoSettings->Bandwidth ) );
	//HAL_Delay(2);
    // Display the Bandwidth (Length : 60 < 100 (Buffer Len))
    printf("\t\"Addr\": \"%08x\",\r\n", demoSettings->RngAddress );
	//HAL_Delay(2);
	
	resultDisplaying = RANGING_DISPLAY_STATE_CONFIG;

}


/*!
 * \brief Display the Ranging Radio setting informations.
 */
static void RangingDisplayUartOutputResultHeader( void )
{
    demoResult = RangingDemoGetResult();

    // Display the Radio RSSI (Length : 13 < 100 (Buffer Len))
    printf("\t\"RSSI\": %d,\r\n", demoResult->RssiValue );
	//HAL_Delay(2);
    // Display the Radio SNR (Length : 25 < 100 (Buffer Len))
    printf("\t\"SNR\": %d,\r\n", demoResult->SnrValue );
	//HAL_Delay(2);
    // Display the Radio FEI (Length : 46 < 100 (Buffer Len))
	printf("\t\"FEI\": \"%08x\",\r\n", (int32_t)demoResult->RngFei );
	//HAL_Delay(2);

	
    if( demoResult->CntPacketRxOK == 0 ){
        // Display the Ranging Result (Length : 86 < 100 (Buffer Len))
        printf("\t\"RngResult\": {\r\n\t\"NbRaws\":%d\r\n\t}\r\n}\r\n}\r\n", (int32_t)demoResult->CntPacketRxOK);
		//HAL_Delay(2);
        resultDisplaying = RANGING_DISPLAY_STATE_IDLE;
    }else{
        // Display the Ranging Result (Length : 98 < 100 (Buffer Len))
        printf("\t\"RngResult\": {\r\n\t\"NbRaws\":%d,\r\n\t\"RawResults\": [\r\n", (int32_t)demoResult->CntPacketRxOK);
		//HAL_Delay(2);
        resultDisplaying = RANGING_DISPLAY_STATE_RESULT_HEADER;
    }
	rangingResultIndex = 0;
}


/*!
 * \brief Display the Ranging Result informations
 */
static void RangingDisplayUartOutputResultRaw( void )
{
    if( rangingResultIndex < (demoResult->RngResultIndex - 1) ){
        printf("\t\t{\"RawRng\": \"%08x\", ", (int32_t)demoResult->RawRngResults[rangingResultIndex] );
		//HAL_Delay(2);
        printf("\"RssiRng\": %d },\r\n", demoResult->RawRngRssi[rangingResultIndex] );
		//HAL_Delay(2);

        resultDisplaying = RANGING_DISPLAY_STATE_RESULT_RAW;
		rangingResultIndex++;

    }else if( rangingResultIndex < demoResult->RngResultIndex ){
        printf("\t\t{\"RawRng\": \"%08x\", ", (int32_t)demoResult->RawRngResults[rangingResultIndex] );
        //HAL_Delay(2);
        printf("\"RssiRng\": %d }\r\n\t],\r\n", demoResult->RawRngRssi[rangingResultIndex] );
        //HAL_Delay(2);
        printf("\t\"DistanceRng\": \"%08x\"\r\n\t}\r\n}\r\n}\r\n", RangingDisplayGetRangingResult(demoResult->RngDistance) );
        //HAL_Delay(2);

        resultDisplaying = RANGING_DISPLAY_STATE_IDLE;
    }else{
        resultDisplaying = RANGING_DISPLAY_STATE_IDLE;
    }
}

/*!
 * \brief State machine to display over UART.
 */
static void RangingDisplayUartOutputContinue( void )
{

    switch( resultDisplaying ){

        case RANGING_DISPLAY_STATE_CONFIG:
            RangingDisplayUartOutputResultHeader( );

        break;

        case RANGING_DISPLAY_STATE_RESULT_HEADER:
            RangingDisplayUartOutputResultRaw( );
        break;

        case RANGING_DISPLAY_STATE_RESULT_RAW:
            RangingDisplayUartOutputResultRaw( );
        break;

        default:
            resultDisplaying = RANGING_DISPLAY_STATE_IDLE;
        break;

    }
}


/*!
 * \brief Get the string information of the Spreading factor
 *
 * \param [in] spreadingFactor   The spreading factor to convert to string.
 *
 * \retval string       The string to display
 */
static char *RangingDisplayGetSpreadingFactor( uint8_t spreadingFactor )
{
    switch( spreadingFactor )
    {
        case LORA_SF5:  return ( char* )"\"SF\": 5";
        case LORA_SF6:  return ( char* )"\"SF\": 6";
        case LORA_SF7:  return ( char* )"\"SF\": 7";
        case LORA_SF8:  return ( char* )"\"SF\": 8";
        case LORA_SF9:  return ( char* )"\"SF\": 9";
        case LORA_SF10: return ( char* )"\"SF\": 10";
        case LORA_SF11: return ( char* )"\"SF\": 11";
        case LORA_SF12: return ( char* )"\"SF\": 12";
        default:        return ( char* )"\"SF\": X";
    }
}

/*!
 * \brief Get the string information of the bandwidth
 *
 * \param [in] bandwidth   The bandwidth to convert to string.
 *
 * \retval string       The string to display
 */
static char *RangingDisplayGetBandwidth( uint8_t bandwidth )
{
    switch( bandwidth )
    {
        case LORA_BW_0200: return ( char* )"\"BW\": 200";
        case LORA_BW_0400: return ( char* )"\"BW\": 400";
        case LORA_BW_0800: return ( char* )"\"BW\": 800";
        case LORA_BW_1600: return ( char* )"\"BW\": 1600";
        default:           return ( char* )"\"BW\": X";
    }
}

/*!
 * \brief Get the formatted ranging result
 *
 * \param [in] rangingResult   The ranging result (double)
 *
 * \retval distance            The distance formatted (uint32_t)
 */
static uint32_t RangingDisplayGetRangingResult( double rangingResult )
{
    return (((uint32_t)rangingResult) << 8u) + (uint32_t)(rangingResult*100 - ((long)rangingResult*100));
}




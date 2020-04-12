#ifndef RANGING_DISPLAY_H
#define RANGING_DISPLAY_H
/*!
 * \file      RangingDisplay.h
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

/****************************************************************************\
 *  Type definition
\****************************************************************************/

/*!
 * \brief Display the Ranging Demo results over UART.
 */
void RangingDisplayUartOutputData( void );

/*!
 * \brief Display the Ranging Demo distance and RSSI over UART.
 */
void RangingDisplayUartOutputDistance( void );

#endif // RANGING_DISPLAY_H

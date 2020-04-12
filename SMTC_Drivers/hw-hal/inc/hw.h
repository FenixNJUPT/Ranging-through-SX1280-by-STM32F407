/*!
 * \file      hw.h
 *
 * \brief     Hardware driver header
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
 
#ifndef __HW_H__
#define __HW_H__

#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* Define the core */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "boardconfig.h"
//HAL
#include "hw-spi.h"
#include "hw-gpio.h"
#include "hw-usart.h"
#include "sx1280.h"
#include "sx1280-hal.h"
#include "utilities.h"


void HwInit( void );

void HwSetLowPower( void );

void SystemClock_Config( void );

void HAL_Delay( uint32_t Delay );

void _Error_Handler(char *, int);

#endif // __HW_H__

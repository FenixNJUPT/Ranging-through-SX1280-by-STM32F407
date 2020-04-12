/*!
 * \file      rf_switch_cg224m6.c
 *
 * \brief     rf switch CG224M6 from CEL driver implementation.
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
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Semtech
 */

#include <stdbool.h>
#include "hw.h"
#include "utilities.h"
#include "rf_switch_cg2214m6.h"


#define ANT_SW_PIN          GPIO_PIN_0
#define ANT_SW_PORT         GPIOB


/***************************************************************************\
 * RAM data
\***************************************************************************/

/***************************************************************************\
 * Local function Prototype
\***************************************************************************/

/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

/*!
 * \brief Init RF Switch
 */
void RfSwitchInit( void ){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*Configure GPIO pins : Antenna switch pins */
    GPIO_InitStruct.Pin = ANT_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ANT_SW_PORT, &GPIO_InitStruct );
}

/*!
 * \brief Deinit RF Switch
 */
void RfSwitchDeinit( void ){
	
	HAL_GPIO_DeInit(ANT_SW_PORT, ANT_SW_PIN);
}

/*!
 * \brief Enable antenna 1 and disable antenna 2
 * \retval None
 */
void SetAntenna1( void ){
	
	HAL_GPIO_WritePin(ANT_SW_PORT, ANT_SW_PIN, GPIO_PIN_SET);		//ANT_SW = 1; // ANT1
}

/*!
 * \brief Enable antenna 2 and disable antenna 1
 * \retval None
 */
void SetAntenna2( void ){
	 
	HAL_GPIO_WritePin(ANT_SW_PORT, ANT_SW_PIN, GPIO_PIN_RESET); 	//ANT_SW = 0; // ANT2
}


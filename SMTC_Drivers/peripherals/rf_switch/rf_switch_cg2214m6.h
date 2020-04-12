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
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Semtech
 */

#ifndef __RF_SWITCH_CG2214M6_H__
#define __RF_SWITCH_CG2214M6_H__

/*!
 * \brief Init RF Switch
 */
void RfSwitchInit( void );

/*!
 * \brief Deinit RF Switch
 */
void RfSwitchDeInit( void );

/*!
 * \brief Enable antenna 1 and disable antenna 2
 * \retval None
 */
void SetAntenna1( void );

/*!
 * \brief Enable antenna 2 and disable antenna 1
 * \retval None
 */
void SetAntenna2( void );

#endif //__RF_SWITCH_CG2214M6_H__

/*!
 * \file      hw-i2c.c
 *
 * \brief     I2C driver implementation
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
#include "i2c.h"

/***************************************************************************\
 * RAM data
\***************************************************************************/
I2c_t	I2c1_Instance;
I2c_t	I2c2_Instance;
static I2cAddrSize  i2cInternalAddrSize;

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 * \param [IN] i2cId  I2C ID
 */
static void I2cMcuInit( I2c_t *obj, I2cId_t i2cId);

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
static void I2cMcuDeInit( I2c_t *obj );

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
static void I2cMcuResetBus( I2c_t *obj );

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of data bytes to write
 *
 * \retval status [SUCCESS, FAIL]
 */
static uint8_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to read
 * \param [IN] size             number of data bytes to read
 *
 * \retval status [SUCCESS, FAIL]
 */
static uint8_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Waits until the given device is in standby mode
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr );


/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 * \param [IN] i2cId  I2C ID
 */
void I2cInit( I2cId_t i2cId )
{
	if(i2cId == I2C_1)
	{
		I2cMcuInit( &I2c1_Instance, i2cId );
		__HAL_RCC_I2C1_CLK_DISABLE( ); // enabled by MX_I2C1_Init()
		MX_I2C1_Init();
		I2c1_Instance.hi2c =  hi2c1;
	}
	// In case of several I2C are defined
//	if(i2cId == I2C_2)
//	{
//		I2cMcuInit( &I2c2_Instance, i2cId );
//		__HAL_RCC_I2C2_CLK_DISABLE( ); // enabled by MX_I2C2_Init()
//		MX_I2C2_Init();
//		I2c2_Instance.hi2c =  hi2c2;
//	}
}

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cDeInit( I2c_t *obj )
{
    I2cMcuDeInit( obj );
}

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cResetBus( I2c_t *obj )
{
    I2cMcuResetBus( obj );
}

/*!
 * \brief Write data to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] data             data to write
 */
uint8_t I2cWrite( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t data )
{

    if( I2cMcuWriteBuffer( obj, deviceAddr, addr, &data, 1u ) == FAIL )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( I2cMcuWriteBuffer( obj, deviceAddr, addr, &data, 1u ) == FAIL )
        {
            return FAIL;
        }
        else
        {
            return SUCCESS;
        }
    }
    else
    {
        return SUCCESS;
    }
}

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of bytes to write
 */
uint8_t I2cWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{

    if( I2cMcuWriteBuffer( obj, deviceAddr, addr, buffer, size ) == FAIL )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( I2cMcuWriteBuffer( obj, deviceAddr, addr, buffer, size ) == FAIL )
        {
            return FAIL;
        }
        else
        {
            return SUCCESS;
        }
    }
    else
    {
        return SUCCESS;
    }
}

/*!
 * \brief Read data from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] data            data to read
 */
uint8_t I2cRead( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *data )
{
    return( I2cMcuReadBuffer( obj, deviceAddr, addr, data, 1 ) );
}

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] buffer          data buffer to read
 * \param [IN] size             number of data bytes to read
 */
uint8_t I2cReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    return( I2cMcuReadBuffer( obj, deviceAddr, addr, buffer, size ) );
}

/***************************************************************************\
 *  PRIVATE functions definition
\***************************************************************************/

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 * \param [IN] i2cId  I2C ID
 */
static void I2cMcuInit( I2c_t *obj, I2cId_t i2cId )
{
    __HAL_RCC_I2C1_CLK_DISABLE( );
    __HAL_RCC_I2C1_CLK_ENABLE( );
    __HAL_RCC_I2C1_FORCE_RESET( );
    __HAL_RCC_I2C1_RELEASE_RESET( );

    i2cInternalAddrSize = I2C_ADDR_SIZE_8;

    obj->I2cId = i2cId;
}

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
static void I2cMcuResetBus( I2c_t *obj )
{
    __HAL_RCC_I2C1_CLK_DISABLE( );
    __HAL_RCC_I2C1_CLK_ENABLE( );
    __HAL_RCC_I2C1_FORCE_RESET( );
    __HAL_RCC_I2C1_RELEASE_RESET( );

	__HAL_RCC_I2C1_CLK_DISABLE( ); // enabled by MX_I2C1_Init()
    MX_I2C1_Init();
}

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
static void I2cMcuDeInit( I2c_t *obj )
{

    HAL_I2C_DeInit( &obj->hi2c );

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE( );
}

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of data bytes to write
 *
 * \retval status [SUCCESS, FAIL]
 */
static uint8_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t     writeStatus = FAIL;
    uint16_t    memAddSize = 0u;

    if( i2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    if( HAL_I2C_Mem_Write( &obj->hi2c, deviceAddr, addr, memAddSize, buffer, size, 2000u ) == HAL_OK ){
        writeStatus = SUCCESS;
    }
    return writeStatus;
}

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to read
 * \param [IN] size             number of data bytes to read
 *
 * \retval status [SUCCESS, FAIL]
 */
static uint8_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t  readStatus = FAIL;
    uint16_t memAddSize = 0u;

    if( i2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    if( HAL_I2C_Mem_Read( &obj->hi2c, deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ){
        readStatus = SUCCESS;
    }

    return readStatus;
}

/*!
 * \brief Waits until the given device is in standby mode
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr )
{
    uint8_t lStatus = FAIL;

    if( HAL_I2C_IsDeviceReady( &obj->hi2c, deviceAddr, 300u, 4096u ) == HAL_OK ){
        lStatus = SUCCESS;
    }

    return lStatus;
}


/***************************************************************************\
 * External Function Definition
\***************************************************************************/
/*!
 * \brief Sets the internal device address size
 *
 * \param [IN] obj              I2C object
 * \param [IN] addrSize         Internal address size
 */
void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
    i2cInternalAddrSize = addrSize;
}

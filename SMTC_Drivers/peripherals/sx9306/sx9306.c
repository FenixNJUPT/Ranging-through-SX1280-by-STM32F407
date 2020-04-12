/*!
 * \file      sx9306.c
 *
 * \brief     SX9306 Capacitive Touch Sensor driver implementation.
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
#include "sx9306.h"
#include "utilities.h"

/***************************************************************************\
 * RAM data
\***************************************************************************/
extern I2c_t	I2c1_Instance;
// The I2C address of the SX9306.
static uint8_t  i2cDeviceAddr;
// Buffer used for reading data with I2C
static uint8_t  readDataBuffer[2u];
// Buffer used for writing data with I2C
static uint8_t  writeDataBuffer[2u];

/***************************************************************************\
 * Local function Prototype
\***************************************************************************/

static bool SX9306Proximity_WriteRegister(
    uint8_t          registerAddr,
    uint8_t         *data,
    uint8_t          size );

static bool SX9306Proximity_ReadRegister(
    uint8_t          registerAddr,
    uint8_t         *data,
    uint8_t          size );

/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

 /*!
 * \brief Initializes the hardware and variables associated with the SX9306.
 */
void SX9306ProximityInit( void ){

    bool i2cResult;

    // Set the I2C address of the Proximity Sensor
    i2cDeviceAddr = SX9306PROXIMITY_I2C_ADDR;

    //Clear interrupt flag
    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_IRQ_SRC, \
                                          &readDataBuffer[0], 1u );

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_IRQMASK;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_IRQ_MASK, \
                                          &writeDataBuffer[0], 1u);
    }

    if( i2cResult == SUCCESS ){
        //Enable sensor 0
        writeDataBuffer[0] = SX9306_SENSOR_EN_0;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_0, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_1;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_1, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_2;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_2, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_3;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_3, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_4;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_4, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_5;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_5, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_6;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_6, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_7;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_7, \
                                          &writeDataBuffer[0], 1u );
    }

    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = SX9306_DEFAULT_CTRL_8;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_8, \
                                          &writeDataBuffer[0], 1u );
    }
}

 /*!
 * \brief Perform a software reset of the SX9306 chip.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityReset( void ){

    uint8_t resetValue = SX9306PROXIMITY_SW_RESET;

    // Write the reset value in the Reset register.
    return SX9306Proximity_WriteRegister( SX9306_REG_RESET, &resetValue, 1u );
}

 /*!
 * \brief Get the current Sensor value. The value indicates the proximity data
 *        provided in parameter.
 *
 * \param [in] proximityData  The proximity data to read.
 *
 * \retval      Value         The value read.
 */
uint16_t SX9306ProximityGetSensorValue( uint8_t proximityData ){

    bool        i2cResult;
    uint16_t    retVal = 0u;

    //Enable Sensor 0
    writeDataBuffer[0] = SX9306_SENSOR_SEL_0;
    i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_SENSORSEL, \
                                        &writeDataBuffer[0], 1u );

    if( i2cResult == SUCCESS ){
        // Get the Proximity Sensor value
        i2cResult = SX9306Proximity_ReadRegister( proximityData, \
                                         &readDataBuffer[0], 2u );

        if( i2cResult == SUCCESS ){
            retVal = (readDataBuffer[1] + (readDataBuffer[0]<<8u));
            if( proximityData == SX9306_DATA_PROXDIFF ){
                if( (retVal & 0x1000u) != 0u ){
                    // ProxDiff is a 12bits signed value.
                    // If the value is negative, return 0.
                    retVal = 0u;
                }
            }
        }
    }

    return retVal;
}


 /*!
 * \brief Set the proximity detection threshold. Low values allow good
 *        sensitivity/distance while higher values allow better noise
 *        immunity.
 *
 * \param [in] thresholdValue  The proximity detection threshold
*                              Value should be set between 0 (threshold : 0)
*                              and 31 (threshold : 1700).
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetDetectionThreshold( uint8_t thresholdValue )
{

    //Check that the value is valid.
    if( thresholdValue < 32u )
    {
        writeDataBuffer[0] = thresholdValue;
        return SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_6, \
                                          &writeDataBuffer[0], 1u );
    }else{
        return FAIL;
    }
}

 /*!
 * \brief Get the proximity detection threshold.
 *
 * \param [out] thresholdValue  The proximity detection threshold.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDetectionThreshold( uint8_t *thresholdValue )
{

    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_6, \
                                          &readDataBuffer[0], 1u );

    if( thresholdValue != (uint8_t*) 0){
        *thresholdValue = readDataBuffer[0];
    }

    return i2cResult;
}

 /*!
 * \brief Set the scan period. Low values will allow fast reaction time
 *        while high values will provide low power consumption.
 *
 * \param [in] periodValue  The active scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetScanPeriod( uint8_t periodValue )
{
    writeDataBuffer[0] = ( periodValue << 4u ) | SX9306_SENSOR_EN_0;
    return SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_0, \
                                          &writeDataBuffer[0], 1u );
}

 /*!
 * \brief Get the scan period.
 *
 * \param [out] periodValue  The active scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetScanPeriod( uint8_t *periodValue )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_0, \
                                          &readDataBuffer[0], 1u );

    if( periodValue != (uint8_t*) 0){
        *periodValue = ( readDataBuffer[0] >> 4u ) & 0x7u ;
    }

    return i2cResult;
}

 /*!
 * \brief Set the capacitance range.
 *
 * \param [in]  range         The typical full scale input capacitance range.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetRange( uint8_t range )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_1, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( readDataBuffer[0] & 0xFC ) | range ;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_1, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the capacitance range.
 *
 * \param [out]  range      The typical full scale input capacitance range.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetRange( uint8_t *range )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_1, \
                                          &readDataBuffer[0], 1u );

    if( range != (uint8_t*) 0){
        *range = ( readDataBuffer[0] & 0x3u );
    }

    return i2cResult;
}

 /*!
 * \brief Set the digital gain factor, the sampling frequency and the
 *        capacitance measurement resolution.
 *
 * \param [in] gain             The digital gain factor
 * \param [in] samplingFreq     The samling frequency
 * \param [in] resolution       The capacitance measurement resolution
 *                              value from 0 (Coarsest) to 7 (finest).
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetGainAndGranularity( uint8_t gain, uint8_t samplingFreq, uint8_t resolution )
{
    writeDataBuffer[0] = ( gain << 5u ) | ( samplingFreq << 3u ) | resolution;
    return SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_2, \
                                          &writeDataBuffer[0], 1u );
}

 /*!
 * \brief Get the digital gain factor, the sampling frequency and the
 *        capacitance measurement resolution.
 *
 * \param [out] gain             The digital gain factor
 * \param [out] samplingFreq     The samling frequency
 * \param [out] resolution       The capacitance measurement resolution
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetGainAndGranularity( uint8_t *gain, uint8_t *samplingFreq, uint8_t *resolution )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_2, \
                                          &readDataBuffer[0], 1u );

    if( gain != (uint8_t*) 0){
        *gain = ( readDataBuffer[0] >> 5u ) & 0x3u;
    }
    if( samplingFreq != (uint8_t*) 0){
        *samplingFreq = ( readDataBuffer[0] >> 3u ) & 0x3u;
    }
    if( resolution != (uint8_t*) 0){
        *resolution = readDataBuffer[0] & 0x7u;
    }

    return i2cResult;
}

 /*!
 * \brief Configure the doze mode and set the doze period.
 *
 * \param [in] enableMode     Enable or Disable Doze mode.
 * \param [in] dozePeriod     The doze scan period. If doze mode disabled,
 *                            the value should be set to 0.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetDozeMode( uint8_t enableMode, uint8_t dozePeriod )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_3, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( enableMode << 6u ) | ( dozePeriod << 4u) | ( readDataBuffer[0] & 0xFu );
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_3, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the doze configuration.
 *
 * \param [out] dozeMode       The doze mode (enabled or disabled).
 * \param [out] dozePeriod     The doze scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDozeMode( uint8_t *dozeMode, uint8_t *dozePeriod )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_3, \
                                          &readDataBuffer[0], 1u );

    if( dozeMode != (uint8_t*) 0){
        *dozeMode = ( readDataBuffer[0] >> 6u ) & 0x1u;
    }
    if( dozePeriod != (uint8_t*) 0){
        *dozePeriod = ( readDataBuffer[0] >> 4u ) & 0x3u;
    }

    return i2cResult;
}

 /*!
 * \brief Set the raw proximity filter strength.
 *
 * \param [in] filterStrength   The raw proximity filter strength
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetProximityFilter( uint8_t filterStrength )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_3, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( readDataBuffer[0] & 0xFCu ) | filterStrength;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_3, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the raw proximity filter strength.
 *
 * \param [out] filterStrength   The raw proximity filter strength
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetProximityFilter( uint8_t *filterStrength )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_3, \
                                          &readDataBuffer[0], 1u );

    if( filterStrength != (uint8_t*) 0){
        *filterStrength = readDataBuffer[0] & 0x3u;
    }

    return i2cResult;
}

 /*!
 * \brief Set the positive and negative average thresholds which will trigger
 *        compensation.
 *
 * \param [in] avgThreshold   The positive and negative average thresholds.
 *                            Thresholds = +/- 128x avgThreshold
 *                            Should not be set below 0x40.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetAverageThreshold( int8_t avgThreshold )
{
    writeDataBuffer[0] = (uint8_t)avgThreshold;
    return SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_4, \
                                          &writeDataBuffer[0], 1u );
}

 /*!
 * \brief Get the positive and negative average thresholds which will trigger
 *        compensation.
 *
 * \param [out] avgThreshold   The positive and negative average thresholds.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetAverageThreshold( int8_t *avgThreshold )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_4, \
                                          &readDataBuffer[0], 1u );

    if( avgThreshold != (int8_t*) 0 ){
        *avgThreshold = (int8_t)readDataBuffer[0];
    }

    return i2cResult;
}

 /*!
 * \brief Set the average debouncer applied to the average threshold and
 *        the average negative and positive filters strength.
 *        Set the value to avgPosFilter>avgNegFilter to filter out (abnormal)
 *        negative events faster.
 *
 * \param [in] avgNegFilter     The average negative filter strength.
 *                              Value from 0 (Off), 1 (Lowest) to 7 (Highest).
 * \param [in] avgPosFilter     The average positive filter strength.
 *                              Value from 0 (Off), 1 (Lowest) to 7 (Highest).
 * \param [in] avgDebouncer     The average debouncer.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetAverageFilterAndDebouncer( uint8_t avgNegFilter, uint8_t avgPosFilter, uint8_t avgDebouncer )
{
    writeDataBuffer[0] = ( avgDebouncer << 6u ) | ( avgNegFilter << 3u ) | avgPosFilter;
    return SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_5, \
                                          &writeDataBuffer[0], 1u );
}

 /*!
 * \brief Get the average debouncer applied to the average threshold and
 *        the average negative and positive filters strength.
 *
 * \param [out] avgNegFilter     The average negative filter strength.
 * \param [out] avgPosFilter     The average positive filter strength.
 * \param [out] avgDebouncer     The average debouncer.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetAverageFilterAndDebouncer( uint8_t *avgNegFilter, uint8_t *avgPosFilter, uint8_t *avgDebouncer )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_5, \
                                          &readDataBuffer[0], 1u );

    if( avgDebouncer != (uint8_t*) 0){
        *avgDebouncer = ( readDataBuffer[0] >> 6u ) & 0x3u;
    }
    if( avgNegFilter != (uint8_t*) 0){
        *avgNegFilter = ( readDataBuffer[0] >> 3u ) & 0x7u;
    }
    if( avgPosFilter != (uint8_t*) 0){
        *avgPosFilter = readDataBuffer[0] & 0x7u;
    }

    return i2cResult;
}

 /*!
 * \brief Enable the automatic compensation triggered by the average
 *        threshold.
 *
 * \param [in] enableCompensation   Enable or disable the automatic
 *                                  compensation.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityEnableCompensation( uint8_t enableCompensation )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( enableCompensation << 7u ) | ( readDataBuffer[0] & 0x7Fu );
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_7, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the compensation mode.
 *
 * \param [out] compensation   Compensation mode (enabled or disabled)
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetCompensation( uint8_t *compensation )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );

    if( compensation != (uint8_t*) 0){
        *compensation = ( readDataBuffer[0] >> 7u ) & 0x1u;
    }

    return i2cResult;
}

 /*!
 * \brief Defines the proximity detection hysteresis applied to proximity
 *        threshold.
 *
 * \param [in] hystValue    The proximity detection hysteresis.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetHysteresis( uint8_t hystValue )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( hystValue << 4u ) | ( readDataBuffer[0] & 0xCFu );
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_7, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the proximity detection hysteresis applied to proximity
 *        threshold.
 *
 * \param [out] hystValue    The proximity detection hysteresis.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetHysteresis( uint8_t *hystValue )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );

    if( hystValue != (uint8_t*) 0){
        *hystValue = ( readDataBuffer[0] >> 4u ) & 0x3u;
    }

    return i2cResult;
}

 /*!
 * \brief Set the close and the far debouncers applied to the proximity
 *        threshold.
 *
 * \param [in] closeDebouncer   The Close debouncer
 * \param [in] farDebouncer     The Far debouncer
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetDebouncer( uint8_t closeDebouncer, uint8_t farDebouncer )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( readDataBuffer[0] & 0xF0u ) | ( closeDebouncer << 2u ) | farDebouncer;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_7, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the close and the far debouncers applied to the proximity
 *        threshold.
 *
 * \param [out] closeDebouncer   The Close debouncer
 * \param [out] farDebouncer     The Far debouncer
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDebouncer( uint8_t *closeDebouncer, uint8_t *farDebouncer )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_7, \
                                          &readDataBuffer[0], 1u );

    if( farDebouncer != (uint8_t*) 0){
        *farDebouncer = readDataBuffer[0] & 0x3u;
    }
    if( closeDebouncer != (uint8_t*) 0){
        *closeDebouncer = ( readDataBuffer[0] >> 2u ) & 0x3u;
    }

    return i2cResult;
}

 /*!
 * \brief Set the stuck timeout. The stuck is a compensation can be
 *        automatically requested if it is detected that the proximity
 *        “Close” state lasts longer than stuck timeout.
 *
 * \param [in] stuckValue     The proximity “stuck” timeout
 *                            Value between 0 (Off) and 15.
 *                   - stuckValue in [1-3] : STUCK = stuckValue x 64 samples
 *                   - stuckValue in [4-7] : STUCK = stuckValue x 128 samples
 *                   - stuckValue in [8-15] : STUCK = stuckValue x 256 samples
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetStuckTimeout( uint8_t stuckValue )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_8, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( stuckValue << 4u )| ( readDataBuffer[0] & 0x0Fu );
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_8, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the stuck timeout.
 *
 * \param [out] stuckValue     The proximity “stuck” timeout
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetStuckTimeout( uint8_t *stuckValue )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_8, \
                                          &readDataBuffer[0], 1u );

    if( stuckValue != (uint8_t*) 0){
        *stuckValue = ( readDataBuffer[0] >> 4 ) & 0xFu;
    }

    return i2cResult;
}

 /*!
 * \brief Set the periodic compensation interval.
 *
 * \param [in] cmpPeriod      The periodic compensation interval.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetCompensationPeriod( uint8_t cmpPeriod )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_8, \
                                          &readDataBuffer[0], 1u );
    if( i2cResult == SUCCESS ){
        writeDataBuffer[0] = ( readDataBuffer[0] & 0xF0u ) | cmpPeriod;
        i2cResult = SX9306Proximity_WriteRegister( SX9306_REG_CONTROL_8, \
                                              &writeDataBuffer[0], 1u );
    }

    return i2cResult;
}

 /*!
 * \brief Get the periodic compensation interval.
 *
 * \param [out] cmpPeriod      The periodic compensation interval.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetCompensationPeriod( uint8_t *cmpPeriod )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_CONTROL_8, \
                                          &readDataBuffer[0], 1u );
    if( cmpPeriod != (uint8_t*) 0){
        *cmpPeriod = readDataBuffer[0] & 0xFu;
    }

    return i2cResult;
}

 /*!
 * \brief Configure the interrupts. It can enable or disable the close, far,
 *        compensation done and conversion interrupts.
 *
 * \param [in] irq      Enable or disable the interrupts.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityEnableIRQ( uint8_t irq )
{
    writeDataBuffer[0] = irq;
    return SX9306Proximity_WriteRegister( SX9306_REG_IRQ_MASK, \
                                          &writeDataBuffer[0], 1u );
}

 /*!
 * \brief Get the interrupt configuration.
 *
 * \param [out] irqMask      Mask of the interrupts enabled.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetIRQMask( uint8_t *irqMask )
{
    bool    i2cResult;

    i2cResult = SX9306Proximity_ReadRegister( SX9306_REG_IRQ_MASK, \
                                          &readDataBuffer[0], 1u );
    if( irqMask != (uint8_t*) 0){
        *irqMask = readDataBuffer[0];
    }

    return i2cResult;
}

 /*!
 * \brief Function to use when an interruption is raised on SX9306_IRQ, it
 *        handles the interrupt by clearing the flag and giving the interrupt
 *        source.
 *
 * \retval      Value         Interrupt Source value
 */
uint8_t SX9306ProximityIRQHandler( void ){

    //Clear interrupt flag
    SX9306Proximity_ReadRegister( SX9306_REG_IRQ_SRC, &readDataBuffer[0], 1u );

    return readDataBuffer[0];
}

/***************************************************************************\
 * Local Function Definition
\***************************************************************************/

static bool SX9306Proximity_ReadRegister( uint8_t registerAddr, uint8_t *data, uint8_t size )
{
    return I2cReadBuffer( &I2c1_Instance, i2cDeviceAddr << 1u, registerAddr, data, size );
}

static bool SX9306Proximity_WriteRegister( uint8_t registerAddr, uint8_t *data, uint8_t size )
{
    return I2cWriteBuffer( &I2c1_Instance, i2cDeviceAddr << 1u, registerAddr, data, size );
}

/***************************************************************************\
 * Test Function Definition
\***************************************************************************/

#define PROXIMITY_AVG_THRESHOLD_VALUE                                     0x80
#define PROXIMITY_RESOLUTION_VALUE                                           7
#define PROXIMITY_AVG_NEG_FILTER_VALUE                                       1
#define PROXIMITY_AVG_POS_FILTER_VALUE                                       7
#define PROXIMITY_PROX_THRESHOLD_VALUE                                      22

uint16_t GetCapacitiveTouchSensor( void ){

	uint16_t value = 0;
    
    // 1- Configure the SX9306 
    SX9306ProximityEnableIRQ( SX9306_IRQ_CLOSEIRQEN );
    SX9306ProximitySetGainAndGranularity( SX9306_GAIN_8, SX9306_FREQ_167KHZ, PROXIMITY_RESOLUTION_VALUE );
    SX9306ProximitySetDozeMode( SX9306_DOZEMODE_DISABLE, 0 );
    SX9306ProximitySetAverageThreshold( PROXIMITY_AVG_THRESHOLD_VALUE );
    SX9306ProximitySetAverageFilterAndDebouncer( PROXIMITY_AVG_NEG_FILTER_VALUE, PROXIMITY_AVG_POS_FILTER_VALUE, SX9306_AVGDEB_OFF );
    SX9306ProximitySetDetectionThreshold( PROXIMITY_PROX_THRESHOLD_VALUE );
	// 2- Get the Proximity Sensor value
	value = SX9306ProximityGetSensorValue( SX9306_DATA_PROXDIFF );

    return value;
}

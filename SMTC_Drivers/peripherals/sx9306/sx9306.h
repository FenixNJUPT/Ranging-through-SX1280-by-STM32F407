#ifndef _SX9306_H
#define _SX9306_H
/*!
 * \file      sx9306.h
 *
 * \brief     SX9306 Capacitive Touch Sensor driver definition.
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


#include <stdint.h>

/****************************************************************************\
 *  Type definition
\****************************************************************************/

#define SX9306PROXIMITY_I2C_ADDR  ( 0x2A ) // Proximity I2C address

#define SX9306PROXIMITY_SW_RESET  ( 0xDE ) // Software Reset value

/*!
 * \brief   SX9306 registers addresses
 */
 //Interrupt & Status registers
#define SX9306_REG_IRQ_SRC         ( 0x00 ) // Interrupt sources
#define SX9306_REG_STATUS          ( 0x01 ) // Status
#define SX9306_REG_IRQ_MASK        ( 0x03 ) // Interrupt mask
// Proximity Sensing Control registers
#define SX9306_REG_CONTROL_0       ( 0x06 ) // Enable and scan period
#define SX9306_REG_CONTROL_1       ( 0x07 ) // Enable Smart SAR, Combined channel feature.
#define SX9306_REG_CONTROL_2       ( 0x08 ) // Resolution, frequence and gain.
#define SX9306_REG_CONTROL_3       ( 0x09 ) // Doze and filter
#define SX9306_REG_CONTROL_4       ( 0x0A ) // Average thresholds
#define SX9306_REG_CONTROL_5       ( 0x0B ) // Average filters and debouncer
#define SX9306_REG_CONTROL_6       ( 0x0C ) // Proximity detection threshold
#define SX9306_REG_CONTROL_7       ( 0x0D ) // Hysteresis and debouncer applied to the Proximity detection threshold
#define SX9306_REG_CONTROL_8       ( 0x0E ) // Compensation interval and Stuck timeout
// Sensor Data Readback registers
#define SX9306_REG_SENSORSEL       ( 0x20 ) // Select which sensor
#define SX9306_REG_PROXUSEFUL      ( 0x21 ) // Instantaneous sensor value
#define SX9306_REG_PROXAVG         ( 0x23 ) // Averaged sensor value
#define SX9306_REG_PROXDIFF        ( 0x25 ) // Diff sensor value
// Software Reset register
#define SX9306_REG_RESET           ( 0x7F ) // Software Reset


/*!
 * \brief Defines the register of the proximity data to read
 */
#define SX9306_DATA_PROXUSEFUL     ( 0x21 ) // Instantaneous sensor value
#define SX9306_DATA_PROXAVG        ( 0x23 ) // Averaged sensor value
#define SX9306_DATA_PROXDIFF       ( 0x25 ) // Diff sensor value (12bits value)
#define SX9306_DATA_PROXOFFSET     ( 0x27 ) // Compensation offset current value.

/*!
 * \brief Mask to enable the interruption
 */
#define SX9306_IRQ_NOIRQ           ( 0x00 ) // Disables interrupts.
#define SX9306_IRQ_CONVDONEIRQEN   ( 0x08 ) // Enables the conversion interrupt.
#define SX9306_IRQ_COMPDONEIRQEN   ( 0x10 ) // Enables the compensation interrupt.
#define SX9306_IRQ_FARIRQEN        ( 0x20 ) // Enables the far interrupt.
#define SX9306_IRQ_CLOSEIRQEN      ( 0x40 ) // Enables the close interrupt.

 /*!
 * \brief Mask to define the interrupt source
 */
#define SX9306_IRQSRC_CONVDONEIRQ  ( 0x08 ) // Enables the conversion interrupt.
#define SX9306_IRQSRC_COMPDONEIRQ  ( 0x10 ) // Enables the compensation interrupt.
#define SX9306_IRQSRC_FARIRQ       ( 0x20 ) // Enables the far interrupt.
#define SX9306_IRQSRC_CLOSEIRQ     ( 0x40 ) // Enables the close interrupt.
#define SX9306_IRQSRC_RESETIRQ     ( 0x80 ) // Enables the reset interrupt.

/*!
 * \brief Mask to enable the Capacitive Sensor pin
 */
#define SX9306_SENSOR_EN_0         ( 0x01 ) // Enable antenna 0 (CS0)
#define SX9306_SENSOR_EN_1         ( 0x02 ) // Enable antenna 1 (CS1)
#define SX9306_SENSOR_EN_2         ( 0x04 ) // Enable antenna 2 (CS2)
#define SX9306_SENSOR_EN_3         ( 0x08 ) // Enable antenna 3 (CS3)

/*!
 * \brief Select the Capacitive Sensor
 */
#define SX9306_SENSOR_SEL_0        ( 0x00 ) // Select antenna 0 (CS0)
#define SX9306_SENSOR_SEL_1        ( 0x01 ) // Select antenna 1 (CS1)
#define SX9306_SENSOR_SEL_2        ( 0x02 ) // Select antenna 2 (CS2)
#define SX9306_SENSOR_SEL_3        ( 0x03 ) // Select antenna 3 (CS3)

/*!
 * \brief Proximity Sensing Control Default Settings
 */
#define SX9306_DEFAULT_IRQMASK     ( 0x00 ) // Default IRQ Mask
#define SX9306_DEFAULT_CTRL_1      ( 0x40 ) // Default range
#define SX9306_DEFAULT_CTRL_2      ( 0x08 ) // Default gain and granularity
#define SX9306_DEFAULT_CTRL_3      ( 0x40 ) // Default doze mode
#define SX9306_DEFAULT_CTRL_4      ( 0x00 ) // Default positive and negative average thresholds
#define SX9306_DEFAULT_CTRL_5      ( 0x00 ) // Default positive filtering and negative filtering strengh and average debouncer.
#define SX9306_DEFAULT_CTRL_6      ( 0x00 ) // Default proximity detection threshold
#define SX9306_DEFAULT_CTRL_7      ( 0x00 ) // Default Compensation configuration, hysteresis value and debouncers
#define SX9306_DEFAULT_CTRL_8      ( 0x00 ) // Default stuck and Compensation period

/*!
 * \brief Defines the Active scan period
 */
#define SX9306_SCANPERIOD_30MS     ( 0x00 )
#define SX9306_SCANPERIOD_60MS     ( 0x01 )
#define SX9306_SCANPERIOD_90MS     ( 0x02 )
#define SX9306_SCANPERIOD_120MS    ( 0x03 )
#define SX9306_SCANPERIOD_150MS    ( 0x04 )
#define SX9306_SCANPERIOD_200MS    ( 0x05 )
#define SX9306_SCANPERIOD_300MS    ( 0x06 )

/*!
 * \brief Defines the typical full scale input capacitance range.
 */
#define SX9306_RANGE_LARGE         ( 0x00 ) // (+/- 7.3pF)
#define SX9306_RANGE_MEDIUM_LARGE  ( 0x01 ) // (+/- 3.7pF)
#define SX9306_RANGE_MEDIUM_SMALL  ( 0x02 ) // (+/- 3.0pF)
#define SX9306_RANGE_SMALL         ( 0x03 ) // (+/- 2.5pF)

/*!
 * \brief Defines the digital gain factor.
 */
#define SX9306_GAIN_OFF            ( 0x00 ) // x1
#define SX9306_GAIN_2              ( 0x01 ) // x2
#define SX9306_GAIN_4              ( 0x02 ) // x4
#define SX9306_GAIN_8              ( 0x03 ) // x8

/*!
 * \brief Defines the sampling frequency.
 */
#define SX9306_FREQ_83KHZ          ( 0x00 ) //83 kHz
#define SX9306_FREQ_125KHZ         ( 0x01 ) //125 kHz
#define SX9306_FREQ_167KHZ         ( 0x02 ) //167 kHz

/*!
 * \brief Disable or enable the doze mode
 */
#define SX9306_DOZEMODE_DISABLE    ( 0x00 )
#define SX9306_DOZEMODE_ENABLE     ( 0x01 )

/*!
 * \brief Defines the Doze scan period.
 */
#define SX9306_DOZEPERIOD_2        ( 0x00 ) //2x SCANPERIOD
#define SX9306_DOZEPERIOD_4        ( 0x01 ) //4x SCANPERIOD
#define SX9306_DOZEPERIOD_8        ( 0x02 ) //8x SCANPERIOD
#define SX9306_DOZEPERIOD_16       ( 0x03 ) //16x SCANPERIOD

/*!
 * \brief Defines Proximity Raw filter strength
 */
#define SX9306_RAWFILTER_OFF       ( 0x00 )
#define SX9306_RAWFILTER_LOW       ( 0x01 )
#define SX9306_RAWFILTER_MEDIUM    ( 0x02 )
#define SX9306_RAWFILTER_HIGH      ( 0x03 )

/*!
 * \brief Defines the average debouncer applied to average threshold.
 */
#define SX9306_AVGDEB_OFF          ( 0x00 ) //Off
#define SX9306_AVGDEB_2            ( 0x01 ) //2 samples
#define SX9306_AVGDEB_4            ( 0x02 ) //4 samples
#define SX9306_AVGDEB_8            ( 0x03 ) //8 samples

/*!
 * \brief Disable or enable the automatic compensation triggered by average
 *        threshold.
 */
#define SX9306_AVGCOMP_DISABLE     ( 0x00 )
#define SX9306_AVGCOMP_ENABLE      ( 0x01 )

/*!
 * \brief Defines the proximity detection hysteresis applied to proximity
 *        threshold.
 */
#define SX9306_HYST_32             ( 0x00 )
#define SX9306_HYST_64             ( 0x01 )
#define SX9306_HYST_128            ( 0x02 )
#define SX9306_HYST_0              ( 0x03 )

/*!
 * \brief Defines the Close debouncer applied to proximity threshold.
 */
#define SX9306_CLOSEDEB_OFF        ( 0x00 ) //Off
#define SX9306_CLOSEDEB_2          ( 0x01 ) //2 samples
#define SX9306_CLOSEDEB_4          ( 0x02 ) //4 samples
#define SX9306_CLOSEDEB_8          ( 0x03 ) //8 samples

/*!
 * \brief Defines the Far debouncer applied to proximity threshold.
 */
#define SX9306_FARDEB_OFF          ( 0x00 ) //Off
#define SX9306_FARDEB_2            ( 0x01 ) //2 samples
#define SX9306_FARDEB_4            ( 0x02 ) //4 samples
#define SX9306_FARDEB_8            ( 0x03 ) //8 samples

/*!
 * \brief Defines the proximity detection threshold
 */
#define SX9306_PROXTHRESH_0        ( 0x00 )
#define SX9306_PROXTHRESH_20       ( 0x01 )
#define SX9306_PROXTHRESH_40       ( 0x02 )
#define SX9306_PROXTHRESH_60       ( 0x03 )
#define SX9306_PROXTHRESH_80       ( 0x04 )
#define SX9306_PROXTHRESH_100      ( 0x05 )
#define SX9306_PROXTHRESH_120      ( 0x06 )
#define SX9306_PROXTHRESH_140      ( 0x07 )
#define SX9306_PROXTHRESH_160      ( 0x08 )
#define SX9306_PROXTHRESH_180      ( 0x09 )
#define SX9306_PROXTHRESH_200      ( 0x0A )
#define SX9306_PROXTHRESH_220      ( 0x0B )
#define SX9306_PROXTHRESH_240      ( 0x0C )
#define SX9306_PROXTHRESH_260      ( 0x0D )
#define SX9306_PROXTHRESH_280      ( 0x0E )
#define SX9306_PROXTHRESH_300      ( 0x0F )
#define SX9306_PROXTHRESH_350      ( 0x10 )
#define SX9306_PROXTHRESH_400      ( 0x11 )
#define SX9306_PROXTHRESH_450      ( 0x12 )
#define SX9306_PROXTHRESH_500      ( 0x13 )
#define SX9306_PROXTHRESH_600      ( 0x14 )
#define SX9306_PROXTHRESH_700      ( 0x15 )
#define SX9306_PROXTHRESH_800      ( 0x16 )
#define SX9306_PROXTHRESH_900      ( 0x17 )
#define SX9306_PROXTHRESH_1000     ( 0x18 )
#define SX9306_PROXTHRESH_1100     ( 0x19 )
#define SX9306_PROXTHRESH_1200     ( 0x1A )
#define SX9306_PROXTHRESH_1300     ( 0x1B )
#define SX9306_PROXTHRESH_1400     ( 0x1C )
#define SX9306_PROXTHRESH_1500     ( 0x1D )
#define SX9306_PROXTHRESH_1600     ( 0x1E )
#define SX9306_PROXTHRESH_1700     ( 0x1F )


/***************************************************************************\
 * External Functions
\***************************************************************************/

 /*!
 * \brief Initializes the hardware and variables associated with the SX9306.
 */
void SX9306ProximityInit( void );


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
uint8_t SX9306ProximitySetDetectionThreshold( uint8_t thresholdValue );

 /*!
 * \brief Set the scan period. Low values will allow fast reaction time
 *        while high values will provide low power consumption.
 *
 * \param [in] periodValue  The active scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetScanPeriod( uint8_t periodValue );

 /*!
 * \brief Set the capacitance range.
 *
 * \param [in]  range         The typical full scale input capacitance range.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetRange( uint8_t range );

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
uint8_t SX9306ProximitySetGainAndGranularity( uint8_t gain, uint8_t samplingFreq, uint8_t resolution );

 /*!
 * \brief Configure the doze mode and set the doze period.
 *
 * \param [in] enableMode     Enable or Disable Doze mode.
 * \param [in] dozePeriod     The doze scan period. If doze mode disabled,
 *                            the value should be set to 0.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetDozeMode( uint8_t enableMode, uint8_t dozePeriod );


 /*!
 * \brief Set the raw proximity filter strength.
 *
 * \param [in] filterStrength   The raw proximity filter strength
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetProximityFilter( uint8_t filterStrength );

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
uint8_t SX9306ProximitySetAverageThreshold( int8_t avgThreshold );


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
uint8_t SX9306ProximitySetAverageFilterAndDebouncer( uint8_t avgNegFilter, uint8_t avgPosFilter, uint8_t avgDebouncer );

 /*!
 * \brief Enable the automatic compensation triggered by the average
 *        threshold.
 *
 * \param [in] enableCompensation   Enable or disable the automatic
 *                                  compensation.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityEnableCompensation( uint8_t enableCompensation );

 /*!
 * \brief Defines the proximity detection hysteresis applied to proximity
 *        threshold.
 *
 * \param [in] hystValue    The proximity detection hysteresis.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetHysteresis( uint8_t hystValue );

 /*!
 * \brief Set the close and the far debouncers applied to the proximity
 *        threshold.
 *
 * \param [in] closeDebouncer   The Close debouncer
 * \param [in] farDebouncer     The Far debouncer
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetDebouncer( uint8_t closeDebouncer, uint8_t farDebouncer );


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
uint8_t SX9306ProximitySetStuckTimeout( uint8_t stuckValue );

 /*!
 * \brief Set the periodic compensation interval.
 *
 * \param [in] cmpPeriod      The periodic compensation interval.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximitySetCompensationPeriod( uint8_t cmpPeriod );

 /*!
 * \brief Configure the interrupts. It can enable or disable the close, far,
 *        compensation done and conversion interrupts.
 *
 * \param [in] irq      Enable or disable the interrupts.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityEnableIRQ( uint8_t irq );

 /*!
 * \brief Get the proximity detection threshold.
 *
 * \param [out] thresholdValue  The proximity detection threshold.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDetectionThreshold( uint8_t *thresholdValue );

 /*!
 * \brief Get the scan period.
 *
 * \param [out] periodValue  The active scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetScanPeriod( uint8_t *periodValue );

 /*!
 * \brief Get the capacitance range.
 *
 * \param [out]  range      The typical full scale input capacitance range.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetRange( uint8_t *range );

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
uint8_t SX9306ProximityGetGainAndGranularity( uint8_t *gain, uint8_t *samplingFreq, uint8_t *resolution );

 /*!
 * \brief Get the doze configuration.
 *
 * \param [out] dozeMode       The doze mode (enabled or disabled).
 * \param [out] dozePeriod     The doze scan period.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDozeMode( uint8_t *dozeMode, uint8_t *dozePeriod );

 /*!
 * \brief Get the raw proximity filter strength.
 *
 * \param [out] filterStrength   The raw proximity filter strength
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetProximityFilter( uint8_t *filterStrength );

 /*!
 * \brief Get the positive and negative average thresholds which will trigger
 *        compensation.
 *
 * \param [out] avgThreshold   The positive and negative average thresholds.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetAverageThreshold( int8_t *avgThreshold );

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
uint8_t SX9306ProximityGetAverageFilterAndDebouncer( uint8_t *avgNegFilter, uint8_t *avgPosFilter, uint8_t *avgDebouncer );

 /*!
 * \brief Get the compensation mode.
 *
 * \param [out] compensation   Compensation mode (enabled or disabled)
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetCompensation( uint8_t *compensation );

 /*!
 * \brief Get the proximity detection hysteresis applied to proximity
 *        threshold.
 *
 * \param [out] hystValue    The proximity detection hysteresis.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetHysteresis( uint8_t *hystValue );

 /*!
 * \brief Get the close and the far debouncers applied to the proximity
 *        threshold.
 *
 * \param [out] closeDebouncer   The Close debouncer
 * \param [out] farDebouncer     The Far debouncer
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetDebouncer( uint8_t *closeDebouncer, uint8_t *farDebouncer );

 /*!
 * \brief Get the stuck timeout.
 *
 * \param [out] stuckValue     The proximity “stuck” timeout
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetStuckTimeout( uint8_t *stuckValue );

 /*!
 * \brief Get the periodic compensation interval.
 *
 * \param [out] cmpPeriod      The periodic compensation interval.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetCompensationPeriod( uint8_t *cmpPeriod );

 /*!
 * \brief Get the interrupt configuration.
 *
 * \param [out] irqMask      Mask of the interrupts enabled.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityGetIRQMask( uint8_t *irqMask );

 /*!
 * \brief Get the current Sensor value. The value indicates the proximity data
 *        provided in parameter.
 *
 * \param [in] proximityData  The proximity data to get.
 *
 * \retval      Value         The value read.
 */
uint16_t SX9306ProximityGetSensorValue( uint8_t proximityData );

 /*!
 * \brief Perform a software reset of the SX9306 chip.
 *
 * \retval      Status        SUCCESS(1) or FAIL(0)
 */
uint8_t SX9306ProximityReset( void );

 /*!
 * \brief Function to use when an interruption is raised on SX9306_IRQ, it
 *        handles the interrupt by clearing the flag and giving the interrupt
 *        source.
 *
 * \retval      Value         Interrupt Source value
 */
uint8_t SX9306ProximityIRQHandler( void );

 /*!
 * \brief Test function configuring the capacitive touch sensor and return
 *        his value.
 *
 * \retval      Value         Value of the capacitive touch
 */
uint16_t GetCapacitiveTouchSensor( void );


#endif //_SX9306_H

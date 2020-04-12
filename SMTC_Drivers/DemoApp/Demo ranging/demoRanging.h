#ifndef DEMO_RANGING_H
#define DEMO_RANGING_H
/*!
 * \file      demoranging.h
 *
 * \brief     Ranging demo implementation
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
 * \brief Defines the type of the demo.
 *  0 : single channel ranging demo, 1 : frequency hopped ranging demo.
 */
#define DEMO_FREQ_HOPPED_RANGING            1 //0


/*!
 * \brief Used to display firmware version on TFT (Utilities menu)
 */
#define FIRMWARE_VERSION    ( ( char* )"Firmware Version: 180718" )

/*!
 * \brief Define min and max ranging channels count
 */
#define DEMO_RNG_CHANNELS_COUNT_MAX          255
#define DEMO_RNG_CHANNELS_COUNT_MIN           10

/*!
 * \brief Define 3 preset central frequencies [Hz]
 */
#define DEMO_CENTRAL_FREQ_PRESET1   2402000000UL
#define DEMO_CENTRAL_FREQ_PRESET2   2450000000UL
#define DEMO_CENTRAL_FREQ_PRESET3   2480000000UL

/*!
 * \brief Define 5 preset ranging addresses
 */
#define DEMO_RNG_ADDR_1             0x10000000
#define DEMO_RNG_ADDR_2             0x32100000
#define DEMO_RNG_ADDR_3             0x20012301
#define DEMO_RNG_ADDR_4             0x20000abc
#define DEMO_RNG_ADDR_5             0x32101230

/*!
 * \brief Define antenna selection for ranging
 */
#define DEMO_RNG_ANT_1              1
#define DEMO_RNG_ANT_2              2
#define DEMO_RNG_ANT_BOTH           0

/*!
 * \brief Define units for ranging distances
 */
#define DEMO_RNG_UNIT_CONV_M        1.0 // not used
#define DEMO_RNG_UNIT_CONV_YD       1.0936
#define DEMO_RNG_UNIT_CONV_MI       6.2137e-4
#define DEMO_RNG_UNIT_SEL_M         0
#define DEMO_RNG_UNIT_SEL_YD        1
#define DEMO_RNG_UNIT_SEL_MI        2

/*!
 * \brief Define min and max Tx power [dBm]
 */
#define DEMO_POWER_TX_MIN           -18
#define DEMO_POWER_TX_MAX           13

typedef enum
{
    DEMO_RANGING_NOT_CONFIGURED,
    DEMO_RANGING_CONFIGURED,
    DEMO_RANGING_RUNNING,
    DEMO_RANGING_TERMINATED,
    DEMO_RANGING_ERROR,
}RangingDemoStatus_t;

/*!
 * \brief Define current demo mode
 */
enum DemoMode
{
    MASTER = 0,
    SLAVE
};

/*!
 * \brief Status of ranging distance
 */
enum RangingStatus
{
    RNG_INIT = 0,
    RNG_PROCESS,
    RNG_VALID,
    RNG_TIMEOUT,
    RNG_PER_ERROR
};

/*!
 * \brief List of states for demo state machine
 */
enum DemoInternalStates
{
    APP_IDLE = 0,               // nothing to do (or wait a radio interrupt)
    APP_RANGING_DONE,
    APP_RANGING_TIMEOUT,
    APP_RANGING_CONFIG,
    APP_RNG,
    APP_RX,                     // Rx done
    APP_RX_TIMEOUT,             // Rx timeout
    APP_RX_ERROR,               // Rx error
    APP_TX,                     // Tx done
    APP_TX_TIMEOUT,             // Tx error
};

/*!
 * \brief Demo Settings structure of Eeprom structure
 */
typedef struct
{
    uint8_t Entity;              // Master or Slave
    uint32_t Frequency;          // Demo frequency
    int8_t TxPower;              // Demo Tx power
	uint8_t SpreadingFactor;     // Demo Spreading factor
	uint8_t Bandwidth;           // Demo Bandwidth
    uint8_t RadioPowerMode;      // Radio Power Mode [0: LDO, 1:DC_DC]
    uint8_t PayloadLength;       // Demo payload length
    uint8_t ModulationType;      // Demo modulation type (LORA, GFSK, FLRC)
    double RngFeiFactor;         // Ranging frequency correction factor
    uint32_t RngAddress;         // Ranging Address
    uint16_t RngFullScale;       // Full range of measuring distance (Ranging)
    uint8_t RngRequestCount;     // Ranging Request Count
    uint8_t RngUnit;             // Ranging distance unit [m]/[mi]
    uint8_t RngStatus;           // Status of ranging distance
    uint8_t RngAntenna;          // Ranging antenna selection
	uint8_t AntennaSwitch;       // Witch antenna connected
    uint16_t RngReqDelay;        // Time between ranging request
    uint16_t RngCalib;           // Ranging Calibration
}DemoSettings_t;

typedef struct{
    uint32_t CntPacketTx;        // Tx packet transmitted
    uint32_t CntPacketRxOK;      // Rx packet received OK
    uint32_t CntPacketRxOKSlave; // Rx packet received OK (slave side)
    uint32_t CntPacketRxKO;      // Rx packet received KO
    uint32_t CntPacketRxKOSlave; // Rx packet received KO (slave side)
    uint16_t RxTimeOutCount;     // Rx packet received KO (by timeout)
    double RngFei;               // Ranging Frequency Error Indicator
    double RngDistance;          // Distance measured by ranging demo
    int8_t RssiValue;            // Demo Rssi Value
    int8_t SnrValue;             // Demo Snr Value (only for LORA mod. type)
    int RngResultIndex;
    double RawRngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
    double RngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
    int8_t RawRngRssi[DEMO_RNG_CHANNELS_COUNT_MAX];
}DemoResult_t;


/***************************************************************************\
 * External Functions
\***************************************************************************/

/*!
 * \brief Initialize the ranging demo.
 *
 * \param [in] demoEntity    The demo entity (either MASTER or SLAVE)
 */
void RangingDemoInitApplication( uint8_t demoEntity );

/*!
 * \brief Set the ranging demo settings.
 *
 * \param [in] nbRequest     The radio to select
 * \param [in] address       The ranging Address
 * \param [in] antenna       The selected antenna
 * \param [in] unitSelect    The selected unit (m, yd or mi)
 */
void RangingDemoSetRangingParameters( uint8_t nbRequest, uint32_t address, uint8_t antenna, uint8_t unitSelect );


/*!
 * \brief Set the ranging radio settings.
 *
 * \param [in] spreadingFactor  The spreading factor value
 * \param [in] bandwidth        The bandwidth value
 * \param [in] codingRate       The coding Rate value
 * \param [in] frequency        The frequency value
 * \param [in] txPower          The TX power value
 */
void RangingDemoSetRadioParameters(
    RadioLoRaSpreadingFactors_t spreadingFactor,
    RadioLoRaBandwidths_t       bandwidth,
    RadioLoRaCodingRates_t      codingRate,
    uint32_t                    frequency,
    int8_t                      txPower );


/*!
 * \brief Run the ranging demo.
 *
 * \retval demoStatus  Indicates the status of the demo.
 */
RangingDemoStatus_t RangingDemoRun( void );

/*!
 * \brief Reset the ranging demo.
 */
void RangingDemoReset( void );

/*!
 * \brief Get the ranging demo settings.
 *
 * \retval DemoSettings   Pointer on the settings.
 */
DemoSettings_t* RangingDemoGetConfiguration( void );

/*!
 * \brief Compute the result distance and get the ranging demo results.
 *
 * \retval DemoResults   Pointer on the results.
 */
DemoResult_t* RangingDemoGetResult( void );

#endif // DEMO_RANGING_H

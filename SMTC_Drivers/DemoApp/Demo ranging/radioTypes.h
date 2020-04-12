#ifndef __RADIO_TYPES_H__
#define __RADIO_TYPES_H__

#include <stdint.h>

typedef enum
{
    STATE_MACHINE_OK,
    STATE_MACHINE_ERROR,
}StateMachineStatus_t;

typedef union RadioIrqFlags
{
    struct Flags                                 //! Structure holding radio IRQs flag
    {
        uint8_t TxDone : 1;                      //!< TxDone interrupt triggered
        uint8_t RxDone : 1;                      //!< TxDone interrupt triggered
        uint8_t TxTimeout : 1;                   //!< Tx timeout interrupt triggered
        uint8_t RxTimeout : 1;                   //!< Rx timeout interrupt triggered
        uint8_t RxError : 1;                     //!< Rx error interrupt triggered
        uint8_t RangingDone : 1;                 //!< Ranging done interrupt triggered
        uint8_t RangingError : 1;                //!< Ranging error interrupt triggered
        uint8_t CadSuccess : 1;
        uint8_t CadFailure : 1;
    }flags;                                      //!< Structure holding every IRQs status
    uint8_t value;                               //!< Raw representation of the IRQs status
}RadioIrqFlags_t;

#endif // __RADIO_TYPES_H__

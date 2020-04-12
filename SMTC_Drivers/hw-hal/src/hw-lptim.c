/*!
 * \file      hw-lptim.c
 *
 * \brief     Timer driver implementation
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

/* Includes -----------------------------------------------------------------*/

#include "hw.h"

/****************************************************************************\
 *  Type definition
\****************************************************************************/

/**
 * TimerFunc type
 *
 *     Type for a timer notification callback. The notification function
 *     is called to indicate that the timer has fired.
 */
typedef void (*TimerFunc)(void *parms);

/***************************************************************************\
 * RAM data
\***************************************************************************/

uint8_t prescaler;
LPTIM_HandleTypeDef LPTimerStruct;
uint8_t isLPTIMInitialised = 0u;

/* Function callback to call when timer has fired */
TimerFunc timerFunction;

/* Parameter of the function callback */
void *timerParam;

/***************************************************************************\
 *  Local functions Prototype
\***************************************************************************/

static void TimerLptimConfig(void);

/***************************************************************************\
 *  APPLICATION INTERFACES functions definition
\***************************************************************************/

/*!
 * \brief Creates an autoreload timer.
 *
 * \param [in] func         A pointer to a function to call when the timer
 *                          expires.
 * \param [in] param        Parameter to pass in func when the timer expires.
 * \param [in] millisec     Time to wait before the timer expiration.
 *
 */
void TimerCreateTimer(void *func, void *param, uint32_t millisec){
    uint32_t time;
    /* Save the parameters */
    timerFunction = (TimerFunc)func;
    timerParam = param;

    // LPTIM must be initialised once
    if ( isLPTIMInitialised == 0){
        TimerLptimConfig();
        isLPTIMInitialised = 1;
    }

    if(millisec < 2000){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
        prescaler = 1;
    } else if((millisec >= 2000) && ((millisec < 4000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV2;
        prescaler = 2;
    } else if((millisec >= 4000) && ((millisec < 8000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV4;
        prescaler = 4;
    } else if((millisec >= 8000) && ((millisec < 16000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV8;
        prescaler = 8;
    } else if((millisec >= 16000) && ((millisec < 32000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV16;
        prescaler = 16;
    } else if((millisec >= 32000) && ((millisec < 64000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
        prescaler = 32;
    } else if((millisec >= 64000) && ((millisec < 128000))){
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV64;
        prescaler = 64;
    } else {
        LPTimerStruct.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
        prescaler = 128;
    }
    HAL_LPTIM_Init(&LPTimerStruct);

    time = ((millisec * LPTCLOCK_IN_HZ)/((1000 * prescaler)));
    if(time > 0xFFFF) {
        time = 0xFFFF;
    }else if( time == 0x0 ){
        time = 0x1;
    }

    /* Start timer */
    HAL_LPTIM_TimeOut_Start_IT(&LPTimerStruct, time, time-1);
}

/*!
 * \brief Cancel the current running timer.
 *
 */
void TimerCancelTimer(void){

    /* Disable the timer */
    LPTimerStruct.Instance->CR = 0;
}

/***************************************************************************\
 * External Function Definition
\***************************************************************************/

void LPTIM1_IRQHandler(void){

    HAL_LPTIM_IRQHandler(&LPTimerStruct);
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim){

    /* call the timer function callback*/
    timerFunction (timerParam);
}

/***************************************************************************\
 * Local Function Definition
\***************************************************************************/
/**
* @brief  Configures the LPTIM peripheral.
*/
static void TimerLptimConfig(void){

    RCC_PeriphCLKInitTypeDef        RCC_PeriphCLKInitStruct;

    /* ### - 1 - Re-target the LSE to Clock the LPTIM Counter ################# */
    /* Select the LSE clock as LPTIM peripheral clock */
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

    if( HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK ){
            while(1);
    }

    __HAL_RCC_LPTIM1_CLK_ENABLE();
    LPTimerStruct.Instance = LPTIM1;
    LPTimerStruct.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    LPTimerStruct.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    LPTimerStruct.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    LPTimerStruct.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    LPTimerStruct.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;

    /* Enable interrupt */
    NVIC_SetPriority(LPTIM1_IRQn,1);
    NVIC_EnableIRQ(LPTIM1_IRQn);

}




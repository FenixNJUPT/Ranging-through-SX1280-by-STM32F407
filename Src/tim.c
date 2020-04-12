/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"


#define CLOCK_IN_HZ 168

typedef void (*TimerFunc)(void *parms);

uint8_t isTIMInitialised = 0u;
uint8_t prescaler;
TimerFunc timerFunction;
void *timerParam;



/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 21000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */






//millisec:测距请求的时间间隔RngReqDelay


void TimerCreateTimer(void *func, void *param, uint32_t millisec){
    uint32_t time;
    /* Save the parameters */
    timerFunction = (TimerFunc)func;
    timerParam = param;

    // LPTIM must be initialised once
    if ( isTIMInitialised == 0){
        MX_TIM1_Init();
        isTIMInitialised = 1;
    }

    if(millisec < 2000){
        htim1.Init.Prescaler = TIM_ICPSC_DIV1;
        prescaler = 1;
    } else if((millisec >= 2000) && ((millisec < 4000))){
        htim1.Init.Prescaler = TIM_ICPSC_DIV2;
        prescaler = 2;
    } else if((millisec >= 4000) && ((millisec < 8000))){
        htim1.Init.Prescaler = TIM_ICPSC_DIV4;
        prescaler = 4;
    } else {
        htim1.Init.Prescaler = TIM_ICPSC_DIV8;
        prescaler = 8;
    }
    HAL_TIM_Base_Init(&htim1);

    time = ((millisec * CLOCK_IN_HZ)/((1000 * prescaler)));
    if(time > 0xFFFF) {
        time = 0xFFFF;
    }else if( time == 0x0 ){
        time = 0x1;
    }

    /* Start timer */
    HAL_TIM_TimeOut_Start_IT(&htim1, time, time-1);
}

void TimerCancelTimer(void){

    /* Disable the timer */
    htim1.Instance->CR1 = 0;
}




/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

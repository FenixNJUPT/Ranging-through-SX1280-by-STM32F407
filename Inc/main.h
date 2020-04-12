/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_nRESET_Pin GPIO_PIN_0
#define RADIO_nRESET_GPIO_Port GPIOA
#define RADIO1_nRESET_Pin GPIO_PIN_1
#define RADIO1_nRESET_GPIO_Port GPIOA
#define RADIO1_NSS_Pin GPIO_PIN_2
#define RADIO1_NSS_GPIO_Port GPIOA
#define RADIO1_BUSY_Pin GPIO_PIN_0
#define RADIO1_BUSY_GPIO_Port GPIOB
#define RADIO1_DIOx_Pin GPIO_PIN_1
#define RADIO1_DIOx_GPIO_Port GPIOB
#define RADIO1_DIOx_EXTI_IRQn EXTI1_IRQn
#define Tim1H_Pin GPIO_PIN_7
#define Tim1H_GPIO_Port GPIOC
#define RADIO_NSS_Pin GPIO_PIN_8
#define RADIO_NSS_GPIO_Port GPIOA
#define Tim1L_Pin GPIO_PIN_10
#define Tim1L_GPIO_Port GPIOC
#define RADIO_BUSY_Pin GPIO_PIN_3
#define RADIO_BUSY_GPIO_Port GPIOB
#define RADIO_DIOx_Pin GPIO_PIN_4
#define RADIO_DIOx_GPIO_Port GPIOB
#define RADIO_DIOx_EXTI_IRQn EXTI4_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

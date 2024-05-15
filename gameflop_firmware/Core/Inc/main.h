/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

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
#define Select_Pin GPIO_PIN_10
#define Select_GPIO_Port GPIOB
#define Select_EXTI_IRQn EXTI15_10_IRQn
#define Start_Pin GPIO_PIN_11
#define Start_GPIO_Port GPIOB
#define Start_EXTI_IRQn EXTI15_10_IRQn
#define A_Pin GPIO_PIN_3
#define A_GPIO_Port GPIOB
#define A_EXTI_IRQn EXTI3_IRQn
#define B_Pin GPIO_PIN_4
#define B_GPIO_Port GPIOB
#define B_EXTI_IRQn EXTI4_IRQn
#define Up_Pin GPIO_PIN_5
#define Up_GPIO_Port GPIOB
#define Up_EXTI_IRQn EXTI9_5_IRQn
#define Left_Pin GPIO_PIN_6
#define Left_GPIO_Port GPIOB
#define Left_EXTI_IRQn EXTI9_5_IRQn
#define Down_Pin GPIO_PIN_7
#define Down_GPIO_Port GPIOB
#define Down_EXTI_IRQn EXTI9_5_IRQn
#define Right_Pin GPIO_PIN_9
#define Right_GPIO_Port GPIOB
#define Right_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
#include "stm32f1xx_hal.h"

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
#define A_clone_Pin GPIO_PIN_2
#define A_clone_GPIO_Port GPIOA
#define I_clone_Pin GPIO_PIN_3
#define I_clone_GPIO_Port GPIOA
#define throttle_Pin GPIO_PIN_4
#define throttle_GPIO_Port GPIOA
#define ghc_Pin GPIO_PIN_13
#define ghc_GPIO_Port GPIOB
#define ghb_Pin GPIO_PIN_14
#define ghb_GPIO_Port GPIOB
#define glc_Pin GPIO_PIN_8
#define glc_GPIO_Port GPIOA
#define glb_Pin GPIO_PIN_9
#define glb_GPIO_Port GPIOA
#define gla_Pin GPIO_PIN_10
#define gla_GPIO_Port GPIOA
#define gha_Pin GPIO_PIN_11
#define gha_GPIO_Port GPIOA
#define I_Pin GPIO_PIN_15
#define I_GPIO_Port GPIOA
#define I_EXTI_IRQn EXTI15_10_IRQn
#define B_Pin GPIO_PIN_4
#define B_GPIO_Port GPIOB
#define B_EXTI_IRQn EXTI4_IRQn
#define A_Pin GPIO_PIN_6
#define A_GPIO_Port GPIOB
#define A_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
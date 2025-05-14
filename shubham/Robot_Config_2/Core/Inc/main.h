/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define S3_Pulse_Pin GPIO_PIN_15
#define S3_Pulse_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_0
#define M1_GPIO_Port GPIOC
#define M2_Pin GPIO_PIN_1
#define M2_GPIO_Port GPIOC
#define S1_Dir_Pin GPIO_PIN_2
#define S1_Dir_GPIO_Port GPIOC
#define S1_Pulse_Pin GPIO_PIN_3
#define S1_Pulse_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define E_Bike_PWM_Pin GPIO_PIN_7
#define E_Bike_PWM_GPIO_Port GPIOA
#define ENA_1_Pin GPIO_PIN_4
#define ENA_1_GPIO_Port GPIOC
#define S3_Dir_Pin GPIO_PIN_5
#define S3_Dir_GPIO_Port GPIOC
#define M3_Pin GPIO_PIN_13
#define M3_GPIO_Port GPIOB
#define M3_PWM_Pin GPIO_PIN_14
#define M3_PWM_GPIO_Port GPIOB
#define P1_Pin GPIO_PIN_11
#define P1_GPIO_Port GPIOA
#define P2_Pin GPIO_PIN_12
#define P2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define S2_Pulse_Pin GPIO_PIN_10
#define S2_Pulse_GPIO_Port GPIOC
#define S2_Dir_Pin GPIO_PIN_11
#define S2_Dir_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_8
#define M1_PWM_GPIO_Port GPIOB
#define M2_PWM_Pin GPIO_PIN_9
#define M2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

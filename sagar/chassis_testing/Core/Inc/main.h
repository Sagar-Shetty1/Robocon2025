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
#define motor_1_Pin GPIO_PIN_0
#define motor_1_GPIO_Port GPIOC
#define motor_2_Pin GPIO_PIN_1
#define motor_2_GPIO_Port GPIOC
#define motor_3_Pin GPIO_PIN_2
#define motor_3_GPIO_Port GPIOC
#define MOTOR_1_PWM_Pin GPIO_PIN_0
#define MOTOR_1_PWM_GPIO_Port GPIOA
#define MOTOR_2_PWM_Pin GPIO_PIN_1
#define MOTOR_2_PWM_GPIO_Port GPIOA
#define MOTOTR_3_PWM_Pin GPIO_PIN_2
#define MOTOTR_3_PWM_GPIO_Port GPIOA
#define dac_dribbling_Pin GPIO_PIN_4
#define dac_dribbling_GPIO_Port GPIOA
#define dac_turret_Pin GPIO_PIN_5
#define dac_turret_GPIO_Port GPIOA
#define pnue_3_Pin GPIO_PIN_2
#define pnue_3_GPIO_Port GPIOB
#define pnue_4_Pin GPIO_PIN_10
#define pnue_4_GPIO_Port GPIOB
#define pnue_1_Pin GPIO_PIN_8
#define pnue_1_GPIO_Port GPIOC
#define pnue_2_Pin GPIO_PIN_9
#define pnue_2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

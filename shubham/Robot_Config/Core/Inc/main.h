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
#define DIR5_Dribbling_Pin GPIO_PIN_15
#define DIR5_Dribbling_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_3
#define DIR4_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PWM_Dribbling_Pin GPIO_PIN_6
#define PWM_Dribbling_GPIO_Port GPIOA
#define Encoders3_A_Pin GPIO_PIN_7
#define Encoders3_A_GPIO_Port GPIOA
#define Step_Pulse_Pin GPIO_PIN_5
#define Step_Pulse_GPIO_Port GPIOC
#define Pneumatics3_Pin GPIO_PIN_2
#define Pneumatics3_GPIO_Port GPIOB
#define Pneumatics4_Pin GPIO_PIN_12
#define Pneumatics4_GPIO_Port GPIOB
#define Encoders4_A_Pin GPIO_PIN_6
#define Encoders4_A_GPIO_Port GPIOC
#define Encoders4_B_Pin GPIO_PIN_7
#define Encoders4_B_GPIO_Port GPIOC
#define Pneumatics1_Pin GPIO_PIN_11
#define Pneumatics1_GPIO_Port GPIOA
#define Pneumatics2_Pin GPIO_PIN_12
#define Pneumatics2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Encoders3_B_Pin GPIO_PIN_4
#define Encoders3_B_GPIO_Port GPIOB
#define PWM_Pin GPIO_PIN_8
#define PWM_GPIO_Port GPIOB
#define PWMB9_Pin GPIO_PIN_9
#define PWMB9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

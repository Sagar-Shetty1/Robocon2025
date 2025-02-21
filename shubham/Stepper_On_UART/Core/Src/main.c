/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<inttypes.h>

#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_PIN GPIO_PIN_8
#define STEP_PORT GPIOA
#define STEP_PIN1 GPIO_PIN_9
#define STEP_PORT1 GPIOA
#define DIR_PIN GPIO_PIN_6
#define DIR_PORT GPIOA
#define DIR_PIN1 GPIO_PIN_7
#define DIR_PORT1 GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxbuff[16];

int slow = 75;

int fast = 400;

int Buff1 = 60;

int Buff2 = -60;

int BuffP = 80;

int BuffN = -80;

int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2;

HAL_StatusTypeDef status;

static uint32_t last_step_time = 0;
static uint32_t last_step_time1 = 0;
static int step_state = 0;
static int step_state1 = 0;
static int direction = 1;
static int direction1 = 1;
int step_delay = 5;
int step_delay1 = 1;
static int stepper_running = 0;  // Toggle flag
static int stepper_running1 = 0;  // Toggle flag
//static int prev_cir = 0;  // Previous button state
static int prev_squ = 0;
static int prev_ll1 = 0;
static int prev_rr1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_UART_Receive(&huart4, rxbuff, 64, 1000);
	HAL_UART_Receive_IT(&huart4, rxbuff, 64);
	status = HAL_UART_Receive(&huart4, rxbuff, 64, 1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//	HAL_StatusTypeDef status;
//
//	status=HAL_UART_Receive(&huart4, rxbuff,64,1000);
//
//
//
//	if (status == HAL_OK) {
//
//		// Convert the received bytes to signed integers
//
//
//
//		lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];
//
//		ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];
//
//		rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];
//
//		ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];
//
//
//
//		up= (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
//
//		down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
//
//		left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
//
//		right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
//
//
//
//		tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
//
//		squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
//
//		cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
//
//		cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
//
//
//
//		ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
//
//		ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
//
//		rr1= (rxbuff[15] & 0x80) ? (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
//
//		rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];
//
//
//
//		// Print the received values
//
////		HAL_Delay(1000);
//		printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", lx, ly, rx, ry, cro, squ, tri, cir);
//
//
//
//		}
//	else {
//
//		ry = 0;
//
//		rx = 0;
//
//		lx = 0;
//
//		ly = 0;
//
//		cro = 0;
//
//		squ = 0;
//
//		tri = 0;
//
//		cir = 0;
//
//		up = 0;
//
//		down = 0;
//
//		left = 0;
//
//		right = 0;
//
//		ll1=0;
//
//		ll2=0;
//
//		rr1=0;
//
//		rr2=0;
//
//	}
//	uint16_t dutycycle;
//	 static int step_count = 0;
//	 static int total_steps = 200;
//	 static uint32_t last_step_time = 0;
//	 static uint32_t last_step_time1 = 0;
//	 static int step_state = 0;
//	 static int step_state1 = 0;
//	 static int direction = 1;
//	 static int direction1 = 1;
//	 int step_delay = 5;
//	 int step_delay1 = 1;
//	 static int stepper_running = 0;  // Toggle flag
//	 static int stepper_running1 = 0;  // Toggle flag
//	 static int prev_cir = 0;  // Previous button state
//	 static int prev_squ = 0;  // Previous button state
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//	 HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
//	 HAL_GPIO_WritePin(DIR_PORT1, DIR_PIN1, direction1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
//
//	 // Toggle stepper motor state when the circle button is pressed
////	 if (cir && !prev_cir) {
////		 printf("Circle Button Pressed\n");
////	     stepper_running = !stepper_running;  // Toggle ON/OFF
////	 }
////	 prev_cir = cir;  // Store previous state
//	 if (squ == 1 && cro == 0 && !prev_squ) {
//		 printf("Square Button Pressed\r\n");
//	     stepper_running1 = !stepper_running1;  // Toggle ON/OFF
//	 }
//	 prev_squ = squ;  // Store previous state
//
//	 if (stepper_running && (HAL_GetTick() - last_step_time) >= step_delay) {
//		 last_step_time = HAL_GetTick();
//
//		 if (step_state == 0) {
//			 HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET); // Step HIGH
//			 step_state = 1;
//		 } else {
//			 HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET); // Step LOW
//			 step_state = 0;
////			 step_count++;
//		 }
//	 }
//
//	 if (stepper_running1 && (HAL_GetTick() - last_step_time1) >= step_delay1) {
//	     last_step_time1 = HAL_GetTick();
//
//	     if (step_state1 == 0) {
//	         HAL_GPIO_WritePin(STEP_PORT1, STEP_PIN1, GPIO_PIN_SET); // Step HIGH
//	         step_state1 = 1;
//	     } else {
//	         HAL_GPIO_WritePin(STEP_PORT1, STEP_PIN1, GPIO_PIN_RESET); // Step LOW
//	         step_state1 = 0;
//	     }
//	 }
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		HAL_GPIO_WritePin(DIR_PORT1, DIR_PIN1,
				direction1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

		// Check if PS5 buttons are pressed
		if (ll1 == 1 && !prev_ll1) {  // LL1 button pressed
			direction1 = 1;  // Set to Anti-clockwise
			stepper_running1 = 1;
		}
		if (rr1 == 1 && !prev_rr1) {  // RR1 button pressed
			direction1 = 0;  // Set to Clockwise
			stepper_running1 = 1;
		}
		if (squ == 1 && !prev_squ) {
			stepper_running1 = 0;
		}

		// Store previous states
		prev_ll1 = ll1;
		prev_rr1 = rr1;
		prev_squ = squ;

		// Run stepper only if stepper_running1 is active
		if (stepper_running1
				&& (HAL_GetTick() - last_step_time1) >= step_delay1) {
			last_step_time1 = HAL_GetTick(); // Update last step time

			if (step_state1 == 0) {
				HAL_GPIO_WritePin(STEP_PORT1, STEP_PIN1, GPIO_PIN_SET); // Step HIGH
				step_state1 = 1;
			} else {
				HAL_GPIO_WritePin(STEP_PORT1, STEP_PIN1, GPIO_PIN_RESET); // Step LOW
				step_state1 = 0;
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA6 PA7 PA8
	 PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//int _write(int file, char *ptr, int len) {
//    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
//    return len;
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {  // Check if it's the correct UART
		lx = (rxbuff[0] & 0x80) ?
				(int32_t) rxbuff[0] - 256 : (int32_t) rxbuff[0];
		ly = (rxbuff[1] & 0x80) ?
				(int32_t) rxbuff[1] - 256 : (int32_t) rxbuff[1];
		rx = (rxbuff[2] & 0x80) ?
				(int32_t) rxbuff[2] - 256 : (int32_t) rxbuff[2];
		ry = (rxbuff[3] & 0x80) ?
				(int32_t) rxbuff[3] - 256 : (int32_t) rxbuff[3];

		up = (rxbuff[8] & 0x80) ?
				(int32_t) rxbuff[8] - 256 : (int32_t) rxbuff[8];
		down = (rxbuff[9] & 0x80) ?
				(int32_t) rxbuff[9] - 256 : (int32_t) rxbuff[9];
		left = (rxbuff[10] & 0x80) ?
				(int32_t) rxbuff[10] - 256 : (int32_t) rxbuff[10];
		right = (rxbuff[11] & 0x80) ?
				(int32_t) rxbuff[11] - 256 : (int32_t) rxbuff[11];

		tri = (rxbuff[6] & 0x80) ?
				(int32_t) rxbuff[6] - 256 : (int32_t) rxbuff[6];
		squ = (rxbuff[5] & 0x80) ?
				(int32_t) rxbuff[5] - 256 : (int32_t) rxbuff[5];
		cir = (rxbuff[7] & 0x80) ?
				(int32_t) rxbuff[7] - 256 : (int32_t) rxbuff[7];
		cro = (rxbuff[4] & 0x80) ?
				(int32_t) rxbuff[4] - 256 : (int32_t) rxbuff[4];

		ll1 = (rxbuff[12] & 0x80) ?
				(int32_t) rxbuff[12] - 256 : (int32_t) rxbuff[12];
		ll2 = (rxbuff[13] & 0x80) ?
				(int32_t) rxbuff[13] - 256 : (int32_t) rxbuff[13];
		rr1 = (rxbuff[14] & 0x80) ?
				(int32_t) rxbuff[14] - 256 : (int32_t) rxbuff[14];
		rr2 = (rxbuff[15] & 0x80) ?
				(int32_t) rxbuff[15] - 256 : (int32_t) rxbuff[15];

		printf(
				"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
				lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1,
				ll2, rr1, rr2);

		// Restart UART reception for next data
		HAL_UART_Receive_IT(&huart4, rxbuff, 64);
	} else {

		ry = 0;

		rx = 0;

		lx = 0;

		ly = 0;

		cro = 0;

		squ = 0;

		tri = 0;

		cir = 0;

		up = 0;

		down = 0;

		left = 0;

		right = 0;

		ll1 = 0;

		ll2 = 0;

		rr1 = 0;

		rr2 = 0;

	}
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

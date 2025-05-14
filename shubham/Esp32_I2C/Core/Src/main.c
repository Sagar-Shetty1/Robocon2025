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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SLAVE_ADDRESS 0x08

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxbuff[10];
volatile uint8_t dataReceived = 0;  // Add this flag to track new data
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2, rL, rR;
typedef enum {
	STOP, FORWARD, BACKWARD, RIGHT, LEFT, CLOCKWISE, ANTICLOCKWISE
} MovementState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		// Set flag to indicate data received
		dataReceived = 1;
	}
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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_I2C_Slave_Receive_IT(&hi2c1, rxbuff, 10);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_I2C_SlaveRxCpltCallback(&hi2c1);
		if (dataReceived) {
			lx = (rxbuff[0] & 0x80) ?
					(int32_t) rxbuff[0] - 256 : (int32_t) rxbuff[0];
			ly = (rxbuff[1] & 0x80) ?
					(int32_t) rxbuff[1] - 256 : (int32_t) rxbuff[1];
			rx = (rxbuff[2] & 0x80) ?
					(int32_t) rxbuff[2] - 256 : (int32_t) rxbuff[2];
			ry = (rxbuff[3] & 0x80) ?
					(int32_t) rxbuff[3] - 256 : (int32_t) rxbuff[3];
			tri = (rxbuff[4] & 0x80) ?
					(int32_t) rxbuff[4] - 256 : (int32_t) rxbuff[4];
			cir = (rxbuff[5] & 0x80) ?
					(int32_t) rxbuff[5] - 256 : (int32_t) rxbuff[5];
			cro = (rxbuff[6] & 0x80) ?
					(int32_t) rxbuff[6] - 256 : (int32_t) rxbuff[6];
			squ = (rxbuff[7] & 0x80) ?
					(int32_t) rxbuff[7] - 256 : (int32_t) rxbuff[7];
			ll2 = (rxbuff[8] & 0x80) ?
					(int32_t) rxbuff[8] - 256 : (int32_t) rxbuff[8];
			rr2 = (rxbuff[9] & 0x80) ?
					(int32_t) rxbuff[9] - 256 : (int32_t) rxbuff[9];

			printf(
					"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
					lx, ly, rx, ry, tri, cir, cro, squ, ll2, rr2);

			// Clear the flag after processing
			dataReceived = 0;
			HAL_I2C_Slave_Receive_IT(&hi2c1, rxbuff, 10);
		}

//		/* chassis */
//		uint16_t dutycycle;
//
//		dutycycle = 0;
//		MovementState current_state = STOP;
//		//motors stop
//		if (ly >= Buff2 && ly <= Buff1 && lx >= Buff2 && lx <= Buff1
//				&& rx >= Buff2 && rx <= Buff1) {
//
//			dutycycle = 0;
//
//			TIM12->CCR1 = dutycycle;
//			TIM10->CCR1 = dutycycle;
//			TIM11->CCR1 = dutycycle;
//			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//		}
//
//		//	  	//forward
//
//		else if (ly >= Buff1 && (lx <= BuffP && lx >= BuffN))
//
//		{
//
//			dutycycle = map(ly, Buff1, 127, 0, fast);
//
//			TIM10->CCR1 = dutycycle;
//			TIM11->CCR1 = dutycycle;
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//			current_state = FORWARD;
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//
//			//	   	HAL_Delay(1000);
//
//		}
//
//		// backward
//
//		else if (ly <= Buff2 && (lx <= BuffP && lx >= BuffN))
//
//		{
//
//			dutycycle = map(ly, -128, Buff2, fast, 0);
//
//			TIM10->CCR1 = dutycycle;
//			TIM11->CCR1 = dutycycle;
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//			current_state = BACKWARD;
//			//	    HAL_Delay(1000);
//
//		}
//
//		//	right
//
//		else if (lx >= Buff1 && (ly <= BuffP && ly >= BuffN))
//
//		{
//
//			dutycycle = map(lx, Buff1, 127, 0, fast);
//
//			TIM12->CCR1 = dutycycle;
//			TIM10->CCR1 = cos30 * dutycycle;
//			TIM11->CCR1 = cos30 * dutycycle;
//			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//			current_state = RIGHT;
//			//	    HAL_Delay(1000);
//
//		}
//
//		// left
//
//		else if (lx <= Buff2 && (ly <= BuffP && ly >= BuffN))
//
//		{
//
//			dutycycle = map(lx, -128, Buff2, fast, 0);
//
//			TIM12->CCR1 = dutycycle;
//			TIM10->CCR1 = cos30 * dutycycle;
//			TIM11->CCR1 = cos30 * dutycycle;
//			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//			current_state = LEFT;
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//
//		}
//
//		//clockwise
//
//		else if (rx >= Buff1 && (ry <= BuffP && ry >= BuffN)) {
//
//			dutycycle = map(rx, Buff1, 127, 0, 100);
//
//			TIM12->CCR1 = dutycycle;
//			TIM10->CCR1 = dutycycle;
//			TIM11->CCR1 = dutycycle;
//			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//			current_state = CLOCKWISE;
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//
//		}
//
//		// anticlockwise
//
//		else if (rx <= Buff2 && (ry <= BuffP && ry >= BuffN)) {
//
//			dutycycle = map(rx, -128, Buff2, 100, 0);
//
//			TIM12->CCR1 = dutycycle;
//			TIM10->CCR1 = dutycycle;
//			TIM11->CCR1 = dutycycle;
//			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
//
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//			current_state = ANTICLOCKWISE;
//
//			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//
//		}
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 32;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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

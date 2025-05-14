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
#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#define STEPS 240
#include "stdio.h"
#include<stdio.h>
#include<inttypes.h>
#include<Math.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
int16_t signed_counter = 0;
int slow = 125;
int fast = 850;
int Buff1 = 50;
int Buff2 = -50;
int BuffP = 50;
int BuffN = -50;
int deadzone = 50;
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2, rL, rR;
//float cos30 = (sqrt(3)/2);
double kp, kd, c, kt, k, ki;
float last_encoder = 0.0f;
const int encoderPPR = 600 * 6;

// stepper
static uint32_t last_step_time = 0;
static uint32_t last_step_time1 = 0;
static uint32_t last_step_time2 = 0;
static int step_state = 0;
static int step_state1 = 0;
static int step_state2 = 0;
static int direction = 1;
static int direction1 = 1;
int step_delay = 1;
int step_delay1 = 1;
int step_delay2 = 1;
static int stepper_running = 0;  // Toggle flag
static int stepper_running1 = 0;  // Toggle flag
static int stepper_running2 = 0;  // Toggle flag
static int ebike_running = 0;  // Toggle flag
static int prev_cir = 0;
static int prev_tri = 0;
static int prev_cro = 0;
static int prev_ll1 = 0;
static int prev_rr1 = 0;
static int prev_ll2 = 0;
static int prev_rr2 = 0;
uint32_t target_steps = 0;
uint32_t current_steps = 0;
uint32_t target_steps1 = 0;
uint32_t target_steps2 = 0;
uint32_t current_steps1 = 0;
uint32_t current_steps2 = 0;

int setpoint = 0;
int error, lasterr;
float output;
float errsum;

unsigned long now, lastTime;

uint8_t rxbuff[16];

uint16_t dutycycle = 0;

// shooting
float dist = 1.7;
float rpm;
float theta = 85;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
int constrain(int value, int min_val, int max_val) {
	if (value < min_val)
		return min_val;
	if (value > max_val)
		return max_val;
	return value;
}
void compute() {
	unsigned long now = HAL_GetTick();
	double timeChange = (double) (now - lastTime);
	error = setpoint - signed_counter;
	double derr = (error - lasterr) / timeChange;
	errsum += error;
	float s = c * error + derr;
	output = kp * error + kd * derr + kt * (k * sign(s));
	output = constrain(output, -500, 500);
	output = abs(output);
	lasterr = error;
	lastTime = now;
//	printf("error = %d \n\r",error);

}
void SetTunings(double Kp, double Ki, double Kd, double Kt, double K) {
	kp = Kp;
	ki = Ki;
	kd = Kd;
	kt = Kt;
	k = K;
}
void calulateProjectileEquation(){
	float calc = sqrtf(9.81/(2*pow(cos(theta), 2)*(dist*tan(theta)-1.39)));
	rpm = (400/M_PI) * dist * calc;

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM13_Init();
	MX_UART4_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* ps5 controller */
		lx = (rxbuff[0] & 0x80) ?
				(int32_t) rxbuff[0] - 256 : (int32_t) rxbuff[0];
		ly = (rxbuff[1] & 0x80) ?
				(int32_t) rxbuff[1] - 256 : (int32_t) rxbuff[1];
		rx = (rxbuff[2] & 0x80) ?
				(int32_t) rxbuff[2] - 256 : (int32_t) rxbuff[2];
		ry = (rxbuff[3] & 0x80) ?
				(int32_t) rxbuff[3] - 256 : (int32_t) rxbuff[3];
		cro = (rxbuff[4] & 0x80) ?
				(int32_t) rxbuff[4] - 256 : (int32_t) rxbuff[4];
		squ = (rxbuff[5] & 0x80) ?
				(int32_t) rxbuff[5] - 256 : (int32_t) rxbuff[5];
		tri = (rxbuff[6] & 0x80) ?
				(int32_t) rxbuff[6] - 256 : (int32_t) rxbuff[6];
		cir = (rxbuff[7] & 0x80) ?
				(int32_t) rxbuff[7] - 256 : (int32_t) rxbuff[7];
		up = (rxbuff[8] & 0x80) ?
				(int32_t) rxbuff[8] - 256 : (int32_t) rxbuff[8];
		down = (rxbuff[9] & 0x80) ?
				(int32_t) rxbuff[9] - 256 : (int32_t) rxbuff[9];
		left = (rxbuff[10] & 0x80) ?
				(int32_t) rxbuff[10] - 256 : (int32_t) rxbuff[10];
		right = (rxbuff[11] & 0x80) ?
				(int32_t) rxbuff[11] - 256 : (int32_t) rxbuff[11];
		ll1 = (rxbuff[12] & 0x80) ?
				(int32_t) rxbuff[12] - 256 : (int32_t) rxbuff[12];
		ll2 = (rxbuff[13] & 0x80) ?
				(int32_t) rxbuff[13] - 256 : (int32_t) rxbuff[13];
		rr1 = (rxbuff[14] & 0x80) ?
				(int32_t) rxbuff[14] - 256 : (int32_t) rxbuff[14];
		rr2 = (rxbuff[15] & 0x80) ?
				(int32_t) rxbuff[15] - 256 : (int32_t) rxbuff[15];

		printf(
				"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
				lx, ly, rx, ry, cro, squ, tri, cir, ll1, rr1, ll2, rr2);
		//	  		printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", lx, ly,
		//	  				rx, ry, cro, squ, tri, cir);

		/* encoder */
		uint16_t raw_counter = __HAL_TIM_GET_COUNTER(&htim2);

		// Convert to signed values
		signed_counter =
				(raw_counter < 32768) ?
						(int16_t) raw_counter : (int16_t) (raw_counter - 65536);

		/* PSMC */
		SetTunings(0.5, 0.5, 0, 10.0, -0.005); // Initial tuning for 50kg·cm motor
		c = 1.5;  // Sliding surface parameter
		if (left == 1)
			setpoint += 100;
		else if (right == 1)
			setpoint -= 100;
		compute();
		//	 	  +
		if (signed_counter < setpoint) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			TIM1->CCR1 = output;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		}

		else if (signed_counter > setpoint) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			TIM1->CCR1 = output;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		} else {
			TIM1->CCR1 = 0;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		}

		printf("Counts = %d \r\n", signed_counter);

		/* stepper */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,
				direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,
				direction1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
				!direction1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

		if (ll1 == 1 && !prev_ll1) {  // LL1 button pressed
			direction1 = 1;  // Set to Anti-clockwise
			stepper_running = 1;
			stepper_running1 = 1;
			target_steps = STEPS;
			target_steps1 = STEPS;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (rr1 == 1 && !prev_rr1) {  // RR1 button pressed
			direction1 = 0;  // Set to Clockwise
			stepper_running = 1;
			stepper_running1 = 1;
			target_steps = STEPS;
			target_steps1 = STEPS;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (ll2 == 1 && !prev_ll2) {  // LL1 button pressed
			direction = 1;  // Set to Anti-clockwise
			stepper_running2 = !stepper_running2;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (rr2 == 1 && !prev_rr2) {  // RR1 button pressed
			direction = 0;  // Set to Clockwise
			stepper_running2 = !stepper_running2;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (cir == 1 && !prev_cir) {
			stepper_running1 = 0;
			current_steps = 0;
			current_steps1 = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		prev_ll1 = ll1;
		prev_rr1 = rr1;
		prev_ll2 = ll2;
		prev_rr2 = rr2;
		prev_cir = cir;
		//		if (stepper_running && (HAL_GetTick() - last_step_time) >= step_delay
		//				&& current_steps < target_steps) {
		//			last_step_time = HAL_GetTick(); // Update last step time
		//
		//			if (step_state == 0) {
		//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // Step HIGH
		//				step_state = 1;
		//				current_steps++;
		//			} else {
		//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // Step LOW
		//				step_state = 0;
		//			}
		//		}
		//		if (stepper_running1 && (HAL_GetTick() - last_step_time1) >= step_delay1
		//				&& current_steps1 < target_steps1) {
		//			last_step_time1 = HAL_GetTick(); // Update last step time
		//
		//			if (step_state1 == 0) {
		//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Step HIGH
		//				step_state1 = 1;
		//				current_steps1++;
		//			} else {
		//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Step LOW
		//				step_state1 = 0;
		//			}
		//		}
		if (stepper_running && (HAL_GetTick() - last_step_time) >= step_delay) {
			last_step_time = HAL_GetTick(); // Update last step time

			if (step_state == 0) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // Step HIGH
				step_state = 1;
				current_steps++;
			} else {

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // Step LOW
				step_state = 0;
			}
		}
		if (stepper_running1
				&& (HAL_GetTick() - last_step_time1) >= step_delay1) {
			last_step_time1 = HAL_GetTick(); // Update last step time

			if (step_state1 == 0) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Step HIGH
				step_state1 = 1;
				current_steps1++;
			} else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Step LOW
				step_state1 = 0;
			}
		}
		if (stepper_running2
				&& (HAL_GetTick() - last_step_time2) >= step_delay2) {
			last_step_time2 = HAL_GetTick(); // Update last step time

			if (step_state2 == 0) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Step HIGH
				step_state2 = 1;
				current_steps2++;
			} else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Step LOW
				step_state2 = 0;
			}
		}

		// Stop when target steps are reached
		//		if (current_steps >= target_steps) {
		//			stepper_running = 0;
		//			current_steps = 0;
		//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		//		}
		//		if (current_steps1 >= target_steps1) {
		//			stepper_running1 = 0;
		//			current_steps1 = 0;
		//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		//		}

		/* shooting */
		calulateProjectileEquation();
		printf("rpm = %.2f \r\n",rpm);
		if (tri == 1 && !prev_tri) {
			ebike_running = 1;
		}
		if (cro == 1 && !prev_cro) {
			ebike_running = 0;
		}
		prev_tri = tri;
		prev_cro = cro;

//		if (ebike_running) {
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
//			TIM1->CCR2 = (70 * 999) / 100;  //rpm 2000 test
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//		} else {
//			TIM1->CCR2 = 0;
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//		}
		if (ebike_running) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
			TIM1->CCR2 = (rpm * 999) / 3500;  //rpm 2000 test
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		} else {
			TIM1->CCR2 = 0;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		}
		//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		//		TIM1->CCR2 = (100 * 999) / 100;  //rpm 2000 test
		//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 179;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void) {

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 179;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 999;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */
	HAL_TIM_MspPostInit(&htim13);

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
	HAL_GPIO_WritePin(GPIOC,
			GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC3 PC5 PC6 PC7
	 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA11 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB13 PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

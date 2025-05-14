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
#include "bno055_stm32.h"
#include "BNO055.h"

#include "stdio.h"
#include<stdio.h>
#include<inttypes.h>
#include<Math.h>
#include <stdbool.h>
#include <stdlib.h>
#define WHEEL_RADIUS        63.5f    // meters
#define ENCODER_COUNTS_PER_REV  349.9844447f // Encoder counts per wheel revolution
#define L                   0.314f     // Distance from robot center to wheels (meters)

#define pi 3.1415926535
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
// comm buffer
char data[12];

// stepper
static uint32_t last_step_time1 = 0;
static int step_state1 = 0;
static int direction1 = 1;
int step_delay = 5;
int step_delay1 = 1;
static int stepper_running1 = 0;  // Toggle flag
static int ebike_running = 0;  // Toggle flag
static int prev_cir = 0;
static int prev_tri = 0;
static int prev_cro = 0;
static int prev_ll1 = 0;
static int prev_rr1 = 0;

// Encoder counters
int16_t signed_counter1 = 0;
int32_t signed_counter2 = 0;
int16_t signed_counter3 = 0;

//BNO055_Data_t imu_data, imuData;

float yaw = 0, q0 = 0, q1 = 0, q2 = 0, q3 = 0;

// Global variables
float x_global = 0.0f;      // Global X position (meters)
float y_global = 0.0f;      // Global Y position (meters)
float theta_prev = 0.0f;    // Previous IMU yaw (radians)
int32_t encoder_prev[3] = { 0 }; // Previous encoder counts for wheels 1, 2, 3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//float sinval = sin(radians(60));
uint8_t rxbuff[16];
int slow = 125;
int fast = 850;
int Buff1 = 50;
int Buff2 = -50;
int BuffP = 50;
int BuffN = -50;
int target_wf, target_wrr, target_wrl, w;
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2, rL, rR;
int deadzone = 50;
int l0 = 310;
//float cos30 = (sqrt(3)/2);
float cos30 = 0.5;

float dist1 = 0.0f;
float dist2 = 0.0f;
float dist3 = 0.0f;
float dist4 = 0.0f;

float last_encoder1 = 0.0f;
float last_encoder2 = 0.0f;
float last_encoder3 = 0.0f;

typedef enum {
	STOP, FORWARD, BACKWARD, RIGHT, LEFT, CLOCKWISE, ANTICLOCKWISE
} MovementState;

// Global position and orientation
volatile float x = 0.0f; // mm
volatile float y = 0.0f; // mm
volatile float theta = 0.0f; // radians

// Encoder state
volatile int32_t prev_encoder5 = 0;
volatile int32_t prev_encoder10 = 0;
volatile int32_t prev_encoder11 = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

void update_odometry(int16_t encoder1, int32_t encoder2, int16_t encoder3) {
	// Calculate the change in encoder values
	int16_t delta1 = encoder1 - last_encoder1;
	int32_t delta2 = encoder2 - last_encoder2;
	int16_t delta3 = encoder3 - last_encoder3;

	// Calculate distance per encoder count
	float wheel_circumference = 2 * M_PI * WHEEL_RADIUS;
	float distance_per_count = (1.0 / ENCODER_COUNTS_PER_REV)
			* (wheel_circumference / 1000);

	// Compute average delta for forward/backward encoders (without abs)
	float avgDelta12 = (delta1 + delta2) / 2.0;

	// Check movement direction based on joystick
	if (ly >= Buff1 && (lx <= BuffP && lx >= BuffN)) {
		// Forward motion - use the actual delta (with sign) to track position
		x += distance_per_count * avgDelta12;
	} else if (ly <= Buff2 && (lx <= BuffP && lx >= BuffN)) {
		// Backward motion - use the actual delta (with sign) to track position
		x += distance_per_count * avgDelta12; // Will be negative when going backward
	}

	if (lx >= Buff1 && (ly <= BuffP && ly >= BuffN)) {
		dist3 = distance_per_count * abs(delta3);  // Using delta3 for y-axis
		y += dist3;
	} else if (lx <= Buff2 && (ly <= BuffP && ly >= BuffN)) {
		dist4 = distance_per_count * abs(delta3);
		y -= dist4;
	}

	// Update last encoder values for the next iteration
	last_encoder1 = encoder1;
	last_encoder2 = encoder2;
	last_encoder3 = encoder3;
}

void resetEncoders() {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

void stopEncoders() {
	HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
}

void startEncoders() {
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
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
	MX_UART4_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_I2C1_Init();
	MX_TIM5_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM13_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM12_Init();
	MX_UART5_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
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

		//		printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
		//				lx, ly, rx, ry, cro, squ, tri, cir, ll1, rr1);
		//		printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", lx, ly,
		//				rx, ry, cro, squ, tri, cir);


		data[0] = rxbuff[4];
		data[1] = rxbuff[5];
		data[2] = rxbuff[6];
		data[3] = rxbuff[7];
		data[4] = rxbuff[8];
		data[5] = rxbuff[9];
		data[6] = rxbuff[10];
		data[7] = rxbuff[11];
		data[8] = rxbuff[12];
		data[9] = rxbuff[13];
		data[10] = rxbuff[14];
		data[11] = rxbuff[15];


		HAL_UART_Transmit(&huart5, (uint8_t*)data, 12, 1000);

		/* 3 encoders */
		uint16_t raw_counter1 = __HAL_TIM_GET_COUNTER(&htim1);
		uint32_t raw_counter2 = __HAL_TIM_GET_COUNTER(&htim2);
		uint16_t raw_counter3 = __HAL_TIM_GET_COUNTER(&htim3);

		// Convert to signed values
		signed_counter1 =
				(raw_counter1 < 32768) ?
						(int16_t) raw_counter1 :
						(int16_t) (raw_counter1 - 65536);
		signed_counter2 =
				(raw_counter2 < 2147483648) ?
						(int32_t) raw_counter2 :
						(int32_t) (raw_counter2 - 4294967296);
		signed_counter3 =
				(raw_counter3 < 32768) ?
						(int16_t) raw_counter3 :
						(int16_t) (raw_counter3 - 65536);

		/* IMU */
		bno055_vector_t v = bno055_getVectorEuler();
		//		printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
		v = bno055_getVectorQuaternion();
		//		printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
		yaw = -(atan2(2.0 * (v.w * v.z + v.x * v.y),
				1.0 - 2.0 * (v.y * v.y + v.z * v.z))) * (180.0 / pi);
		//				printf("Yaw: %.2f\r\n", yaw);
		printf("Encoder position: %d %d %d X: %.2f Y: %.2f\r\n",
				signed_counter1, signed_counter2, signed_counter3, x, y);

		/* 3 wheel */
		uint16_t dutycycle;

		dutycycle = 0;
		MovementState current_state = STOP;
		/* chassis */
		//motors stop
		if (ly >= Buff2 && ly <= Buff1 && lx >= Buff2 && lx <= Buff1
				&& rx >= Buff2 && rx <= Buff1) {

			dutycycle = 0;

			TIM12->CCR1 = dutycycle;
			TIM10->CCR1 = dutycycle;
			TIM11->CCR1 = dutycycle;
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

		}

		//	  	//forward

		else if (ly >= Buff1 && (lx <= BuffP && lx >= BuffN))

		{

			dutycycle = map(ly, Buff1, 127, 0, fast);

			TIM10->CCR1 = dutycycle;
			TIM11->CCR1 = dutycycle;
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			current_state = FORWARD;
			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

			//	   	HAL_Delay(1000);

		}

		// backward

		else if (ly <= Buff2 && (lx <= BuffP && lx >= BuffN))

		{

			dutycycle = map(ly, -128, Buff2, fast, 0);

			TIM10->CCR1 = dutycycle;
			TIM11->CCR1 = dutycycle;
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			current_state = BACKWARD;
			//	    HAL_Delay(1000);

		}

		//	right

		else if (lx >= Buff1 && (ly <= BuffP && ly >= BuffN))

		{

			dutycycle = map(lx, Buff1, 127, 0, fast);

			TIM12->CCR1 = dutycycle;
			TIM10->CCR1 = cos30 * dutycycle;
			TIM11->CCR1 = cos30 * dutycycle;
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			current_state = RIGHT;
			//	    HAL_Delay(1000);

		}

		// left

		else if (lx <= Buff2 && (ly <= BuffP && ly >= BuffN))

		{

			dutycycle = map(lx, -128, Buff2, fast, 0);

			TIM12->CCR1 = dutycycle;
			TIM10->CCR1 = cos30 * dutycycle;
			TIM11->CCR1 = cos30 * dutycycle;
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			current_state = LEFT;
			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

		}

		//clockwise

		else if (rx >= Buff1 && (ry <= BuffP && ry >= BuffN)) {

			dutycycle = map(rx, Buff1, 127, 0, 100);

			TIM12->CCR1 = dutycycle;
			TIM10->CCR1 = dutycycle;
			TIM11->CCR1 = dutycycle;
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			current_state = CLOCKWISE;
			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

		}

		// anticlockwise

		else if (rx <= Buff2 && (ry <= BuffP && ry >= BuffN)) {

			dutycycle = map(rx, -128, Buff2, 100, 0);

			TIM12->CCR1 = dutycycle;
			TIM10->CCR1 = dutycycle;
			TIM11->CCR1 = dutycycle;
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			current_state = ANTICLOCKWISE;

			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

		}

		if (current_state == CLOCKWISE || current_state == ANTICLOCKWISE) {
			stopEncoders();
		} else {
			startEncoders();
		}

		update_odometry(signed_counter1, signed_counter2, signed_counter3);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 180 - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1000 - 1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 179;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 999;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 179;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 999;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */
	HAL_TIM_MspPostInit(&htim11);

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 179;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 999;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */
	HAL_TIM_MspPostInit(&htim12);

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
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

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
	/* DMA1_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			GPIO_PIN_15 | M1_Pin | M2_Pin | M3_Pin | stepper_direction_1_Pin
					| ENA_1_Pin | stepper_pulse_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | P1_Pin | P2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_2 | GPIO_PIN_10 | P3_Pin | stepper_pulse_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC15 M1_Pin M2_Pin M3_Pin
	 stepper_direction_1_Pin ENA_1_Pin stepper_pulse_1_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_15 | M1_Pin | M2_Pin | M3_Pin
			| stepper_direction_1_Pin | ENA_1_Pin | stepper_pulse_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin P1_Pin P2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | P1_Pin | P2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 PB10 P3_Pin stepper_pulse_2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | P3_Pin
			| stepper_pulse_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

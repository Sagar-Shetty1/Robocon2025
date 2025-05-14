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
#include<Math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
//#define WHEEL_RADIUS        63.5f    // meters
//#define ENCODER_COUNTS_PER_REV  349.9844447f // Encoder counts per wheel revolution
//#define L                   0.314f     // Distance from robot center to wheels (meters)
#define STEPS 240

// Global variables
float x_global = 0.0f;      // Global X position (meters)
float y_global = 0.0f;      // Global Y position (meters)
float theta_prev = 0.0f;    // Previous IMU yaw (radians)
int32_t encoder_prev[3] = { 0 }; // Previous encoder counts for wheels 1, 2, 3

#define pi 3.1415926535
#define PI 3.1415926535

#define l0 0.3

#define I2C_SLAVE_ADDRESS 0x08
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
double wfl, wfr, wrl, wrr, wz;
const double r = 75;
double target_wf = 0, target_wrl = 0, target_wrr = 0;
double current_wf = 0, current_wrl = 0, current_wrr = 0;
const double decelerationRate = 30.0;
double w;
double rL, thetaL, rR, thetaR;
const int deadzone = 40;

static uint32_t last_step_time = 0;
static uint32_t last_step_time1 = 0;
static int step_state = 0;
static int step_state1 = 0;
static int direction1 = 1;
int step_delay = 1;
int step_delay1 = 1;
static int stepper_running = 0;  // Toggle flag
static int stepper_running1 = 0;  // Toggle flag
static int ebike_running = 0;  // Toggle flag
static int prev_cir = 0;
static int prev_tri = 0;
static int prev_cro = 0;
static int prev_squ = 0;
static int prev_down = 0;
static int prev_ll1 = 0;
static int prev_rr1 = 0;
static int prev_ll2 = 0;
static int prev_rr2 = 0;
uint32_t target_steps = 0;
uint32_t current_steps = 0;
uint32_t target_steps1 = 0;
uint32_t current_steps1 = 0;
uint32_t drib_speed = 70;

// Encoder counters
int16_t signed_counter1 = 0;
int32_t signed_counter2 = 0;
int16_t signed_counter3 = 0;

uint8_t rxbuff[16];
volatile uint8_t dataReceived = 0;
int slow = 125;
int fast = 850;
int Buff1 = 50;
int Buff2 = -50;
int BuffP = 50;
int BuffN = -50;
//int target_wf, target_wrr, target_wrl, w;
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2;
//int deadzone = 50;
//int l0 = 310;
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

//BNO055_Data_t imu_data, imuData;

float yaw = 0, q0 = 0, q1 = 0, q2 = 0, q3 = 0;

// pneumatics
static int pneumatic_open = 0;

//servo
static int servoDir = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
static void MX_TIM14_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == I2C2) {
//		// Set flag to indicate data received
//		dataReceived = 1;
//	}
//}
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    HAL_I2C_Slave_Receive_IT(hi2c, rxbuff, 16);
//}
// Global variables for I2C communication
#define I2C_BUFFER_SIZE 16
uint8_t rxbuff[I2C_BUFFER_SIZE] = { 0 };
volatile uint8_t i2c_data_ready = 0;
uint32_t last_i2c_reception = 0;
#define I2C_TIMEOUT_MS 1000  // 1 second timeout for I2C communication

// Initialize I2C communication
void init_i2c_slave(void) {
	// Clear the receive buffer initially
	memset(rxbuff, 0, I2C_BUFFER_SIZE);

	// Start the slave receiver in interrupt mode
	if (HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, I2C_BUFFER_SIZE) != HAL_OK) {
		// Handle initialization error
		Error_Handler();
	}

	// Record the initialization time
	last_i2c_reception = HAL_GetTick();
}

// I2C Slave receive complete callback
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {
		// Set flag to indicate data received
		i2c_data_ready = 1;

		// Update the reception timestamp
		last_i2c_reception = HAL_GetTick();

		// Restart the slave receive in interrupt mode
		if (HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, I2C_BUFFER_SIZE)
				!= HAL_OK) {
			// If restarting receive fails, handle the error
			Error_Handler();
		}
	}
}

// I2C Error callback
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {
		// Clear the receive buffer on error
		memset(rxbuff, 0, I2C_BUFFER_SIZE);

		// Reset the ready flag
		i2c_data_ready = 0;

		// Clear all error flags
		hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

		// Re-enable the I2C peripheral if it was disabled due to error
		if (hi2c->State == HAL_I2C_STATE_READY) {
			// Reset the I2C peripheral
			HAL_I2C_DeInit(hi2c);
			HAL_I2C_Init(hi2c);
		}

		// Restart reception
		HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, I2C_BUFFER_SIZE);
	}
}

// Check I2C communication status
uint8_t check_i2c_communication(void) {
	uint32_t current_time = HAL_GetTick();

	// Check for timeout
	if (current_time - last_i2c_reception > I2C_TIMEOUT_MS) {
		// No communication for too long, restart I2C
		memset(rxbuff, 0, I2C_BUFFER_SIZE);
		i2c_data_ready = 0;

		// Abort any ongoing transfer
		HAL_I2C_Master_Abort_IT(&hi2c2, I2C_SLAVE_ADDRESS);

		// Restart reception
		HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, I2C_BUFFER_SIZE);

		// Update the timestamp
		last_i2c_reception = current_time;

		return 0; // Communication failed
	}

	return 1; // Communication OK
}

// Process I2C data - call this in your main loop
void process_i2c_data(void) {
	// Check if I2C communication is working
	if (!check_i2c_communication()) {
		// Handle communication failure
		printf("I2C communication failure\r\n");
		return;
	}

	// Process data only if new data is available
	if (i2c_data_ready) {
		// Parse the received data
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
		ll1 = (rxbuff[8] & 0x80) ?
				(int32_t) rxbuff[8] - 256 : (int32_t) rxbuff[8];
		rr1 = (rxbuff[9] & 0x80) ?
				(int32_t) rxbuff[9] - 256 : (int32_t) rxbuff[9];
		ll2 = (rxbuff[10] & 0x80) ?
				(int32_t) rxbuff[10] - 256 : (int32_t) rxbuff[10];
		rr2 = (rxbuff[11] & 0x80) ?
				(int32_t) rxbuff[11] - 256 : (int32_t) rxbuff[11];
		up = (rxbuff[12] & 0x80) ?
				(int32_t) rxbuff[12] - 256 : (int32_t) rxbuff[12];
		down = (rxbuff[13] & 0x80) ?
				(int32_t) rxbuff[13] - 256 : (int32_t) rxbuff[13];
		right = (rxbuff[14] & 0x80) ?
				(int32_t) rxbuff[14] - 256 : (int32_t) rxbuff[14];
		left = (rxbuff[15] & 0x80) ?
				(int32_t) rxbuff[15] - 256 : (int32_t) rxbuff[15];

		// Print data for debugging
//		printf(
//				"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %d %d\r\n",
//				lx, ly, rx, ry, tri, cir, cro, squ, ll1, rr1, ll2, rr2, up,
//				down, right, left, ebike_running, drib_speed);

		// Reset the data ready flag
		i2c_data_ready = 0;
	} else {
		lx = 0, ly = 0, rx = 0, ry = 0, cro = 0, squ = 0, tri = 0, cir = 0, up = 0, down = 0, left = 0, right = 0, ll1 = 0, rr1 = 0, ll2 = 0, rr2 = 0;
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
//}
long map(long x, long in_min, long in_max, long out_min, long out_max) {

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

//void update_odometry(int16_t encoder1, int32_t encoder2, int16_t encoder3) {
//	// Calculate the change in encoder values
//	int16_t delta1 = encoder1 - last_encoder1;
//	int32_t delta2 = encoder2 - last_encoder2;
//	int16_t delta3 = encoder3 - last_encoder3;
//
//	// Calculate distance per encoder count
//	float wheel_circumference = 2 * M_PI * WHEEL_RADIUS;
//	float distance_per_count = (1.0 / ENCODER_COUNTS_PER_REV)
//			* (wheel_circumference / 1000);
//
//	// Compute average delta for forward/backward encoders (without abs)
//	float avgDelta12 = (delta1 + delta2) / 2.0;
//
//	// Check movement direction based on joystick
//	if (ly >= Buff1 && (lx <= BuffP && lx >= BuffN)) {
//		// Forward motion - use the actual delta (with sign) to track position
//		x += distance_per_count * avgDelta12;
//	} else if (ly <= Buff2 && (lx <= BuffP && lx >= BuffN)) {
//		// Backward motion - use the actual delta (with sign) to track position
//		x += distance_per_count * avgDelta12; // Will be negative when going backward
//	}
//
//	if (lx >= Buff1 && (ly <= BuffP && ly >= BuffN)) {
//		dist3 = distance_per_count * abs(delta3);  // Using delta3 for y-axis
//		y += dist3;
//	} else if (lx <= Buff2 && (ly <= BuffP && ly >= BuffN)) {
//		dist4 = distance_per_count * abs(delta3);
//		y -= dist4;
//	}
//
//	// Update last encoder values for the next iteration
//	last_encoder1 = encoder1;
//	last_encoder2 = encoder2;
//	last_encoder3 = encoder3;
//}

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

float constrain(float val, float min_val, float max_val) {
	if (val < min_val) {
		return min_val;
	} else if (val > max_val) {
		return max_val;
	} else {
		return val;
	}
}

void rectToPolar() {
	wz = rx;
	if (abs(lx) < deadzone)
		lx = 0;
	// else lx = (lx > 0) ? lx - deadzone : lx + deadzone;

	if (abs(ly) < deadzone)
		ly = 0;
	// else ly = (ly > 0) ? ly - deadzone : ly + deadzone;

	if (abs(rx) < deadzone)
		rx = 0;
	// else rx = (rx > 0) ? rx - deadzone : rx + deadzone;

	if (abs(ry) < deadzone)
		ry = 0;
	// else ry = (ry > 0) ? ry - deadzone : ry + deadzone;
	rL = sqrt(lx * lx + ly * ly);
	thetaL = atan2(lx, ly);
	rR = sqrt(rx * rx + ry * ry);
	thetaR = atan2(ry, rx);
}

void compute3wheel() {
	// Calculate joystick-based velocities
	const double theta = -30 * pi / 180;
	double lx_rotated = lx * cos(theta) - ly * sin(theta);
	double ly_rotated = lx * sin(theta) + ly * cos(theta);
	double vx = ly_rotated;
	double vy = lx_rotated;
	w = rx * 2.5;
	double speedFactorL = 15 * (rL / 127 + rR / 127) / 2;
	// Modify wheel velocities to include yaw correction
	target_wf = ((-0.5 * vx) + (sqrt(3) / 2) * vy + l0 * w) * speedFactorL;
	target_wrr = ((-0.5 * vx) - (sqrt(3) / 2) * vy + l0 * w) * speedFactorL;
	target_wrl = (vx + l0 * w) * speedFactorL;
	// target_wf=(-0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
	// target_wrr = (0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
	// target_wrl = (-vx + l0*w)*speedFactorL;
	// Constrain target wheel speeds
//	target_wf = constrain(target_wf, -950, 950);
//	target_wrr = constrain(target_wrr, -950, 950);
//	target_wrl = constrain(target_wrl, -950, 950);
}

void applyDeceleration() {
	// Gradually approach target speeds for each wheel
	current_wf += (target_wf - current_wf) / decelerationRate;
	current_wrl += (target_wrl - current_wrl) / decelerationRate;
	current_wrr += (target_wrr - current_wrr) / decelerationRate;

	// Serial.print("wf =");
	// Serial.print(current_wf);
	// Serial.print("wrr =");
	// Serial.print(current_wrr);
	// Serial.print("wrl =");
	// Serial.println(current_wrl);

	// Update motor directions and speeds based on decelerated values
	if (current_wf >= 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, current_wf);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -current_wf);
	}

	if (current_wrl >= 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, current_wrl);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, -current_wrl);
	}

	if (current_wrr >= 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, current_wrr);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, -current_wrr);
	}
}

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle) {
	uint32_t pulse_length = 100 + (angle * (500 - 100) / 180);
	__HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
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
	MX_TIM14_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
//	HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, 16);
//
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	// Initialize I2C slave communication
	init_i2c_slave();

	// Set default values
	memset(rxbuff, 0, I2C_BUFFER_SIZE);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* ps5 controller */
		process_i2c_data();
//		HAL_I2C_SlaveRxCpltCallback(&hi2c2);
//
//		lx = (rxbuff[0] & 0x80) ?
//				(int32_t) rxbuff[0] - 256 : (int32_t) rxbuff[0];
//		ly = (rxbuff[1] & 0x80) ?
//				(int32_t) rxbuff[1] - 256 : (int32_t) rxbuff[1];
//		rx = (rxbuff[2] & 0x80) ?
//				(int32_t) rxbuff[2] - 256 : (int32_t) rxbuff[2];
//		ry = (rxbuff[3] & 0x80) ?
//				(int32_t) rxbuff[3] - 256 : (int32_t) rxbuff[3];
//		tri = (rxbuff[4] & 0x80) ?
//				(int32_t) rxbuff[4] - 256 : (int32_t) rxbuff[4];
//		cir = (rxbuff[5] & 0x80) ?
//				(int32_t) rxbuff[5] - 256 : (int32_t) rxbuff[5];
//		cro = (rxbuff[6] & 0x80) ?
//				(int32_t) rxbuff[6] - 256 : (int32_t) rxbuff[6];
//		squ = (rxbuff[7] & 0x80) ?
//				(int32_t) rxbuff[7] - 256 : (int32_t) rxbuff[7];
//		ll1 = (rxbuff[8] & 0x80) ?
//				(int32_t) rxbuff[8] - 256 : (int32_t) rxbuff[8];
//		rr1 = (rxbuff[9] & 0x80) ?
//				(int32_t) rxbuff[9] - 256 : (int32_t) rxbuff[9];
//		ll2 = (rxbuff[10] & 0x80) ?
//				(int32_t) rxbuff[10] - 256 : (int32_t) rxbuff[10];
//		rr2 = (rxbuff[11] & 0x80) ?
//				(int32_t) rxbuff[11] - 256 : (int32_t) rxbuff[11];
//		up = (rxbuff[12] & 0x80) ?
//				(int32_t) rxbuff[12] - 256 : (int32_t) rxbuff[12];
//		down = (rxbuff[13] & 0x80) ?
//				(int32_t) rxbuff[13] - 256 : (int32_t) rxbuff[13];
//		right = (rxbuff[14] & 0x80) ?
//				(int32_t) rxbuff[14] - 256 : (int32_t) rxbuff[14];
//		left = (rxbuff[15] & 0x80) ?
//				(int32_t) rxbuff[15] - 256 : (int32_t) rxbuff[15];
//
		printf(
				"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %d %d\r\n",
				lx, ly, rx, ry, tri, cir, cro, squ, ll1, rr1, ll2, rr2, up,
				down, right, left, ebike_running, drib_speed);
//
//		printf(
//				"Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %l d %ld %ld %ld %ld %ld %.2f %.2f %.2f %d\r\n",
//				lx, ly, rx, ry, tri, cir, cro, squ, ll1, rr1, ll2, rr2, up,
//				down, right, left, target_wf, target_wrl, target_wrr, ebike_running);

//			HAL_I2C_Slave_Receive_IT(&hi2c2, rxbuff, 16);

		/* 3 wheel */
		MovementState current_state = STOP;

		rectToPolar();
		compute3wheel();
		applyDeceleration();

		if (current_state == CLOCKWISE || current_state == ANTICLOCKWISE) {
			stopEncoders();
		} else {
			startEncoders();
		}

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
		//		bno055_vector_t v = bno055_getVectorEuler();
		//		//		printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
		//		v = bno055_getVectorQuaternion();
		//		//		printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
		//		yaw = -(atan2(2.0 * (v.w * v.z + v.x * v.y),
		//				1.0 - 2.0 * (v.y * v.y + v.z * v.z))) * (180.0 / pi);
		////				printf("Yaw: %.2f\r\n", yaw);
		//		printf("Encoder position: %d %d %d X: %.2f Y: %.2f\r\n",
		//				signed_counter1, signed_counter2, signed_counter3, x, y);
		/* Stepper */
		//		stepper_running1 = 1;
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
		if (cir == 1 && !prev_cir) {
			stepper_running = 0;
			stepper_running1 = 0;
			current_steps = 0;
			current_steps1 = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		prev_ll1 = ll1;
		prev_rr1 = rr1;
		prev_cir = cir;
		if (stepper_running && (HAL_GetTick() - last_step_time) >= step_delay
				&& current_steps < target_steps) {
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
		if (stepper_running1 && (HAL_GetTick() - last_step_time1) >= step_delay1
				&& current_steps1 < target_steps1) {
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
		// Stop when target steps are reached
		if (current_steps >= target_steps) {
			stepper_running = 0;
			current_steps = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		if (current_steps1 >= target_steps1) {
			stepper_running1 = 0;
			current_steps1 = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}

		/* Dribbling */
		if (tri == 1 && !prev_tri) {
			ebike_running = 1;
		}
		if (cro == 1 && !prev_cro) {  // cro button pressed
			printf("cross pressed");
			//			direction1 = 0;  // Set to Clockwise
			ebike_running = 0;
		}
		if (ll2 == 1 && !prev_ll2) {
			drib_speed = drib_speed - 2;
		}
		if (rr2 == 1 && !prev_rr2) {
			drib_speed = drib_speed + 2; //62 speed for passing 50 for dribbling
		}
		if (down == 1 && !prev_down) {
			drib_speed = 70;
		}
		if (drib_speed < 0) {
			drib_speed = 0;
		}
		if (drib_speed > 100) {
			drib_speed = 100;
		}
		prev_ll2 = ll2;
		prev_rr2 = rr2;
		prev_tri = tri;
		prev_cro = cro;
		prev_down = down;
		if (ebike_running) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//			TIM1->CCR3 = (drib_speed * 999) / 100;  //rpm 2000 test
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
					((drib_speed * 999) / 100));
		} else {
			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
//			TIM1->CCR3 = (0 * 999) / 100;
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((0 * 999) / 100));
		}

		// pneumatics
		if (squ == 1) {
			pneumatic_open = 1;
		} else {
			pneumatic_open = 0;
		}
		//		prev_squ = squ;
		if (pneumatic_open) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		}

		// servo motor
//		if (left == 1)
//		{
//
//			htim3.Instance->CCR3 = 50; // duty cycle is 1 ms (0 degrees)
//
//		} else if (right == 1) {
//
//			htim3.Instance->CCR3 = 125; // duty cycle is 2 ms (180 degrees)
//
//		}
		if (right == 1) {//net open
//			Set_Servo_Angle(&htim5, TIM_CHANNEL_1, 180);
			htim1.Instance->CCR1 = 75; // 90 degree
			htim1.Instance->CCR2 = 75;
			htim1.Instance->CCR3 = 75;
		} else if (left == 1) {//net close
//			Set_Servo_Angle(&htim5, TIM_CHANNEL_1, 90); // duty cycle is 2 ms (180 degrees)
			htim1.Instance->CCR1 = 125; // 180 degree
			htim1.Instance->CCR2 = 125;
			htim1.Instance->CCR3 = 125;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

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
	hi2c1.Init.ClockSpeed = 100000;
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
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 32;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	htim1.Init.Prescaler = 899;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 900 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
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
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	htim5.Init.Prescaler = 179;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 999;
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
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 179;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 999;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

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
	HAL_GPIO_WritePin(GPIOC,
			S3_Pulse_Pin | M1_Pin | M2_Pin | S1_Dir_Pin | S1_Pulse_Pin
					| ENA_1_Pin | S3_Dir_Pin | S2_Pulse_Pin | S2_Dir_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | LD2_Pin | P1_Pin | P2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_2 | M3_Pin | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : S3_Pulse_Pin M1_Pin M2_Pin S1_Dir_Pin
	 S1_Pulse_Pin ENA_1_Pin S3_Dir_Pin S2_Pulse_Pin
	 S2_Dir_Pin */
	GPIO_InitStruct.Pin = S3_Pulse_Pin | M1_Pin | M2_Pin | S1_Dir_Pin
			| S1_Pulse_Pin | ENA_1_Pin | S3_Dir_Pin | S2_Pulse_Pin | S2_Dir_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 LD2_Pin P1_Pin P2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | LD2_Pin | P1_Pin | P2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB2 M3_Pin PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | M3_Pin | GPIO_PIN_15;
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

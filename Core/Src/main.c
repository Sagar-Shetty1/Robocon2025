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
#include "main.h"
#include<stdio.h>
#include<inttypes.h>
#include<Math.h>
#include <stdbool.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
static uint32_t last_step_time1 = 0;
static int step_state1 = 0;
static int direction1 = 1;
int step_delay = 5;
int step_delay1 = 1;
static int stepper_running1 = 0;  // Toggle flag
static int prev_squ = 0;
static int prev_ll1 = 0;
static int prev_rr1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxbuff[16];
int slow = 75;
int fast = 400;
int Buff1 = 60;
int Buff2 = -60;
int BuffP = 80;
int BuffN = -80;
int target_wf, target_wrr, target_wrl, w;
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1,
		ll2, rr2, rL, rR;
int deadzone = 50;
int l0 = 310;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
}
int constrain(int value, int min_val, int max_val) {
	if (value < min_val)
		return min_val;
	if (value > max_val)
		return max_val;
	return value;
}
void compute3wheel() {
	// Calculate joystick-based velocities
	const double theta = -30 * pi / 180;
	double lx_rotated = lx * cos(theta) - ly * sin(theta);
	double ly_rotated = lx * sin(theta) + ly * cos(theta);
	double vx = ly_rotated;
	double vy = lx_rotated;
	w = rx;
	target_wf = ((-0.5 * vx) + (sqrt(3) / 2) * vy + l0 * w);
	target_wrr = ((-0.5 * vx) - (sqrt(3) / 2) * vy + l0 * w);
	target_wrl = (vx + l0 * w);

	// Constrain target wheel speeds
	target_wf = constrain(target_wf, -800, 800);
	target_wrr = constrain(target_wrr, -800, 800);
	target_wrl = constrain(target_wrl, -800, 800);
}
void Channel_A_Config(void)
{
  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020024; // address of alternate function register of port A
  uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
  uint32_t *pPortAPullUpReg =   (uint32_t*)0x4002000C; // address of port A pull up register

  *pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
  *pAFRLPortAReg   |= 0x00010000; // alternate function of Timer 2 Channel 1 enabled
  *pPortAModeReg   &= 0x3FFFFFFF;
  *pPortAModeReg   |= 0x80000000; // port A Pin 15 configured for 'Alternate function'
  *pPortAPullUpReg |= 0x40000000;    // port A Pin 15 pulled up
}

void Channel_B_Config(void)
{
  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
  uint32_t *pAFRLPortBReg   =   (uint32_t*)0x40020420; // address of alternate function register of port B
  uint32_t *pPortBModeReg   =   (uint32_t*)0x40020400; // address of port B mode register
  uint32_t *pPortBPullUpReg =   (uint32_t*)0x4002040C; // address of port B pull up register

  *pAHB1ClkCtrlReg |= 0x2;        // port B clock enable
  *pAFRLPortBReg   |= 0x00100000; // alternate function of Timer 2 Channel 2 enabled
  *pPortBModeReg   &= 0xFFFFFF3F;
  *pPortBModeReg   |= 0x00000080; // port B Pin 3 configured for 'Alternate function'
  *pPortBPullUpReg |= 0x0040;     // port B Pin 3 pulled up
}



void Encoder_Init(uint16_t ARR_Value)
{
	 uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB2 clock control register
	 uint32_t *pTimer2ArrReg   =   (uint32_t*)0x4000002C; // address of timer 2 auto reload register
	 uint32_t *pTimer2CCMR1Reg =   (uint32_t*)0x40000018; // address of timer 2 capture/compare register
	 uint32_t *pTimer2CcerReg  =   (uint32_t*)0x40000020; // address of timer 2 capture/compare enable register
     uint32_t *pTimer2SmcrReg  =   (uint32_t*)0x40000008; // address of timer 2 slave mode control register
     uint32_t *pTimer2CR1Reg   =   (uint32_t*)0x40000000; // address of timer 2 control register 1

    *pAPB1ClkCtrlReg |= 0x1;           //timer 2 clock enable
    *pTimer2ArrReg   =  ARR_Value - 1; // maximum count value for counter
    *pTimer2CCMR1Reg |= 0x1;   // mapping 'Input capture 1' mode in timer 2 Channel 1 and configuring it as input
    *pTimer2CCMR1Reg |= 0x100; // mapping 'Input capture 2' mode in timer 2 Channel 2 and configuring it as input
    *pTimer2CcerReg  |= 0x2;   // channel 1 triggered at rising edge
    *pTimer2CcerReg  |= 0x20;  // channel 2 triggered at rising edge
    *pTimer2SmcrReg  |= 0x3;   // counter counts up/down on both channel 1 and channel 2 edges depending on the level of the other input.
    *pTimer2CR1Reg   |= 0x1;   // counter enable
}
uint16_t Get_Encoder_Counts()
{
	 uint32_t *pTimer2CNTReg   =   (uint32_t*)0x40000024; // address of timer 2 counter register
	 return *pTimer2CNTReg; // reading counts from counter register

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
	Encoder_Init(65535);
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
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart4, rxbuff, 16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
//			  	  	  	  	  	  	  lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];
//			  		  	  	 	 	   ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];
//			  		  	  	 	 	   rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];
//			  		  	  	 	 	   ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];
//			  		  	  	 	 	   cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
//			  		  	  	 	 	   squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
//			  		  	  	 	 	   tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
//			  		  	  	 	 	   cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
//			  		  	  	 	 	   up= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
//			  		  	  	 	 	   down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
//			  		  	  	 	 	   left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
//			  		  	  	 	 	   right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
//			  		  	  	 	 	   ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
//			  		  	  	 	 	   ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
//			  		  	  	 	 	   rr1= (rxbuff[15] & 0x80) ? (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
//			  		  	  	 	 	   rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];
		if (abs(lx) < deadzone)
			lx = 0;
		if (abs(ly) < deadzone)
			ly = 0;
		if (abs(rx) < deadzone)
			rx = 0;
		else
			rx = (rx > 0) ? rx - deadzone : rx + deadzone;
		if (abs(ry) < deadzone)
			ry = 0;
		lx = (lx * 950) / 127;  // Assuming joystick range is -127 to +127
		ly = (ly * 950) / 127;
		rx = (rx * 950) / 127;
		rL = sqrt(lx * lx + ly * ly);
		rR = sqrt(rx * rx + ry * ry);
		//		printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
		//				lx, ly, rx, ry, cro, squ, tri, cir, ll1, rr1);
		//		  printf("Received Integers: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", lx, ly, rx, ry, cro, squ, tri, cir);
		uint16_t counter = Get_Encoder_Counts();
		printf("Encoder 1 : %u\r\n",counter);
		compute3wheel();
		if (target_wf >= 0) {
			TIM1->CCR1 = target_wf;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
		} else {
			TIM1->CCR1 = -target_wf;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		}
		if (target_wrr >= 0) {
			TIM1->CCR2 = target_wrr;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		} else {
			TIM1->CCR2 = -target_wrr;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
		}
		if (target_wrl >= 0) {
			TIM1->CCR3 = target_wrl;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		} else {
			TIM1->CCR3 = -target_wrl;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,
				direction1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

		if (ll1 == 1 && !prev_ll1) {  // LL1 button pressed
			direction1 = 1;  // Set to Anti-clockwise
			stepper_running1 = 1;
		}
		if (rr1 == 1 && !prev_rr1) {  // RR1 button pressed
			direction1 = 0;  // Set to Clockwise
			stepper_running1 = 1;
		}
		if (cir == 1 && !prev_squ) {
			stepper_running1 = 0;
		}
		prev_ll1 = ll1;
		prev_rr1 = rr1;
		prev_squ = cir;
		if (stepper_running1 && (HAL_GetTick() - last_step_time1) >= step_delay1) {
			last_step_time1 = HAL_GetTick(); // Update last step time

			if (step_state1 == 0) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Step HIGH
				step_state1 = 1;
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Step LOW
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 180-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(stepper_direction_GPIO_Port, stepper_direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, stepper_speed_Pin|motor1_Pin|motor2_Pin|motor3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : stepper_direction_Pin */
  GPIO_InitStruct.Pin = stepper_direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(stepper_direction_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : stepper_speed_Pin motor1_Pin motor2_Pin motor3_Pin */
  GPIO_InitStruct.Pin = stepper_speed_Pin|motor1_Pin|motor2_Pin|motor3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void Error_Handler(void)
{
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

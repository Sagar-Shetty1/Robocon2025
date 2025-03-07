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

#include "BNO055.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rxbuff[16];

int slow=75;

int fast=500;

int Buff1=30;

int Buff2=-30;

int BuffP=40;

int BuffN=-40;


int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1, ll2, rr2;
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

UART_HandleTypeDef huart4;

BNO055_Data_t imu_data;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive(&huart4, rxbuff, 64,500);



   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 //  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
 //  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
 //  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 //  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //MAIN CODE


	  	  	HAL_StatusTypeDef status;

	  	  	status=HAL_UART_Receive(&huart4, rxbuff,64,500);



	  	  	if (status == HAL_OK)

	  	  	{

	  	   // Convert the received bytes to signed integers



	  	   lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];

	  	   ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];

	  	   rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];

	  	   ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];



	  	   cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
	  //
	  	   squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
	  //
	  //	   tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
	  //
	  //	   cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
	  //
	  //	   up= (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
	  //
	  //	   down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
	  //
	  //	   left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
	  //
	  //	   right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
	  //
	  //	   ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
	  //
	  //	   ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
	  //
	  //	   rr1= (rxbuff[15] & 0x80) ? (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
	  //
	  //	   rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];



	  	   // Print the received values

	  	   printf("Received Integers:\n");

	  	   printf("lx: %ld ", lx);

	  	   printf("ly: %ld ", ly);

	  	   printf("rx: %ld ", rx);

	  	   printf("ry: %ld \n", ry);


	  //
	  	   printf("cro: %ld\n", cro);
	  //
	  	   printf("squ: %ld\n", squ);
	  //
	  //	   printf("tri: %ld\n", tri);
	  //
	  //	   printf("cir: %ld\n", cir);
	  //
	  //
	  //
	  //	   printf("up: %ld\n", up);
	  //
	  //	   printf("down: %ld\n", down);
	  //
	  //	   printf("left: %ld\n", left);
	  //
	  //	   printf("right: %ld\n", right);
	  //
	  //
	  //
	  //	   printf("ll1: %ld\n", ll1);
	  //
	  //	   printf("ll2: %ld\n", ll2);
	  //
	  //	   printf("rr1: %ld\n", rr1);
	  //
	  //	   printf("rr2: %ld\n", rr2);



	  	   }

	  	   else
	  	   {

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

	  	   ll1=0;

	  	   ll2=0;

	  	   rr1=0;

	  	   rr2=0;

	  	   }

	  	  	uint16_t dutycycle;






	  	  	//stop
	  	  	if(ly>=Buff2 && ly<=Buff1 && lx>=Buff2 && lx<=Buff1 && rx>=Buff2 && rx<=Buff1){

	  			//motor 1
	  		   HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);

	  		   //motor 2
	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);

	  		   //motor 3
	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);

	  	  	}

	  	  	//forward
	  	  	else if( ly>=Buff1 && (lx<=BuffP && lx>=BuffN) ){

	  //	  		long map(long x, long in_min, long in_max, long out_min, long out_max)
	  //	  		{
	  //	  		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	  //	  		}
	  	  		dutycycle=map(ly,Buff1,127,0,fast);
	  //	  		int slow=75;
	  //	  		int fast=400;
	  //	  		int Buff1=20;
	  //	  		int Buff2=-20;
	  //	  		int BuffP=40;
	  //	  		int BuffN=-40;

	  	  		//motor 1
	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);

	  		   //motor 2
	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  		   //motor 3
	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);


	  	  	}
	  	  	//backward
	  	  	else if(ly<=Buff2 && (lx<=BuffP && lx>=BuffN) )

	  	   {

	  			 dutycycle=map(ly,-128,Buff2,fast,0);

	  			 //motor 1
	  			   HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, SET);
	  			   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);

	  			   //motor 2
	  			   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, SET);
	  			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  			   //motor 3
	  			   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  			   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);


	  	   }

	  	  	//RIGHT
	  	  	else if(lx>=Buff1 && (ly<=BuffP && ly>=BuffN))

	  	   {

	  	  		dutycycle=map(lx,Buff1,127,0,fast);

	  	  		//motor 1
	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, SET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		   //motor 2
	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, (0.86 * dutycycle));

	  		   //motor 3
	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (0.86 * dutycycle));


	  	   }
	  	  	//left

	  	  	else if(lx<=Buff2 && (ly<=BuffP && ly>=BuffN) )

	  	   {

	  	  		dutycycle=map(lx,-128,Buff2,fast,0);
	  	  		//motor 1
	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		   //motor 2
	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, SET);
	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, (0.86 * dutycycle));

	  		   //motor 3
	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, SET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (0.86 * dutycycle));


	  	   }

	  	  //top-right
	  	    else if(lx>70 && ly>70)
	  	  	  	  	{

	  	  	  	  		dutycycle=map(rx,-128,Buff2,400,0);

	  	  	  	  		//motor 1
	  	  	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, SET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  	  	  		   //motor 2
	  	  	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  	  	  		   //motor 3
	  	  	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, SET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);

	  	  	  	   }
	  	  	//top-left
	  	  else if(lx < -70 && ly > 70)
	  		  	  	{

	  		  	  		dutycycle=map(rx,-128,Buff2,400,0);

	  		  	  		//motor 1
	  		  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		  		   //motor 2
	  		  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, SET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);

	  		  		   //motor 3
	  		  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, SET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);

	  		  	   }
	  	  //bottom-right
	  	    else if(lx > 70 && ly < -70)
	  		  	  	{

	  		  	  		dutycycle=map(rx,-128,Buff2,400,0);

	  		  	  		//motor 1
	  		  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, SET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		  		   //motor 2
	  		  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);

	  		  		   //motor 3
	  		  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);

	  		  	   }
	  	  //bottom-left
	  	    else if(lx < -70 && ly < -70)
	  		  	  	{

	  		  	  		dutycycle=map(rx,-128,Buff2,400,0);

	  		  	  		//motor 1
	  		  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		  		   //motor 2
	  		  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, SET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  		  		   //motor 3
	  		  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, SET);
	  		  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);

	  		  	   }

	  	  	//ANTICLOCKWISE
	  	  	else if(rx<=Buff2 && (ry<=BuffP && ry>=BuffN) )
	  	  	{

	  	  		dutycycle=map(rx,-128,Buff2,400,0);

	  	  		//motor 1
	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  		   //motor 2
	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  		   //motor 3
	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, RESET);
	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);

	  	   }
	  	  //CLOCKWISE
	  	  	  	  	else if(rx>=Buff1 && (ry<=BuffP && ry>=BuffN) )
	  	  	  	  	{

	  	  	  	  		dutycycle=map(rx,Buff1,127,0,400);
	  	  	  	  		//motor 1
	  	  	  	  		HAL_GPIO_WritePin(motor_1_GPIO_Port, motor_1_Pin, SET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, dutycycle);

	  	  	  		   //motor 2
	  	  	  		   HAL_GPIO_WritePin(motor_2_GPIO_Port, motor_2_Pin, SET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, dutycycle);

	  	  	  		   //motor 3
	  	  	  		   HAL_GPIO_WritePin(motor_3_GPIO_Port, motor_3_Pin, SET);
	  	  	  		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycycle);


	  	  	  	   }



	  	  if (BNO055_GetAllData(&hi2c1, &imu_data) == HAL_OK) {
//	  	  	  		   Print accelerometer data (m/s^2)
	  	  	  		  printf("Accel: X=%.2f, Y=%.2f, Z=%.2f m/s^2\n",
	  	  	  				 imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);

//	  	  	  		   Print gyroscope data (degrees/s)
	  	  	  		  printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f deg/s\n",
	  	  	  				 imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
	  	  	  		  printf("\n\n");
//	  	  	  		   Print orientation data
	  	  	  		  printf("Euler: Yaw=%.2f, Roll=%.2f, Pitch=%.2f deg\n",
	  	  	  				 imu_data.euler_h, imu_data.euler_r, imu_data.euler_p);
	  	  	  		  printf("\n\n");
	  	  	  	  }

	  	  	  	  HAL_Delay(100); // Read at 10Hz
	  	  	  HAL_Delay(10);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 180-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 180-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 180-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, motor_3_Pin|motor_1_Pin|motor_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motor_3_Pin motor_1_Pin motor_2_Pin */
  GPIO_InitStruct.Pin = motor_3_Pin|motor_1_Pin|motor_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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

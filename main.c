/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define LPF_REG 0x1A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t data;
uint8_t buffer[2],tuffer[6],cuffer[6]; // ham gyro ve acc değerlerinin tutulduğu listeler
int16_t gyro_raw[3],acc_raw[3];        // anlamlandırılmış ham değerler
float gyro_cal[3],acc_cal[3]; // kalibrasyon offsetlerini tutan liste
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro;
float angle_pitch_acc, angle_roll_acc, angle_pitch_acc_sum, angle_roll_acc_sum,acc_total_vector_sum;
float angle_pitch,angle_roll;
int16_t raw_temp;
float temp;
int i;
float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
HAL_StatusTypeDef set_gyro;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // PWR_MGMT_1 CNFG
   data = 0x00;
   HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
   //kullanılan birimin pointer olarak döndürülmesi, cihaz slave adresi, register adresi, register boyutu, verinin pointer
   //olarak döndürülmesi, verinin boyutu , zaman aşımıı değeri
   // GYRO CNFG --> +-500 derece/saniye --> 08
   data = 0x08;
   HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
   // ACC CNFG --> +-8g --> 10
   data = 0x10;
   HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
   data = 0x03;
   HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, LPF_REG, 1, &data, 1, HAL_MAX_DELAY);

   for(i=0; i<2000; i++)
   {
 	  prevtime2 = time2;
 	  time2 = HAL_GetTick();
 	  elapsedtime2=(time2-prevtime2)*1000;

 	  cuffer[0]=0x43;
 	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY);
 	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY);

 	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
 	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
 	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

 	  gyro_cal[0] += gyro_raw[0];
 	  gyro_cal[1] += gyro_raw[1];
 	  gyro_cal[2] += gyro_raw[2];

 	  HAL_Delay(3);

   }

   gyro_cal[0] /= 2000;
   gyro_cal[1] /= 2000;
   gyro_cal[2] /= 2000;

   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
   HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  prevtime1 = time1;
	  time1 = HAL_GetTick();
	  elapsedtime1=(time1-prevtime1)*1000;

	  tuffer[0]=0x3B;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,tuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,tuffer,6,HAL_MAX_DELAY);

	  // Acc Raw Values
	  acc_raw[0] = (tuffer[0] << 8 | tuffer[1]);
	  acc_raw[1] = (tuffer[2] << 8 | tuffer[3]);
	  acc_raw[2] = (tuffer[4] << 8 | tuffer[5]);


	  buffer[0]=0x41;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,buffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,buffer,2,HAL_MAX_DELAY);

	  // Temperature Values
	  raw_temp = (buffer[0] << 8 | buffer[1]);
	  temp = (raw_temp / 340.0) + 36.53;


	  cuffer[0]=0x43;
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY);

	  // Gyro Raw Values
	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  gyro_raw[0] -= gyro_cal[0];
	  gyro_raw[1] -= gyro_cal[1];
	  gyro_raw[2] -= gyro_cal[2];


	  angle_pitch_gyro += gyro_raw[0] * 0.0000611;
	  angle_roll_gyro += gyro_raw[1] * 0.0000611;

	  angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
	  angle_roll_gyro -= angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);


	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	  //57.296 = 1 / (3.142 / 180)
	  angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector)* 57.296;
	  angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector)* -57.296;


	  angle_pitch_acc -= 0;//0.05;
	  angle_roll_acc -= 0;//-1.32;

	  if(set_gyro){
		  angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004;
		  angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004;
		}
	  else
	  {
		  angle_pitch = angle_pitch_acc;
		  set_gyro = true;
		}

	  while((HAL_GetTick() - prevtime)*1000 < 4000);
	  prevtime = HAL_GetTick();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

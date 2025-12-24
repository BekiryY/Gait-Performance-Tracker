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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nrf24l01.h"
#include <stdio.h>
#include <string.h>

/* --- DEFINES --- */
#define MPU6050_ADDR 0x68 << 1
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75

/* --- SENSOR STRUCT --- */
typedef struct {
    float ax; float ay; float az;
    float gx; float gy; float gz;
    float temp;
} SensorData_t;

typedef struct __attribute__((packed)) {
    uint16_t period;
    uint16_t intensity;
} StepData_t;

typedef struct __attribute__((packed)) {
	uint16_t step_initial_count; 		// initial step count  2B
    StepData_t steps[5]; // Array of 5 steps takes up 20B
    float temp;          // Single temperature reading for the batch  //4B
} sentData_t; 		//26B

// State Variables (Place these above main)
uint8_t batch_index = 0;       // Track which step (0-4) we are filling
uint32_t step_count = 0;       // Global step counter
uint32_t last_step_time = 0;   // Time marker for previous step
uint8_t is_above_threshold = 0;// Lock flag

#define GYRO_TH 175.0

#define OPERATION_2G 16384.0
#define OPERATION_4G 8192.0
#define OPERATION_8G 4096.0
#define OPERATION_16G 2048.0

#define OPERATION_250 131.0
#define OPERATION_500 65.5
#define OPERATION_1000 32.75
#define OPERATION_2000 16.375
// Define these floats outside the loop:
//for device 1
#define ACCEL_X_OFFSET 0
#define ACCEL_Y_OFFSET  0
#define ACCEL_Z_OFFSET 0.135

/* --- GLOBAL VARIABLES --- */
SensorData_t data_imu; // This holds the actual sensor values
sentData_t sentData;
uint8_t TxAddress[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

// FIX 1: Correct Size (Do NOT subtract 1 for binary structs)
const uint8_t MyDataSize = sizeof(SensorData_t);

/* --- HANDLES --- */
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* --- PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void UART_SendString(char *pString);

/* --- HELPER FUNCTION --- */
static void UART_SendString(char *pString) {
    HAL_UART_Transmit(&huart2, (uint8_t*)pString, strlen(pString), HAL_MAX_DELAY);
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

static void LowPowerDelay(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();
    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();

    while ((HAL_GetTick() - start) < delay_ms)
    {
        // Enter Sleep Mode (CPU disabled, Peripherals+SysTick enabled)
        // Wakes up every 1ms via SysTick interrupt to check loop condition
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
}


int main(void)
{
  /* MCU Configuration */



  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  NRF24_Init(&hspi1, GPIOA, GPIO_PIN_9, GPIOC, GPIO_PIN_7);
  NRF24_SetTXAddress(TxAddress);
  NRF24_SetRFChannel(90);
  NRF24_SetDataRate(NRF24_DR_250KBPS);
  NRF24_SetOutputPower(NRF24_PA_LOW);
  NRF24_SetCRCLength(NRF24_CRC_16);
  NRF24_SetTXMode();

  MX_USART2_UART_Init();

  MX_I2C1_Init();

  /* --- NRF24L01 Initialization --- */
  UART_SendString("NRF24L01 Transmitter Initialized.\r\n");

  /* --- MPU6050 Initialization --- */
  char log_buffer[100]; // Buffer for printing
  uint8_t check;
  uint8_t i2c_reg_val;  // Renamed from 'data' to avoid confusion

  printf("--- Program Started ---\r\n");
  	  HAL_Delay(100); // Give it time to send

  	  // 1. Check if I2C device is found
  	  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2, 100) != HAL_OK) {
  		UART_SendString("!!! I2C Device Not Ready. Check wiring. Stuck in Error_Handler().\r\n");
  		Error_Handler();
  	  }

  	UART_SendString("I2C Device Ready.\r\n");
  	  HAL_Delay(100);

  	  // 2. Check the WHO_AM_I register
  	  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 100);

  	  if (check == 0x70) // 0x68 is the default MPU-6050 Who Am I value
  	  {
  		printf("MPU6050 WHO_AM_I check SUCCESS (0x%X). Waking up sensor...\r\n", check);

  		  // Configure accelerometer ±2g
  		i2c_reg_val = 0x08;
  		  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &i2c_reg_val, 1, 100);

  		  // Configure gyroscope ±1000°/s
  		i2c_reg_val = 0x10;
  		  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &i2c_reg_val, 1, 100);

  		  // Wake up the MPU6050
  		i2c_reg_val = 0x00;
  		  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &i2c_reg_val, 1, 100);


  	  } else {
  		printf("!!! MPU6050 WHO_AM_I check FAILED. Value was: 0x%X. Check AD0 pin.\r\n", check);

  		  Error_Handler();
  	  }

  	  UART_SendString("Setup Complete. Entering main loop...\r\n\r\n");
  	  HAL_Delay(100);

  	while (1)
  	  {
  	      uint8_t buffer[14];
  	      // Read 14 bytes (Accel, Temp, Gyro)
  	      HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 100);

  	      if (status == HAL_OK)
  	      {
  	          // 1. Parse raw data
  	          //int16_t ax_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
  	          //int16_t ay_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
  	          //int16_t az_raw = (int16_t)(buffer[4] << 8 | buffer[5]);
  	          int16_t tp_raw = (int16_t)(buffer[6] << 8 | buffer[7]);
  	          //int16_t gx_raw = (int16_t)(buffer[8] << 8 | buffer[9]);
  	          int16_t gy_raw = (int16_t)(buffer[10] << 8 | buffer[11]);
  	          int16_t gz_raw = (int16_t)(buffer[12] << 8 | buffer[13]);

  	          // 2. Convert to float
  	          //data_imu.ax = ax_raw / OPERATION_4G + ACCEL_X_OFFSET;
  	          //data_imu.ay = ay_raw / OPERATION_4G + ACCEL_Y_OFFSET;
  	          //data_imu.az = az_raw / OPERATION_4G + ACCEL_Z_OFFSET;
  	          data_imu.temp = (tp_raw / 310.0f) + 18.53f;
  	          //data_imu.gx = gx_raw / OPERATION_1000;
  	          data_imu.gy = gy_raw / OPERATION_1000;
  	          data_imu.gz = gz_raw / OPERATION_1000;

  	          // 3. Calculate Gyro Swing
  	          float gyro_diff = fabsf(data_imu.gy - data_imu.gz);

  	          uint32_t current_time = HAL_GetTick();
  	          uint32_t time_diff = current_time - last_step_time;

  	          // --- STEP DETECTION LOGIC ---
  	          if (gyro_diff > GYRO_TH && !is_above_threshold)
  	          {
  	              is_above_threshold = 1; // Lock

  	              // CASE 1: Valid Step (Between 300ms and 2000ms)
  	              if ((time_diff >= 250 && time_diff <= 2500) || step_count == 0)
  	              {
  	                  step_count += 1;

  	                  // -- Batching Logic --
  	                  if (batch_index == 0) {
  	                      sentData.step_initial_count = step_count;
  	                  }

  	                  sentData.steps[batch_index].period = (uint16_t)time_diff;
  	                  sentData.steps[batch_index].intensity = (uint16_t)gyro_diff;

  	                  batch_index++;
  	                  last_step_time = current_time;

  	                  // Check if Batch is Full (5 steps)
  	                  if (batch_index >= 5)
  	                  {
  	                      sentData.temp = data_imu.temp;

  	                      // Transmit Full Batch
  	                      NRF24_TX_Result_t res = NRF24_Transmit((uint8_t*)&sentData, sizeof(sentData));

  	                      while(res != NRF24_TX_OK)
  	                      {
  	                          UART_SendString(">> FULL BATCH SENT: FAILED. Retrying in 5s...\r\n");
  	                          LowPowerDelay(5000);
  	                          res = NRF24_Transmit((uint8_t*)&sentData, sizeof(sentData));
  	                      }

  	                      UART_SendString(">> FULL BATCH SENT: OK\r\n");
  	                      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Blink LED

  	                      batch_index = 0; // Reset
  	                  }
  	              }
  	              // CASE 2: New Start (Pause detected > 2000ms)
  	              else if (time_diff > 2500)
  	              {
  	                   last_step_time = current_time;
  	              }
  	          }
  	          // Reset Lock
  	          else if (gyro_diff < 75.0f)
  	          {
  	              is_above_threshold = 0;
  	          }

  	          // --- TIMEOUT FLUSH LOGIC ---
  	          // If data pending AND no steps for > 2 seconds
  	          if (batch_index > 0 && (current_time - last_step_time > 2500))
  	          {
  	              // Fill remaining slots with 0
  	              for(int i = batch_index; i < 5; i++) {
  	                  sentData.steps[i].period = 0;
  	                  sentData.steps[i].intensity = 0;
  	              }

  	              sentData.temp = data_imu.temp;

  	              // Transmit Partial Batch
  	              NRF24_TX_Result_t res = NRF24_Transmit((uint8_t*)&sentData, sizeof(sentData));

  	              while(res != NRF24_TX_OK)
  	              {
  	                  UART_SendString(">> TIMEOUT FLUSH: FAILED. Retrying in 5s...\r\n");
  	                  LowPowerDelay(5000);
  	                  res = NRF24_Transmit((uint8_t*)&sentData, sizeof(sentData));
  	              }

  	              UART_SendString(">> TIMEOUT FLUSH: OK\r\n");
  	              HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  	              batch_index = 0; // Buffer cleared
  	          }

  	          // --- PLOTTER ---
  	          sprintf(log_buffer, "Diff:%.2f,Thresh%.2f:.0\r\n", gyro_diff, GYRO_TH);
  	          UART_SendString(log_buffer);
  	      }
  	      else
  	      {
  	          UART_SendString("I2C Error.\r\n");
  	      }

  	      HAL_Delay(20); // 50Hz Loop
  	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	  hi2c1.Init.Timing = 0x00201D2B;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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

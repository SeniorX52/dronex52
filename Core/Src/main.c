/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "motor.h"
#include "UART.h"
#include "BMP280.h"
#include "MPU6050.h"
#include "PID.h"
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE  50
uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t bufferIndex=0;
MPU6050_HandleTypeDef mpu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pair(void);

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  Motor_Init();
  //pair();
  mpu.hi2c = &hi2c2;
  HAL_Delay(2000);
  MPU6050_Init(&mpu);
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart2, &rx_buffer[bufferIndex], 1);
  BMP280_Init(&hi2c1);
  HAL_Delay(1000);
  BMP280_CalibData calibData;
  BMP280_ReadCalibrationData(&hi2c1, &calibData);
  HAL_Delay(1000);
  PID_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  PID_ControlLoop();
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void pair() {
//	  // Allow time for modules to stabilize
//		HAL_UART_Transmit(&huart3, (uint8_t*)"AT+UART=9600,1,0\r\n", strlen("AT+UART=9600,1,0\r\n"), HAL_MAX_DELAY);
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+ROLE=0\r\n", strlen("AT+ROLE=0\r\n"), HAL_MAX_DELAY);  // Slave mode
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+PSWD=1234\r\n", strlen("AT+PSWD=1234\r\n"), HAL_MAX_DELAY);  // Set PIN
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+NAME=MySlave\r\n", strlen("AT+NAME=MySlave\r\n"), HAL_MAX_DELAY);  // Optional name
//	    HAL_Delay(500);
//	    // Configure Master (UART2)
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+UART=9600,1,0\r\n", strlen("AT+UART=9600,1,0\r\n"), HAL_MAX_DELAY);
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+ROLE=1\r\n", strlen("AT+ROLE=1\r\n"), HAL_MAX_DELAY);  // Master mode
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CMODE=0\r\n", strlen("AT+CMODE=0\r\n"), HAL_MAX_DELAY);  // Pair with specific address
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+PSWD=1234\r\n", strlen("AT+PSWD=1234\r\n"), HAL_MAX_DELAY);  // Set PIN
//	    HAL_Delay(500);
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+INIT\r\n", strlen("AT+INIT\r\n"), HAL_MAX_DELAY);  // Initialize
//	    HAL_Delay(500);
//
//	    // Bind to the specific slave address
//	    char bind_cmd[40];
//	    snprintf(bind_cmd, sizeof(bind_cmd), "AT+BIND=00:22:05:00:51:88\r\n");
//	    HAL_UART_Transmit(&huart2, (uint8_t*)bind_cmd, strlen(bind_cmd), HAL_MAX_DELAY);
//	    HAL_Delay(5000);
//
//	    // Attempt to connect to the slave
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+LINK=00:22:05:00:51:88\r\n", strlen("AT+LINK=00:22:05:00:51:88\r\n"), HAL_MAX_DELAY);
//	    HAL_Delay(10000);
//	    // Check connection state
//	    uint8_t buffer[100] = {0};
//	    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+STATE?\r\n", strlen("AT+STATE?\r\n"), HAL_MAX_DELAY);
//	    HAL_UART_Receive(&huart2, buffer, sizeof(buffer), 5000);  // 5-second timeout
//
//	    if (strstr((char*)buffer, "CONNECTED") != NULL) {
//	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // Turn on connected LED
//	    } else {
//	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//	    	 HAL_UART_Transmit(&huart2, (uint8_t*)"AT+ORGL\r\n", strlen("AT+ORGL\r\n"), HAL_MAX_DELAY);
//	    	 HAL_UART_Transmit(&huart3, (uint8_t*)"AT+ORGL\r\n", strlen("AT+ORGL\r\n"), HAL_MAX_DELAY);
//
//	    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2)
    {
    	if(rx_buffer[bufferIndex]=='\n' || rx_buffer[bufferIndex]=='\r' ){
    		rx_buffer[bufferIndex]='\0';
            if (rx_buffer[0] != '#')
            {
                // Clear the buffer
                memset(rx_buffer, 0, sizeof(rx_buffer));
                bufferIndex = 0;
                UART_sendString("Command does not start with '#', clearing buffer.\n");
            }
            else
            {
                Controller_executeCommand((char*)rx_buffer);
                memset(rx_buffer, 0, sizeof(rx_buffer));
                bufferIndex = 0;
            }

    	}
    	else{
    		if(bufferIndex<RX_BUFFER_SIZE-1){
    			bufferIndex++;
    		}
    		else{
    			UART_sendString((char*)"Instruction is too large\n");
    			memset(rx_buffer, 0, sizeof(rx_buffer));
    			bufferIndex=0;
    		}
    	}

    }
    HAL_UART_Receive_IT(&huart2, &rx_buffer[bufferIndex], 1);

}
void UART_send_Char(char data) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&data, 1, HAL_MAX_DELAY);
}
void UART_receive_Data(char* data,uint16_t size)
{
    HAL_UART_Receive(&huart2, (uint8_t*)data, size, 1000);
}

void UART_send_Data(char* data)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 1000);
}
void UART_sendNumber(int32_t TxNumber) {
    // Handle special case for 0
    if (TxNumber == 0) {
        UART_send_Char('0');
        return;
    }

    // Buffer to hold the number as a string
    char buffer[12]; // Enough to hold max int32_t value and null terminator
    int index = 0;

    // Handle negative numbers
    if (TxNumber < 0) {
        UART_send_Char('-');
        TxNumber = -TxNumber; // Make the number positive for further processing
    }

    // Convert number to string in reverse order
    while (TxNumber != 0) {
        buffer[index++] = (TxNumber % 10) + '0';
        TxNumber /= 10;
    }

    // Send the string in correct order
    for (int i = index - 1; i >= 0; i--) {
        UART_send_Char(buffer[i]);
    }
}


void UART_sendString(char *str) {
    while (*str) {
        HAL_UART_Transmit(&huart2, (uint8_t *)str, 1, HAL_MAX_DELAY);
        str++;
    }
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

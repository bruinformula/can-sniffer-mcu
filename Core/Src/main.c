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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "canbus.h"
#define SD_BUFFER_SIZE 512
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
//CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
struct		CANmessage temp_msg;
uint16_t 	sd_state = 0;
char 		sd_buffer[SD_BUFFER_SIZE];
uint16_t 	sd_idx = 0;
uint32_t 	total_frames_logged = 0;
FATFS 		fs; // Working area for the file system
FRESULT 	res; // To store the result
FIL 		myFile;
UINT 		bytesWritten;
char 		logData[] = "Test string\r\n";
//CAN_TxHeaderTypeDef   TxHeader; /* Header containing the information of the transmitted frame */
//CAN_RxHeaderTypeDef   RxHeader; ; /* Header containing the information of the received frame */
//uint8_t               TxData[8] = {0};  /* Buffer of the data to send */
//uint8_t               RxData[8]; /* Buffer of the received data */
//uint32_t              TxMailbox;  /* The number of the mail box that transmitted the Tx message */
//uint8_t 			  tx;
//uint8_t 			  rx;
//static struct CANmessage CANqueue[CANQUEUE_SIZE];
//static uint32_t CANqueue_write_index;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Logging_Task(void);
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
  MX_USB_PCD_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  CANbus_Init();
  HAL_CAN_Start(&hcan);
  CAN_EnableCollection();
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_IRQHandler(&hcan);

//  HAL_CAN_Receive_IT(&hcan, CAN_RX_FIFO0);


	res = f_mount(&fs, "", 1);

	if (res == FR_OK) {
		sd_state=1;
	}

//	if (f_open(&myFile, "LOG.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
//		f_write(&myFile, logData, strlen(logData), &bytesWritten);
//		f_close(&myFile);
//	}

//	if (f_open(&myFile, "LOG.TXT", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK){
//		f_lseek(&myFile, f_size(&myFile));
//	}
	memset(sd_buffer, 0, 512); // Clean the buffer
	sd_idx = 0;                // Reset index
	HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  Logging_Task();



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
////	CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
////
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
//  sFilterConfig.FilterBank = 0;
//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  sFilterConfig.FilterIdHigh = 0x0000;
//  sFilterConfig.FilterIdLow = 0x0000;
//  sFilterConfig.FilterMaskIdHigh = 0x0000;
//  sFilterConfig.FilterMaskIdLow = 0x0000;
//  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//  sFilterConfig.FilterActivation = ENABLE;
//  sFilterConfig.SlaveStartFilterBank = 14;
//  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK){
//	  //filter configuration error
//	  Error_Handler();
//  }
//  //Starting CAN peripheral
//  if (HAL_CAN_Start(&hcan) != HAL_OK){
//	  //start error
//	  Error_Handler();
//  }
//  //Activate CAN RX notification on FIFO0
//  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)){
//	  //notification error
//	  Error_Handler();
//  }
////
  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Logging_Task(void){
	while(CAN_Dequeue(&temp_msg)){
		char line[64];

		int len = sprintf(line, "%03lX,%d,%02X%02X%02X%02X%02X%02X%02X%02X\n",
                temp_msg.Id, temp_msg.DLC,
                temp_msg.Data[0], temp_msg.Data[1], temp_msg.Data[2], temp_msg.Data[3],
                temp_msg.Data[4], temp_msg.Data[5], temp_msg.Data[6], temp_msg.Data[7]);

		// Add the IRQ count as a "fake" data byte to see if it's changing
//		int len = sprintf(line, "%03lX,%d,IRQ:%lu\n",
//		                  temp_msg.Id, temp_msg.DLC, can_rx_irq_count);

		if (sd_idx + len >= 512){
			UINT bw;
			if (f_open(&myFile, "DATA4.TXT", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
				f_lseek(&myFile, myFile.fsize);
				f_write(&myFile, sd_buffer, sd_idx, &bw);
				f_close(&myFile);
			}
			sd_idx = 0;
		}

		memcpy(&sd_buffer[sd_idx], line, len);
		sd_idx += len;

		total_frames_logged++;
	}

}



//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
//	{
//		if (GPIO_PIN == GPIO_PIN_13)
//		{
//			if (led_state == 0){
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//				led_state = 1;
//			}
//			else{
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//				led_state = 0;
//			}
//		}
//
//	}
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

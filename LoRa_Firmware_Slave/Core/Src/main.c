/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1278.h"
#include "SX1278_hw.h"
#include "string.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
SX1278_pack_t ack;
SX1278_pack_t packet_sent;
SX1278_pack_t previous_pack;
int ret;
int retry = 0;
char buffer[512];
int count = 0;
int message = 0;
int message_length;
int send_packets = 0;
char ackBuffer[512];
int state = 0;
int retack;
uint32_t time_val_pr = 0;
uint32_t time_val_nw = 0;
uint8_t comp=0;
int ack_retrying = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len) {
	int i;
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	for (i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
//initialize LoRa module
  SX1278_hw.dio0.port = DIO0_GPIO_Port;
  SX1278_hw.dio0.pin = DIO0_Pin;
  SX1278_hw.nss.port = NSS_GPIO_Port;
  SX1278_hw.nss.pin = NSS_Pin;
  SX1278_hw.reset.port = RESET_GPIO_Port;
  SX1278_hw.reset.pin = RESET_Pin;
  SX1278_hw.spi = &hspi2;

  SX1278.hw = &SX1278_hw;
  printf("Configuring LoRa module\r\n");
  //node addr is 1
  SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM,120, SX1278_LORA_SF_7,
		  SX1278_LORA_BW_250KHZ, SX1278_LORA_CR_4_5,1, SX1278_LORA_CRC_DIS, 10);
  printf("Done configuring LoRaModule\r\n");
  ret = SX1278_LoRaEntryTx(&SX1278, 255, 2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  switch(state)
		  {
		  case 0:
			  packet_sent.packnum = message;
			  message_length = sprintf(buffer,"Hello D");
//			  HAL_Delay(1000);
			  ret = SX1278_LoRaEntryTx(&SX1278, 255, 2000);
			  ret = SX1278_LoRaTxPacket(&SX1278, &packet_sent, (uint8_t*)buffer, 255, 2000, 2);
			  if (ret == 1){
				  SX1278_copypacket(&previous_pack, &packet_sent);
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  state = 1 ; message++;
			  }
			  else state = 0;
			  break;

		  case 1:
			  //wait for ack
//			  HAL_Delay(200);
			  retack = SX1278_LoRaEntryRx(&SX1278, 255, 2000);
			  HAL_Delay(800);
			  retack = SX1278_LoRaRxPacketACK(&SX1278, &ack, &packet_sent);
			  comp = SX1278_Compare(&SX1278, &ack, &packet_sent);
			  if (retack > 0)state = 0;
			  else {
				  while(ack_retrying) {
					  HAL_TIM_Base_Start_IT(&htim16);
					  state = 1;
				  }
				  HAL_TIM_Base_Stop(&htim16);
				  state = 2;
				  ack_retrying = 1;
			  }
			  break;

		  case 2:

			  if (retry <= 3){
//				  message_length = sprintf(buffer,"Hello D");
//				  HAL_Delay(1000);
				  ret = SX1278_LoRaEntryTx(&SX1278, 255, 2000);
				  ret = SX1278_LoRaTxPacketRetry(&SX1278, &previous_pack, 2,2000);
				  if (ret == 1){
					  state = 1;retry ++;packet_sent.retry = retry;}
				  else state = 2;}
			  if (retry > 3){
				  state = 0;retry = 0;packet_sent.retry = retry;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim16)
	{
		ack_retrying = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DIO0_Pin){

	printf("interrupt occured\r\n");
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
//  __disable_irq();
//  while (1)
//  {
//  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

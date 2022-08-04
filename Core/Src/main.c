/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "hid_commands.h"
#include "shifterG27.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// USB Handler
extern USBD_HandleTypeDef hUsbDeviceFS;

// Objects
usbMapping_t SimCentralizerHID = {{0,0,0,0},0};
ShifterG27_t Shifter = {0,0,0};

// Values
uint8_t gear = 0;
uint8_t seq = 0;
uint8_t mode = 0;
uint8_t mode_tmp1 = 0;
uint8_t mode_tmp2 = 0;
uint8_t shift = 0;

// ADC Channel
uint32_t adcChannel[3] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2
};

// ADC Axis
uint8_t adcAxis[3] = {0,0,0};

// SPI Buffer
uint8_t rx_buffer[2] = {0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// To use ADC multiple channels (variable from MX_ADC1_Init)
ADC_ChannelConfTypeDef sConfig = {0};

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// G27 SPI ButtonBox
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); 							// Start SPI Communication
	HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, rx_buffer, sizeof(rx_buffer), 3000); 	// Get SPI Datas

	switch(status)
	{
		case HAL_OK:
			Shifter.button = 0;
			if ((rx_buffer[0] & 0xff) != 0xff)   // if all bits of rx_buffer[0] is 1 assume shifter is disconnected
			{
				// Reverse gear
				if (rx_buffer[0] == 96) Shifter.reverse = 1;
				else Shifter.reverse = 0;
				
				// Mode button
				if (rx_buffer[0] == 1) mode_tmp1 = 1;
				else mode_tmp1 = 0;

				// Shift button
				if (rx_buffer[0] == 3) shift = 1;
				else shift = 0;

				// Red buttons
				if (rx_buffer[0] == 6) Shifter.button = 1;
				if (rx_buffer[0] == 7) Shifter.button = 11; // Shift + Button 1
				if (rx_buffer[0] == 12) Shifter.button = 2;
				if (rx_buffer[0] == 15) Shifter.button = 12; // Shift + Button 2
				
				// Black Dpad
				if (rx_buffer[1] == 1) Shifter.button = 3;
				if (rx_buffer[1] == 12) Shifter.button = 4;
				if (rx_buffer[1] == 3) Shifter.button = 5;
				if (rx_buffer[1] == 6) Shifter.button = 6;

				// Black buttons
				if (rx_buffer[1] == 224) Shifter.button = 7;
				if (rx_buffer[1] == 96) Shifter.button = 8;
				if (rx_buffer[1] == 24) Shifter.button = 9;
				if (rx_buffer[1] == 48) Shifter.button = 10;
			}
			break;

		case HAL_TIMEOUT:
		case HAL_BUSY:
		case HAL_ERROR: Error_Handler();
		default: break;
	}
	
	// Close SPI Communication with G27 Shifter
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


	// Axis : Read ADC
	for (uint8_t i=0; i<3; ++i) {
		sConfig.Channel = adcChannel[i]; 				// Change channel
		HAL_ADC_ConfigChannel(&hadc1, &sConfig); 		// Apply changes
		HAL_ADC_Start(&hadc1); 							// Start ADC module
		HAL_ADC_PollForConversion(&hadc1, 10); 			// Request for conversion
		adcAxis[i] = HAL_ADC_GetValue(&hadc1) >> 4; 	// Read ADC value and store it into an axis (>> 4 means for 12bits -> 8bits)
	}
	
	// Axis : Set values
	SimCentralizerHID.z = adcAxis[2] - 128;
	
	// Shifter : Get Axis & Reverse
	Shifter.xAxis = adcAxis[0];
	Shifter.yAxis = adcAxis[1];


	// Shifter : Mode H/Seq
	if (mode_tmp1 == 1) {
		mode_tmp2 = 1;
	}
	else if (mode_tmp1 == 0) {
		if (mode_tmp1 != mode_tmp2) {
			mode ^= 1;
			gear = 0;
			mode_tmp2 = 0;
		}
    }

	// Shifter : Set Gear
    if (!mode)
    {
        // H Shifter
        gear = getGear(&Shifter);       								// Read active H gear
		
		// If gear engaged
        if(gear != 0) {
			if (gear == 9) pressButton(&SimCentralizerHID, gear - 1); 	// Send USB active H reverse
			else pressButton(&SimCentralizerHID, gear); 				// Send USB active H gear
		}
    }
    else
    {
		// Sequential Up
        if (getGear(&Shifter) == 4)
		{
			// Send USB gear up
            pressButton(&SimCentralizerHID, 9);
            USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&SimCentralizerHID, sizeof(usbMapping_t));
			// Buffer variable for Switch State
			seq = 4;
        } 
        // Sequential Down
        else if (getGear(&Shifter) == 3)
		{
			// Send USB gear down
            pressButton(&SimCentralizerHID, 10);
            USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&SimCentralizerHID, sizeof(usbMapping_t));
			// Buffer variable for Switch State
			seq = 3;
		}
		else if (getGear(&Shifter) == 0)
		{
			if (seq == 0) {
				seq = 0; 								// Do nothing (just reset seq to 0)
			}
			else if (seq == 4) {
				if (gear < 8) gear++; 					// Up Gear
            	else if (gear == 9) gear = 0; 			// Reverse to Neutral
				seq = 0;
			}
			else if (seq == 3) {
				if ((gear > 0) && (gear < 9)) gear--; 	// Down Gear
				else gear = 9; 							// Neutral to Reverse
				seq = 0;
			}
		}
    }

	// G27 ButtonBox
	if (Shifter.button) {
		if (shift) pressButton(&SimCentralizerHID, Shifter.button + 20); 	// +20 because : 6 Gears + 1 Gear7 + 1 Reverse + 2 Sequential + Shift
		else pressButton(&SimCentralizerHID, Shifter.button + 10); 			// +10 because : 6 Gears + 1 Gear7 + 1 Reverse + 2 Sequential
	}
	else releaseButtons(&SimCentralizerHID); 								// If no button

	// HID : Send HID report
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&SimCentralizerHID, sizeof(usbMapping_t));

	// Reset Shifter gear
	releaseShifter(&SimCentralizerHID);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */


  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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


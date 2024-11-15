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
#include "string.h"
#include "stm32l4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern SPI_HandleTypeDef hspi1;  // De SPI interface die we gebruiken
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//UART
#define RxBuf_SIZE 10 //16 bits geeft waarde van 0 tm 65535, dus maximaal 5 decimalen + \r\n dus 7 decimalen maximaal. Om errors te voorkomen is de buffer 10
#define MainBuf_SIZE 10

//MAX4820
extern SPI_HandleTypeDef hspi2;
#define MAX4820_CS_PIN    GPIO_PIN_5   // De GPIO pin die verbonden is met CS
#define MAX4820_CS_PORT   GPIOB        // De GPIO poort die verbonden is met CS


//Registers adres definitie van de DAC80501
#define ADDR_NOOP_register 0b00000000 //No operation, Hex 0x00
#define ADDR_DEVID_register 0b00000001 //Device identification, Hex 0x01
#define ADDR_SYNC_register 0b0000 0010 //Synchronisation, Hex 0x02
#define ADDR_CONFIG_register 0b00000011 //Configuration, Hex 0x03
#define ADDR_GAIN_register 0b00000100 //Gain, Hex 0x04
#define ADDR_TRIGGER_register 0b00000101 //Trigger, Hex 0x05
#define ADDR_STATUS_register 0b00000111 //Status, Hex 0x07
#define ADDR_DAC_DATA_register 0b00001000 //Digital-to-analog converter, Hex 0x08

//Register Data van de DAC80501
#define DATA_NOOP 0x00
	//#define DATA_DEVID_register 0b0000 0001 0001 0101, kan worden uitgelezen
#define DATA_SYNC_register 0x00
#define DATA_CONFIG_register 0b0000000100000000 //disabling internal reference & power-down mode
#define DATA_GAIN_register 0x00 //referencespanning delen door 1 & gain op 1 zetten
#define DATA_TRIGGER_register 0x0A //dit reset de dac  & zet de dac in asynchronous mode(default)
	//#define DATA_STATUS_register 0x01 geeft aan of DAC wordt uitgeschakeld, 0x00 geeft aan dat dac is ingeschakeld
#define DAC80501_SPI_TIMEOUT 100



//SS pin van de DAC, verander zonodig
#define DAC_SS_Pin GPIO_PIN_6
#define DAC_SS_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
char TxBuf[RxBuf_SIZE];
uint16_t convertedString;

//MAX4820
uint8_t MAX4820_SS_Pin = GPIO_PIN_5; //port B

//DAC80501
uint8_t DacAdres = 0b10010010;
uint16_t DATA__DAC_DATA_register;
HAL_StatusTypeDef check;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
//MAX4820
void MAX4820_Select(void)
{
    HAL_GPIO_WritePin(MAX4820_CS_PORT, MAX4820_CS_PIN, GPIO_PIN_RESET); // CS low
}
void MAX4820_Deselect(void)
{
    HAL_GPIO_WritePin(MAX4820_CS_PORT, MAX4820_CS_PIN, GPIO_PIN_SET);   // CS high
}

void MAX4820_SendData(uint8_t data)
{
	uint8_t txData[1];
	txData[0] = data;

    MAX4820_Select();  // Activeren van CS
    HAL_SPI_Transmit(&hspi2, txData, 1, HAL_MAX_DELAY);  // Verzenden van 8 bits data
    MAX4820_Deselect();  // Deactiveren van CS
}

void MAX4820_5V_pos(void)
{
	MAX4820_SendData(0b00000110); //dec: 6
}


void MAX4820_60V_pos(void)
{
	MAX4820_SendData(0b00000101); //dec:5
}

void MAX4820_5V_neg(void)
{
	MAX4820_SendData(0b00001010); //dec: 10
}


void MAX4820_60V_neg(void)
{
	MAX4820_SendData(0b00001001); //dec: 9
}


HAL_StatusTypeDef DAC80501Reset(void)
{
    HAL_StatusTypeDef status;
    uint8_t txData[3];

    // Stel het adres en de data in voor het trigger-register van de DAC80501
    txData[0] = ADDR_TRIGGER_register;                     // Registeradres
    txData[1] = (DATA_TRIGGER_register >> 8 ) & 0xFF;      // MSB van de 16-bits data
    txData[2] = (DATA_TRIGGER_register & 0xFF);              // LSB van de 16-bits data

    // Zet de Chip Select (CS/SS) lijn laag om SPI-communicatie te starten
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);

    // Verzend de 24-bits data via SPI
    status = HAL_SPI_Transmit(&hspi1, txData, 3, DAC80501_SPI_TIMEOUT);

    // Zet de Chip Select (CS/SS) lijn hoog om SPI-communicatie te beëindigen
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);

    // Retourneer de status van de SPI-communicatie (HAL_OK of foutcode)
    return status;
}

HAL_StatusTypeDef DAC80501Config(void)
{
	 HAL_StatusTypeDef status;
	 uint8_t txData[3];

	 txData[0] =  ADDR_CONFIG_register;                     // Registeradres
	 txData[1] = (DATA_CONFIG_register >> 8 ) & 0xFF;      // MSB van de 16-bits data
	 txData[2] = (DATA_CONFIG_register & 0xFF);              // LSB van de 16-bits data

	 // Zet de Chip Select (CS/SS) lijn laag om SPI-communicatie te starten
	 HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);

	 // Verzend de 24-bits data via SPI
	 status = HAL_SPI_Transmit(&hspi1, txData, 3, DAC80501_SPI_TIMEOUT);

	 // Zet de Chip Select (CS/SS) lijn hoog om SPI-communicatie te beëindigen
	 HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);

	 // Retourneer de status van de SPI-communicatie (HAL_OK of foutcode)
	 return status;
}

HAL_StatusTypeDef DAC80501Gain(void)
{
	 HAL_StatusTypeDef status;
	 uint8_t txData[3];

	 txData[0] = ADDR_GAIN_register ;                     // Registeradres
	 txData[1] = (DATA_GAIN_register >> 8 ) & 0xFF;      // MSB van de 16-bits data
	 txData[2] = (DATA_GAIN_register & 0xFF);              // LSB van de 16-bits data

	 // Zet de Chip Select (CS/SS) lijn laag om SPI-communicatie te starten
	 HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);

	 // Verzend de 24-bits data via SPI
	 status = HAL_SPI_Transmit(&hspi1, txData, 3, DAC80501_SPI_TIMEOUT);

	 // Zet de Chip Select (CS/SS) lijn hoog om SPI-communicatie te beëindigen
	 HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);

	 // Retourneer de status van de SPI-communicatie (HAL_OK of foutcode)
	 return status;
}

HAL_StatusTypeDef DAC80501DataVoltage5V(uint16_t voltage_mV)  //data(getal) moet in mV zijn van 0 tot 5000mV
{
	//Zorgen dat spanning tussen 0-5000mV zit
	if(voltage_mV > 5000){ //Houdt de spanning onder de 5000mV
		voltage_mV = 5000;
	}
	if(voltage_mV < 0){ //Houdt de spanning boven de 0mV. Werkt niet goed vanwerge unsigned naar signed conversion bij functie aanroep
		voltage_mV = 0;
	}

    HAL_StatusTypeDef status;
    uint16_t convertedDataVoltage = (uint16_t)(((uint32_t)voltage_mV * 65535) / 5000); // 0-5000mV omzetten naar 16 bits waarde die naar de dac gestuurd kan worden. gebruik gehele getallen
    uint8_t txData[3];

    // Stel het adres en de data in voor het trigger-register van de DAC80501
    txData[0] = ADDR_DAC_DATA_register;                     // Registeradres
    txData[1] = (convertedDataVoltage >> 8 ) & 0xFF;      // MSB van de 16-bits data
    txData[2] = (convertedDataVoltage & 0xFF);              // LSB van de 16-bits data

    // Zet de Chip Select (CS/SS) lijn laag om SPI-communicatie te starten
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);

    // Verzend de 24-bits data via SPI
    status = HAL_SPI_Transmit(&hspi1, txData, 3, DAC80501_SPI_TIMEOUT);

    // Zet de Chip Select (CS/SS) lijn hoog om SPI-communicatie te beëindigen
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);

    // Retourneer de status van de SPI-communicatie (HAL_OK of foutcode)
    return status;
}

HAL_StatusTypeDef DAC80501DataVoltage60V(uint16_t voltage_mV)  //data(getal) moet in mV zijn van 0 tot 60000mV
{
	//Zorgen dat spanning tussen 0-5000mV zit
	if(voltage_mV > 60000){ //Houdt de spanning onder de 5000mV
		voltage_mV = 60000;
	}
	if(voltage_mV < 0){ //Houdt de spanning boven de 0mV. Werkt niet goed vanwerge unsigned naar signed conversion bij functie aanroep
		voltage_mV = 0;
	}

    HAL_StatusTypeDef status;
    uint16_t convertedDataVoltage = (uint16_t)(((uint32_t)voltage_mV * 65535) / (5000*12)); // 0-60000mV omzetten naar 16 bits waarde die naar de dac gestuurd kan worden. gebruik gehele getallen
    uint8_t txData[3];

    // Stel het adres en de data in voor het trigger-register van de DAC80501
    txData[0] = ADDR_DAC_DATA_register;                     // Registeradres
    txData[1] = (convertedDataVoltage >> 8 ) & 0xFF;      // MSB van de 16-bits data
    txData[2] = (convertedDataVoltage & 0xFF);              // LSB van de 16-bits data

    // Zet de Chip Select (CS/SS) lijn laag om SPI-communicatie te starten
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);

    // Verzend de 24-bits data via SPI
    status = HAL_SPI_Transmit(&hspi1, txData, 3, DAC80501_SPI_TIMEOUT);

    // Zet de Chip Select (CS/SS) lijn hoog om SPI-communicatie te beëindigen
    HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);

    // Retourneer de status van de SPI-communicatie (HAL_OK of foutcode)
    return status;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart->Instance == USART2){
		memcpy(MainBuf, RxBuf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}

	//Kies een van de vier
	convertedString = (uint16_t)atoi((char* )RxBuf);
	MAX4820_60V_pos();DAC80501DataVoltage60V(convertedString);


	//MAX4820_5V_pos();
	//MAX4820_60V_neg();
	//MAX4820_5V_neg();
	HAL_UART_Transmit(&huart2, MainBuf, Size, 1000);

	memset(RxBuf,0,strlen(RxBuf));

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
   __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
   HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


   MAX4820_60V_pos();
  // MAX4820_5V_neg();
  // MAX4820_60V_neg();
  while (1)
  {

	  //DAC80501DataVoltage5V(4000);
//	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  //Deze code wordt gebruikt om de CPOL initial value op 1 te zetten. verander alleen de GPIO_InitStruct.Pin pin & HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); poort
    GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_5; // GPIOD Pin 1 is the SPI2_CLK
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      if (hspi1.Init.CLKPolarity==SPI_POLARITY_HIGH) {
        GPIO_InitStruct.Pull = GPIO_PULLUP;
      } else {
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      }
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      HAL_Delay(1);
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, SS_MAX4820_Pin|SS_DAC80501_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SS_MAX4820_Pin SS_DAC80501_Pin */
  GPIO_InitStruct.Pin = SS_MAX4820_Pin|SS_DAC80501_Pin;
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

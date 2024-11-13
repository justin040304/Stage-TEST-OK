/* USER CODE BEGIN Header */
//   Voor het instellen van de DAC moet de CPOL = 1 en CPHA = 0(1 Edge). een ding is dat de CPOL = 1 wordt nadat eerste bericht is verstuurd.
// dus voeg de code in "static void MX_SPI1_Init(void)" onder het kopje USER CODE BEGIN SPI1_Init 2


  /* USER CODE END SPI1_Init 2 */

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
#include "stm32l4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern SPI_HandleTypeDef hspi1;  // De SPI interface die we gebruiken
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




//Registers adres definitie van de DAC80501
#define ADDR_NOOP_register 0x00 //No operation, Hex 0x00
#define ADDR_DEVID_register 0x01 //Device identification, Hex 0x01
#define ADDR_SYNC_register 0x02 //Synchronisation, Hex 0x02
#define ADDR_CONFIG_register 0x03 //Configuration, Hex 0x03
#define ADDR_GAIN_register 0x04 //Gain, Hex 0x04
#define ADDR_TRIGGER_register 0x05 //Trigger, Hex 0x05
#define ADDR_STATUS_register 0x07 //Status, Hex 0x07
#define ADDR_DAC_DATA_register 0x08 //Digital-to-analog converter, Hex 0x08

//Register Data van de DAC80501
#define DATA_NOOP 0x0000
	//#define DATA_DEVID_register 0b0000 0001 0001 0101, kan worden uitgelezen
#define DATA_SYNC_register 0x0000
//#define DATA_CONFIG_register 0x0101 //disabling internal reference & power-down mode
//#define DATA_GAIN_register 0x00 //referencespanning delen door 1 & gain op 1 zetten
#define DATA_TRIGGER_register 0x000A //dit reset de dac  & zet de dac in asynchronous mode(default)
	//#define DATA_STATUS_register 0x01 geeft aan of DAC wordt uitgeschakeld, 0x00 geeft aan dat dac is ingeschakeld
#define DAC80501_SPI_TIMEOUT 100

//#define DATA_CONFIG_register 0b0000000100000000 //disabling internal reference & power-down mode
//#define DATA_GAIN_register 0x00 //referencespanning delen door 1 & gain op 1 zetten
#define DATA_CONFIG_register 0x0000 //enabling internal reference & dis power-down mode
#define DATA_GAIN_register 0x0001 //referencespanning delen door 1 & gain op 2 zetten

//SS pin van de DAC, verander zonodig
#define DAC_SS_Pin GPIO_PIN_6
#define DAC_SS_Port GPIOB




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t DacAdres = 0b10010010;
uint16_t DATA__DAC_DATA_register;
uint8_t dummyData = 0b00000000;



HAL_StatusTypeDef check;

//stuur eerst DACadres, Register adres en dan data respectievelijk

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */



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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //SPI starten zonder daadwerkelijk iets aan te sturen
//  HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi1, &dummyData, 1, DAC80501_SPI_TIMEOUT);
//  HAL_GPIO_WritePin(DAC_SS_Port, DAC_SS_Pin, GPIO_PIN_SET);
//  HAL_Delay(1);

  //DAC80501Reset();
 // DAC80501Config();
//  DAC80501Gain();


//DAC80501DataVoltage5V(4999); //getal moet in mV zijn 0-5000mV
//  DAC80501DataVoltage60V();

  while (1)
  {
	  DAC80501DataVoltage60V(30000);
	  HAL_Delay(1000);
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
  //Deze code wordt gebruikt om de CPOL initial value op 1 te zetten. verander alleen de GPIO_InitStruct.Pin pin & HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); poort
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//       GPIO_InitStruct.Pin = GPIO_PIN_5; // GPIOD Pin 1 is the SPI2_CLK
//       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//       if (hspi1.Init.CLKPolarity==SPI_POLARITY_HIGH) {
//         GPIO_InitStruct.Pull = GPIO_PULLUP;
//       } else {
//         GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//       }
//       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//       GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
//       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//       HAL_Delay(1);

       GPIO_InitTypeDef GPIO_InitStruct = {0};
       // Configureer SCK Pin
       GPIO_InitStruct.Pin = GPIO_PIN_5; // GPIOA Pin 5 voor SPI1_SCK
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       if (hspi1.Init.CLKPolarity == SPI_POLARITY_HIGH) {
           GPIO_InitStruct.Pull = GPIO_PULLUP;
       } else {
           GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       }
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
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

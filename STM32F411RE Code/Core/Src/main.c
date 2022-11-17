/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "inttypes.h"
#include "fonts.h"
#include "ssd1331.h"
#include "stdio.h"
#include "debounce.h"
#include "stdlib.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void passwordInput(int oneInput, int twoInput, int threeInput, char *passwordEntered);
void passwordMatch(char *passwordEntered, int *ifMatch);
void motorRotate(int8_t dir);
void tempDisplay(double temp);
void tempMeasure(void);
void buzzerAlarm(uint8_t match);
/* USER CODE BEGIN PFP */
uint16_t buttonPinOne = GPIO_PIN_3; //PB3
uint16_t buttonPinTwo = GPIO_PIN_4; //PB4
uint16_t buttonPinThree = GPIO_PIN_5; //PB5
uint16_t buttonPinFour = GPIO_PIN_10; //PB10
char port = 'B';
int8_t mode = 0;
int activatedPassword[] = {1111, 2222, 3333, 1233};

int stepsCounter = 0;
int32_t stepRequired = 400;
int flag = 0;
int stepEnable = 0;
int state = 1;
int timeStamp = 0;

uint16_t adcValue = 0;
double temperatureMeasured = 0;
double avgTemperature = 0;


	//IWDG_HandleTypeDef hiwdg;


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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  ssd1331_init();
  /* USER CODE BEGIN 2 */
  char myMessage1[] = "Smart";
  char myMessage2[] = "Access";
  char myMessage3[] = "Control";
  char myMessage4[] = "System";

  deBounceInit(buttonPinOne, port, mode);
  deBounceInit(buttonPinTwo, port, mode);
  deBounceInit(buttonPinThree, port, mode);
  deBounceInit(buttonPinFour, port, mode);

  /* USER CODE BEGIN 2 */

  int portB3Contains = 1;
  int portB4Contains = 1;
  int portB5Contains = 1;
  int portB10Contains = 1;
  int ifMatch = 0;
  char passwordEntered[] = "0000";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  switch(state) //To enter stages
  {
	  case 1:
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
		  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
		  while(portB10Contains) //Check if the Ok button is pushed down
		  {
			  ssd1331_clear_screen(BLACK);
			  ssd1331_display_string(0, 0, myMessage1, FONT_1608, WHITE);
			  ssd1331_display_string(0, 16, myMessage2, FONT_1608, WHITE);
			  ssd1331_display_string(0, 32, myMessage3, FONT_1608, WHITE);
			  ssd1331_display_string(0, 48, myMessage4, FONT_1608, WHITE);

			  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
			  while(portB10Contains) // Continue till Ok button is pressed
			  {
				  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
				  tempMeasure();
			  }
		  }

		  state = 2;

		  break;



	  case 2:
		  portB3Contains = deBounceReadPin(buttonPinOne, port, mode);
		  portB4Contains = deBounceReadPin(buttonPinTwo, port, mode);
		  portB5Contains = deBounceReadPin(buttonPinThree, port, mode);
		  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);

          //Check if the first pin of password (One, Two, Three Pin Input) or Cancel button is pushed down
		  while(portB3Contains && portB4Contains && portB5Contains && portB10Contains)
		  {
			  ssd1331_clear_screen(BLACK);
			  ssd1331_display_string(1, 1, "Enter Pin", FONT_1608, WHITE);

              //Continue till the first pin of password (One, Two, Three Pin Input) or Cancel button is pushed down

			  while(portB3Contains && portB4Contains && portB5Contains && portB10Contains)
			  {
				  portB3Contains = deBounceReadPin(buttonPinOne, port, mode);
				  portB4Contains = deBounceReadPin(buttonPinTwo, port, mode);
				  portB5Contains = deBounceReadPin(buttonPinThree, port, mode);
				  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
				  tempMeasure();
			  }
		  }
			  if (portB10Contains == 0) // if Cancel Button is pressed, go to state 4
			  {
				state = 1;
			  }

			  // if the first pin of password (One, Two, Three Input) button is pushed down, record the 4 digits pin
			  // entered by customer then continue till customer press 'OK' button and go to state 5
			  else
			  {
				  //record the 4 digits pin entered by customer
				  passwordInput(portB3Contains, portB4Contains, portB5Contains, passwordEntered);
				  state = 3;
				  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
					  while(portB10Contains) // Continue till Customer press 'OK'
					  {
					  portB10Contains = deBounceReadPin(buttonPinFour, port, mode);
					  tempMeasure();
					  }
			  }
			  break;


	  case 3:
		  ssd1331_clear_screen(BLACK);
		  ssd1331_display_string(1, 1, "Processing", FONT_1608, WHITE);
		  ssd1331_display_string(1, 16, "...", FONT_1608, WHITE);
		  HAL_Delay(500); //Print the "Processing" for 500s
		  //Check if password Input by customer match one of the activated passwords
		  passwordMatch(passwordEntered, &ifMatch);
		  if (ifMatch == 1) //if match, go to state 1 and print'Successful'
		  {
			  ssd1331_clear_screen(BLACK);
			  ssd1331_display_string(1, 1, "Successful!", FONT_1608, WHITE);
			  printf("Someone just entered the password successfully!\n");
			  stepEnable = 1; //turn on the motor
			  timeStamp = HAL_GetTick();
			  motorRotate(1);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
			  state = 4;
		  }
		  else if (ifMatch == 0) //if does not match, go to state 6
		  {
			  state = 5;
		  }
		  break;

	  case 4:
		  ssd1331_display_string(1, 1, "Successful!", FONT_1608, WHITE);
		  stepEnable = 1; //turn on the motor
		  tempMeasure();

		  break;

	  case 5:
		  ssd1331_clear_screen(BLACK);
		  ssd1331_display_string(1, 1, "Invaild", FONT_1608, WHITE);
		  ssd1331_display_string(1, 16, "Password!", FONT_1608, WHITE);
		  printf("Alert!! Someone just entered Invaild Password!\n");
		  buzzerAlarm(0);
		  state = 1; //go to state 1
		  break;

	  default:
		  state = 1; //go to state 1
	  	  }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void passwordInput(int oneInput, int twoInput, int threeInput, char *passwordEntered)
{
	if (oneInput == 0) //check if the first pin of password entered by customer is 1
	{
		passwordEntered[0] = '1';
	}
	else if (twoInput == 0) //check if the first pin of password entered by customer is 2
	{
		passwordEntered[0] = '2';
	}
	else if (threeInput == 0) //check if the first pin of password entered by customer is 3
	{
		passwordEntered[0] = '3';
	}
	//printf("*\r\n");
	HAL_Delay(100);
	ssd1331_display_string(0, 32, "*", FONT_1608, WHITE);

	for (uint8_t j = 1; j < 4; j++) //check if the second to fourth pin entered by customer
	{
	    oneInput = deBounceReadPin(buttonPinOne, port, mode);
		twoInput = deBounceReadPin(buttonPinTwo, port, mode);
		threeInput = deBounceReadPin(buttonPinThree, port, mode);
		while (oneInput && twoInput && threeInput) //continue till the second to fourth pin entered by customer
		{
			oneInput = deBounceReadPin(buttonPinOne, port, mode);
			twoInput = deBounceReadPin(buttonPinTwo, port, mode);
			threeInput = deBounceReadPin(buttonPinThree, port, mode);
		}

		if (oneInput == 0) //check if the pin of password entered by customer is 1
		{
			passwordEntered[j] = '1';
		}
		else if (twoInput == 0) //check if the pin of password entered by customer is 2
		{
			passwordEntered[j] = '2';
		}
		else if (threeInput == 0) //check if the pin of password entered by customer is 3
		{
			passwordEntered[j] = '3';
		}
		//printf("*\r\n");
		ssd1331_display_string((j * 20), 32, "*", FONT_1608, WHITE);
		HAL_Delay(100);
	}
}


void passwordMatch(char *passwordEntered, int *ifMatch)
{
	int passwordInt = atoi(passwordEntered);
	*ifMatch = 0;
	for (uint8_t i = 0; i < 9; i++) // To loop all index of activated passwords
	{
		// check if the password entered by customer match one of the activated passwords
		if (passwordInt == activatedPassword[i])
		{
			*ifMatch = 1;
		}
	}
}

void motorRotate(int8_t dir)
{
	stepEnable = 1;
	if (dir == 1) // open the door
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); //0E -- enable
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET); //FR -- 0 backward, forward 1
	}
	else if (dir == -1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); //0E -- enable
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET); //FR -- 0 backward, forward 1
	}
}

void tempDisplay(double temp)
{
	char tempString[5];
	sprintf(tempString,"%.1f", temp);

	ssd1331_display_string(60, 50, tempString, 12, GREEN);
	ssd1331_display_string(85, 50, "C", 12, GREEN);
}

void tempMeasure()
{
	  double sum = 0; //accumulator of measured temperature values
	  double sampleMeasured[10] = {0};

	  // find the average temperature
	  for(uint16_t i = 0; i < 10; i++) //calculate the accumulated measured temperature
	  {
		  if (HAL_ADC_Start(&hadc1) !=HAL_OK)
		  {
			Error_Handler();
		  }

		  if (HAL_ADC_PollForConversion(&hadc1, 2) !=HAL_OK)
		  {
			Error_Handler();
		  }
		  else{
			  adcValue = HAL_ADC_GetValue(&hadc1);
			  //temperatureMeasured = (((adcValue + 67.385) / 1231.3) - 0.5258) * 100;
			  temperatureMeasured = ((3.3 * adcValue / 4096) - 0.5258) * 100;
			  sampleMeasured[i] = temperatureMeasured;
			  sum = sum + sampleMeasured[i];
		  }
	  }
	  avgTemperature = sum / 10; //average the accumulated measured temperature
	  tempDisplay(avgTemperature);
	  if (avgTemperature > 23)
	  {
		  printf("Fire!\n");
		  printf("Temperature reach %.1f\n", avgTemperature);
		  HAL_Delay(2000);
	  }
}

void buzzerAlarm(uint8_t match)
{
	for(uint8_t k = 0; k < 2; k++) //To make sure the buzzer will turn on for 5 seconds (500ms x 5 = 2.5s for each)
	{
		if (match == 1)
		{
		  for(uint32_t i = 46999; i <= 49999; i = i + 1000)
		  {
			  TIM2->CCR1 = i;
			  tempMeasure();
			  HAL_Delay(500);

		  }
		}
		else
		{	for(uint32_t j = 11999; j >= 9999; j = j-500)
			 {
				TIM2->CCR1 = j;
				tempMeasure();
				HAL_Delay(500);
			 }

		}
	}
	TIM2->CCR1 = 0;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */
/*

  ADC_ChannelConfTypeDef sConfig = {0};


  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC; //ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  */

	HAL_StatusTypeDef rc;

	/* Initialize ADC */
	__HAL_RCC_ADC1_CLK_ENABLE();
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	rc = HAL_ADC_Init(&hadc1);
	if (rc != HAL_OK){
		Error_Handler();
	}

	ADC_ChannelConfTypeDef sConfig;

	/** Configure Regular Channel*/
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	rc = HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	if (rc != HAL_OK){
		Error_Handler();
	}

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
HAL_TIM_MspPostInit(&htim2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_Base_Start_IT(&htim2);

}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Timer Interrupt Setting
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	if (stepEnable == 1)
	{
		stepsCounter = stepsCounter + 1;
		flag = 1;

		if (HAL_GetTick() > 8000 + timeStamp)
		{
			state = 1;
			motorRotate(-1);
		}
		if (stepsCounter/2 == stepRequired)
		{
			stepsCounter = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET); //ENABLE -- 0 disable
			stepEnable = 0;
		}
	}

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SSD1331_RES_Pin|SSD1331_DC_Pin|SSD1331_CS_Pin|GPIO_PIN_10
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SSD1331_RES_Pin SSD1331_DC_Pin SSD1331_CS_Pin */
  GPIO_InitStruct.Pin = SSD1331_RES_Pin|SSD1331_DC_Pin|SSD1331_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = buttonPinOne | buttonPinTwo | buttonPinThree |buttonPinFour;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, buttonPinOne | buttonPinTwo | buttonPinThree | buttonPinFour, GPIO_PIN_RESET);


  //For Stepper Motor
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  /*Configure GPIO pins : PC0 PC1 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET); //0E -- 0 default disable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET); //FR -- 0 backward, forward 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET); //step -- 0,1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET); //RST -- 0->RESET, 1->SET

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

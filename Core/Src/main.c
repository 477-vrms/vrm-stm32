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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx.h"

#include <string.h> // for memset()
#include <stdio.h> // for printf()
#include <math.h>   // for M_PI
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t myRxData[10];
volatile uint8_t RxReady = 0;
volatile uint8_t RxByte = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03
#define CMD_ACTION_GROUP_RUN    0x06
#define CMD_ACTION_GROUP_STOP   0x07
#define CMD_ACTION_GROUP_SPEED  0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0F
#define BUFF_VAL 				100

#define BATTERY_VOLTAGE       0x0F
#define ACTION_GROUP_RUNNING  0x06
#define ACTION_GROUP_STOPPED  0x07
#define ACTION_GROUP_COMPLETE 0x08

#define GET_LOW_BYTE(A) (uint8_t)((A))

#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//============================================================================
// Configure Timer 1 and the PWM output pins.
// Parameters: none
//============================================================================
void setup_tim1()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //Enable port A
    GPIOA->MODER |= 0x20000; //Set pins 8,9,10,11 for alternate function use
    GPIOA->AFR[1] |= 0x2; //Rout timer to external pins
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable TIM1 clock
    TIM1->BDTR |= TIM_BDTR_MOE; //Set BDTR MOE
    TIM1->PSC = 1-1; //Set PSC
    TIM1->ARR = 2400-1; //Set ARR
    TIM1->CCMR1 |= 0x60; //Set CCMR1 for PWM
    TIM1->CCER |= TIM_CCER_CC1E; //Enable output 1
    TIM1->CCR1 = (2400-1) * 0.9;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void send_data(int data)
{
    while((USART3->ISR & USART_ISR_TXE) == 0); //Wait for transmit set
    USART3->TDR = data; //Write data to transmitter
}

void send_dataPi(int data)
{
    while((USART2->ISR & USART_ISR_TXE) == 0); //Wait for transmit set
    USART2->TDR = data; //Write data to transmitter
}

void setup_uart3()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	//Set alternate function for PA1 and PA2 (Tx/Rx)
	//Set pins 10 and 11 for alternate function (10 code)
	GPIOC->MODER |= 0xA00000;
	GPIOC->MODER &= ~(0x500000);

	//Set PC10 and PC11 alternate function to AFR1 (Uart3 Tx/Rx respectively)
	GPIOC->AFR[1] |= 0x1100;

	//Set baud rate to 9600 by dividing 48MHz clock by 9600
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600;

    USART3->CR1 &= ~USART_CR1_UE; //Disable USART
    USART3->CR1 &= ~USART_CR1_M; //Set word size of 8 bits
    USART3->CR2 &= ~USART_CR2_STOP; //Set 1 stop bit
    USART3->CR1 &= ~USART_CR1_PCE; //Disable parity
    USART3->CR1 &= ~USART_CR1_OVER8; //Oversample by 16x

	//Enable Tx and Rx and USART
	USART3->CR1 |= 0xD; //1101

    while((USART3->ISR & USART_ISR_TEACK) == 0); //Wait for transmit acknowledge
    while((USART3->ISR & USART_ISR_REACK) == 0); //Wait for receive acknowledge
}

void fillbuffer(uint8_t buf[10], uint8_t data_len, uint16_t pos, uint16_t t, uint8_t servoID, uint8_t cmd)
{
	buf[0] = FRAME_HEADER;
	buf[1] = FRAME_HEADER;
	buf[2] = data_len;
	buf[3] = CMD_SERVO_MOVE;
	buf[4] = 1;
	buf[5] = GET_LOW_BYTE(t);
	buf[6] = GET_HIGH_BYTE(t);
	buf[7] = servoID;
	buf[8] = GET_LOW_BYTE(pos);
	buf[9] = GET_HIGH_BYTE(pos);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE BEGIN 2 */
  uint8_t buf[10];
  /*uint16_t pos3 = 1500; // 500 - 2000
  uint16_t claw_pos = 1500; // 1500 - 2500
  uint16_t pos245 = 1500; // 500 - 2500*/

  /* USER CODE END 2 */
  uint8_t j_data[BUFF_VAL];
  /* Infinite loop */

  while (1)
  {
	  int count = 0;
	  HAL_UART_Receive(&huart3,j_data,BUFF_VAL,1000);
	  int releaseFlag = 1;
	  uint8_t* j_scanner = j_data;
	  int j_val = 0;
	  while(releaseFlag)
	  {
		  while(j_data[count]!=':' && releaseFlag)
		  {
			  j_scanner++;
			  count++;
			  j_val = j_val*10 + (int)(j_data[count]-'0');
			  if(j_data[count] == '$' || count >= BUFF_VAL)
				  releaseFlag = 0;
		  }
		  j_scanner++;
		  count++;
	  }
	  //uint8_t j1 = j_data[0];
	  uint8_t j2 = j_data[1];
	  uint8_t j3 = j_data[2];
	  //uint8_t j4 = j_data[3];
	  //uint8_t j5 = j_data[4];
	  uint8_t j6 = j_data[5];
	  uint8_t j7 = j_data[6];
	  uint8_t j8 = j_data[7];

	  //correct data						 UNITY	       SERVO LIMITS
	  j8 = (j8 - 1500) * 10;   //0 to 100    :  1500-2500
	  j7 = (j7+90)*11 + 500;   // -90 to 90  :  500 - 2500
	  j6 = (j6+60)*13.6 + 500; //-60 to 50   :  500 - 2000
	  //j5 = (j5+180)*0 + 0;     //-180 to 180 :  N/A
	  //j4 = 0; 						 //--------------------------
	  j3 = j3*14.2 + 500;      //0 to 140    :  500 - 2500
	  j2 = (j2+50)*20 + 500;   //-50 to 50   :  500 - 2500
	  //j1 = (j1+90)*0 + 0;      //-90 to 90   :  N/A



	/* Servo ID:
	 * 1: (not available)
	 * 2: rotate claw
	 * 3: top
	 * 4: middle
	 * 5: bottom
	 * 6: claw
	 * 7: (not available)
	 * 8: (not available)

	 J# ID:
	 * J1: Rotate Base (not available)
	 * J2: Bottom
	 * J3: Middle
	 * J4: (zerod out)
	 * J5: Spin (not available)
	 * J6: Top
	 * J7: spin claw
	 * J8: Claw (1-100 button)
	 */

	if(j2 > 2500)
		j2 = (uint8_t)2500;
	else if(j2 < 500)
		j2 = (uint8_t)500;

	if(j3 > 2500)
		j3 = (uint8_t)2500;
	else if(j3 < 500)
		j3 = (uint8_t)500;

	if(j6 > 2000)
		j6 = (uint8_t)2000;
	else if(j6 < 500)
		j6 = (uint8_t)500;

	if(j7 > 2500)
		j7 = (uint8_t)2500;
	else if(j7 < 500)
		j7 = (uint8_t)500;

	if(j8 > 2500)
		j8 = (uint8_t)2500;
	else if(j8 < 1500)
		j8 = (uint8_t)1500;

	//Claw : j8 sID 6
	/*fillbuffer(buf, 8, j8, 1000, 6, 1);
	for(const uint8_t *c = buf; *c; c++)
		send_data(*c);

	//Rotate Claw : j7 sID 2
	fillbuffer(buf, 8, j7, 1000, 2, 1);
	for(const uint8_t *c = buf; *c; c++)
		send_data(*c);

	//Top : J6 sID 3
	fillbuffer(buf, 8, j6, 1000, 3, 1);
	for(const uint8_t *c = buf; *c; c++)
		send_data(*c);

	//Middle: J3 sID 4
	fillbuffer(buf, 8, j3, 1000, 4, 1);
	for(const uint8_t *c = buf; *c; c++)
		send_data(*c);

	//Bottom: J2 sID 5
	fillbuffer(buf, 8, j2, 1000, 5, 1);
	for(const uint8_t *c = buf; *c; c++)
		send_data(*c);
*/
	HAL_UART_Transmit(&huart3,j_data, 10,1000);
  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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


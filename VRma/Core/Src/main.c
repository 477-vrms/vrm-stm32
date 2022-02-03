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

#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03
#define CMD_ACTION_GROUP_RUN    0x06
#define CMD_ACTION_GROUP_STOP   0x07
#define CMD_ACTION_GROUP_SPEED  0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0F


#define BATTERY_VOLTAGE       0x0F
#define ACTION_GROUP_RUNNING  0x06
#define ACTION_GROUP_STOPPED  0x07
#define ACTION_GROUP_COMPLETE 0x08

#define GET_LOW_BYTE(A) (uint8_t)((A))

#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void setup_tim1();
void setup_usart3();

//============================================================================
// setup_tim1()    (Autotest #1)
// Configure Timer 1 and the PWM output pins.
// Parameters: none
//============================================================================
void setup_tim1()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //Enable port A
    GPIOA->MODER |= 0xaa0000; //Set pins 8,9,10,11 for alternate function use
    GPIOA->AFR[1] |= 0x2222; //Rout timer to external pins
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable TIM1 clock
    TIM1->BDTR |= TIM_BDTR_MOE; //Set BDTR MOE
    TIM1->PSC = 1-1; //Set PSC
    TIM1->ARR = 2400-1; //Set ARR
    TIM1->CCMR1 |= 0x6060; //Set CCMR1 for PWM
    TIM1->CCMR2 |= 0x6060; //Set CCMR2 for PWM
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE; //Enable preload
    TIM1->CCER |= TIM_CCER_CC1E; //Enable output 1
    TIM1->CCER |= TIM_CCER_CC2E; //Enable output 2
    TIM1->CCER |= TIM_CCER_CC3E; //Enable output 3
    TIM1->CCER |= TIM_CCER_CC4E; //Enable output 4
    TIM1->CCR1 = 500;
    TIM1->CR1 |= TIM_CR1_CEN;

}

void send_data(int data)
{
    while((USART3->ISR & USART_ISR_TXE) == 0); //Wait for transmit set
    USART3->TDR = data; //Write data to transmitter
}

void setup_uart3()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	//Set alternate function for PC10 and PC11 (Tx/Rx)
	GPIOC->MODER |= 0x200000; //Set pins 8,9,10,11 for alternate function use
	GPIOC->MODER &= ~(0x100000); //Set pins 8,9,10,11 for alternate function use

	GPIOC->MODER |= 0x800000;
	GPIOC->MODER &= ~(0x400000);

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

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	setup_tim1();
	setup_uart3();
	uint8_t buf[10];

	uint16_t pos3 = 1500; // 500 - 2000
	uint16_t claw_pos = 1500; // 1500 - 2500
	uint16_t pos245 = 1500; // 500 - 2500

	int off3 = 5;
	int offClaw = 5;
	int off245 = 5;

	while (1)
	{
		if(pos3 > 2000)
			off3 = -5;
		else if(pos3 < 500)
			off3 = 5;

		if(claw_pos > 2000)
			offClaw = -2;
		else if(claw_pos < 1500)
			offClaw = 2;

		if(pos245 > 2500)
			off245 = - 5;
		else if(pos245 < 500)
			off245 = 5;

		pos3 = pos3 + off3;
		pos245 = pos245 + off245;
		claw_pos = claw_pos + offClaw;

		//Claw
		fillbuffer(buf, 8, claw_pos, 1000, 6, 1);
		for(const uint8_t *c = buf; *c; c++)
			send_data(*c);
		//Rotate Claw
		fillbuffer(buf, 8, pos245, 1000, 2, 1);
		for(const uint8_t *c = buf; *c; c++)
			send_data(*c);
		//Top
		fillbuffer(buf, 8, pos3, 1000, 3, 1);
		for(const uint8_t *c = buf; *c; c++)
			send_data(*c);
		//Middle
		fillbuffer(buf, 8, pos245, 1000, 4, 1);
		for(const uint8_t *c = buf; *c; c++)
			send_data(*c);
		//Bottom
		fillbuffer(buf, 8, pos245, 1000, 5, 1);
		for(const uint8_t *c = buf; *c; c++)
			send_data(*c);
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


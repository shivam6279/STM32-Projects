#include "stm32.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include <math.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

void GPIO_init() {
	RCC->AHB2ENR |= 0b1111111;
}

//---------------------

void TIM1_on() {
	TIM1->CR1 |= 1;
}

void TIM2_on() {
	TIM2->CR1 |= 1;
}

void TIM3_on() {
	TIM3->CR1 |= 1;
}

void TIM4_on() {
	TIM4->CR1 |= 1;
}

void TIM6_on() {
	TIM6->CR1 |= 1;
}

void TIM7_on() {
	TIM7->CR1 |= 1;
}

//---------------------

void TIM8_on() {
	TIM8->CR1 |= 1;
}

void TIM1_off() {
	TIM1->CR1 &= ~1;
}

void TIM2_off() {
	TIM2->CR1 &= ~1;
}

void TIM3_off() {
	TIM3->CR1 &= ~1;
}

void TIM4_off() {
	TIM4->CR1 &= ~1;
}

void TIM6_off() {
	TIM6->CR1 &= ~1;
}

void TIM7_off() {
	TIM7->CR1 &= ~1;
}

void TIM8_off() {
	TIM8->CR1 &= ~1;
}

//---------------------

void TIM1_init() {
    RCC->APB2ENR |= 1 << 11;

    //PA[8] - 1P
    GPIOA->AFR[1] |= 0b0110;
	GPIOA->MODER &= ~(1 << 16);
	GPIOA->MODER |= 1 << 17;
	GPIOA->OSPEEDR |= 0b11 < 16;

    //PA[7] - 1N
    GPIOA->AFR[0] |= 0b0110 << 28;
	GPIOA->MODER &= ~(1 << 14);
	GPIOA->MODER |= 1 << 15;
	GPIOA->OSPEEDR |= 0b11 < 14;

    //PA[9] - 2P
    GPIOA->AFR[1] |= 0b0110 << 4;
	GPIOA->MODER &= ~(1 << 18);
	GPIOA->MODER |= 1 << 19;
	GPIOA->OSPEEDR |= 0b11 < 18;

    //PB[0] - 2N
    GPIOB->AFR[0] |= 0b0110;
	GPIOB->MODER &= ~1;
	GPIOB->MODER |= 1 << 1;
	GPIOB->OSPEEDR |= 0b11;

    //PA[10] - 3P
    GPIOA->AFR[1] |= 0b0110 << 8;
	GPIOA->MODER &= ~(1 << 20);
	GPIOA->MODER |= 1 << 21;
	GPIOA->OSPEEDR |= 0b11 < 20;

    //PF[0] - 3N
    GPIOF->AFR[0] |= 0b0110;
	GPIOF->MODER &= ~1;
	GPIOF->MODER |= 1 << 1;
	GPIOF->OSPEEDR |= 0b11;

    TIM1->CR1 = 0;
    TIM1->CR2 = 0;

    TIM1->CR1 |= 1 << 7; //Auto-preload enable    
//    TIM1->CR1 |= 0b01 << 5; //Center-aligned mode 1
    TIM1->CR1 |= 0b10 << 5; //Center-aligned mode 2

    TIM1->CCMR1 = 0;
    TIM1->CCMR1 |= 0b110 << 12;
    TIM1->CCMR1 |= 0b110 << 4;

    TIM1->CCMR2 = 0;
    TIM1->CCMR2 |= 0b110 << 4;

    TIM1->CCMR2 = 0;
	TIM1->CCMR2 |= 0b110 << 4;



    TIM1->PSC = 0;

    TIM1->CNT = 0;

    TIM1->CCER = 0;
    TIM1->CCER |= 0b010101010101; //enable outputs

	TIM1->CCER |= TIM_CCER_CC1NP;
	TIM1->CCER |= TIM_CCER_CC2NP;
	TIM1->CCER |= TIM_CCER_CC3NP;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	//OC4
//	TIM1->CCMR2 |= 0b001 << 12;
//	TIM1->CCER |= 1 << 16;
	TIM1->CCR4 = 0;
//	TIM1->DIER |= 1 << 4;

	//OC5
//    TIM1->CCMR3 = 0;
//	TIM1->CCMR3 |= 0b110 << 4;
//	TIM1->CCER |= 1 << 16;
//	TIM1->CCR5 = 0;

	TIM1->BDTR |= 1 << 15;

	NVIC_SetPriority(TIM1_CC_IRQn, 2);
	NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM1->CR1 |= 1;
}

void TIM2_init() {
    RCC->APB1ENR1 |= 1;

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;

    TIM2->CR1 |= 1 << 7;

    TIM2->ARR = 1000;
    TIM2->PSC = 170 - 1;

    TIM2->CNT = 0;

    TIM2->EGR |= 1;

    TIM2->DIER |= 1;

    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);

//	TIM2->CR1 |= 1;
}

void TIM3_init() {
    RCC->APB1ENR1 |= 1 << 1;

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;

    TIM3->CR1 |= 1 << 7;

    float freq = 50000;
    float f = (float)SYSCLK_FREQ / freq;
    TIM3->ARR = (uint16_t)f;
	TIM3->PSC = 0;

    TIM3->CNT = 0;

    TIM3->DIER |= 1;
//    TIM3->CR1 |= 1;
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM4_init() {
    RCC->APB1ENR1 |= 1 << 2;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;

    TIM4->CR1 |= 1 << 7;

    TIM4->ARR |= 0xFFFFF;
    TIM4->PSC = 0;

    TIM4->CNT = 0;

    TIM4->EGR |= 1;

    TIM4->DIER |= 1;

    TIM4->SR &= ~(0x1);

    // TIM4->CR1 |= 1;
    NVIC_SetPriority(TIM4_IRQn, 5);
    NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM6_init() {
    RCC->APB1ENR1 |= 1 << 4;

    TIM6->CR1 = 0;
    TIM6->CR2 = 0;

    float freq = 10000;
    float f = (float)SYSCLK_FREQ / freq;
    // TIM6->ARR |= ((uint32_t)f & 0x000FFFFF);
    // TIM6->ARR = (uint32_t)f;
    TIM6->ARR |= 0xFFFFF;
    TIM6->PSC = (uint16_t)f - 1;

    TIM6->CNT = 0;

    TIM6->EGR |= 1;

    // TIM6->CR1 |= 1;
}

void delay_ms(uint16_t d) {
	TIM6->CNT = 0;
	TIM6->PSC = 17000 - 1;
	TIM6->EGR |= 1;
	TIM6->CR1 |= 1;
	while(TIM6->CNT < d*10);
	TIM6->CR1 &= ~1;
}

void delay_us(uint16_t d) {
	TIM6->CNT = 0;
	TIM6->PSC = 170 - 1;
	TIM6->EGR |= 1;
	TIM6->CR1 |= 1;
	while(TIM6->CNT < d);
	TIM6->CR1 &= ~1;
}

void TIM8_init() {
	RCC->APB2ENR |= 1 << 13;
	RCC->APB2ENR |= 0b1111111;

	//PA15 - TIM8_CH1
	GPIOA->AFR[1] |= 0b0010 << 28;
	GPIOA->MODER &= ~(1<<30);
	GPIOA->MODER |= 1<<31;

	//PB8 - TIM8_CH2
	GPIOB->AFR[1] |= 0b1010;
	GPIOB->MODER &= ~(1<<16);
	GPIOB->MODER |= 1<<17;

	//PB6 <> TIM8_ETR
	GPIOB->AFR[0] |= 0b0110 << 24;
	GPIOB->MODER &= ~(1<<12);
	GPIOB->MODER |= 1<<13;

    TIM8->CR1 = 0;

    TIM8->CR2 = 0;
    TIM8->SMCR = 0b0011;
    TIM8->CCMR1 |= 0b01 << 8;
    TIM8->CCMR1 |= 0b01;
    TIM8->ECR = 1;

    TIM8->ARR |= 0xFFFFF;
    TIM8->CNT = 0;

    TIM8->CR1 |= 1;
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 21;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

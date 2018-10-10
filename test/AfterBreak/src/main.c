/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);

		if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		else
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
}

void InitializeLEDs()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_13;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioStructure);

//    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
}

void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 7200-1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 1000-1;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);			// >>>INTERRUPT<<< enable
    TIM_Cmd(TIM2, ENABLE);
}

void EnableTimerInterrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void GPIO_Ini(){
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gpio);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gpio);

}

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_Ini();
	InitializeLEDs();
	InitializeTimer();
	EnableTimerInterrupt();


	while(1){
//		if(!(GPIOB->IDR == GPIO_Pin_13) )
//		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))

		if((GPIOB->IDR & GPIO_Pin_13) != 0 )
			GPIOB->BSRR = GPIO_Pin_12;

		else
			GPIOB->BRR = GPIO_Pin_12;

//dummy blink
//        int timerValue = TIM_GetCounter(TIM2);
//        if (timerValue < 500)
//            GPIOC->BSRR = GPIO_Pin_13;
//        else
//            GPIOC->BRR = GPIO_Pin_13;
	}
}

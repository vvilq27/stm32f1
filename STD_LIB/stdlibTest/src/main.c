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
			
void delay(int time)
	{
	    int i;
	    for (i = 0; i < time * 4000; i++) {}
	}


int main(void)
{
	//podlaczanie zegarow
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//timer clk enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef dma;

	//timer setup
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = 1000 - 1;
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);	//start timer

	//timer interrupt setup
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	//DMA setup
	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)src_buffer;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	dma.DMA_MemoryBaseAddr = (uint32_t)dst_buffer;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_BufferSize = BUFFER_SIZE;
	dma.DMA_M2M = DMA_M2M_Enable;
	DMA_Init(DMA1_Channel1, &dma);

	//konfiguracja portow
	GPIO_StructInit(&gpio);  // domyœlna konfiguracja
	gpio.GPIO_Pin = GPIO_Pin_13;  // konfigurujemy pin 5
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;  // jako wyjœcie
	GPIO_Init(GPIOC, &gpio);  // inicjalizacja modu³u GPIOA

	for(;;){
//        GPIO_SetBits(GPIOC, GPIO_Pin_13); // zapalenie diody
//        delay(100);
//        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // zgaszenie diody
//        delay(400);
	}
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        else
            GPIO_SetBits(GPIOC, GPIO_Pin_13);

    }
}

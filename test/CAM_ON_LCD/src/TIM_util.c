#include "stm32f10x.h"
#include "../inc/st7735.h"

 TIM_TimeBaseInitTypeDef tim;
 GPIO_InitTypeDef gpio;
 TIM_OCInitTypeDef  channel;

	extern volatile uint32_t cnt;
	uint32_t prevCnt;
	uint16_t diff;
	extern volatile char cnt_s[5];
	char frameCnt[2];

void TIM2_setup(void){

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 7200-1;
	tim.TIM_Period = 10000-1;
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void TIM4_PWM(void){
	 GPIO_StructInit(&gpio);
	 gpio.GPIO_Pin = GPIO_Pin_6;
	 gpio.GPIO_Speed = GPIO_Speed_50MHz;
	 gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_Init(GPIOB, &gpio);

	 //tim freq setup
	 TIM_TimeBaseStructInit(&tim);
	 tim.TIM_CounterMode = TIM_CounterMode_Up;
	 tim.TIM_Prescaler = 8;		//1 for 8 mhz
	 tim.TIM_Period = 2 - 1;
	 TIM_TimeBaseInit(TIM4, &tim);


	 TIM_OCStructInit(&channel);
	  channel.TIM_OCMode = TIM_OCMode_PWM1;
	  channel.TIM_OutputState = TIM_OutputState_Enable;
	  channel.TIM_Pulse = 1;
	  TIM_OC1Init(TIM4, &channel);

	  TIM_Cmd(TIM4, ENABLE);
}

void EnableTimerInterrupt(void)
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void TIM2_IRQHandler(void){
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

//		diff = cnt - prevCnt;
//        ST7735_Clear(0x0000);
        ST7735_PutStr5x7(10, 40,itoa(cnt,cnt_s, 10), RGB565(128, 128, 128));
//		ST7735_PutStr5x7(10, 20,itoa(diff,frameCnt, 10), RGB565(128, 128, 128));
//		ST7735_PutStr5x7(10, 30,itoa(cnt,cnt_s, 10), RGB565(128, 128, 128));
//		ST7735_PutStr5x7(10, 40,itoa(prevCnt,cnt_s, 10), RGB565(128, 128, 128));
//		prevCnt = cnt;

		if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		else
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}


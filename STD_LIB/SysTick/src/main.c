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
			
unsigned long int SysTick_Config_Mod(unsigned long int SysTick_CLKSource, unsigned long int Ticks);
extern volatile unsigned long int Licznikms;
void Delay(unsigned long int Opoznieniems){
	Licznikms = 0;
	while((Licznikms) < Opoznieniems);
}


int main(void)
{
	RCC_Config();
	//set systic freq to 1hz
	//72mhz / 8 / 9M = 1hz
	if(SysTick_Config_Mod(SysTick_CLKSource_HCLK_Div8, 9000000ul))
		while(1);


	GPIO_InitTypeDef gpio; // obiekt gpio z konfiguracja portow GPIO

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // uruchomienie zegara modulu GPIO

	GPIO_StructInit(&gpio); // domyslna konfiguracja
	gpio.GPIO_Pin = GPIO_Pin_13; // konfigurujemy pin 5
	gpio.GPIO_Mode = GPIO_Mode_Out_PP; // jako wyjscie
	GPIO_Init(GPIOC, &gpio); // inicjalizacja modulu GPIOA

//	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	for(;;){

//		for(uint32_t i = 0; i < 2500000; i++);

	}
}

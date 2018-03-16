/*
 * My_lib.c
 *
 *  Created on: 15 mar 2018
 *      Author: arazu
 */

#include "stm32f10x.h"

void send_char(char c)
{
 while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
 USART_SendData(USART2, c);
}

int __io_putchar(int c)
{
 if (c=='\n')
 send_char('\r');
 send_char(c);
 return c;
}

void pinToggle(GPIO_TypeDef* gpioPort, uint16_t pin){
	if(GPIO_ReadOutputDataBit(gpioPort, pin))
		GPIO_ResetBits(gpioPort, pin);
	else
		GPIO_SetBits(gpioPort, pin);
}

//Configure system clock
void RCC_Config(void){
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);	//turn on hse
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //wait for HSE startup

	if(HSEStartUpStatus == SUCCESS){
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);

		//setup HCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		//set PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_SYSCLK_Div2);

		//set PCLK = HSE*9 = 72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		//turn on PLL
		RCC_PLLCmd(ENABLE);

		//wait for PLL to startup
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		//set PLL as clock signal source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		//wait for PLL to be set as system clock
		while(RCC_GetSYSCLKSource() != 0x08);

		//place for code to setup peripherals clock


		//turn on GPIO B clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		//turn on all ports
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	}
}

void GPIO_Config(void){
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gpio);
}

void NVIC_Config(void){
//set vector table address
#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}
/*
 * example for 72mhz clk
 * to make systic count 1 sec periods you have to use these params:
 * SysTick_Config_Mod(SysTick_CLKSource_HCLK_Div8, 9000000ul)
 *
 * interrupt handler is SysTick_Handler()
 */


unsigned long int SysTick_Config_Mod(unsigned long int SysTick_CLKSource, unsigned long int Ticks){
	unsigned long int Settings;

	assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));

	//check if start val doesnt exceed max allowed value
	if(Ticks > SysTick_LOAD_RELOAD_Msk)		return(1);

	//set counter start value
	SysTick ->LOAD = (Ticks & SysTick_LOAD_RELOAD_Msk) - 1;

	//set interrupt prirority
	NVIC_SetPriority(SysTick_IRQn, 0);

	//set actual counter value
	SysTick -> VAL = 0;

	//set SysTick_IRQ and counter markers
	Settings = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	//choose clock settings marker
	if(SysTick_CLKSource == SysTick_CLKSource_HCLK){
		Settings |= SysTick_CLKSource_HCLK;
	} else{
		Settings &= SysTick_CLKSource_HCLK_Div8;
	}

	//save setup to the SysTick control register and start counter
	SysTick -> CTRL = Settings;
	return(0);
}

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include "../inc/delay.h"
#include "../inc/st7735.h"


/*
 * 				SETUP
 * PB12		A0
 * PB13		SCK
 * PB15		MOSI (no MISO required)
 *
 * PB6		RST
 * PB7		CS
 * all setup in st7735.h
 *
 *			Default colors sending order
 *	RGB sent in 2 bytes
 *	5 bits RED, 6 bits GREEN, 5 bits BLUE
 *	0bRRRR RGGG-GGGB BBBB
 */
	// Screen connection
	// SCK  -> PB3
	// A0   -> PB4
	// SDA  -> PB5
	// RST  -> PB6
	// CS   -> PB7


void InitializeBlinker()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_13;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioStructure);

//    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
}

	volatile uint32_t cnt = 0;
	volatile char cnt_s[5];

int main(void)
{
	ST7735_Init();
	ST7735_AddrSet(0,0,159,127);
	ST7735_Clear(0x0000);
	ST7735_Orientation(scr_180);

	InitializeBlinker();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	stmForCameraSetup();
	TIM2_setup();
	EnableTimerInterrupt();


	while(1) {
		while((GPIOB->IDR & GPIO_Pin_10)){
			ST7735_PutStr5x7(10, 10,itoa(cnt,cnt_s, 10), RGB565(128, 128, 128));
		}
			cnt++;

		while(!(GPIOB->IDR & GPIO_Pin_10)){

		}

//		ST7735_PutStr5x7(10, 60,itoa(TIM_GetCounter(TIM2),cnt_s, 10), RGB565(128, 128, 128));
	}//end main loop
}//end main

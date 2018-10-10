#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include "delay.h"
#include "st7735.h"


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



void InitializeLEDs()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);

//    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
}

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


int main(void)
{
	ST7735_Init();
	ST7735_AddrSet(0,0,159,127);
	ST7735_Clear(0x0000);
	InitializeBlinker();

	ST7735_Orientation(scr_180);
//	ST7735_PutStr5x7(0,0,"Hello fucking bitch!",RGB565(255,0,0));

	uint32_t cnt = 0;
	char cnt_s[5];
	while(1) {
		if((GPIOB->IDR & GPIO_Pin_10) != 0 ){
			GPIOC->BSRR = GPIO_Pin_13;
		}
		else{
			GPIOC->BRR = GPIO_Pin_10;
			cnt++;

			ST7735_PutStr5x7(50, 70,itoa(cnt,cnt_s, 10), RGB565(128, 128, 128));
		}

	}
}


//============NOTES==============

//display picture
//	for(int i = 0; i < 128; i++){
//		for(int j= 0; j < 128; j++){
//			ST7735_Pixel(i, j, pic[16384-j*128+i]);
//		}
//	}
/* color testing
		int num = 0;
		int change = 1;
		int r = 0b1111100000000000;
		int g = 0b111100000000;
		int b = 0b000011110000;
		int bb = 0b000000001111;

		change = 1;
		for(int i = 0; i < 32; i++){
			ST7735_FillRect(3, 3, 90, 120, num);
			change/=2;
			change++;
			num = (change << 11);
			num |= (change << 0);
			change*=2;
			num |= (change << 5);
			Delay_ms(50);
		}
		*/

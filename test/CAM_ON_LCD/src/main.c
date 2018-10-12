#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include "../inc/delay.h"
#include "../inc/st7735.h"
#include "ov7670.h"


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

	volatile uint16_t fcnt = 0;
	volatile uint16_t pcnt = 0;
	volatile char cnt_s[6];
	uint8_t row[160];

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	ST7735_Init();
	ST7735_AddrSet(0,0,159,127);
	ST7735_Clear(0x0000);
	ST7735_Orientation(scr_180);

	InitializeBlinker();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	twoWireInit();

	stmForCameraSetup();
	camInit();
	setRes(QQVGA);
	setColorSpace(YUV422);

	TIM2_setup();
//	EnableTimerInterrupt();
	uint8_t i = 0;
	uint8_t j = 0;

	while(1) {

		//doesnt work, make array and write it after scan
		//put scan value on screen

		//initialize port as input with pullup
		//why spi its not sending every row?


		GPIO_SetBits(GPIOC, GPIO_Pin_13);

		//picture going
		while(!(GPIOB->IDR & GPIO_Pin_11)){
			//high - row going
			while((GPIOB->IDR & GPIO_Pin_10)){
				//pix going
				i++;
				//low
				while(!(GPIOB->IDR & GPIO_Pin_1)){
				}
				row[i] = (GPIOA->IDR & 0xff);

				//high
				while((GPIOB->IDR & GPIO_Pin_1)){
				}

				//low
				while(!(GPIOB->IDR & GPIO_Pin_1)){
				}
				//high
				while(!(GPIOB->IDR & GPIO_Pin_1)){
					if((GPIOB->IDR & GPIO_Pin_10))
						break;
				}

			}
			i=0;
			ST7735_PutStr5x7(10, 40,itoa(row[80],cnt_s, 10), RGB565(128, 128, 128));
			ST7735_PutStr5x7(10, 50,itoa(row[81],cnt_s, 10), RGB565(128, 128, 128));
			ST7735_PutStr5x7(10, 60,itoa(row[82],cnt_s, 10), RGB565(128, 128, 128));

			//low, wait for new row
			while(!(GPIOB->IDR & GPIO_Pin_10)){
				//without it, it wont let go out of "parent" while
				if((GPIOB->IDR & GPIO_Pin_11))
					break;
			}
		}

		GPIO_ResetBits(GPIOC, GPIO_Pin_13);

		//next pic indicator
		while((GPIOB->IDR & GPIO_Pin_11)){
		}

	}//end main loop

}//end main
//======NOTES=========
//		ST7735_PutStr5x7(10, 60,itoa(TIM_GetCounter(TIM2),cnt_s, 10), RGB565(128, 128, 128));

//		ST7735_PutStr5x7(10, 20,itoa(fcnt,cnt_s, 10), RGB565(128, 128, 128));
//		ST7735_PutStr5x7(10, 40,itoa(pcnt,cnt_s, 10), RGB565(128, 128, 128));

//				ST7735_Pixel(j, i, RGB565(pixVal,pixVal,pixVal));

//for(int x = 0; x < 160; x++){
//	uint8_t pixVal = row[x];
//	ST7735_Pixel(5, x, RGB565(pixVal,pixVal,pixVal));
//}



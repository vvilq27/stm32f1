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

//CAMERA
//PB1 PIXCLK
//PB10	HREF
//PB11 VSYNC

#define row_size	160
GPIO_InitTypeDef gpioStructure;

volatile uint16_t fcnt = 0;
volatile uint16_t pcnt = 0;
volatile char cnt_s[6];
uint8_t row[row_size];






void InitializeBlinker()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpioStructure.GPIO_Pin = GPIO_Pin_13;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioStructure);

//    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
}

void InitializeInputPort(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	gpioStructure.GPIO_Pin = 0xFF;
	gpioStructure.GPIO_Mode = GPIO_Mode_IPU; // jako wejscie z rezystorem pull-up
    GPIO_Init(GPIOA, &gpioStructure); // port GPIOC

    gpioStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11;
	gpioStructure.GPIO_Mode = GPIO_Mode_IPU; // jako wejscie z rezystorem pull-up
	GPIO_Init(GPIOB, &gpioStructure); // port GPIOC
}

//void copy_dma()
//{
// DMA_Cmd(DMA1_Channel1, DISABLE);
// DMA_ClearFlag(DMA_ISR_TCIF1);
// DMA_SetCurrDataCounter(DMA1_Channel1, BUFFER_SIZE);
//
// DMA_Cmd(DMA1_Channel1, ENABLE);
// while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
//}

void dma_init(){
	DMA_InitTypeDef dma;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryBaseAddr = (uint32_t)row;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = 1;
	dma.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA1_Channel1, &dma);
	 DMA_Cmd(DMA1_Channel1, ENABLE);
	 USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	 //code to check
}


int main(void){

	ST7735_Init();
	ST7735_AddrSet(0,0,159,127);
	ST7735_Clear(0x0000);
	ST7735_Orientation(scr_180);

	InitializeBlinker();
	InitializeInputPort();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	EXTI_InitTypeDef exti;

	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line13;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);


	twoWireInit();
	uartEnable(72);
//	dma_init();

	stmForCameraSetup();
//	camInit();
//	setRes(QQVGA);
//	setColorSpace(YUV422);

//	TIM2_setup();
//	EnableTimerInterrupt();
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t pixVal = 0;
	uint8_t buf[10];
	uint8_t nums[] = {48,49,50,51,52,53,54,55};


	GPIOC->BSRR |= GPIO_Pin_13;
//	const struct regval_list ov7670_default_regs[255];
	const struct regval_list *camera_defaults = ov7670_default_regs;

		for(int l = 0; l < 8; l++){
			uint8_t reg_addr = camera_defaults->reg_num;
			uint8_t reg_val = camera_defaults->value;//cameraReadByte(reg_addr);
			uint8_t reg_cam = cameraReadByte(reg_addr);

//			ST7735_PutChar5x7(5, 2+l*10, nums[l]+1, RGB565(128, 128, 128));
			ST7735_PutNumber5x7(5, 2+l*12, reg_addr, RGB565(128, 128, 128));
			ST7735_PutNumber5x7(25, 2+l*12, reg_val, RGB565(128, 128, 128));
			ST7735_PutNumber5x7(45, 2+l*12, reg_cam, RGB565(128, 128, 128));
			camera_defaults++;
		}

//		ST7735_PutNumber5x7(100,55,254,RGB565(128, 128, 128));

	//adresy sie nie zgadzaja sproboj odczytac tablice inaczej
		for(int l = 0; l < 20; l++){
				uint8_t reg_addr = camera_defaults->reg_num;
				uint8_t reg_val = camera_defaults->value;//cameraReadByte(reg_addr);

//				uartSend(reg_addr);
//				uartSends(" ");
//				ST7735_PutChar5x7(5, 2+l*12, nums[l], RGB565(128, 128, 128));
//				ST7735_PutStr5x7(itoa(reg_val, buf, 10), 25, 2+l*12, RGB565(128, 128, 128));
				camera_defaults++;
			}

	while(1) {


		//kamera dziwnie odpowiada,
		//nie mozna zmienic formatu video
		// w wierszu jest az 128 pixeli
		//zdebuguj sccb, upewnij sie ze rejestry sie dobrze wpisuja

//		GPIO_SetBits(GPIOC, GPIO_Pin_13);

		//picture going
/*		while(!(GPIOB->IDR & GPIO_Pin_11)){
			//high - row going
			while((GPIOB->IDR & GPIO_Pin_10)){

				i++;
				//low
				while(!(GPIOB->IDR & GPIO_Pin_1)){GPIOC->BRR |= GPIO_Pin_13;}
//				pixVal = (GPIOA->IDR & 0xff);
//				uartSend(pixVal);

				row[i] = (GPIOA->IDR & 0xff);
				//high
				while((GPIOB->IDR & GPIO_Pin_1));

				//low
				while(!(GPIOB->IDR & GPIO_Pin_1));

				//high
				while((GPIOB->IDR & GPIO_Pin_1)){
					//if HREF low, exit loop, row finished
					if(!(GPIOB->IDR & GPIO_Pin_10))
						break;
				}

			}//no data now
			uartSend(i);
			uartSend(" ");
			j++;
			uartSend(j);
//			for(int x = 0; x < 120; x++){
//				pixVal = row[x];
//				ST7735_Pixel(x, j, RGB565(pixVal,pixVal,pixVal));
//			}
			i=0;

			//low href, wait for new row
			while(!(GPIOB->IDR & GPIO_Pin_10)){
				//without it, it wont let go out of "parent" while
				if((GPIOB->IDR & GPIO_Pin_11))
					break;
			}
		}//end of picture

		j=0;
//		GPIO_ResetBits(GPIOC, GPIO_Pin_13);

		//VSYNC high, next pic indicator
		while((GPIOB->IDR & GPIO_Pin_11)){
		}
*/


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

//			ST7735_PutStr5x7(10, 40,itoa(row[80],cnt_s, 10), RGB565(128, 128, 128));
//			ST7735_PutStr5x7(10, 50,itoa(row[81],cnt_s, 10), RGB565(128, 128, 128));
//			ST7735_PutStr5x7(10, 60,itoa(row[82],cnt_s, 10), RGB565(128, 128, 128));

/* pixel test display
			j=row[80];
			ST7735_Pixel(5, 6, RGB565(j,j,j));
			j=row[81];
			ST7735_Pixel(5, 7, RGB565(j,j,j));
			j=row[82];
			ST7735_Pixel(5, 8, RGB565(j,j,j));
*/




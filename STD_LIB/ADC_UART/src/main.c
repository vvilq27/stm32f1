/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stdio.h>
#include <stdint.h>
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

//=======================================
//					MAIN
//=======================================

int main(void){
 GPIO_InitTypeDef gpio;
 USART_InitTypeDef uart;
 ADC_InitTypeDef adc;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

 RCC_ADCCLKConfig(RCC_PCLK2_Div6);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

 //UART
 GPIO_StructInit(&gpio);
 gpio.GPIO_Pin = GPIO_Pin_2;
 gpio.GPIO_Mode = GPIO_Mode_AF_PP;
 GPIO_Init(GPIOA, &gpio);

 gpio.GPIO_Pin = GPIO_Pin_3;
 gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 GPIO_Init(GPIOA, &gpio);

 USART_StructInit(&uart);
 uart.USART_BaudRate = 115200;
 USART_Init(USART2, &uart);
 USART_Cmd(USART2, ENABLE);

 //ADC
 /*
 gpio.GPIO_Pin = GPIO_Pin_0;
 gpio.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_Init(GPIOA, &gpio);

 ADC_StructInit(&adc);
 adc.ADC_ContinuousConvMode = ENABLE;
 adc.ADC_NbrOfChannel = 1;
 adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
 ADC_Init(ADC1, &adc);

 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
 ADC_Cmd(ADC1, ENABLE);

 ADC_ResetCalibration(ADC1);
 while (ADC_GetResetCalibrationStatus(ADC1));

 ADC_StartCalibration(ADC1);
*/
 //GPIO
 GPIO_StructInit(&gpio); // domyslna konfiguracja
 gpio.GPIO_Pin = GPIO_Pin_13; // konfigurujemy pin 5
 gpio.GPIO_Mode = GPIO_Mode_Out_PP; // jako wyjscie
 GPIO_Init(GPIOC, &gpio); // inicjalizacja modulu GPIOA

 GPIO_StructInit(&gpio); // domyslna konfiguracja
 gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; // konfigurujemy pin 5
 gpio.GPIO_Mode = GPIO_Mode_IPD; // jako wyjscie
 GPIO_Init(GPIOB, &gpio); // inicjalizacja modulu GPIOA

/*
 while (ADC_GetCalibrationStatus(ADC1));

 ADC_SoftwareStartConvCmd(ADC1, ENABLE);
*/

 GPIO_SetBits(GPIOC, GPIO_Pin_13);
 //=======================================
 //					LOOP
 //=======================================

//no clock on ADC or port A
 while (1) {
//	 uint16_t adc = ADC_GetConversionValue(ADC1);
//	 float v = (float)adc * 3.3f / 4096.0f;

	 if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))
		 printf("BIT 4 is high \r\n");
	 if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
		 printf("BIT 5 is high\r\n");

	 for(int i = 0; i < 50000; i++);
 }//end while


}//end main

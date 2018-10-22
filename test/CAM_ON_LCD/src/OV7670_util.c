/*
 * OV7670_util.c
 *
 *  Created on: 27 wrz 2018
 *      Author: arazu
 */

#include "../inc/stm32f103.h"
#include "stm32f10x.h"

 TIM_TimeBaseInitTypeDef tim;
 GPIO_InitTypeDef gpio;
 TIM_OCInitTypeDef  channel;

void twoWireClk(char v) {
    pinOutput(GPIOC_BASE, 14, v);
}

void twoWireData(char v) {
    pinOutput(GPIOC_BASE, 15, v);
}

void twoWireAsOutput() {
    pinMode(GPIOC_BASE, 15, PIN_MODE_OUT_SLOW, PIN_CNF_O_PP);
}

void twoWireAsInput() {
    pinMode(GPIOC_BASE, 15, PIN_MODE_IN, PIN_CNF_I_FLT);
}

char twoWireRead() {
    return pinInput(GPIOC_BASE, 15);
}

void twoWireDelay() {
    int n = 50; while (n--) asm("nop");
}

void twoWireStart() {
    twoWireClk(1);
    twoWireDelay();
    twoWireData(1);
    twoWireDelay();
    twoWireData(0);
    twoWireDelay();
    twoWireClk(0);
    twoWireDelay();
}

void twoWireStop() {
    twoWireData(0);
    twoWireDelay();
    twoWireClk(1);
    twoWireDelay();
    twoWireData(1);
    twoWireDelay();
    twoWireRead();
    twoWireDelay();
}

//bit changes on falling CLK
//bit is read on rising edge
//MSB goes 1st
//high bit after sent 8 data bits is camera response
int twoWireWriteByte(unsigned char b) {
    int i;
    for (i = 0; i < 8; i++) {
        twoWireData((b & 0x80) ? 1 : 0);
        b <<= 1;
        twoWireDelay();
        twoWireClk(1);
        twoWireDelay();
        twoWireClk(0);
    }
    twoWireAsInput();
    twoWireDelay();
    twoWireClk(1);
    twoWireDelay();
    i = twoWireRead();
    twoWireClk(0);
    twoWireDelay();
    twoWireAsOutput();
    return i;
}

unsigned char twoWireReadByte() {
    int i;
    unsigned char b = 0;
    twoWireAsInput();
    for (i = 0; i < 8; i++) {
        b <<= 1;
        twoWireDelay();
        twoWireClk(1);
        twoWireDelay();
        b |= twoWireRead();
        twoWireClk(0);
    }
    twoWireAsOutput();
    twoWireData(1);
    twoWireDelay();
    twoWireClk(1);
    twoWireDelay();
    twoWireClk(0);
    return b;
}

//14 CLK, 15 DATA
void twoWireInit() {
    pinMode(GPIOC_BASE, 14, PIN_MODE_OUT_SLOW, PIN_CNF_O_PP);
    pinOutput(GPIOC_BASE, 14, 1);
    twoWireAsOutput();
    twoWireDelay();
}

int cameraReadByte(unsigned char addr) {
    unsigned char res;
    twoWireStart();
    twoWireWriteByte(0x42);
    twoWireWriteByte(addr);
    twoWireStop();
    twoWireStart();
    twoWireWriteByte(0x43);
    res = twoWireReadByte();
    twoWireStop();
    return res;
}

void cameraWriteByte(unsigned char addr, unsigned char v) {
    unsigned char res;
    int a = 0;
    twoWireStart();
    a |= twoWireWriteByte(0x42);
    a |= (twoWireWriteByte(addr) << 1);
    a |= (twoWireWriteByte(v) << 2);
    twoWireStop();
}

void stmForCameraSetup(void){
	 GPIO_StructInit(&gpio);
	 gpio.GPIO_Pin = GPIO_Pin_6;
	 gpio.GPIO_Speed = GPIO_Speed_50MHz;
	 gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_Init(GPIOB, &gpio);

	 //tim freq setup
	 TIM_TimeBaseStructInit(&tim);
	 tim.TIM_CounterMode = TIM_CounterMode_Up;
	 tim.TIM_Prescaler = 16-1;		//1 for 8 mhz
	 tim.TIM_Period = 8 - 1;
	 TIM_TimeBaseInit(TIM4, &tim);


	 TIM_OCStructInit(&channel);
	  channel.TIM_OCMode = TIM_OCMode_PWM1;
	  channel.TIM_OutputState = TIM_OutputState_Enable;
	  channel.TIM_Pulse = 4;
	  TIM_OC1Init(TIM4, &channel);

	  TIM_Cmd(TIM4, ENABLE);
}

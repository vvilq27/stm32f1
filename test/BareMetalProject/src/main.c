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
#include "stm32f103.h"
#include "util.h"
			

// This section can go into a header file if wanted
// Define some types for readibility
#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char

#define HSE_Value       ((uint32_t) 25000000) /* Value of the External oscillator in Hz */
#define HSI_Value       ((uint32_t)  8000000) /* Value of the Internal oscillator in Hz*/

// Define the base addresses for peripherals
#define PERIPH_BASE     ((uint32_t) 0x40000000)
#define SRAM_BASE       ((uint32_t) 0x20000000)

#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x20000)

#define AFIO_BASE       (APB2PERIPH_BASE + 0x0000) //   AFIO base address is 0x40010000
#define GPIOA_BASE      (APB2PERIPH_BASE + 0x0800) //  GPIOA base address is 0x40010800
#define USART1_BASE     (APB2PERIPH_BASE + 0x3800) // USART1 base address is 0x40013800

#define RCC_BASE        ( AHBPERIPH_BASE + 0x1000) //   RCC base address is 0x40021000
#define FLASH_BASE      ( AHBPERIPH_BASE + 0x2000) // FLASH base address is 0x40022000

#define NVIC_BASE       ((uint32_t) 0xE000E100)

#define STACKINIT       0x20008000
#define DELAY           7200000

#define AFIO            ((AFIO_type  *)   AFIO_BASE)
#define GPIOA           ((GPIO_type  *)  GPIOA_BASE)
#define RCC             ((RCC_type   *)    RCC_BASE)
#define FLASH           ((FLASH_type *) FLASH_BASE)
#define USART1          ((USART_type *) USART1_BASE)
#define NVIC            ((NVIC_type  *)   NVIC_BASE)

/*
 * Register Addresses
 */
typedef struct
{
	uint32_t CRL;      /* GPIO port configuration register low,      Address offset: 0x00 */
	uint32_t CRH;      /* GPIO port configuration register high,     Address offset: 0x04 */
	uint32_t IDR;      /* GPIO port input data register,             Address offset: 0x08 */
	uint32_t ODR;      /* GPIO port output data register,            Address offset: 0x0C */
	uint32_t BSRR;     /* GPIO port bit set/reset register,          Address offset: 0x10 */
	uint32_t BRR;      /* GPIO port bit reset register,              Address offset: 0x14 */
	uint32_t LCKR;     /* GPIO port configuration lock register,     Address offset: 0x18 */
} GPIO_type;

typedef struct
{
	uint32_t SR;       /* Address offset: 0x00 */
	uint32_t DR;       /* Address offset: 0x04 */
	uint32_t BRR;      /* Address offset: 0x08 */
	uint32_t CR1;      /* Address offset: 0x0C */
	uint32_t CR2;      /* Address offset: 0x10 */
	uint32_t CR3;      /* Address offset: 0x14 */
	uint32_t GTPR;     /* Address offset: 0x18 */
} USART_type;

typedef struct
{
	uint32_t CR;       /* RCC clock control register,                Address offset: 0x00 */
	uint32_t CFGR;     /* RCC clock configuration register,          Address offset: 0x04 */
	uint32_t CIR;      /* RCC clock interrupt register,              Address offset: 0x08 */
	uint32_t APB2RSTR; /* RCC APB2 peripheral reset register,        Address offset: 0x0C */
	uint32_t APB1RSTR; /* RCC APB1 peripheral reset register,        Address offset: 0x10 */
	uint32_t AHBENR;   /* RCC AHB peripheral clock enable register,  Address offset: 0x14 */
	uint32_t APB2ENR;  /* RCC APB2 peripheral clock enable register, Address offset: 0x18 */
	uint32_t APB1ENR;  /* RCC APB1 peripheral clock enable register, Address offset: 0x1C */
	uint32_t BDCR;     /* RCC backup domain control register,        Address offset: 0x20 */
	uint32_t CSR;      /* RCC control/status register,               Address offset: 0x24 */
	uint32_t AHBRSTR;  /* RCC AHB peripheral clock reset register,   Address offset: 0x28 */
	uint32_t CFGR2;    /* RCC clock configuration register 2,        Address offset: 0x2C */
} RCC_type;

typedef struct
{
	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t AR;
	uint32_t RESERVED;
	uint32_t OBR;
	uint32_t WRPR;
} FLASH_type;

typedef struct
{
	uint32_t EVCR;      /* Address offset: 0x00 */
	uint32_t MAPR;      /* Address offset: 0x04 */
	uint32_t EXTICR1;   /* Address offset: 0x08 */
	uint32_t EXTICR2;   /* Address offset: 0x0C */
	uint32_t EXTICR3;   /* Address offset: 0x10 */
	uint32_t EXTICR4;   /* Address offset: 0x14 */
	uint32_t MAPR2;     /* Address offset: 0x18 */
} AFIO_type;

typedef struct
{
	uint32_t   ISER[8];     /* Address offset: 0x000 - 0x01C */
	uint32_t  RES0[24];     /* Address offset: 0x020 - 0x07C */
	uint32_t   ICER[8];     /* Address offset: 0x080 - 0x09C */
	uint32_t  RES1[24];     /* Address offset: 0x0A0 - 0x0FC */
	uint32_t   ISPR[8];     /* Address offset: 0x100 - 0x11C */
	uint32_t  RES2[24];     /* Address offset: 0x120 - 0x17C */
	uint32_t   ICPR[8];     /* Address offset: 0x180 - 0x19C */
	uint32_t  RES3[24];     /* Address offset: 0x1A0 - 0x1FC */
	uint32_t   IABR[8];     /* Address offset: 0x200 - 0x21C */
	uint32_t  RES4[56];     /* Address offset: 0x220 - 0x2FC */
	uint8_t   IPR[240];     /* Address offset: 0x300 - 0x3EC */
	uint32_t RES5[644];     /* Address offset: 0x3F0 - 0xEFC */
	uint32_t       STIR;    /* Address offset:         0xF00 */
} NVIC_type;


void set_system_clock_to_72Mhz(void)
{
	// Necessary wait states for Flash for high speeds
	FLASH->ACR = 0x12;
	// Enable HSE
	RCC->CR |= (1 << 16);
	// Wait untill HSE settles down
	while (!(RCC->CR & (1 << 17)));
	// Set PREDIV2 division factor to 5
	RCC->CFGR2 |= (0b0100 << 4);
	// Set PLL2 multiplication factor to 8
	RCC->CFGR2 |= (0b0110 << 8);
	// Enable PLL2
	RCC->CR |= (1 << 26);
	// Wait untill PLL2 settles down
	while (!(RCC->CR & (1 << 27)));
	// Set PLL2 as PREDIV1 clock source
	RCC->CFGR2 |= (1 << 16);
	// Set PREDIV1 division factor to 5
	RCC->CFGR2 |= (0b0100 << 0);
	// Select Prediv1 as PLL source
	RCC->CFGR |= (1 << 16);
	// Set PLL1 multiplication factor to 9
	RCC->CFGR |= (0b0111 << 18);
	// Set APB1 to 36MHz
	RCC->CFGR |= 1 << 10;
	// Enable PLL
	RCC->CR |= (1 << 24);
	// Wait untill PLL settles down
	while (!(RCC->CR & (1 << 25)));
	// Finally, choose PLL as the system clock
	RCC->CFGR |= (0b10 << 0);
}

void delay(int time)
{
    int i;
    for (i = 0; i < time * 4000; i++) {}
}

void timer2SetupToggleOutput(unsigned short prescale, unsigned short timeout) {
//    REG_L(RCC_BASE, RCC_APB2ENR) |= (1 << 0); // AFIO clock

//    REG_L(AFIO_BASE, AFIO_MAPR) &= ~(7 << 24); // JTAG/SW bits clear
//    REG_L(AFIO_BASE, AFIO_MAPR) |= 4 << 24; // disable JTAG/SW

//    REG_L(AFIO_BASE, AFIO_MAPR) &= ~(3 << 8); // no TIM2 remap
//    REG_L(AFIO_BASE, AFIO_MAPR) |= 1 << 8; // TIM2 CH1/2 remapped
//    pinMode(GPIOA_BASE, 15, PIN_MODE_OUT, PIN_CNF_O_APP);
    REG_L(RCC_BASE, RCC_APB1ENR) |= (1 << 0); // timer 2 enabled

    /*
    REG_S(TIM2_BASE, TIM_PSC) = prescale - 1; // divide by prescaler
    REG_S(TIM2_BASE, TIM_ARR) = timeout - 1;
    REG_S(TIM2_BASE, TIM_CNT) = 0;
    REG_S(TIM2_BASE, TIM_CCR1) = timeout - 1;
    REG_S(TIM2_BASE, TIM_CCMR1) = 0x30; // toggle output compare
    REG_S(TIM2_BASE, TIM_CCER) = 1; // output compare enable
    REG_S(TIM2_BASE, TIM_CR1) = 1; // start
*/

//    REG_S(TIM2_BASE, TIM_PSC) = 16000 - 1; // divide by prescaler
//    REG_S(TIM2_BASE, TIM_ARR) = 1000 - 1;	//period
//    REG_S(TIM2_BASE, TIM_CNT) = 0;
    TIM2->PSC =7200-1;
    TIM2->ARR = 10000-1;
    TIM2->EGR |= (1<<0);
    TIM2->CR1 |= TIM_CR1_CEN;

}
void delay_ms(volatile uint32_t s)
{
	for(s; s>0; s--){
		// Reset the timer
		TIM2->EGR |= 0x0001;
		// Wait until timer reaches to 1000
		// It is 1000 becuase timer is running at 1 MHz and 1000 will
		//   generate 1 milli-second
 		while(TIM2->CNT < 1000);
	}
}



int main(void)
{
//	set_system_clock_to_72Mhz();
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	timer2SetupToggleOutput(1,3);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gpio);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &gpio);

	uartEnable(0x271);
	/*
	// Enable alternate function clock. Bit 0 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 0);
	// Enable GPIOA clock. Bit 2 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 2);
	// Enable clock for UART1 clock. Bit 14 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 14);

	// Make GPIOA Pin 9,10 (PA9, PA10) alternate-function output (0b1010)
	GPIOA->CRH &= 0xFFFFF00F;
	GPIOA->CRH |= 0x00000BB0;

	// Enable USART
	USART1->CR1 |= (1 << 13);
	// Word length - leave default (8 data)
	USART1->CR1 |= (0 << 12);
	// Number of stop bits - leave default (1 stop)
	USART1->CR2 |= (0b00 << 12);
	// Baud rate
	// BRR should be 468.75 for 9600 baud rate
	// Thus manista is 468 (0x1d4) and fraction is 12 (0xc) (12/16 is .75)
	// Making it 0x1d4c
	USART1->BRR = 0x1d4c;
	// Transmitter enable
	USART1->CR1 |= (1 << 3);

	GPIO_SetBits(GPIOC, GPIO_Pin_13);
*/

/*
 * 		TIMER 3
 */
	// Enable clock for that module for TIM3. Bit1 in RCC APB1ENR register
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Reset CR1 just in case
	TIM3->CR1 = 0x0000;

	// fCK_PSC / (PSC[15:0] + 1)
	// 72 Mhz / 71 + 1 = 1 Mhz timer clock speed
	TIM3->PSC = 7200;

	// This is set to max value (0xFFFF) since we manually check
	//   if the value reach to 1000 in the delay_ms function
	TIM3->ARR = 10000;

	// Finally enable TIM3 module
	TIM3->CR1 |= (1 << 0);

/*
 * 		TIMER 1 PWM
 */

	// Enable clock for TIM1 clock. Bit 11 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 11);

//	// Make GPIOE Pin 14 (PE14) alternate-function output (0b1010)
//	GPIOE->CRH &= 0xF0FFFFFF;
//	GPIOE->CRH |= 0x0B000000;

//	// Make PE8 & PE15 output to fix LEDs
//	GPIOE->CRH &= 0x0FFFFFF0;
//	GPIOE->CRH |= 0x20000002;
//	GPIOE->ODR = 0;

//	// MAP The TIM1 CH4 to PE14 pin (defaults to PA8)
//	//  write 0b11 to bits 7:6
//	AFIO->MAPR |= 0x000000C0;

	// Reset CR1 just in case
//	TIM1->CR1 = 0x0000;

	// Set prescaler
	// fCK_PSC / (PSC[15:0] + 1)
	// 72 Mhz / 71 + 1 = 1 Mhz timer clock speed
	TIM1->PSC = 7200-1;
	// Set period to 5000
	TIM1->ARR = 50000;
	// Set duty cycle
	TIM1->CCR4 = 5000;

	// Enable Capture/Compare 4 interrupt
	TIM1->DIER = (1 << 4);

	// Select pwm
	// Preload enable and PWM mode 1 for CH4

	TIM1->CCMR2 |= 0x6800;

	// Enable CH4 output and polarity active-high
	TIM1->CCER |= 0x1000;

	// Enable Main Output
	TIM1->BDTR |= (1 << 15);

//	enable_interrupt(TIM1_CC_IRQn);

	// Finally enable TIM1 module
	TIM1->CR1 |= (1 << 0);

	while(1){
		uartSends("abc");
		uartSendHex(0x11,2);
//		if(!(GPIOB->IDR == GPIO_Pin_13) )
//		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))

//		if((GPIOB->IDR & GPIO_Pin_13) != 0 )
//			GPIOB->BSRR = GPIO_Pin_12;
//
//		else
//			GPIOB->BRR = GPIO_Pin_12;
//
//		USART1->DR = 'a' + i;
//		while(!(USART1->SR & (1 << 6)));

		if(TIM3->CNT > 100)
			GPIOC->BSRR |= GPIO_Pin_13;
		else
			GPIOC->BRR = GPIO_Pin_13;
	}
}

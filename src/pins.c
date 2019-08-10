/*
 * pins.c
 *
 *  Created on: 03.07.2019
 *      Author: DS
 */

#include "stm32f10x.h"

int fAPB1 = 8*1000000; //max 36M
int fAPB2 = 128*1000000; //max 72M
int fAHB = 128*1000000; //max 72M
void configureClocks(){
	//set clock to 8xPPL MHz
	/*enable clocks*/
	//
	RCC->CR |= RCC_CR_HSEON;
	//RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2;
	RCC->CFGR = RCC_CFGR_PLLMULL16 | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV16;
	while (!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	//RCC->CR &= ~RCC_CR_HSION;

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

void configureGPIO(){
	//
	// out, od, 2MHz: 0x2=0b0010
	// out, pp, 2MHz: 0x6=0b0110
	// out, pp, 50MHz, 0x7=0b0111
	// out, alt, pp, 50MHz: 0xB=0b1011
	// out, alt, pp, 2MHz: 0xA=0b1010
	// out, alt, od, 2MHz: 0xE=0b1110

	// in, fl: 0x4=0b0100
	// in, an: 0x0=0b0000

	//======================================
	// PA0..7 (D0..7) - in, fl.
				///76543210
	GPIOA->CRL = 0x44444444;
	//--------------------------------------
	// PA8 (MCO-XCLK) = out, alt, pp, 50MHz
	// PA9 (USART1 TX) = out, alt, pp, 2MHz
	// PA10 (USART1 RX) = in, float
	// PA11 (PCLK) = in
				///54321098
	GPIOA->CRH = 0x444444AB;
	//GPIOA->CRH = ((GPIOA->CRH & 0xFFFFFF00) | 0x000000AA);
	//======================================
	// PB6 (SCL1) = out, alt, od, I2C1
	// PB7 (SDA1) = out, alt, od, I2C1
				///76543210
	GPIOB->CRL = 0xEE444444;
	//--------------------------------------
	// PB10 (HREF) = in
	// PB11 (VSYNC) = in
	// PB12 (RST) = out, pp
	// PB13 (PWUP) = out, pp
				///54321098
	GPIOB->CRH = 0x44664444;

	// reset OV7670
	GPIOB->BSRR |= GPIO_BSRR_BS12 | GPIO_BSRR_BS13;
	volatile int t;
	for(t=0; t<200; ++t) asm("NOP"); //reset
	GPIOB->BSRR |= GPIO_BSRR_BR12 | GPIO_BSRR_BS13;
	//======================================
	// not avail.
				///76543210
	//GPIOC->CRL = 0x44444444;
	//--------------------------------------
	// PC13 (built-in LED) = out, pp, 50MHz
	// PC14 (GND) = out, pp, 2MHz
	// PC15 (VDD) = out, pp, 2MHz
				///54321098
	GPIOC->CRH = 0x66744444;
	GPIOC->CRH = ((GPIOC->CRH & 0x000FFFFF) | 0x66600000);
	//GPIOC->BSRR |= (1<<14) | (1<<15);
}

void configureMCO(){
	//main clock output for OV7670
	RCC->CFGR |= RCC_CFGR_MCO_HSI;
}

void configureUART(){
	//USART1->BRR = 72000000/4000000; //BaudRate_Register=CPU_Freq / Desired_Baudrate
	USART1->BRR = fAPB2/4000000;
	/*Default configuration: 8N1 */
	/* Enable: USART, Rx not empty interrupt, Tx, Rx */
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE;
	//USART1->DR = 'K'; //"ready"
	NVIC_EnableIRQ(USART1_IRQn);
}

void configureI2C(){
	//master
	int F = fAPB1/1000000;
	I2C1->CR2 |= F; // MHz
	// I2C slow mode - 100kHz
	// duty cycle - 50:50
	I2C1->CCR |= F*50;
	// max rise time in Sm = 1000ns
	I2C1->TRISE |= F*10 +1;
	I2C1->CR1 |= I2C_CR1_PE;
}

void configureEXTI(){

	//AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
	//AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
	AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;

	EXTI->RTSR |=  EXTI_RTSR_TR11;
	//EXTI->RTSR |=  EXTI_RTSR_TR10;
	//EXTI->FTSR |= EXTI_FTSR_TR11;
	//EXTI->FTSR |= EXTI_FTSR_TR10;
	EXTI->IMR |= EXTI_IMR_MR11;
	///EXTI->IMR |= EXTI_IMR_MR10;

	//NVIC_EnableIRQ(EXTI2_IRQn);
	//NVIC_EnableIRQ(EXTI4_IRQn);
	//NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void configureTimer(){
	/*
	TIM3->CCMR2 |= TIM_CCMR2_CC3S_0;
	TIM3->CCER |= TIM_CCER_CC3P;
	TIM3->CCER |= TIM_CCER_CC3E;

	TIM3->DIER |= TIM_DIER_CC3DE; // DMA

	TIM3->CNT=0; // Reset CouNT register
	//TIM3->PSC=7;
	//TIM3->CR1 |= TIM_CR1_CEN;
	*/
	//=================================
	//T1C4 as input capture to generate DMA req. on PCLK rising edge
	TIM1->CCMR2 |= TIM_CCMR2_CC4S_0;  //
	TIM1->CCER |= TIM_CCER_CC4P; //
	TIM1->CCER |= TIM_CCER_CC4E; //
	TIM1->DIER |= TIM_DIER_CC4DE; // DMA
	TIM1->CNT=0; // Reset CouNT register

}

void configureADC(){
	/* configure ADC1: order of channels*/
	ADC1->SQR3 |= 0b00010; /*1st: channel 2 (on PA2 pin)*/
	/* number of channels =0b00000 +1 = 1 */
		/*
		 * Set conversion time
		 * I don't care about speed, so I set the longest possible
		 * (but most precise): 239.5 cycles
		 * (With 8MHz (default) clock it takes ~30us
		 */
	ADC1->SMPR2 |= 0b000000111;

	/* Discontinuous mode: 1 channel per conversion */
	ADC1->CR1 |= ADC_CR1_DISCEN;
	/* Start conversion with writing SWSTART bit */
	ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
	/* Turn on ADC1 */
	ADC1->CR2 |= ADC_CR2_ADON;

	/*
	* Start calibration procedure
	* and wait till it ends
	*/

	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);
}

//char I2C_read(char);
void I2C_write(char, char);

//register, value, ...
char RV1[] = {

		//QQVGA, YUV according to IG
		0x11, 0x02, //prescaler
		0x12, 0x00,
		0x0c, 0x04,
		0x3e, 0x1a,
		0x70, 0x3a,
		0x71, 0x35,
		0x72, 0x22,
		0x73, 0xf2,
		0xa2, 0x02,
		0x15, 0x20,	//gate PCLK via HREF
		0xff, 0xff	//terminate
};
char RV2[] = {
		//QVGA, YUV according to IG
		0x11, 0x0F, //prescaler
		0x12, 0x00,
		0x0c, 0x04,
		0x3e, 0x19,
		0x70, 0x3a,
		0x71, 0x35,
		0x72, 0x11,
		0x73, 0xf1,
		0xa2, 0x02,
		0x15, 0x20,	//gate PCLK via HREF
		0xff, 0xff //terminate
};
char RV3[] = {
		//QVGA, YUV according to IG
		0x11, 0x0A, //prescaler
		0x12, 0x00,
		0x0c, 0x00,
		0x3e, 0x00,
		0x70, 0x3a,
		0x71, 0x35,
		0x72, 0x11,
		0x73, 0xf0,
		0xa2, 0x02,
		//0x56, 0x60, //contrast
		//0x41, 0x30, //sharpness
		0x15, 0x20,	//gate PCLK via HREF
		0xff, 0xff //terminate
};
void configureCamera(void){
	volatile int i;
	for(i=100000;i>0;--i); //give time to start
	I2C_write(0x12, 0x80); //RESET
	for(i=100000;i>0;--i);

	i = 0;
	char* p = RV3;
	while(p[i] != 0xff){
		I2C_write(p[i], p[i+1]);
		i=i+2;
	}
}

void configureDMA(){
	/*
	DMA1_Channel2->CNDTR = 2;
	DMA1_Channel2->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel2->CMAR = (uint32_t)&dma_test;
	//DMA1_Channel2->CMAR = (uint32_t)&(GPIOA->IDR);
	DMA1_Channel2->CCR |= DMA_CCR2_MINC | DMA_CCR2_CIRC | DMA_CCR2_DIR; //| DMA_CCR3_CIRC
	//DMA1_Channel2->CCR |= DMA_CCR2_EN;
	*/
	DMA1_Channel4->CNDTR = 0xFFFF;
	DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
	//DMA1_Channel4->CMAR = (uint32_t)&dma_test;
	DMA1_Channel4->CMAR = (uint32_t)&(GPIOA->IDR);
	DMA1_Channel4->CCR |= (DMA_CCR4_MINC&0) | (DMA_CCR4_CIRC) | DMA_CCR4_DIR;
	//DMA1_Channel4->CCR |= DMA_CCR4_EN;
}

void blink(){
	GPIOC->BSRR |= ((GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13); //blink
}

int r4=0;
char RX=0;
char Rxr=0;
char Rxv=0;
__attribute__((interrupt)) void USART1_IRQHandler(void){
	if(USART1->SR & USART_SR_RXNE){
		RX = USART1->DR; //MUST-READ
		switch(RX){
		case '1': //capture
			// on data receive: action

			USART1->DR = 'k';//
			/*
			 *         __                      __
			 * _______/  \____________________/  \_Vsync
			 *   _           _   _   _   _
			 * _/ \_________/ \_/ \_/ \_/ \________ Pclk
			 *
			 *           +DMA                  +dma
			 */
			while(!(USART1->SR & USART_SR_TXE));
			//if((GPIOB->IDR & GPIO_IDR_IDR11))
			//while(GPIOB->IDR & GPIO_IDR_IDR11);

			while(!(GPIOB->IDR & GPIO_IDR_IDR11));

			GPIOC->BSRR = GPIO_BSRR_BR13;
			TIM1->CR1 |= TIM_CR1_CEN;
			DMA1_Channel4->CCR |= DMA_CCR4_EN;
			//for(int i=0; i<100;++i)asm("nop");
			//NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			//EXTI->PR = 0xFFFF;
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			//while((GPIOB->IDR & GPIO_IDR_IDR11));
			//for(int i=0; i<0xFFFF; ++i) asm("nop");
			//for(volatile int i=0; i<130455; ++i);
			while(GPIOB->IDR & GPIO_IDR_IDR11);

			//while(!(GPIOB->IDR & GPIO_IDR_IDR11));
			//DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
			//TIM1->CR1 &= ~TIM_CR1_CEN;
			break;
		case '9': //command
			NVIC_DisableIRQ(USART1_IRQn);
			USART1->DR = 'r';
			while(!(USART1->SR & USART_SR_RXNE));
			Rxr = USART1->DR;
			USART1->DR = 'v';
			while(!(USART1->SR & USART_SR_RXNE));
			Rxv = USART1->DR;
			I2C_write(Rxr, Rxv);
			USART1->DR = 'i';
			NVIC_ClearPendingIRQ(USART1_IRQn);
			NVIC_EnableIRQ(USART1_IRQn);
			break;
		case 'R': //reset all
			break;
		case 'r': //reset camera
			break;
		}
	}
}

int cnt_HP=0;
int cnt_HF=0;
int cnt_VP=0;
int cnt_VF=0;
__attribute__((interrupt)) void EXTI15_10_IRQHandler(void){
	//if(EXTI->PR & EXTI_PR_PR11){ //_VSYNC_
		EXTI->PR = EXTI_PR_PR11;
		//for(int i=0; i<100; ++i) asm("nop");
		if((GPIOB->IDR & GPIO_IDR_IDR11)){// rising

			//while(GPIOB->IDR & GPIO_IDR_IDR11);
			//USART1->DR = 'V';
			++cnt_VP;
			GPIOC->BSRR = GPIO_BSRR_BS13;
			DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
			TIM1->CR1 &= ~TIM_CR1_CEN;
			NVIC_DisableIRQ(EXTI15_10_IRQn);
		}
		//else{ //falling
			//USART1->DR = 'v';
		//	++cnt_VF;
		//}
}

__attribute__((interrupt)) void SysTick_Handler(void){
	blink();
	return;
}

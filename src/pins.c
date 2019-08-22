/*
 * pins.c
 *
 *  Created on: 03.07.2019
 *      Author: DS
 */

#include "stm32f10x.h"
#include "ov7670.h"


int fAPB1 = 8*1000000;
int fAPB2 = 128*1000000;
int fAHB = 128*1000000;

void configureClocks(){
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
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

void configureGPIO(){
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
	// PA11 (PCLK) = in, fl
				///54321098
	GPIOA->CRH = 0x444444AB;
	//GPIOA->CRH = ((GPIOA->CRH & 0xFFFFFF00) | 0x000000AA);
	//======================================
	// PB0 (sensor OUT) = in, fl
	// PB6 (SCL1) = out, alt, od, I2C1
	// PB7 (SDA1) = out, alt, od, I2C1
				///76543210
	GPIOB->CRL = 0xEE444444;
	//--------------------------------------
	// PB10 (HREF) = in, fl
	// PB11 (VSYNC) = in, fl
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
				/////76543210
	//GPIOC->CRL = 0x44444444;
	//--------------------------------------
	// PC13 (built-in LED) = out, pp, 50MHz
	// PC14 (GND) = out, pp, 2MHz
	// PC15 (VDD) = out, pp, 2MHz
				///54321098
	GPIOC->CRH = 0x66644444;
	GPIOC->BSRR |= GPIO_BSRR_BR14 | GPIO_BSRR_BS15;
}

void configureMCO(){
	// Main Clock Output for OV7670 XCLK
	// source: HSI (8MHz)
	RCC->CFGR |= RCC_CFGR_MCO_HSI;
}

void configureUART(){
	// BaudRate_Register = CPU_Freq / Desired_Baudrate
	USART1->BRR = fAPB2/4000000;
	//Default configuration: 8N1, no changes
	// Enable: USART, Rx not empty interrupt, Tx, Rx
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE;
	// Enable interrupts
	NVIC_EnableIRQ(USART1_IRQn);
}

void configureI2C(){
	// Configure I2C1: Sm (100kHz), duty cycle: 50:50, Rise time: 1000ns (max for Sm)
	int F = fAPB1/1000000; //clock, MHz

	//Simplified formulas from RM:
	I2C1->CR2 |= F; // input clock, MHz
	I2C1->CCR |= F*50; // Sm (100KHz), duty cycle: 50:50
	I2C1->TRISE |= F*10 +1; // rise time: 1000ns
	// enable I2C
	I2C1->CR1 |= I2C_CR1_PE;
}

void configureEXTI(){
	// Configure EXTernal Interrupt on PB11 (VSYNC)
	AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB; // line 11 - port B
	EXTI->RTSR |=  EXTI_RTSR_TR11; // interrupt on rising edge
	EXTI->IMR |= EXTI_IMR_MR11; // unmask interrupt
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
	// Timer 1, Channel 4 as input capture to generate DMA req. on PCLK rising edge
	// More intuitive approach would be to generate DMA request on rising edge of PCLK
	// However my uC doesn't support it, so I use timer in input capture mode.
	// By default the timer starts counting on rising edge, counts until falling edge
	// and then latches measured value. Then it can also generate interrupt or DMA request.

	// Here I need timer only to generate DMA request on rising edge.
	// I need only to reverse polarity.
	// Measured value is ignored.

	TIM1->CCMR2 |= TIM_CCMR2_CC4S_0;  // input capture mode on channel 4
	TIM1->CCER |= TIM_CCER_CC4P; // reverse polarity: stop measuring on rising edge
	TIM1->CCER |= TIM_CCER_CC4E; // enable
	TIM1->DIER |= TIM_DIER_CC4DE; // DMA reqest on enable
	TIM1->CNT=0; // Reset CouNT register
	// don't enable TIM1 yet!

}

void configureADC(){
	// configure ADC1: order of channels
	ADC1->SQR3 = 8; /*1st: channel 2 (on PB0 pin)*/

		/*
		 * Set conversion time
		 * I don't care about speed, so I set the longest possible
		 * (but most precise): 239.5 cycles
		 * (With 8MHz (default) clock it takes ~30us
		 */
	ADC1->SMPR2 |= 0b000000111;

	// Discontinuous mode: 1 channel per conversion
	ADC1->CR1 |= ADC_CR1_DISCEN;
	// Start conversion with writing SWSTART bit
	ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
	// Turn on ADC1
	ADC1->CR2 |= ADC_CR2_ADON;

	//Start calibration procedure and wait till it ends
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);
}

void configureCamera(void){
	volatile int i; // counter

	for(i=100000;i>0;--i); // give time to start
	I2C_write(0x12, 0x80); // RESET
	for(i=100000;i>0;--i); // give some time to start after reset

	i = 0;
	// use pairs of Register and Value to configure camera
	char* p = RV_vga_yuv;
	while(p[i] != 0xff){ //all pairs are terminated with 0xff, 0xff
		I2C_write(p[i], p[i+1]);
		i=i+2;
	}
}

void configureDMA(){
	// Configure DMA1:
	// 8bits from GPIOA to USART1
	//
	DMA1_Channel4->CNDTR = 0xFFFF; // doesn't matter?
	DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel4->CMAR = (uint32_t)&(GPIOA->IDR);
	// circular: don't stop transfering;
	DMA1_Channel4->CCR |= DMA_CCR4_CIRC | DMA_CCR4_DIR;
	// don't enable DMA yet!
}

void blink(){
	//toggle state of PC13 (built-in LED)
	GPIOC->BSRR |= ((GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13);
}

char RX=0; //received data
// variables for interfacing with I2C
char i2c_register = 0xFF;
char i2c_value = 0;
char i2c_read = 0;

// next received value will be treated as:
// 0: command
// r: register for i2c
// v: value for i2c
char usart_receiving = 0;

__attribute__((interrupt)) void USART1_IRQHandler(void){
	// usart will always receive one byte on one interrupt,
	// and sent one byte
	// exception: sending frame (however, data is sent outside this
	// interrupt, but via DMA requests

	if(USART1->SR & USART_SR_RXNE){
		RX = USART1->DR;

		switch(usart_receiving){
		case 0:
			switch(RX){
			case 0:
				USART1->DR = '0';
				break;
			case '1': //capture
				while(!(USART1->SR & USART_SR_TXE));
				USART1->DR = 'C'; // response
				/*
				 *         __                      __
				 * _______/  \____________________/  \_Vsync
				 *   _           _   _   _   _
				 * _/ \_________/ \_/ \_/ \_/ \________ Pclk
				 *
				 *           +DMA                  +dma
				 */
				while(!(GPIOB->IDR & GPIO_IDR_IDR11)); // wait while VSYNC is LOW
				GPIOC->BSRR = GPIO_BSRR_BR13; // LED on
				TIM1->CR1 |= TIM_CR1_CEN; // enable TIM1
				DMA1_Channel4->CCR |= DMA_CCR4_EN; // enable DMA1
				NVIC_EnableIRQ(EXTI15_10_IRQn); // enable interrupts on VSYNC
				while(GPIOB->IDR & GPIO_IDR_IDR11); //wait while VSYNC is HIGH
				NVIC_DisableIRQ(USART1_IRQn);
				// now DMA will send data via uart and enable them on rising VSYNC
				break;
			case '2': // read distance
				while(!(USART1->SR & USART_SR_TXE));
				USART1->DR = ((GPIOB->IDR & GPIO_IDR_IDR0) ? '_' : '#');
				break;

			case '3': // set register
				USART1->DR = 'r';
				usart_receiving = 'r';
				break;
			case '4': // get register
				USART1->DR = i2c_register;
				break;
			case '5': // set value
				USART1->DR = 'v';
				usart_receiving = 'v';
				break;
			case '6': // get value
				USART1->DR = i2c_value;
				break;
			case '7': // I2C write
				USART1->DR = 'W';
				I2C_write(i2c_register, i2c_value);
				break;
			case '8': // I2C read
				i2c_read = I2C_read(i2c_register);
				USART1->DR = i2c_read;
				break;
			case 'R': //TODO: reset all
				USART1->DR = 'x';
				break;
			case 'r': //TODO: reset camera
				USART1->DR = 'x';
				break;
			default: // unrecognized
				USART1->DR = '?';
				break;
			}
			break;
		case 'r': // save register
			USART1->DR = 'r';
			i2c_register = RX;
			usart_receiving = 0; // normal mode
			break;
		case 'v': // save value
			USART1->DR = 'v';
			i2c_value = RX;
			usart_receiving = 0; // normal mode
			break;
		default: // unrecognized mode
			USART1->DR = 'X';
			usart_receiving=0;
		}
	}
	return;
}

__attribute__((interrupt)) void EXTI15_10_IRQHandler(void){
		EXTI->PR = EXTI_PR_PR11; // clear Pending Register
		if((GPIOB->IDR & GPIO_IDR_IDR11)){ // rising edge - END OF FRAME
			GPIOC->BSRR = GPIO_BSRR_BS13; // turn off LED
			DMA1_Channel4->CCR &= ~DMA_CCR4_EN; // disable DMA1
			TIM1->CR1 &= ~TIM_CR1_CEN; // disable TIM1
			NVIC_DisableIRQ(EXTI15_10_IRQn); // disable interrupts [from VSYNC]
			NVIC_EnableIRQ(USART1_IRQn); // enable USART interrupt
		}
}

__attribute__((interrupt)) void SysTick_Handler(void){
	blink();
	return;
}

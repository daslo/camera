/*
 * main.c
 *
 *  Created on: 04.07.2019
 *      Author: DS
 */

#include "stm32f10x.h"
void configureClocks();
void configureGPIO();
void configureUART();
void configureI2C();
void configureMCO();
void configureEXTI();
void configureTimer();
void configureDMA();
void configureCamera();

int main(void) {
	configureClocks();
	configureGPIO();
	//configureADC();
	configureMCO();
	configureI2C();
	configureUART();
	configureEXTI();
	configureCamera();
	configureTimer();
	configureDMA();

	GPIOC->BSRR = GPIO_BSRR_BS13; //LED off
	//SysTick_Config(16000000);

	for(;;);
}

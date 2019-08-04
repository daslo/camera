/*
 * ov7670.c
 *
 *  Created on: 03.07.2019
 *      Author: DS
 */

#include "stm32f10x.h"


char I2C_read(char reg){
	char tmp=0;

	// send adr. and reg. to read
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = 0x42; //0x42; // write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)); //0x0400 -> ACK fail
	tmp = I2C1->SR2;
	//delay();
	I2C1->DR = reg; // register to read
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	//delay();
	I2C1->CR1 |= I2C_CR1_STOP;

	// send adr. and read register
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	//I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->DR = 0x42 + 0x01; // read address
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	tmp = I2C1->SR2;
	I2C1->CR1 |= I2C_CR1_STOP;
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	tmp = I2C1->DR;

	return tmp;
}

void I2C_write(char reg, char val){
	char tmp=0;

	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = 0x42; //0x42; // write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)); //0x0400 -> ACK fail
	tmp = I2C1->SR2;
	I2C1->DR = reg; // register to write to
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->DR = val; // value
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	//delay();
	I2C1->CR1 |= I2C_CR1_STOP;
}

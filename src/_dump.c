//================================================
	/*
	if(EXTI->PR & EXTI_PR_PR10){
		EXTI->PR = EXTI_PR_PR10; //_HREF_
		if(GPIOB->IDR & GPIO_IDR_IDR10){// rising
			USART1->DR = 'H';
			++cnt_HP;
			//TIM3->CR1 &= ~TIM_CR1_CEN;
			//DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
			//NVIC_DisableIRQ(EXTI15_10_IRQn);
		}
		else{ //falling
			USART1->DR = 'h';
			++cnt_HF;
			//DMA1_Channel2->CCR |= DMA_CCR2_EN;
			//TIM3->CR1 |= TIM_CR1_CEN;
		}
	}
	*/

//================================================

/*
	//QQVGA, YUV according to IG
	I2C_write(0x11, 0x02); //IG:01, set higher prescaler
	I2C_write(0x12, 0x00); //00
	I2C_write(0x0C, 0x04); //04
	I2C_write(0x3E, 0x1a); //1a
	I2C_write(0x70, 0x3a); //3a
	I2C_write(0x71, 0x35); //35
	I2C_write(0x72, 0x22); //22
	I2C_write(0x73, 0xF2); //f2
	I2C_write(0xA2, 0x02); //02
	// gate PCLK via HREF
	I2C_write(0x15, 0x20); //no PLCK tgl on blank
	*/
//================================================
/*
{REG_COM3, COM3_DCWEN}, // enable downsamp/crop/window

    {REG_COM14, 0x1a},	// divide by 4
    {0x72, 0x22},		// downsample by 4
    {0x73, 0xf2},		// divide by 4
    {REG_HSTART,0x16},
    {REG_HSTOP,0x04},
    {REG_HREF,0xa4},
    {REG_VSTART,0x02},
    {REG_VSTOP,0x7a},
    {REG_VREF,0x0a},
{0xff, 0xff} END MARKER
*//*
{REG_COM7, 0x0},	 Selects YUV mode
    {REG_RGB444, 0},	 No RGB444 please
    {REG_COM1, 0},
    {REG_COM15, COM15_R00FF},
    {REG_COM9, 0x6A},   128x gain ceiling; 0x8 is reserved bit
    {0x4f, 0x80},		 "matrix coefficient 1"
    {0x50, 0x80},		 "matrix coefficient 2"
    {0x51, 0},		 vb
    {0x52, 0x22},		 "matrix coefficient 4"
    {0x53, 0x5e},		 "matrix coefficient 5"
    {0x54, 0x80},		 "matrix coefficient 6"
    {REG_COM13,COM13_GAMMA|COM13_UVSAT},
{0xff, 0xff},
*/
//================================================
//================================================
	//AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
	//AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;

	//EXTI->IMR = EXTI_IMR_MR14 | EXTI_IMR_MR11 | EXTI_IMR_MR10;
	//EXTI->RTSR = EXTI_RTSR_TR14 | EXTI_RTSR_TR11 | EXTI_RTSR_TR10;
//================================================
//================================================
/*
void configureTimer(){
	// TIM4: 72MHz -> 36MHz
	TIM4->PSC = 2;
	TIM4->CNT = 0;
	TIM4->ARR = 1;

	TIM4->CCR4 = 1;
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_0;
	TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}
*/
//================================================
//================================================
	/*
	 * Verbose configuration
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_0;

	GPIOC->CRH |= GPIO_CRH_MODE14_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF14_0;
	GPIOC->BSRR |= GPIO_BSRR_BR14;


	GPIOC->CRH |= GPIO_CRH_MODE15_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF15_0;
	GPIOC->BSRR |= GPIO_BSRR_BS15;

	//UART1

	GPIOA->CRH |= GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA->CRH |= GPIO_CRH_MODE9_1;
	GPIOA->CRH &= ~GPIO_CRH_MODE9_0;


	// default configuration
	//UART0 (BT?)

	//I2C_ (SDIO)

	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~GPIO_CRL_CNF6_0;
	GPIOB->CRL |= GPIO_CRL_MODE6_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE6_0;

	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~GPIO_CRL_CNF7_0;
	GPIOB->CRL |= GPIO_CRL_MODE7_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE7_0;


	//clk_out/PWM

	GPIOB->CRH |= GPIO_CRH_CNF9_1;
	GPIOB->CRH &= ~GPIO_CRH_CNF9_0;
	GPIOB->CRH |= GPIO_CRH_MODE9_1;
	GPIOB->CRH &= ~GPIO_CRH_MODE9_0;
	//Px7..0 (D7..0)

	//GPIO_in: PCLK, VSYNC, HREF


	GPIOB->CRH |= GPIO_CRH_MODE12_1;
	GPIOB->CRH &= ~GPIO_CRH_CNF12_0;
	GPIOB->BSRR |= GPIO_BSRR_BR12;

	GPIOB->CRH |= GPIO_CRH_MODE13_1;
	GPIOB->CRH &= ~GPIO_CRH_CNF13_0;
	GPIOB->BSRR |= GPIO_BSRR_BS13;
	*/
//================================================
//================================================
/*

// Enable Acknowledgment
	I2Cx->CR1 |= I2C_CR1_ACK;
	// Clear POS flag
	I2Cx->CR1 &= ~I2C_CR1_POS; // NACK position current

	// Initiate START sequence
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait for EV5
	if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_EV5) == I2C_ERROR) return I2C_ERROR;

	// Send the slave address (EV5)
	I2Cx->DR = SlaveAddress | I2C_OAR1_ADD0; // Last bit set (receiver mode)

	// Wait for EV6
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_ADDR) == I2C_ERROR) return I2C_ERROR;

	// There are can be three cases:
	//   read 1 byte
	//   read 2 bytes
	//   read more than 2 bytes
	if (nbytes == 1) {
		// Receive 1 byte (AN2824 figure 2)
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable I2C acknowledgment

		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void)I2Cx->SR1; // Clear ADDR bit
		(void)I2Cx->SR2;
		I2Cx->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
		__enable_irq();

		// Wait for RxNE flag (receive buffer not empty) EV7
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_RXNE) == I2C_ERROR) return I2C_ERROR;

		// Read received byte
*buf = (uint8_t)I2Cx->DR;
*/
//================================================
//================================================
	/*
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = 0x42; //0x42; // write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)); //0x0400 -> ACK fail
	tmp = I2C1->SR2;
	//delay();
	I2C1->DR = 0x15; // register to read
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	//delay();
	I2C1->CR1 |= I2C_CR1_STOP;
	*/
//================================================
//================================================
/*
The following example shows how to capture the counter value in TIMx_CCR1 when TI1
input rises. To do this, use the following procedure:
[[[• Program the needed input filter duration with respect to the signal connected to the
timer (by programming the ICxF bits in the TIMx_CCMRx register if the input is one of
the TIx inputs). Let’s imagine that, when toggling, the input signal is not stable during at
must five internal clock cycles. We must program a filter duration longer than these five
clock cycles. We can validate a transition on TI1 when eight consecutive samples with
the new level have been detected (sampled at f DTS frequency). Then write IC1F bits to
0011 in the TIMx_CCMR1 register.]]]
• Select the edge of the active transition on the TI1 channel by writing the CC1P bit to 0
in the TIMx_CCER register (rising edge in this case).
[[[• Program the input prescaler. In our example, we wish the capture to be performed at
each valid transition, so the prescaler is disabled (write IC1PS bits to 00 in the
TIMx_CCMR1 register).]]]

When an input capture occurs:
• The TIMx_CCR1 register gets the value of the counter on the active transition.
• CC1IF flag is set (interrupt flag). CC1OF is also set if at least two consecutive captures
occurred whereas the flag was not cleared.
• An interrupt is generated depending on the CC1IE bit.
• A DMA request is generated depending on the CC1DE bit.
*/
//================================================
//================================================


/*
 * Channel configuration procedure
The following sequence should be followed to configure a DMA channelx (where x is the
channel number).
1. Set the peripheral register address in the DMA_CPARx register. The data will be
moved from/ to this address to/ from the memory after the peripheral event.
2. Set the memory address in the DMA_CMARx register. The data will be written to or
read from this memory after the peripheral event.
3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
After each peripheral event, this value will be decremented.
4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
5. Configure data transfer direction, circular mode, peripheral & memory incremented
mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
DMA_CCRx register
6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
As soon as the channel is enabled, it can serve any DMA request from the periphe
 */
//void sendUART(char);
//================================================
//================================================
/*
	 * OBS
	if(EXTI->PR & EXTI_PR_PR11){ //_VSYNC_
		EXTI->PR |= EXTI_PR_PR11;
		i1++;
		if(send==2){
			EXTI->PR |= EXTI_PR_PR11;
			EXTI->IMR &= ~EXTI_IMR_MR14;
			NVIC_DisableIRQ(EXTI15_10_IRQn);
			NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			send = 3;
			sendUART(0xBB);
			sendUART(0xBB);

		}
		if(send==1){
			sendUART(0xAA);
			sendUART(0xAA);
			send = 2;
			EXTI->IMR |= EXTI_IMR_MR14;
		}
	}
	}
	*/

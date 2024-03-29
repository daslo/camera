/*
 * ov7670.h
 *
 *  Created on: 22.08.2019
 *      Author: ds
 */

#ifndef OV7670_H_
#define OV7670_H_

void I2C_write(char, char);
char I2C_read(char);

// pairs of Registers and Values
char RV_qqvga_yuv[] = {
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
		0x15, 0x20, //gate PCLK via HREF
		0xff, 0xff	//terminate
};
char RV_qvga_yuv[] = {
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
		0x15, 0x20, //gate PCLK via HREF
		0xff, 0xff  //terminate
};
char RV_vga_yuv[] = {
		//VGA, YUV according to IG
		0x11, 0x0A, //prescaler
		0x12, 0x00,
		0x0c, 0x00,
		0x3e, 0x00,
		0x70, 0x3a,
		0x71, 0x35,
		0x72, 0x11,
		0x73, 0xf0,
		0xa2, 0x02,
		0x15, 0x20,	//gate PCLK via HREF
		0xff, 0xff //terminate
};

#endif /* OV7670_H_ */

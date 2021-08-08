/*
 * ov7670.c
 *
 *  Created on: May 18, 2021
 *      Author: Simen
 */


//#include "stm32l4xx_hal.h"
#include "ov7670.h"

extern I2C_HandleTypeDef hi2c2;
extern DCMI_HandleTypeDef hdcmi;


//add do while to check HAL status?
void ov7670_read(uint8_t reg_addr)
{
	uint8_t buffer[4] = {0};
	HAL_I2C_Master_Transmit(&hi2c2, OV7670_WRITE_ADDR, &reg_addr, 1, 100);
	HAL_I2C_Master_Receive(&hi2c2, OV7670_WRITE_ADDR, buffer, 1, 100);
	Serial_com(&buffer[0], 1);
}

void ov7670_write(uint8_t reg_addr, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2, OV7670_WRITE_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT , &data, 1, 100); // I2C_MEMADD_SIZE_8BIT is defined in the HAL library
}

void ov7670_write_Delay(uint8_t reg_addr, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2, OV7670_WRITE_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT , &data, 1, 100); // I2C_MEMADD_SIZE_8BIT is defined in the HAL library
	HAL_Delay(20);
}

uint8_t ov7670_read_return(uint8_t reg_addr)
{
	uint8_t buffer[4] = {0};
	HAL_I2C_Master_Transmit(&hi2c2, OV7670_WRITE_ADDR, &reg_addr, 1, 100);
	HAL_I2C_Master_Receive(&hi2c2, OV7670_WRITE_ADDR, buffer, 1, 100);
	return buffer[0];
}


void ov7670_hard_reset()
{
	HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void ov7670_init()
{
  //ov7670_hard_reset();
  ov7670_write(0x12, 0x80);  // soft reset
  HAL_Delay(500);			 // Needs a delay after reset otherwise all registers remain 0x00 after writing them

  // Configure camera
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1); // it works, but MIT recommend 20 ms - why?
  }
  HAL_Delay(500);
}

void ov7670_read_all_reg()
{
 uint8_t buffer[4] = {0};
 for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
	buffer[0] = OV7670_reg[i][0];
	Serial_com(&buffer[0], 1);
	//HAL_UART_Transmit(&hlpuart1, buffer, 1, 10); // Prints register number
	ov7670_read(OV7670_reg[i][0]);				 // Prints register content stored on OV7670
	HAL_Delay(1000);
  }
}

void ov7670_reg_check()
{
 uint8_t buffer[1] = {00};
 uint8_t counter = 0;
 uint8_t tot_reg_num = 0;
 for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
	 tot_reg_num++;
	if (ov7670_read_return(OV7670_reg[i][0]) == OV7670_reg[i][1])
	{
		counter++;
	}
	HAL_Delay(10);
  }
 if (counter == tot_reg_num)
 {
	 HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
	 buffer[0] = 0xAA;
	 Serial_com(&buffer[0], 1);
	 //HAL_UART_Transmit(&hlpuart1, buffer, 1, 10);
 } else
 {
	 HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
	 buffer[0] = 0xFA;
	 Serial_com(&buffer[0], 1);
	 //HAL_UART_Transmit(&hlpuart1, buffer, 1, 10);
 }
}


void capture_image(void)
{
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frame_buffer, RGB565_QVGA_SIZE);
	//HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) frame_buffer, RGB565_QVGA_SIZE);
}






//int ov7670_init_MIT(char *format, int n)
uint8_t ov7670_init_MIT(uint32_t n) //n=frame buffer size
    {

//        if (ReadReg(REG_PID) != 0x76)           // check id camera
//        {
//            return 0;
//        }
	    ov7670_write(0x12, 0x80);  // soft reset
	    HAL_Delay(500);			 // Needs a delay after reset otherwise all registers remain 0x00 after writing them
//        Reset();                                 // Resets all registers to default values
//        Reset();                                 // Resets all registers to default values

        ov7670_write_Delay(REG_RGB444, 0x00);              // Disable RGB444
        ov7670_write_Delay(REG_COM10, 0x02);               // 0x02   VSYNC negative (http://nasulica.homelinux.org/?p=959)
        ov7670_write_Delay(REG_MVFP, 0x27);                // mirror image

        ov7670_write_Delay(REG_CLKRC, 0x80);               // prescaler x1
        ov7670_write_Delay(DBLV, 0x0a);                    // bypass PLL

        ov7670_write_Delay(REG_COM11, 0x0A) ;
        ov7670_write_Delay(REG_TSLB, 0x04);                // 0D = UYVY  04 = YUYV
        ov7670_write_Delay(REG_COM13, 0x88);               // connect to REG_TSLB


//        if((strcmp("BAW", format) == 0) || (strcmp("YUV", format) == 0)|| (strcmp("RAW", format) == 0))     // YUV
//        {
//            ov7670_write_Delay(REG_COM7, 0x00);           // YUV
//            ov7670_write_Delay(REG_COM17, 0x00);          // color bar disable
//            ov7670_write_Delay(REG_COM3, 0x04);
//            ov7670_write_Delay(REG_COM15, 0xC0);          // Set normal rgb with Full range
//
//        }else
//        if((strcmp("RGB", format) == 0))                // RGB565
//        {
            ov7670_write_Delay(REG_COM7, 0x04);           // RGB + color bar disable
            ov7670_write_Delay(REG_RGB444, 0x00);         // Disable RGB444
            ov7670_write_Delay(REG_COM15, 0x10);          // Set rgb565 with Full range    0xD0
            ov7670_write_Delay(REG_COM3, 0x04);
            ov7670_write_Delay(REG_CLKRC, 0x80);          // prescaler x1
//        }

        ov7670_write_Delay(0x70, 0x3A);                   // Scaling Xsc
        ov7670_write_Delay(0x71, 0x35);                   // Scaling Ysc
        ov7670_write_Delay(0xA2, 0x02);                   // pixel clock delay

        if(n == 19200)              // 160*120
        {
            ov7670_write_Delay(REG_COM14, 0x1a);          // divide by 4
            ov7670_write_Delay(0x72, 0x22);               // downsample by 4
            ov7670_write_Delay(0x73, 0xf2);               // divide by 4
            ov7670_write_Delay(REG_HREF, 0xa4);
            ov7670_write_Delay(REG_HSTART, 0x16);
            ov7670_write_Delay(REG_HSTOP, 0x04);
            ov7670_write_Delay(REG_VREF, 0x0a);
            ov7670_write_Delay(REG_VSTART, 0x02);
            ov7670_write_Delay(REG_VSTOP, 0x7a);

            ov7670_write_Delay(0x7a, 0x20);
            ov7670_write_Delay(0x7b, 0x1c);
            ov7670_write_Delay(0x7c, 0x28);
            ov7670_write_Delay(0x7d, 0x3c);
            ov7670_write_Delay(0x7e, 0x5a);
            ov7670_write_Delay(0x7f, 0x68);
            ov7670_write_Delay(0x80, 0x76);
            ov7670_write_Delay(0x81, 0x80);
            ov7670_write_Delay(0x82, 0x88);
            ov7670_write_Delay(0x83, 0x8f);
            ov7670_write_Delay(0x84, 0x96);
            ov7670_write_Delay(0x85, 0xa3);
            ov7670_write_Delay(0x86, 0xaf);
            ov7670_write_Delay(0x87, 0xc4);
            ov7670_write_Delay(0x88, 0xd7);
            ov7670_write_Delay(0x89, 0xe8);

            ov7670_write_Delay(0x13, 0xe0);
            ov7670_write_Delay(0x00, 0x00);
            ov7670_write_Delay(0x10, 0x00);
            ov7670_write_Delay(0x0d, 0x40);
            ov7670_write_Delay(0x14, 0x18);
            ov7670_write_Delay(0xa5, 0x05);
            ov7670_write_Delay(0xab, 0x07);
            ov7670_write_Delay(0x24, 0x95);
            ov7670_write_Delay(0x25, 0x33);
            ov7670_write_Delay(0x26, 0xe3);
            ov7670_write_Delay(0x9f, 0x78);
            ov7670_write_Delay(0xa0, 0x68);
            ov7670_write_Delay(0xa1, 0x03);
            ov7670_write_Delay(0xa6, 0xd8);
            ov7670_write_Delay(0xa7, 0xd8);
            ov7670_write_Delay(0xa8, 0xf0);
            ov7670_write_Delay(0xa9, 0x90);
            ov7670_write_Delay(0xaa, 0x94);
            ov7670_write_Delay(0x13, 0xe5);

            ov7670_write_Delay(0x0e, 0x61);
            ov7670_write_Delay(0x0f, 0x4b);
            ov7670_write_Delay(0x16, 0x02);

            ov7670_write_Delay(0x21, 0x02);
            ov7670_write_Delay(0x22, 0x91);
            ov7670_write_Delay(0x29, 0x07);
            ov7670_write_Delay(0x33, 0x0b);
            ov7670_write_Delay(0x35, 0x0b);
            ov7670_write_Delay(0x37, 0x1d);
            ov7670_write_Delay(0x38, 0x71);
            ov7670_write_Delay(0x39, 0x2a);
            ov7670_write_Delay(0x3c, 0x78);
            ov7670_write_Delay(0x4d, 0x40);
            ov7670_write_Delay(0x4e, 0x20);
            ov7670_write_Delay(0x69, 0x00);

            ov7670_write_Delay(0x74, 0x10);
            ov7670_write_Delay(0x8d, 0x4f);
            ov7670_write_Delay(0x8e, 0x00);
            ov7670_write_Delay(0x8f, 0x00);
            ov7670_write_Delay(0x90, 0x00);
            ov7670_write_Delay(0x91, 0x00);
            ov7670_write_Delay(0x92, 0x00);

            ov7670_write_Delay(0x96, 0x00);
            ov7670_write_Delay(0x9a, 0x80);
            ov7670_write_Delay(0xb0, 0x84);
            ov7670_write_Delay(0xb1, 0x0c);
            ov7670_write_Delay(0xb2, 0x0e);
            ov7670_write_Delay(0xb3, 0x82);
            ov7670_write_Delay(0xb8, 0x0a);

            ov7670_write_Delay(0x43, 0x0a);
            ov7670_write_Delay(0x44, 0xf0);
            ov7670_write_Delay(0x45, 0x34);
            ov7670_write_Delay(0x46, 0x58);
            ov7670_write_Delay(0x47, 0x28);
            ov7670_write_Delay(0x48, 0x3a);
            ov7670_write_Delay(0x59, 0x88);
            ov7670_write_Delay(0x5a, 0x88);
            ov7670_write_Delay(0x5b, 0x44);
            ov7670_write_Delay(0x5c, 0x67);
            ov7670_write_Delay(0x5d, 0x49);
            ov7670_write_Delay(0x5e, 0x0e);
            ov7670_write_Delay(0x64, 0x04);
            ov7670_write_Delay(0x65, 0x20);
            ov7670_write_Delay(0x66, 0x05);
            ov7670_write_Delay(0x94, 0x04);
            ov7670_write_Delay(0x95, 0x08);

            ov7670_write_Delay(0x6c, 0x0a); //AWB control
            ov7670_write_Delay(0x6d, 0x55); //AWB control
            ov7670_write_Delay(0x6e, 0x11); //AWB control
            ov7670_write_Delay(0x6f, 0x9f); //AWB control
            ov7670_write_Delay(0x6a, 0x40); //GGAIN
            ov7670_write_Delay(0x01, 0x40); // BLUE GAIN (AGC) - default 0x80 - max FF
            ov7670_write_Delay(0x02, 0x40); // RED GAIN (AGC) - default 0x80 - max FF
            ov7670_write_Delay(0x13, 0xe7); //Banding filter - depends on exposure time - se 3.3. automatic exposure time (AEC)
            ov7670_write_Delay(0x15, 0x02);

            ov7670_write_Delay(0x4f, 0x80);
            ov7670_write_Delay(0x50, 0x80);
            ov7670_write_Delay(0x51, 0x00);
            ov7670_write_Delay(0x52, 0x22);
            ov7670_write_Delay(0x53, 0x5e);
            ov7670_write_Delay(0x54, 0x80);
            ov7670_write_Delay(0x58, 0x9e);

            ov7670_write_Delay(0x41, 0x08);
            ov7670_write_Delay(0x3f, 0x00);
            ov7670_write_Delay(0x75, 0x05);
            ov7670_write_Delay(0x76, 0xe1);
            ov7670_write_Delay(0x4c, 0x00);
            ov7670_write_Delay(0x77, 0x01);
            ov7670_write_Delay(0x3d, 0xc1);
            ov7670_write_Delay(0x4b, 0x09);
            ov7670_write_Delay(0xc9, 0x60);
            ov7670_write_Delay(0x41, 0x38);
            ov7670_write_Delay(0x56, 0x40);

            ov7670_write_Delay(0x34, 0x11);
            ov7670_write_Delay(0x3b, 0x02);
            ov7670_write_Delay(0xa4, 0x88);
            ov7670_write_Delay(0x96, 0x00);
            ov7670_write_Delay(0x97, 0x30);
            ov7670_write_Delay(0x98, 0x20);
            ov7670_write_Delay(0x99, 0x30);
            ov7670_write_Delay(0x9a, 0x84);
            ov7670_write_Delay(0x9b, 0x29);
            ov7670_write_Delay(0x9c, 0x03);
            ov7670_write_Delay(0x9d, 0x4c);
            ov7670_write_Delay(0x9e, 0x3f);
            ov7670_write_Delay(0x78, 0x04);

            ov7670_write_Delay(0x79, 0x01);
            ov7670_write_Delay(0xc8, 0xf0);
            ov7670_write_Delay(0x79, 0x0f);
            ov7670_write_Delay(0xc8, 0x00);
            ov7670_write_Delay(0x79, 0x10);
            ov7670_write_Delay(0xc8, 0x7e);
            ov7670_write_Delay(0x79, 0x0a);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x0b);
            ov7670_write_Delay(0xc8, 0x01);
            ov7670_write_Delay(0x79, 0x0c);
            ov7670_write_Delay(0xc8, 0x0f);
            ov7670_write_Delay(0x79, 0x0d);
            ov7670_write_Delay(0xc8, 0x20);
            ov7670_write_Delay(0x79, 0x09);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x02);
            ov7670_write_Delay(0xc8, 0xc0);
            ov7670_write_Delay(0x79, 0x03);
            ov7670_write_Delay(0xc8, 0x40);
            ov7670_write_Delay(0x79, 0x05);
            ov7670_write_Delay(0xc8, 0x30);
            ov7670_write_Delay(0x79, 0x26);
            ov7670_write_Delay(0x09, 0x03);
            ov7670_write_Delay(0x3b, 0x42);

            ov7670_write_Delay(0xff, 0xff);   /* END MARKER */

        }
        if(n == 76800)              // 320*240
        {
            ov7670_write_Delay(REG_COM14, 0x19);
            ov7670_write_Delay(0x72, 0x11);
            ov7670_write_Delay(0x73, 0xf1);
            ov7670_write_Delay(REG_HREF, 0x24);
            ov7670_write_Delay(REG_HSTART, 0x16);
            ov7670_write_Delay(REG_HSTOP, 0x04);
            ov7670_write_Delay(REG_VREF, 0x0a);
            ov7670_write_Delay(REG_VSTART,0x02);
            ov7670_write_Delay(REG_VSTOP, 0x7a);

            ov7670_write_Delay(0x7a, 0x20);
            ov7670_write_Delay(0x7b, 0x1c);
            ov7670_write_Delay(0x7c, 0x28);
            ov7670_write_Delay(0x7d, 0x3c);
            ov7670_write_Delay(0x7e, 0x55);
            ov7670_write_Delay(0x7f, 0x68);
            ov7670_write_Delay(0x80, 0x76);
            ov7670_write_Delay(0x81, 0x80);
            ov7670_write_Delay(0x82, 0x88);
            ov7670_write_Delay(0x83, 0x8f);
            ov7670_write_Delay(0x84, 0x96);
            ov7670_write_Delay(0x85, 0xa3);
            ov7670_write_Delay(0x86, 0xaf);
            ov7670_write_Delay(0x87, 0xc4);
            ov7670_write_Delay(0x88, 0xd7);
            ov7670_write_Delay(0x89, 0xe8);

            ov7670_write_Delay(0x13, 0xe0);
            ov7670_write_Delay(0x00, 0x00);
            ov7670_write_Delay(0x10, 0x00);
            ov7670_write_Delay(0x0d, 0x00);
            ov7670_write_Delay(0x14, 0x28);
            ov7670_write_Delay(0xa5, 0x05);
            ov7670_write_Delay(0xab, 0x07);
            ov7670_write_Delay(0x24, 0x75);
            ov7670_write_Delay(0x25, 0x63);
            ov7670_write_Delay(0x26, 0xA5);
            ov7670_write_Delay(0x9f, 0x78);
            ov7670_write_Delay(0xa0, 0x68);
            ov7670_write_Delay(0xa1, 0x03);
            ov7670_write_Delay(0xa6, 0xdf);
            ov7670_write_Delay(0xa7, 0xdf);
            ov7670_write_Delay(0xa8, 0xf0);
            ov7670_write_Delay(0xa9, 0x90);
            ov7670_write_Delay(0xaa, 0x94);  //AEC Algorithm Selection
            ov7670_write_Delay(0x13, 0xe5);

            ov7670_write_Delay(0x0e, 0x61);
            ov7670_write_Delay(0x0f, 0x4b);
            ov7670_write_Delay(0x16, 0x02);
            ov7670_write_Delay(0x21, 0x02);
            ov7670_write_Delay(0x22, 0x91);
            ov7670_write_Delay(0x29, 0x07);
            ov7670_write_Delay(0x33, 0x0b);
            ov7670_write_Delay(0x35, 0x0b);
            ov7670_write_Delay(0x37, 0x1d);
            ov7670_write_Delay(0x38, 0x71);
            ov7670_write_Delay(0x39, 0x2a);
            ov7670_write_Delay(0x3c, 0x78);
            ov7670_write_Delay(0x4d, 0x40);
            ov7670_write_Delay(0x4e, 0x20);
            ov7670_write_Delay(0x69, 0x00);
            ov7670_write_Delay(0x6b, 0x00);
            ov7670_write_Delay(0x74, 0x19);
            ov7670_write_Delay(0x8d, 0x4f);
            ov7670_write_Delay(0x8e, 0x00);
            ov7670_write_Delay(0x8f, 0x00);
            ov7670_write_Delay(0x90, 0x00);
            ov7670_write_Delay(0x91, 0x00);
            ov7670_write_Delay(0x92, 0x00);
            ov7670_write_Delay(0x96, 0x00);
            ov7670_write_Delay(0x9a, 0x80);
            ov7670_write_Delay(0xb0, 0x84);
            ov7670_write_Delay(0xb1, 0x0c);
            ov7670_write_Delay(0xb2, 0x0e);
            ov7670_write_Delay(0xb3, 0x82);
            ov7670_write_Delay(0xb8, 0x0a);
            ov7670_write_Delay(0x43, 0x14);
            ov7670_write_Delay(0x44, 0xf0);
            ov7670_write_Delay(0x45, 0x34);
            ov7670_write_Delay(0x46, 0x58);
            ov7670_write_Delay(0x47, 0x28);
            ov7670_write_Delay(0x48, 0x3a);
            ov7670_write_Delay(0x59, 0x88);
            ov7670_write_Delay(0x5a, 0x88);
            ov7670_write_Delay(0x5b, 0x44);
            ov7670_write_Delay(0x5c, 0x67);
            ov7670_write_Delay(0x5d, 0x49);
            ov7670_write_Delay(0x5e, 0x0e);
            ov7670_write_Delay(0x64, 0x04);
            ov7670_write_Delay(0x65, 0x20);
            ov7670_write_Delay(0x66, 0x05);
            ov7670_write_Delay(0x94, 0x04);
            ov7670_write_Delay(0x95, 0x08);

            ov7670_write_Delay(0x6c, 0x0a); //AWB control
			ov7670_write_Delay(0x6d, 0x55); //AWB control
			ov7670_write_Delay(0x6e, 0x11); //AWB control
			ov7670_write_Delay(0x6f, 0x9f); //AWB control
			ov7670_write_Delay(0x6a, 0x40); //GGAIN
			ov7670_write_Delay(0x01, 0x40); // BLUE GAIN (AGC) - default 0x80 - max FF
			ov7670_write_Delay(0x02, 0x40); // RED GAIN (AGC) - default 0x80 - max FF
			//0x13 is important - [2] on/off AGC automatic gain - [1] on/off AWB automatic white balance
			ov7670_write_Delay(0x13, 0xe7); //Banding filter - depends on exposure time - se 3.3. automatic exposure time (AEC)
            //0xef value - all is enabled except banding
			//ov7670_write_Delay(0x15, 0x02); // VSYNC negative

            ov7670_write_Delay(0x4f, 0x80);
            ov7670_write_Delay(0x50, 0x80);
            ov7670_write_Delay(0x51, 0x00);
            ov7670_write_Delay(0x52, 0x22);
            ov7670_write_Delay(0x53, 0x5e);
            ov7670_write_Delay(0x54, 0x80);
            ov7670_write_Delay(0x58, 0x9e);
            ov7670_write_Delay(0x41, 0x08);
            ov7670_write_Delay(0x3f, 0x00);
            ov7670_write_Delay(0x75, 0x05);
            ov7670_write_Delay(0x76, 0xe1);
            ov7670_write_Delay(0x4c, 0x00);
            ov7670_write_Delay(0x77, 0x01);
            ov7670_write_Delay(0x3d, 0xc2);
            ov7670_write_Delay(0x4b, 0x09);
            ov7670_write_Delay(0xc9, 0x60);
            ov7670_write_Delay(0x41, 0x38);
            ov7670_write_Delay(0x56, 0x40);
            ov7670_write_Delay(0x34, 0x11);
            ov7670_write_Delay(0x3b, 0x02);
            ov7670_write_Delay(0xa4, 0x89);
            ov7670_write_Delay(0x96, 0x00);
            ov7670_write_Delay(0x97, 0x30);
            ov7670_write_Delay(0x98, 0x20);
            ov7670_write_Delay(0x99, 0x30);
            ov7670_write_Delay(0x9a, 0x84);
            ov7670_write_Delay(0x9b, 0x29);
            ov7670_write_Delay(0x9c, 0x03);
            ov7670_write_Delay(0x9d, 0x4c);
            ov7670_write_Delay(0x9e, 0x3f);
            ov7670_write_Delay(0x78, 0x04);
            ov7670_write_Delay(0x79, 0x01);
            ov7670_write_Delay(0xc8, 0xf0);
            ov7670_write_Delay(0x79, 0x0f);
            ov7670_write_Delay(0xc8, 0x00);
            ov7670_write_Delay(0x79, 0x10);
            ov7670_write_Delay(0xc8, 0x7e);
            ov7670_write_Delay(0x79, 0x0a);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x0b);
            ov7670_write_Delay(0xc8, 0x01);
            ov7670_write_Delay(0x79, 0x0c);
            ov7670_write_Delay(0xc8, 0x0f);
            ov7670_write_Delay(0x79, 0x0d);
            ov7670_write_Delay(0xc8, 0x20);
            ov7670_write_Delay(0x79, 0x09);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x02);
            ov7670_write_Delay(0xc8, 0xc0);
            ov7670_write_Delay(0x79, 0x03);
            ov7670_write_Delay(0xc8, 0x40);
            ov7670_write_Delay(0x79, 0x05);
            ov7670_write_Delay(0xc8, 0x30);
            ov7670_write_Delay(0x79, 0x26);
            ov7670_write_Delay(0x09, 0x03);
            ov7670_write_Delay(0x3b, 0x42);

            ov7670_write_Delay(0xff, 0xff);   /* END MARKER */

        }
        if(n == 307200)             // 640*480
        {
            ov7670_write_Delay(REG_CLKRC, 0x01);
            ov7670_write_Delay(REG_TSLB,  0x04);
            ov7670_write_Delay(REG_COM7, 0x01);
            ov7670_write_Delay(DBLV, 0x4a);
            ov7670_write_Delay(REG_COM3, 0);
            ov7670_write_Delay(REG_COM14, 0);

            ov7670_write_Delay(REG_HSTART, 0x13);
            ov7670_write_Delay(REG_HSTOP, 0x01);
            ov7670_write_Delay(REG_HREF, 0xb6);
            ov7670_write_Delay(REG_VSTART, 0x02);
            ov7670_write_Delay(REG_VSTOP, 0x7a);
            ov7670_write_Delay(REG_VREF, 0x0a);
            ov7670_write_Delay(0x72, 0x11);
            ov7670_write_Delay(0x73, 0xf0);

            /* Gamma curve values */
            ov7670_write_Delay(0x7a, 0x20);
            ov7670_write_Delay(0x7b, 0x10);
            ov7670_write_Delay(0x7c, 0x1e);
            ov7670_write_Delay(0x7d, 0x35);
            ov7670_write_Delay(0x7e, 0x5a);
            ov7670_write_Delay(0x7f, 0x69);
            ov7670_write_Delay(0x80, 0x76);
            ov7670_write_Delay(0x81, 0x80);
            ov7670_write_Delay(0x82, 0x88);
            ov7670_write_Delay(0x83, 0x8f);
            ov7670_write_Delay(0x84, 0x96);
            ov7670_write_Delay(0x85, 0xa3);
            ov7670_write_Delay(0x86, 0xaf);
            ov7670_write_Delay(0x87, 0xc4);
            ov7670_write_Delay(0x88, 0xd7);
            ov7670_write_Delay(0x89, 0xe8);

            /* AGC and AEC parameters.  Note we start by disabling those features,
            then turn them only after tweaking the values. */
            ov7670_write_Delay(0x13, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT);
            ov7670_write_Delay(0x00, 0);
            ov7670_write_Delay(0x10, 0);
            ov7670_write_Delay(0x0d, 0x40);
            ov7670_write_Delay(0x14, 0x18);
            ov7670_write_Delay(0xa5, 0x05);
            ov7670_write_Delay(0xab, 0x07);
            ov7670_write_Delay(0x24, 0x95);
            ov7670_write_Delay(0x25, 0x33);
            ov7670_write_Delay(0x26, 0xe3);
            ov7670_write_Delay(0x9f, 0x78);
            ov7670_write_Delay(0xa0, 0x68);
            ov7670_write_Delay(0xa1, 0x03);
            ov7670_write_Delay(0xa6, 0xd8);
            ov7670_write_Delay(0xa7, 0xd8);
            ov7670_write_Delay(0xa8, 0xf0);
            ov7670_write_Delay(0xa9, 0x90);
            ov7670_write_Delay(0xaa, 0x94);
            ov7670_write_Delay(0x13, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC);

            /* Almost all of these are magic "reserved" values.  */
            ov7670_write_Delay(0x0e, 0x61);
            ov7670_write_Delay(0x0f, 0x4b);
            ov7670_write_Delay(0x16, 0x02);
            ov7670_write_Delay(0x1e, 0x27);
            ov7670_write_Delay(0x21, 0x02);
            ov7670_write_Delay(0x22, 0x91);
            ov7670_write_Delay(0x29, 0x07);
            ov7670_write_Delay(0x33, 0x0b);
            ov7670_write_Delay(0x35, 0x0b);
            ov7670_write_Delay(0x37, 0x1d);
            ov7670_write_Delay(0x38, 0x71);
            ov7670_write_Delay(0x39, 0x2a);
            ov7670_write_Delay(0x3c, 0x78);
            ov7670_write_Delay(0x4d, 0x40);
            ov7670_write_Delay(0x4e, 0x20);
            ov7670_write_Delay(0x69, 0);
            ov7670_write_Delay(0x6b, 0x0a);
            ov7670_write_Delay(0x74, 0x10);
            ov7670_write_Delay(0x8d, 0x4f);
            ov7670_write_Delay(0x8e, 0);
            ov7670_write_Delay(0x8f, 0);
            ov7670_write_Delay(0x90, 0);
            ov7670_write_Delay(0x91, 0);
            ov7670_write_Delay(0x96, 0);
            ov7670_write_Delay(0x9a, 0);
            ov7670_write_Delay(0xb0, 0x84);
            ov7670_write_Delay(0xb1, 0x0c);
            ov7670_write_Delay(0xb2, 0x0e);
            ov7670_write_Delay(0xb3, 0x82);
            ov7670_write_Delay(0xb8, 0x0a);

            /* More reserved magic, some of which tweaks white balance */
            ov7670_write_Delay(0x43, 0x0a);
            ov7670_write_Delay(0x44, 0xf0);
            ov7670_write_Delay(0x45, 0x34);
            ov7670_write_Delay(0x46, 0x58);
            ov7670_write_Delay(0x47, 0x28);
            ov7670_write_Delay(0x48, 0x3a);
            ov7670_write_Delay(0x59, 0x88);
            ov7670_write_Delay(0x5a, 0x88);
            ov7670_write_Delay(0x5b, 0x44);
            ov7670_write_Delay(0x5c, 0x67);
            ov7670_write_Delay(0x5d, 0x49);
            ov7670_write_Delay(0x5e, 0x0e);
            ov7670_write_Delay(0x6c, 0x0a);
            ov7670_write_Delay(0x6d, 0x55);
            ov7670_write_Delay(0x6e, 0x11);
            ov7670_write_Delay(0x6f, 0x9f);
            ov7670_write_Delay(0x6a, 0x40);
            ov7670_write_Delay(0x01, 0x40);
            ov7670_write_Delay(0x02, 0x60);
            ov7670_write_Delay(0x13, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB);

            /* Matrix coefficients */
            ov7670_write_Delay(0x4f, 0x80);
            ov7670_write_Delay(0x50, 0x80);
            ov7670_write_Delay(0x51, 0);
            ov7670_write_Delay(0x52, 0x22);
            ov7670_write_Delay(0x53, 0x5e);
            ov7670_write_Delay(0x54, 0x80);
            ov7670_write_Delay(0x58, 0x9e);

            ov7670_write_Delay(0x41, 0x08);
            ov7670_write_Delay(0x3f, 0);
            ov7670_write_Delay(0x75, 0x05);
            ov7670_write_Delay(0x76, 0xe1);
            ov7670_write_Delay(0x4c, 0);
            ov7670_write_Delay(0x77, 0x01);
            ov7670_write_Delay(0x3d, 0xc3);
            ov7670_write_Delay(0x4b, 0x09);
            ov7670_write_Delay(0xc9, 0x60);
            ov7670_write_Delay(0x41, 0x38);
            ov7670_write_Delay(0x56, 0x40);

            ov7670_write_Delay(0x34, 0x11);
            ov7670_write_Delay(0x3b, COM11_EXP|COM11_HZAUTO);
            ov7670_write_Delay(0xa4, 0x88);
            ov7670_write_Delay(0x96, 0);
            ov7670_write_Delay(0x97, 0x30);
            ov7670_write_Delay(0x98, 0x20);
            ov7670_write_Delay(0x99, 0x30);
            ov7670_write_Delay(0x9a, 0x84);
            ov7670_write_Delay(0x9b, 0x29);
            ov7670_write_Delay(0x9c, 0x03);
            ov7670_write_Delay(0x9d, 0x4c);
            ov7670_write_Delay(0x9e, 0x3f);
            ov7670_write_Delay(0x78, 0x04);

            /* Extra-weird stuff.  Some sort of multiplexor register */
            ov7670_write_Delay(0x79, 0x01);
            ov7670_write_Delay(0xc8, 0xf0);
            ov7670_write_Delay(0x79, 0x0f);
            ov7670_write_Delay(0xc8, 0x00);
            ov7670_write_Delay(0x79, 0x10);
            ov7670_write_Delay(0xc8, 0x7e);
            ov7670_write_Delay(0x79, 0x0a);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x0b);
            ov7670_write_Delay(0xc8, 0x01);
            ov7670_write_Delay(0x79, 0x0c);
            ov7670_write_Delay(0xc8, 0x0f);
            ov7670_write_Delay(0x79, 0x0d);
            ov7670_write_Delay(0xc8, 0x20);
            ov7670_write_Delay(0x79, 0x09);
            ov7670_write_Delay(0xc8, 0x80);
            ov7670_write_Delay(0x79, 0x02);
            ov7670_write_Delay(0xc8, 0xc0);
            ov7670_write_Delay(0x79, 0x03);
            ov7670_write_Delay(0xc8, 0x40);
            ov7670_write_Delay(0x79, 0x05);
            ov7670_write_Delay(0xc8, 0x30);
            ov7670_write_Delay(0x79, 0x26);

            ov7670_write_Delay(0xff, 0xff); /* END MARKER */
        }

        return 1;
    }

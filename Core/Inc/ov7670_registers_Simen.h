/*
 * ov7670_registers_Simen.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Simen
 */

#ifndef INC_OV7670_REGISTERS_SIMEN_H_
#define INC_OV7670_REGISTERS_SIMEN_H_

#define REG_BATT 0xFF

static const uint8_t OV7670_reg[][2] = {

//		/* Color mode related */
//		{0x12, 0x14},   // QVGA, RGB
//		{0x8C, 0x00},   // RGB444 Disable
//		{0x40, 0x10 + 0xc0},   // RGB565, 00 - FF - fullscale is 0xD0 (0x10+0xc0) - MIT use 0x10
//		//{0x40, 0x90}, //limits range to 01-FE
//		//{0x3A, 0x04 + 8},   // UYVY (why?) - Not my comment
//		{0x3A, 0x0c}, //Same as 0x04+8 - 11: V Y U Y - greenish
//		/*From Application Note - Simen*/
//		//{0x3a, 0x04},
//		//   {0x3A, 0x14},
//		//   {0x67, 0xc8}, //fixed UV values instead of chip output
//		//   {0x68, 0x80}, //fixed UV values instead of chip output
//
//
//		//{0x3A, 0x2c}, //this gives a black image - not inverted?
//		//{0x3A, 0x08},   //10: U Y V Y - greenish
//		//{0x3A, 0x04},   //01: Y V Y U - inverted
//		//{0x3A, 0x00}, //00: Y U Y V - don't work
//		//{0x3A, 0x1c},
//		//{0x3A, 0x14},
//		{0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
//		{0xB0, 0x84}, // important
//
//
//
//		/* clock related */
//		{0x0C, 0x04},  // DCW enable
//		{0x3E, 0x19},  // manual scaling, pclk/=2
//		{0x70, 0x3A},  // scaling_xsc
//		{0x71, 0x35},  // scaling_ysc
//		{0x72, 0x11}, // down sample by 2
//		{0x73, 0xf1}, // DSP clock /= 2
//
//		/* windowing (empirically decided...) */
//		{0x17, 0x16},   // HSTART
//		{0x18, 0x04},   // HSTOP
//		{0x32, 0x80},   // HREF
//		{0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
//		{0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
//		{0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

		//--------------------- MIT

		/*default setup*/
		//{0x8C, 0x00},   // same - Disable RGB444
		//{0x15, 0x02},   // 0x02   VSYNC negative (http://nasulica.homelinux.org/?p=959)
		//{0x1e, 0x27},   // mirror image

		//{0x11, 0x80},   // prescaler x1
		//{0x6b, 0x0a},   // bypass PLL

		//{0x3b, 0x0A},
		//{0x3a, 0x04},   // 0D = UYVY  04 = YUYV
		//{0x3d, 0x88},   // NEQ - connect to REG_TSLB

		/*RGB565*/
		//{0x12, 0x04},   // NEQ - RGB + color bar disable
		{0x12, 0x14}, //_SIMEN
		{0x8c, 0x00},   // S - Disable RGB444
		{0x40, 0x10},   // NEQ Set rgb565 with Full range    0xD0
		{0x3a, 0x04},   // 0D = UYVY  04 = YUYV
		//{0x3A, 0x0c}, //SIMEN

		/*Clock related*/
		{0x0C, 0x04},  // DCW enable
		{0x3E, 0x19},  // manual scaling, pclk/=2
		{0x70, 0x3A},  // scaling_xsc
		{0x71, 0x35},  // scaling_ysc
		{0x72, 0x11}, // down sample by 2
		{0x73, 0xf1}, // DSP clock /= 2

		//{0x0c, 0x04},	// S - DCW enable
//		{0x11, 0x80},   // prescaler x1

		//{0x70, 0x3A},   // S - Scaling Xsc
		//{0x71, 0x35},   // S - Scaling Ysc
//		{0xA2, 0x02},   // M - pixel clock delay

		//{0x3E, 0x19}, // S - manual scaling, pclk/=2 	- same
		//{0x72, 0x11}, // S - down sample by 2 			- same
		//{0x73, 0xf1}, // S - DSP clock /= 2 			- same

		/*Windowing*/
		{0x17, 0x16},   // HSTART
		{0x18, 0x04},   // HSTOP
		{0x32, 0x80},   // HREF
		{0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
		{0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
		{0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

//		{0x32, 0x24}, //HREF   - differerent
		//{0x17, 0x16}, //HSTART - same
		//{0x18, 0x04}, //HSTOP  - same
		//{0x03, 0x0a}, //VREF   - same
		//{0x19, 0x03},   // - SIMEN: VSTART =  14 ( = 3 * 4 + 2)
		//{0x1a, 0x7b},   // - SIMEN: VSTOP  = 494 ( = 123 * 4 + 2)
//		{0x19, 0x02}, //VSTART - different
//		{0x1a, 0x7a}, //VSTOP  - different

		/* gamma curve */
		{0x7a, 0x20},
		{0x7b, 0x1c},
		{0x7c, 0x28},
		{0x7d, 0x3c},
		{0x7e, 0x55},
		{0x7f, 0x68},
		{0x80, 0x76},
		{0x81, 0x80},
		{0x82, 0x88},
		{0x83, 0x8f},
		{0x84, 0x96},
		{0x85, 0xa3},
		{0x86, 0xaf},
		{0x87, 0xc4},
		{0x88, 0xd7},
		{0x89, 0xe8},

		{0x13, 0xe0},
		{0x00, 0x00},
		{0x10, 0x00},
		{0x0d, 0x00},
		{0x14, 0x28},
		{0xa5, 0x05},
		{0xab, 0x07},
		{0x24, 0x75},
		{0x25, 0x63},
		{0x26, 0xA5},
		{0x9f, 0x78},
		{0xa0, 0x68},
		{0xa1, 0x03},
		{0xa6, 0xdf},
		{0xa7, 0xdf},
		{0xa8, 0xf0},
		{0xa9, 0x90},
		{0xaa, 0x94},  //AEC Algorithm Selection
		{0x13, 0xe5},

		{0x0e, 0x61},
		{0x0f, 0x4b},
		{0x16, 0x02},
		{0x21, 0x02},
		{0x22, 0x91},
		{0x29, 0x07},
		{0x33, 0x0b},
		{0x35, 0x0b},
		{0x37, 0x1d},
		{0x38, 0x71},
		{0x39, 0x2a},
		{0x3c, 0x78},
		{0x4d, 0x40},
		{0x4e, 0x20},
		{0x69, 0x00},
		{0x6b, 0x00},
		{0x74, 0x19},
		{0x8d, 0x4f},
		{0x8e, 0x00},
		{0x8f, 0x00},
		{0x90, 0x00},
		{0x91, 0x00},
		{0x92, 0x00},
		{0x96, 0x00},
		{0x9a, 0x80},
		{0xb0, 0x84},
		{0xb1, 0x0c},
		{0xb2, 0x0e},
		{0xb3, 0x82},
		{0xb8, 0x0a},
		{0x43, 0x14},
		{0x44, 0xf0},
		{0x45, 0x34},
		{0x46, 0x58},
		{0x47, 0x28},
		{0x48, 0x3a},
		{0x59, 0x88},
		{0x5a, 0x88},
		{0x5b, 0x44},
		{0x5c, 0x67},
		{0x5d, 0x49},
		{0x5e, 0x0e},
		{0x64, 0x04},
		{0x65, 0x20},
		{0x66, 0x05},
		{0x94, 0x04},
		{0x95, 0x08},

		{0x6c, 0x0a}, //AWB control
		{0x6d, 0x55}, //AWB control
		{0x6e, 0x11}, //AWB control
		{0x6f, 0x9f}, //AWB control
		{0x6a, 0x40}, //GGAIN
		{0x01, 0x40}, // BLUE GAIN (AGC) - default 0x80 - max FF
		{0x02, 0x40}, // RED GAIN (AGC) - default 0x80 - max FF
		//0x13 is important - [2] on/off AGC automatic gain - [1] on/off AWB automatic white balance
		{0x13, 0xe7}, //Banding filter - depends on exposure time - se 3.3. automatic exposure time (AEC)
		//0xef value - all is enabled except banding
		//{0x15, 0x02}, // VSYNC negative

		/*Color matrix*/
		{0x4f, 0x80},
		{0x50, 0x80},
		{0x51, 0x00},
		{0x52, 0x22},
		{0x53, 0x5e},
		{0x54, 0x80},
		{0x58, 0x9e},

		{0x41, 0x08},
		{0x3f, 0x00},
		{0x75, 0x05},
		{0x76, 0xe1},
		{0x4c, 0x00},
		{0x77, 0x01},
		{0x3d, 0xc2},
		{0x4b, 0x09},
		{0xc9, 0x60},
		{0x41, 0x38},
		{0x56, 0x40},
		{0x34, 0x11},
		{0x3b, 0x02},
		{0xa4, 0x89},
		{0x96, 0x00},
		{0x97, 0x30},
		{0x98, 0x20},
		{0x99, 0x30},
		{0x9a, 0x84},
		{0x9b, 0x29},
		{0x9c, 0x03},
		{0x9d, 0x4c},
		{0x9e, 0x3f},
		{0x78, 0x04},
		{0x79, 0x01},
		{0xc8, 0xf0},
		{0x79, 0x0f},
		{0xc8, 0x00},
		{0x79, 0x10},
		{0xc8, 0x7e},
		{0x79, 0x0a},
		{0xc8, 0x80},
		{0x79, 0x0b},
		{0xc8, 0x01},
		{0x79, 0x0c},
		{0xc8, 0x0f},
		{0x79, 0x0d},
		{0xc8, 0x20},
		{0x79, 0x09},
		{0xc8, 0x80},
		{0x79, 0x02},
		{0xc8, 0xc0},
		{0x79, 0x03},
		{0xc8, 0x40},
		{0x79, 0x05},
		{0xc8, 0x30},
		{0x79, 0x26},
		{0x09, 0x03},
		{0x3b, 0x42},

		{REG_BATT, REG_BATT},   /* END MARKER */


};

#endif /* INC_OV7670_REGISTERS_SIMEN_H_ */

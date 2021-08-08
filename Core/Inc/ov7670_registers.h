/*
 * ov7670_registers.h
 *
 *  Created on: May 18, 2021
 *      Author: Simen
 */

#ifndef INC_OV7670_REGISTERS_H_
#define INC_OV7670_REGISTERS_H_

#define REG_BATT 0xFF

static const uint8_t OV7670_reg[][2] = {
  /* Color mode related */
  {0x12, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {0x40, 0x10 + 0xc0},   // RGB565, 00 - FF - fullscale is 0xD0 (0x10+0xc0) - MIT use 0x10
  //{0x40, 0x90}, //limits range to 01-FE
  //{0x3A, 0x04 + 8},   // UYVY (why?) - Not my comment
  {0x3A, 0x0c}, //Same as 0x04+8 - 11: V Y U Y - greenish
  /*From Application Note - Simen*/
   //{0x3a, 0x04},
//   {0x3A, 0x14},
//   {0x67, 0xc8}, //fixed UV values instead of chip output
//   {0x68, 0x80}, //fixed UV values instead of chip output


  //{0x3A, 0x2c}, //this gives a black image - not inverted?
  //{0x3A, 0x08},   //10: U Y V Y - greenish
  //{0x3A, 0x04},   //01: Y V Y U - inverted
  //{0x3A, 0x00}, //00: Y U Y V - don't work
  //{0x3A, 0x1c},
   //{0x3A, 0x14},
  {0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // important



  /* clock related */
  {0x0C, 0x04},  // DCW enable
  {0x3E, 0x19},  // manual scaling, pclk/=2
  {0x70, 0x3A},  // scaling_xsc
  {0x71, 0x35},  // scaling_ysc
  {0x72, 0x11}, // down sample by 2
  {0x73, 0xf1}, // DSP clock /= 2

  /* windowing (empirically decided...) */
  {0x17, 0x16},   // HSTART
  {0x18, 0x04},   // HSTOP
  {0x32, 0x80},   // HREF
  {0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  /* color matrix coefficient */
////#if 0
//  {0x4f, 0xb3},
//  {0x50, 0xb3},
//  {0x51, 0x00},
//  {0x52, 0x3d},
//  {0x53, 0xa7},
//  {0x54, 0xe4},
//  {0x58, 0x9e},
//#else - This is same as MIT
  {0x4f, 0x80},
  {0x50, 0x80},
  {0x51, 0x00},
  {0x52, 0x22},
  {0x53, 0x5e},
  {0x54, 0x80},
  {0x58, 0x9e},
//#endif
  //MY VERSION
//    {0x4f, 0x80},
//    {0x50, 0x80},
//    {0x51, 0x00},
//    {0x52, 0x22},
//    {0x53, 0x5e},
//    {0x54, 0x80},
//    {0x58, 0x9e},
//ov7670_write_Delay(0x4f, 0x80);
//ov7670_write_Delay(0x50, 0x80);
//ov7670_write_Delay(0x51, 0x00);
//ov7670_write_Delay(0x52, 0x22);
//ov7670_write_Delay(0x53, 0x5e);
//ov7670_write_Delay(0x54, 0x80);
//ov7670_write_Delay(0x58, 0x9e);

  //This type of GAIN gives alot of noice.
  /* 3a */
  //{0x13, 0x84}, // default turns on AWB
//  {0x14, 0x0a},   // AGC Ceiling = 2x /default 4x       - IF I ENABLE THIS NO IMAGE DATA AT ALL
//  {0x5F, 0x2f},   // AWB B Gain Range (empirically decided)
//                  // without this bright scene becomes yellow (purple). might be because of color matrix
//  {0x60, 0x98},   // AWB R Gain Range (empirically decided)
//  {0x61, 0x70},   // AWB G Gain Range (empirically decided)
  {0x41, 0x38},   // edge enhancement, de-noise, AWG gain enabled - REMOVING THIS - NO MORE INVERTED COLOR - MUCH MORE STABLE

  //FROM MIT
//{0x6c, 0x0a}, //AWB control
//{0x6d, 0x55}, //AWB control
//{0x6e, 0x11}, //AWB control
//{0x6f, 0x9f}, //AWB control
//{0x6a, 0x40}, //GGAIN
//{0x01, 0x40}, // BLUE GAIN (AGC) - default 0x80 - max FF
//{0x02, 0x40}, // RED GAIN (AGC) - default 0x80 - max FF
////0x13 is important - [2] on/off AGC automatic gain - [1] on/off AWB automatic white balance
//{0x13, 0xe7}, //Banding filter - depends on exposure time - se 3.3. automatic exposure time (AEC)
// //0xef value - all is enabled except banding
////ov7670_write_Delay(0x15, 0x02); // VSYNC negative


  /* gamma curve */
//#if 1 - works good with adjusting gain on reg 0x69
  {0x7b, 16},
  {0x7c, 30},
  {0x7d, 53},
  {0x7e, 90},
  {0x7f, 105},
  {0x80, 118},
  {0x81, 130},
  {0x82, 140},
  {0x83, 150},
  {0x84, 160},
  {0x85, 180},
  {0x86, 195},
  {0x87, 215},
  {0x88, 230},
  {0x89, 244},
  {0x7a, 16},
//#else
  /* gamma = 1 */ // THIS IS VERY GREEN
//  {0x7b, 4},
//  {0x7c, 8},
//  {0x7d, 16},
//  {0x7e, 32},
//  {0x7f, 40},
//  {0x80, 48},
//  {0x81, 56},
//  {0x82, 64},
//  {0x83, 72},
//  {0x84, 80},
//  {0x85, 96},
//  {0x86, 112},
//  {0x87, 144},
//  {0x88, 176},
//  {0x89, 208},
//  {0x7a, 64},
//#endif

  //MY VERSION - MIT
//	{0x7a, 0x20},
//	{0x7b, 0x1c},
//	{0x7c, 0x28},
//	{0x7d, 0x3c},
//	{0x7e, 0x55},
//	{0x7f, 0x68},
//	{0x80, 0x76},
//	{0x81, 0x80},
//	{0x82, 0x88},
//	{0x83, 0x8f},
//	{0x84, 0x96},
//	{0x85, 0xa3},
//	{0x86, 0xaf},
//	{0x87, 0xc4},
//	{0x88, 0xd7},
//	{0x89, 0xe8},

//	ov7670_write_Delay(0x7a, 0x20);
//	ov7670_write_Delay(0x7b, 0x1c);
//	ov7670_write_Delay(0x7c, 0x28);
//	ov7670_write_Delay(0x7d, 0x3c);
//	ov7670_write_Delay(0x7e, 0x55);
//	ov7670_write_Delay(0x7f, 0x68);
//	ov7670_write_Delay(0x80, 0x76);
//	ov7670_write_Delay(0x81, 0x80);
//	ov7670_write_Delay(0x82, 0x88);
//	ov7670_write_Delay(0x83, 0x8f);
//	ov7670_write_Delay(0x84, 0x96);
//	ov7670_write_Delay(0x85, 0xa3);
//	ov7670_write_Delay(0x86, 0xaf);
//	ov7670_write_Delay(0x87, 0xc4);
//	ov7670_write_Delay(0x88, 0xd7);
//	ov7670_write_Delay(0x89, 0xe8);


  //FIX GAIN
  //{0x69,0x06}, //Bx1.5, Rx1.25, Gx1
  //{0x69,0x07},	 //Bx1.75, Rx1.25, Gx1
  //{0x69,0x0B},	 //Bx1.75, Rx1.5, Gx1


  /* fps */
//  {0x6B, 0x4a}, //PLL  x4
    {0x11, 0x00}, // pre-scalar = 1/1   (night mode: 0x11, 0x03)
//  {0x11, 0x03},
//  {0x3b, 0x0a},
  /* others */
  //{0x1E, 0x31}, //mirror flip
  {0x42, 0x08}, // test bar

  {REG_BATT, REG_BATT},
};



#endif /* INC_OV7670_REGISTERS_H_ */

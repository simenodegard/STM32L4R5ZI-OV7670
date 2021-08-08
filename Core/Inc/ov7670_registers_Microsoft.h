/*
 * ov7670_registers_Microsoft.h
 *
 *  Created on: Jun 9, 2021
 *      Author: Simen
 */

#ifndef INC_OV7670_REGISTERS_MICROSOFT_H_
#define INC_OV7670_REGISTERS_MICROSOFT_H_

#define REG_BATT 0xFF

/*
 Register values selectively picked from https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/media/i2c/ov7670.c
*/

static const uint8_t OV7670_reg[][2] = {
  /* Color Mode Configuration */
  {0x12, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {0x40, 0x10 + 0xC0},   // RGB565, 00 - FF
  {0x3A, 0x0C}, // UYVY (why?) (Default)
  {0x3D, 0xC0}, // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // important

  /* Clock Configuration */
  {0x0C, 0x04},  // DCW enable
  {0x3E, 0x19},  // manual scaling, PCLK /= 2
  {0x70, 0x3A},  // scaling_xsc
  {0x71, 0x35},  // scaling_ysc
  {0x72, 0x11},  // down sample by 2
  {0x73, 0xF1},  // DSP clock /= 2

  /* Windowing (Empirically Decided) */
  {0x17, 0x16},  // HSTART
  {0x18, 0x04},  // HSTOP
  {0x32, 0x80},  // HREF (Default)
  {0x19, 0x03},  // VSTART = 14 (= 3 * 4 + 2) (Default)
  {0x1A, 0x7B},  // VSTOP = 494 (= 123 * 4 + 2) (Default)
  {0x03, 0x0A},  // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  /* Color Matrix Coefficients */
  {0x4F, 0x80},
  {0x50, 0x80},
  {0x51, 0x00},
  {0x52, 0x22},
  {0x53, 0x5E},
  {0x54, 0x80},
  {0x58, 0x9E},

  /* Auto Edge Enhancement, Auto Denoising, AWG Gain Enable */
  {0x41, 0x38},
  /* Brightness */
  //{0x55, 0xC0},  // User-defined
  /* Contrast */
  //{0x56, 0x40}, // User-defined

  /* AGC and AEC parameters.  Note we start by disabling those features,
     then turn them only after tweaking the values. */
  {0x13, 0xE0},
  {0xA5, 0x05},
  {0xAB, 0x07},
  {0x24, 0x95},
  {0x25, 0x33},
  {0x26, 0xE3},
  {0x9F, 0x78},
  {0xA0, 0x68},
  {0xA1, 0x03}, /* Magic */
  {0xA6, 0xD8},
  {0xA7, 0xD8},
  {0xA8, 0xF0},
  {0xA9, 0x90},
  {0xAA, 0x94},
  {0x13, 0xE5},

  /* White Balance Magic Numbers*/
  {0x43, 0x0A},
  {0x44, 0xF0},
  {0x45, 0x34},
  {0x46, 0x58},
  {0x47, 0x28},
  {0x48, 0x3A},
  {0x59, 0x88},
  {0x5A, 0x88},
  {0x5B, 0x44},
  {0x5C, 0x67},
  {0x5D, 0x49},
  {0x5E, 0x0E},
  {0x6A, 0x40},
  {0x6C, 0x0A},
  {0x6D, 0x55},
  {0x6E, 0x11},
  {0x6F, 0x9F}, /* "9e for advance AWB" */
  {0x01, 0x40},
  {0x02, 0x60},
  {0x13, 0xE7},

  /* Additional Magic Values */
  {0x0E, 0x61},
  {0x0F, 0x4B},
  {0x16, 0x02},
  {0x21, 0x02},
  {0x22, 0x91},
  {0x29, 0x07},
  {0x33, 0x0B},
  {0x35, 0x0B},
  {0x37, 0x1D},
  {0x38, 0x71},
  {0x39, 0x2A},
  {0x3C, 0x78},
  {0x4D, 0x40},
  {0x4E, 0x20},
  {0x69, 0x00},
  {0x6B, 0x4A},
  {0x74, 0x10},
  {0x8D, 0x4F},
  {0x8E, 0x00},
  {0x8F, 0x00},
  {0x90, 0x00},
  {0x91, 0x00},
  {0x96, 0x00},
  {0x9A, 0x00},
  {0xB1, 0x0C},
  {0xB2, 0x0E},
  {0xB3, 0x82},
  {0xB8, 0x0A},

  {0x77, 0x01},
  {0xC9, 0x60},
  {0x3B, 0x02},
  {0x97, 0x30},
  {0x98, 0x20},
  {0x99, 0x30},
  {0x9A, 0x84},
  {0x9B, 0x29},
  {0x9C, 0x03},
  {0x9D, 0x4C},
  {0x9E, 0x3F},
  {0x78, 0x04},

  /* Gamma Coefficients */
  {0x7A, 0x20},
  {0x7B, 0x10},
  {0x7C, 0x1E},
  {0x7D, 0x35},
  {0x7E, 0x5A},
  {0x7F, 0x69},
  {0x80, 0x76},
  {0x81, 0x80},
  {0x82, 0x88},
  {0x83, 0x8F},
  {0x84, 0x96},
  {0x85, 0xA3},
  {0x86, 0xAF},
  {0x87, 0xC4},
  {0x88, 0xD7},
  {0x89, 0xE8},

  /* Pre-Scalar */
  {0x11, 0x00},

  //{0x42, 0x08}, // test bar
  //{0x71, 0xC0},

  {REG_BATT, REG_BATT},
};

#endif /* INC_OV7670_REGISTERS_MICROSOFT_H_ */
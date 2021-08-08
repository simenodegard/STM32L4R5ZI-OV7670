/*
 * ov7670.h
 *
 *  Created on: May 18, 2021
 *      Author: Simen
 */

#include "main.h"
//#include "ov7670_registers.h"
//#include "ov7670_registers_v2.h"
//#include "ov7670_registers_Amin.h"
//#include "ov7670_registers_Amin_V2.h"
//#include "ov7670_registers_Microsoft.h"
#include "ov7670_registers_MIT.h" // just defines of the registers names
//#include "ov7670_registers_Simen.h"
//#include "ov7670_registers_SIMEN_V2.h"
#include "ov7670_registers_SIMEN_V3.h"

#ifndef INC_OV7670_H_
#define INC_OV7670_H_


#define OV7670_WRITE_ADDR 0x42 		//page 11: The device slave addresses are 42 for write and 43 for read
#define OV7670_READ_ADDR 0x43
#define RGB565_QVGA_SIZE 38400	// 76800/2 Number of 32-bit words -> 2 pixels per word - se page 32 DCMI doc - 38.4 kB
#define RGB565_QVGA_SIZE_8BIT 153600  //38400*4

extern uint32_t frame_buffer[RGB565_QVGA_SIZE];

void ov7670_read(uint8_t reg_addr);
void ov7670_write(uint8_t reg_addr, uint8_t data);
void ov7670_write_Delay(uint8_t reg_addr, uint8_t data);
uint8_t ov7670_read_return(uint8_t reg_addr);
void ov7670_hard_reset();
void ov7670_init();
void ov7670_read_all_reg();
void ov7670_reg_check();
void capture_image();
uint8_t ov7670_init_MIT(uint32_t n);

#endif /* INC_OV7670_H_ */

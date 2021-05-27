/*
 * ov7670.h
 *
 *  Created on: May 18, 2021
 *      Author: Simen
 */

#include "main.h"
#include "ov7670_registers.h"
//#include "ov7670_registers_v2.h"
//#include "ov7670_registers_Amin.h"

#ifndef INC_OV7670_H_
#define INC_OV7670_H_


#define OV7670_WRITE_ADDR 0x42 		//page 11: The device slave addresses are 42 for write and 43 for read
#define OV7670_READ_ADDR 0x43

void ov7670_read(uint8_t reg_addr);
void ov7670_write(uint8_t reg_addr, uint8_t data);
uint8_t ov7670_read_return(uint8_t reg_addr);
void ov7670_hard_reset();
void ov7670_init();
void ov7670_read_all_reg();
void ov7670_reg_check();


#endif /* INC_OV7670_H_ */

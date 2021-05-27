/*
 * ov7670.c
 *
 *  Created on: May 18, 2021
 *      Author: Simen
 */


//#include "stm32l4xx_hal.h"
#include "ov7670.h"

extern I2C_HandleTypeDef hi2c2;



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

//
//RET camera_init()
//{
//  return ov7670_init(&hdcmi, &hdma_dcmi, &hi2c2);
//}
//
//RET camera_config(uint32_t mode)
//{
//  uint32_t ov7670Mode;
//  switch (mode){
//  case CAMERA_MODE_QVGA_RGB565:
//    ov7670Mode = OV7670_MODE_QVGA_RGB565;
//    break;
//  case CAMERA_MODE_QVGA_YUV:
//    ov7670Mode = OV7670_MODE_QVGA_YUV;
//    break;
//  default:
//    printf("camera mode %d is not supported\n", mode);
//    return RET_ERR;
//  }
//  return ov7670_config(ov7670Mode);
//}


void ov7670_init()
{
//	uint8_t buffer[0];
//	buffer[0] = 0x01;
//  //ov7670_hard_reset();
  ov7670_write(0x12, 0x80);  // soft reset
  HAL_Delay(500);			 // Needs a delay after reset otherwise all registers are 0x00

  // Configure camera
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1);
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




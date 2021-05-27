/*
 * uart.c
 *
 *  Created on: May 25, 2021
 *      Author: Simen
 */


#include "uart.h"

void Serial_com(uint8_t *s, uint32_t size) {
	uint32_t counter = 0;
	while (counter < size) {
		while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
			; // Wait for Empty
		LL_LPUART_TransmitData8(LPUART1, *s++);
		counter++;
	}
}

/*
 *  Implemented a few Low Level function to support stio.h function like printf
 *
 *  Created on: 21.10.2019
 *      Author: Adrian Wojak
 */

#include "main.h"

//define UART resource to output/input
/*extern UART_HandleTypeDef huart2;

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++){ __io_putchar( *ptr++ );}
	return len;
}
*/

int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return(ch);
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
	__io_putchar(*ptr++);
	}
	return len;
}
// TODO: Implement others function for read data etc.


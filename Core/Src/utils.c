/*
 * utils.c
 *
 *  Created on: 9 янв. 2022 г.
 *      Author: timagr615
 */
//#include "main.h"
//#include <string.h>
//extern UART_HandleTypeDef huart1;
#include "utils.h"

void print_float(uint8_t *str, float data){
	//sprintf(str, " %d.%05d  ",(uint32_t)fabs(data), (uint16_t)((data - (uint32_t)data)*100000.));
	sprintf(str, "  %5.9f  ", data);
	printf(str);
	//HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);
}

void print_mat(uint8_t *str, double data){
	sprintf(str, "%6.7f", data);
	printf(str);
	//HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);
}

void assert_fail(uint8_t* file, uint32_t line)
{
	uint8_t error_str[200];
	sprintf(error_str, "Wrong parameters value: file %s on line %d\r\n", file, (int)line);
	printf(error_str);
	//HAL_UART_Transmit(&huart1, error_str, strlen((char *)error_str), 0xFFFF);
}

/*
 * utils.h
 *
 *  Created on: 9 янв. 2022 г.
 *      Author: timagr615
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_
#include "main.h"
#include <string.h>

//extern UART_HandleTypeDef huart1;

#define assert_p(expr) ((expr) ? (void)0 : assert_fail((uint8_t *)__FILE__, __LINE__))

void print_float(uint8_t *str, float data);
void print_mat(uint8_t *str, double data);

void assert_fail(uint8_t* file, uint32_t line);


#endif /* INC_UTILS_H_ */

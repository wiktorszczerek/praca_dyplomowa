/*
 * shared_variables.h
 *
 *  Created on: Sep 1, 2020
 *      Author: wszcz
 */

#ifndef INC_SHARED_VARIABLES_H_
#define INC_SHARED_VARIABLES_H_

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;


uint8_t button_pressed = 0;

#endif /* INC_SHARED_VARIABLES_H_ */

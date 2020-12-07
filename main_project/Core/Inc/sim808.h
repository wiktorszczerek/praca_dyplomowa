/*
 * sim808.h
 *
 *  Created on: Dec 7, 2020
 *      Author: wszcz
 */
#ifndef INC_SIM808_H_
#define INC_SIM808_H_
#define UART_BUFFER_SIZE 24
#include "includes.h"
#include "shared_variables.h"

void sim808_power_on();
void sim808_power_off();
ERRORS sim808_establish_baudrate();
ERRORS sim808_init();
ERRORS sim808_send_sms(char* text);


#endif /* INC_SIM808_H_ */

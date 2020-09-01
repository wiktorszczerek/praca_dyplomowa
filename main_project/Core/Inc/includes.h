/*
 * includes.h
 *
 *  Created on: Sep 1, 2020
 *      Author: wszcz
 */

#ifndef INC_INCLUDES_H_
#define INC_INCLUDES_H_

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_hal.h"

typedef enum ERRORS
{
	NULL_STATE,
	OK,
	CRITICAL_ERROR,
	EEPROM_DATA_INVALID,
	EEPROM_READ_ERROR
}ERRORS;

typedef enum ERROR_CATEGORIES
{
	EEPROM
}ERROR_CATEGORIES;


#endif /* INC_INCLUDES_H_ */

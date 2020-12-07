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
#include "main.h"
#include "errno.h"

typedef enum ERRORS
{
	NULL_STATE,
	OK,
	CRITICAL_ERROR,
	THRESHOLD_LEVEL_EXCEEDED,
	EEPROM_DATA_INVALID,
	EEPROM_SENSOR_VERSION_INVALID,
	EEPROM_READ_ERROR,
	RTC_SETUP_DATE_ERROR,
	RTC_SETUP_TIME_ERROR,
	RTC_SET_ALARM_ERROR,
	SIM808_ERROR,
	SIM808_SIM_LOCKED,
	SIM808_SIM_UNLOCKED,
	SIM808_ECHO_ENABLED,
	SIM808_MESSAGE_ERROR
}ERRORS;



#endif /* INC_INCLUDES_H_ */

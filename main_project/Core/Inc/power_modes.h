/*
 * power_modes.h
 *
 *  Created on: 8 wrz 2020
 *      Author: wszcz
 */

#ifndef INC_POWER_MODES_H_
#define INC_POWER_MODES_H_

#include "includes.h"
ERRORS rtc_setup();
ERRORS rtc_set_alarm_in_seconds(RTC_HandleTypeDef* hrtc, int seconds);

void power_mode_sleep(RTC_HandleTypeDef* hrtc);

#endif /* INC_POWER_MODES_H_ */

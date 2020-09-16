/*
 * power_modes.c
 *
 *  Created on: 8 wrz 2020
 *      Author: wszcz
 */
#include "power_modes.h"


ERRORS rtc_set_alarm_in_seconds(RTC_HandleTypeDef* hrtc, int seconds)
{
	RTC_AlarmTypeDef 	salarmstructure;
	RTC_TimeTypeDef 	stimestructure;

	HAL_RTC_GetTime(hrtc, &stimestructure, RTC_FORMAT_BIN);

	salarmstructure.Alarm = RTC_ALARM_A;
	salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
	salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
	salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	salarmstructure.AlarmTime.Hours = stimestructure.Hours;
	salarmstructure.AlarmTime.Minutes = stimestructure.Minutes;
	salarmstructure.AlarmTime.Seconds = stimestructure.Seconds + seconds;
	salarmstructure.AlarmTime.SubSeconds = 0x56;
	if(HAL_RTC_SetAlarm_IT(hrtc, &salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		return RTC_SET_ALARM_ERROR;
	}
	return OK;
}



void power_mode_sleep()
{
	/* Set all GPIO in analog state to reduce power consumption */
	GPIO_AnalogState_Config();

	/* Set the System clock to 24 MHz (MSI) */
	SystemClock_24MHz();

	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	/* Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

	/* Suspend Tick increment to prevent wakeup by Systick interrupt.         */
	/* Otherwise the Systick interrupt will wake up the device within 1ms     */
	/* (HAL time base).                                                       */
	HAL_SuspendTick();

	/* Switch off all clock enable ... */
	RCC->AHB1SMENR = 0x0;
	RCC->AHB2SMENR = 0x0;
	RCC->AHB3SMENR = 0x0;
	RCC->APB1SMENR1 = 0x0;
	RCC->APB1SMENR2 = 0x0;
	RCC->APB2SMENR = 0x0;

	/* Enter SLEEP Mode, Main regulator is ON */
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void power_mode_select(int mode)
{
	switch(mode)
	{
		case 0:
		{
			power_mode_sleep();
			break;
		}
		case 1:
		{
			//TODO ???
		}
	}
}


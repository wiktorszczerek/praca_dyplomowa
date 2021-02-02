/*
 * sensors.h
 *
 *  Created on: 23 paź 2020
 *      Author: wszcz
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

//INTERVALS (POWER MODES)
#define CARBON_MONOXIDE_SLEEP_TIME 			0x2710 //for 32kHz it's 5s.
#define CARBON_MONOXIDE_THREE_MINUTES		36
#define CARBON_MONOXIDE_TWO_MINUTES			24
#define INFINITE_SLEEP_TIME					0

//ETANOL - loops and delay between measurements.
#define ETHANOL_LOOPS						100 //150 * 200ms = 30000ms = 30s. - should be enough -> far too much...
#define ETHANOL_DELAY_BETWEEN_MEASUREMENTS	200
#define STANDARD_DELAY_AFTER_MEASUREMENT	100

#endif /* INC_SENSORS_H_ */

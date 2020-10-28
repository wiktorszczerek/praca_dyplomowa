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
#define INFINITE_SLEEP_TIME					0

//ETANOL - loops and delay between measurements.
#define ETHANOL_LOOPS						150 //150 * 200ms = 30000ms = 30s. - should be enough
#define ETHANOL_DELAY_BETWEEN_MEASUREMENTS	200

#endif /* INC_SENSORS_H_ */

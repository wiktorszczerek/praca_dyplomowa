/*
 * eeprom.h
 *
 *  Created on: Sep 1, 2020
 *      Author: wszcz
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "includes.h"
#include "shared_variables.h"

#define ONE_HEX_BYTE							2

#define STRUCT_SIZE								8

#define VERSION_NUM								44									//0-255
#define SENSOR_ID 								1									//0-255
#define SENSOR_TYPE								ETHANOL						//enum SENSOR_TYPES
//this coefficient is usually 1-5 nA/ppm -> so f.e. 2.220 will look like: DEC = 2 , FRAC = 220
#define CURRENT_PER_PPM_COEFFICIENT_DEC			1									//depends on the sensor, but ususally >1
#define CURRENT_PER_PPM_COEFFICIENT_FRAC		940									//0-999
#define THRESHOLD								4095								//0-4095 for 12b configured ADC
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER	40890									//0 when factory-reset (0-65356)
#define SENSOR_FIRED_COUNTER					65356								//0 when factory-reset (0-16 777 215)

#define EEPROM_DATA_START						'<'
#define EEPROM_DATA_END							'>'

#define SIZE_OF_SPRINTF_FLAG					4+1


//PARAMETERS SIZE (IN HEX SIGNS)
#define VERSION_NUM_SIZE							ONE_HEX_BYTE
#define SENSOR_ID_SIZE								ONE_HEX_BYTE
#define SENSOR_TYPE_SIZE							ONE_HEX_BYTE
#define CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE		ONE_HEX_BYTE
#define CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE		ONE_HEX_BYTE+1
#define THRESHOLD_SIZE								ONE_HEX_BYTE+1
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE	2*ONE_HEX_BYTE
#define SENSOR_FIRED_COUNTER_SIZE					3*ONE_HEX_BYTE


//EEPROM ADDRESSES
#define EEPROM_ADDRESS								0xA0
#define EEPROM_DATA_START_ADDR						0x00
#define VERSION_NUM_ADDR							0x01		//+2
#define SENSOR_ID_ADDR								0x03		//+2
#define SENSOR_TYPE_ADDR							0x05		//+2
#define CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR		0x07		//+2
#define CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR		0x09		//+3
#define THRESHOLD_ADDR								0x0C		//+3
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR	0x0F		//+4
#define SENSOR_FIRED_COUNTER_ADDR					0x13		//+6
#define EEPROM_DATA_END_ADDR						0x19

typedef enum SENSOR_TYPES
{
	CARBON_MONOXIDE,
	ETHANOL,
	DINITROGEN_DIOXIDE,
	METHANE
}SENSOR_TYPES;


//STRUCTS
/**
 * A structure to hold the info about sensor plugged into the circuit.
 */
struct sensor_data {
	uint8_t version_num; /**< version of this info. CAN'T BE 0 (ZERO) - THIS VALUE IS USED ONLY FOR EEPROM ERROR CHECKS. */
	uint8_t sensor_id;
	uint8_t sensor_type;
	uint8_t current_per_ppm_coefficient_dec;
	uint16_t current_per_ppm_coefficient_frac;
	uint16_t threshold;
	uint16_t device_turned_on_with_sensor_counter;
	uint32_t sensor_fired_counter;
};

struct sensor_info
{
	//(each field +1 because of the \0 sign, needed in formatting)
	/*@{*/
	char version_num[VERSION_NUM_SIZE+1]; /**< version of this info. CAN'T BE 0 (ZERO) - THIS VALUE IS USED ONLY FOR EEPROM ERROR CHECKS. */
	char sensor_id[SENSOR_ID_SIZE+1];
	char sensor_type[SENSOR_TYPE_SIZE+1];
	char current_per_ppm_coefficient_dec[CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE+1];
	char current_per_ppm_coefficient_frac[CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE+1];
	char threshold[THRESHOLD_SIZE+1];
	char device_turned_on_with_sensor_counter[DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE+1];
	char sensor_fired_counter[SENSOR_FIRED_COUNTER_SIZE+1];
	/*@}*/
};

//FUNCTIONS
void format_to_hex(uint16_t what, uint8_t num_of_signs, char* result);
void create_sensor_info(struct sensor_info* si);
void write_sensor_info_to_eeprom(struct sensor_info* si);
void zero_eeprom_useful_mem();

void sensor_info_init(struct sensor_info* si);
ERRORS read_sensor_data_from_eeprom(struct sensor_data* sd);

//DEBUG ONLY
ERRORS read_sensor_info_from_eeprom(struct sensor_info* si);
void show_read_sensor_info(struct sensor_info* si);
void show_read_sensor_data(struct sensor_data* sd);

#endif /* INC_EEPROM_H_ */

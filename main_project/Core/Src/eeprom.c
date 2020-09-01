/*
 * eeprom.c
 *
 *  Created on: Sep 1, 2020
 *      Author: wszcz
 */

#include "eeprom.h"

typedef enum SENSOR_TYPES
{
	CARBON_MONOXIDE,
	ETHANOL,
	DINITROGEN_DIOXIDE,
	METHANE
}SENSOR_TYPES;


void format_to_hex(uint16_t what, uint8_t num_of_signs, char* result)
{
	char flag[SIZE_OF_SPRINTF_FLAG];
	sprintf(flag,"%%0%1dx",num_of_signs);
	sprintf(result,flag,what);
}

void create_sensor_info(struct sensor_info* si)
{
	format_to_hex(VERSION_NUM, VERSION_NUM_SIZE, si->version_num);
	format_to_hex(SENSOR_ID, SENSOR_ID_SIZE, si->sensor_id);
	SENSOR_TYPES type_holder = SENSOR_TYPE;
	format_to_hex((uint8_t)type_holder, SENSOR_TYPE_SIZE, si->sensor_type);
	format_to_hex(CURRENT_PER_PPM_COEFFICIENT_DEC, CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE, si->current_per_ppm_coefficient_dec);
	format_to_hex(CURRENT_PER_PPM_COEFFICIENT_FRAC, CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE,si->current_per_ppm_coefficient_frac);
	format_to_hex(THRESHOLD, THRESHOLD_SIZE,si->threshold);
	format_to_hex(DEVICE_TURNED_ON_WITH_SENSOR_COUNTER, DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE,si->device_turned_on_with_sensor_counter);
	format_to_hex(SENSOR_FIRED_COUNTER, SENSOR_FIRED_COUNTER_SIZE,si->sensor_fired_counter);
}

void send_to_eeprom_byte_by_byte(uint16_t where, char* what)
{
	char msg[80];
	for(uint16_t i = 0; i<strlen(what); ++i)
	{
		if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, where+i, 1, (uint8_t*)(&(what[i])), sizeof(what[i]), HAL_MAX_DELAY)!=HAL_OK)
		{
			sprintf(msg,"WRITE OPERATION FAILED at %d index byte in %s.\r\n", i, what);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
		}
		HAL_Delay(500);
	}
}

void read_from_eeprom_single_byte(uint16_t addr, uint8_t* read_buffer)
{
	if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, addr, 1, read_buffer, sizeof(*read_buffer), HAL_MAX_DELAY)!= HAL_OK)
	{
		char msg[] = "EEPROM_READ_ERROR";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
	}
}

ERRORS read_from_eeprom_byte_by_byte(uint16_t addr, uint8_t num_of_bytes, uint8_t* read_buffer)
{
	for(uint8_t i=0;i<num_of_bytes;++i)
	{
		if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, addr+i, 1, (uint8_t*)(read_buffer+i), sizeof(read_buffer[i]), HAL_MAX_DELAY)!= HAL_OK)
		{
			/*char msg[] = "EEPROM_READ_ERROR";
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);*/
			return EEPROM_READ_ERROR;
		}
		//HAL_Delay(500);
	}
	return OK;
}


void send_to_eeprom_start_and_end_markers()
{
	char start = EEPROM_DATA_START, end = EEPROM_DATA_END;
	char msg[80];
	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, EEPROM_DATA_START_ADDR, 1, (uint8_t*)&start, sizeof(start), HAL_MAX_DELAY)!=HAL_OK)
	{
		sprintf(msg,"WRITE OPERATION FAILED at start eeprom data");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
	}
	HAL_Delay(500);

	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, EEPROM_DATA_END_ADDR, 1, (uint8_t*)&end, sizeof(end), HAL_MAX_DELAY)!=HAL_OK)
	{
		sprintf(msg,"WRITE OPERATION FAILED at end eeprom data");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
	}
	HAL_Delay(500);
}

void write_sensor_info_to_eeprom(struct sensor_info* si)
{
	char msg_write[80];
	sprintf(msg_write,"Sending data to EEPROM...\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
	send_to_eeprom_start_and_end_markers();
	send_to_eeprom_byte_by_byte(VERSION_NUM_ADDR, si->version_num);
	send_to_eeprom_byte_by_byte(SENSOR_ID_ADDR, si->sensor_id);
	send_to_eeprom_byte_by_byte(SENSOR_TYPE_ADDR, si->sensor_type);
	send_to_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR, si->current_per_ppm_coefficient_dec);
	send_to_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR, si->current_per_ppm_coefficient_frac);
	send_to_eeprom_byte_by_byte(THRESHOLD_ADDR, si->threshold);
	send_to_eeprom_byte_by_byte(DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR, si->device_turned_on_with_sensor_counter);
	send_to_eeprom_byte_by_byte(SENSOR_FIRED_COUNTER_ADDR, si->sensor_fired_counter);
	sprintf(msg_write,"Data saved to EEPROM!\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
}

void zero_eeprom_useful_mem()
{
	uint8_t zero_arr[100];
	memset(zero_arr, 0, 100);
	char msg_write[80];
	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0x00, 1, zero_arr, 100, HAL_MAX_DELAY)!=HAL_OK)
		{
			sprintf(msg_write,"WRITE OPERATION FAILED at zeroing EEPROM.\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
		}
	HAL_Delay(5000);
}

ERRORS check_eeprom_data()
{
	char data_start_check = 0, data_end_check = 0;
	read_from_eeprom_single_byte(EEPROM_DATA_START_ADDR, (uint8_t*)&data_start_check);
	read_from_eeprom_single_byte(EEPROM_DATA_END_ADDR, (uint8_t*)&data_end_check);
	if(data_start_check != '<' || data_end_check != '>')
	{
		/*TODO:
			flash diodes because sensor or sensor's circuit is dead somehow
			and set flag for turning off the whole device
		*/
		return EEPROM_DATA_INVALID;
	}
	return OK;
}

void add_end_chars_to_sensor_info(struct sensor_info* si)
{
	si->version_num[VERSION_NUM_SIZE] 													= '\0';
	si->sensor_id[SENSOR_ID_SIZE] 														= '\0';
	si->sensor_type[SENSOR_TYPE_SIZE]													= '\0';
	si->current_per_ppm_coefficient_dec[CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE] 			= '\0';
	si->current_per_ppm_coefficient_frac[CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE] 		= '\0';
	si->threshold[THRESHOLD_SIZE]														= '\0';
	si->device_turned_on_with_sensor_counter[DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE] = '\0';
	si->sensor_fired_counter[SENSOR_FIRED_COUNTER_SIZE]									= '\0';
}



ERRORS read_sensor_data_from_eeprom(struct sensor_info* si)
{
	if(check_eeprom_data() != OK)
		return EEPROM_DATA_INVALID;


	if(read_from_eeprom_byte_by_byte(VERSION_NUM_ADDR, VERSION_NUM_SIZE, (uint8_t*)(si->version_num)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(SENSOR_ID_ADDR, SENSOR_ID_SIZE, (uint8_t*)(si->sensor_id)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(SENSOR_TYPE_ADDR, SENSOR_TYPE_SIZE, (uint8_t*)(si->sensor_type)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR, CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE,
			(uint8_t*)(si->current_per_ppm_coefficient_dec)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR, CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE,
			(uint8_t*)(si->current_per_ppm_coefficient_frac)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(THRESHOLD_ADDR, THRESHOLD_SIZE, (uint8_t*)(si->threshold)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR, DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE,
			(uint8_t*)(si->device_turned_on_with_sensor_counter)) != OK)
	{
		return CRITICAL_ERROR;
	}
	if(read_from_eeprom_byte_by_byte(SENSOR_FIRED_COUNTER_ADDR, SENSOR_FIRED_COUNTER_SIZE,
			(uint8_t*)(si->sensor_fired_counter)) != OK)
	{
		return CRITICAL_ERROR;
	}

	add_end_chars_to_sensor_info(si);

	return OK;
}

void sensor_info_init(struct sensor_info* si)
{
	memset(si->version_num, '0', VERSION_NUM_SIZE);
	memset(si->sensor_id, '0', SENSOR_ID_SIZE);
	memset(si->sensor_type, '0', SENSOR_TYPE_SIZE);
	memset(si->current_per_ppm_coefficient_dec, '0', CURRENT_PER_PPM_COEFFICIENT_DEC_SIZE);
	memset(si->current_per_ppm_coefficient_frac, '0', CURRENT_PER_PPM_COEFFICIENT_FRAC_SIZE);
	memset(si->threshold, '0', THRESHOLD_SIZE);
	memset(si->device_turned_on_with_sensor_counter, '0', DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_SIZE);
	memset(si->sensor_fired_counter, '0', SENSOR_FIRED_COUNTER_SIZE);
}

void show_read_sensor_data(struct sensor_info* si)
{
	char msg_buffer[256];
	sprintf(msg_buffer,"+=========EEPROM=========+\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"VERSION NUMBER: %s\n\r", si->version_num);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"SENSOR ID: %s\n\r", si->sensor_id);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"SENSOR TYPE: %s\n\r", si->sensor_type);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"CURRENT_DEC: %s\n\r", si->current_per_ppm_coefficient_dec);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"CURRENT FRAC: %s\n\r", si->current_per_ppm_coefficient_frac);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"THRESHOLD: %s\n\r", si->threshold);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"DEVICE ON COUNTER: %s\n\r", si->device_turned_on_with_sensor_counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"SENSOR FIRED COUNTER: %s\n\r", si->sensor_fired_counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
	sprintf(msg_buffer,"+========================+\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, strlen(msg_buffer)+1, 10);
}

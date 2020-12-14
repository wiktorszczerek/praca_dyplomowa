/*
 * sim808.c
 *
 *  Created on: Dec 7, 2020
 *      Author: wszcz
 */
#include "sim808.h"
uint8_t recv[UART_BUFFER_SIZE];
uint8_t response_received = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	response_received = 1;
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void sim808_power_on()
{
	HAL_GPIO_WritePin(SIM808_PWR_GPIO_Port, SIM808_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(SIM808_PWR_GPIO_Port, SIM808_PWR_Pin, GPIO_PIN_RESET);
	HAL_Delay(5000);
}

void sim808_power_off()
{
	HAL_GPIO_WritePin(SIM808_PWR_GPIO_Port, SIM808_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(SIM808_PWR_GPIO_Port, SIM808_PWR_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}

void clear_uart_buffer()
{
  for(uint8_t i=0;i<UART_BUFFER_SIZE;++i)
  {
	  recv[i] = '\0';
  }
}

ERRORS sim808_send_and_check_response(char* cmd, uint8_t response_size)
{
	response_received = 0;
	clear_uart_buffer();
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	HAL_UART_Receive_DMA(&huart1, recv, response_size);
	while(!response_received){};
//	HAL_UART_Transmit(&huart2, recv, UART_BUFFER_SIZE, 1000);
	if(!strcmp((char*)cmd, "\r\nERROR\r\n"))
	{
		return SIM808_ERROR;
	}
	if(!strcmp(cmd,"AT+CPIN?\r"))
	{
		if(!strcmp((char*)recv, "\r\n+CPIN: SIM PIN\r\n\r\nOK"))
			return SIM808_SIM_LOCKED;
		else
			return SIM808_SIM_UNLOCKED;
	}
	else if(!strcmp(cmd, "AT\r"))
	{
		if(strcmp((char*)recv,"\r\nOK\r\n"))
		{
			return SIM808_ECHO_ENABLED;
		}
		else return OK;
	}
	else if(!strcmp(cmd, "AT+CMGS=\"+48665055722\"\r"))
	{
		if(!strcmp((char*)recv, "\r\n>"))
			return OK;
		else return SIM808_MESSAGE_ERROR;
	}
	return OK;
}

ERRORS sim808_establish_baudrate()
{
  ERRORS local_error = sim808_send_and_check_response("AT\r", 6);
  if(local_error == SIM808_ECHO_ENABLED)
	  local_error=sim808_send_and_check_response("ATE0\r", 10);

  if(local_error == SIM808_ERROR)
	  return CRITICAL_ERROR;

  return OK;
}

ERRORS sim808_init()
{
	ERRORS local_error = sim808_send_and_check_response("AT+CPIN?\r", 22);
	if(local_error == SIM808_SIM_LOCKED)
		local_error=sim808_send_and_check_response("AT+CPIN=\"1022\"\r",22);

	if(local_error == SIM808_ERROR)
		return CRITICAL_ERROR;

	HAL_Delay(10000);

	if((local_error=sim808_send_and_check_response("AT+CFUN=1\r", 6))==SIM808_ERROR)
		return CRITICAL_ERROR;
	if((local_error=sim808_send_and_check_response("AT+CMGF=1\r", 6))==SIM808_ERROR)
		return CRITICAL_ERROR;
	if((local_error=sim808_send_and_check_response("AT+CSCS=\"GSM\"\r", 6))==SIM808_ERROR)
		return CRITICAL_ERROR;

	return OK;
}

ERRORS sim808_send_sms(char* text)
{
	char* to_send = (char*)malloc(strlen(text)+1);
	sprintf(to_send, text);
	sprintf(to_send+strlen(text),"");
	ERRORS local_error = sim808_send_and_check_response("AT+CMGS=\"+48665055722\"\r", 3);
	if(local_error == OK)
		local_error = sim808_send_and_check_response(to_send, 13);
	if(local_error == SIM808_ERROR)
		return CRITICAL_ERROR;
	return OK;
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//+++++++++++++++++++++++++++++++++++++++
//Specify type of sensor here (uncomment the proper line)
//+++++++++++++++++++++++++++++++++++++++
//#define SENSOR_CARBON_MONOXIDE
#define SENSOR_ETHANOL
//#define SENSOR_NITROGEN_DIOXIDE
//#define SENSOR_METHANE

typedef enum SENSOR_TYPES
{
	CARBON_MONOXIDE,
	ETHANOL,
	NITROGEN_DIOXIDE,
	METHANE
}SENSOR_TYPES;


#define ONE_HEX_BYTE							2
struct sensor_info
{
	char version_num[ONE_HEX_BYTE+1];
	char sensor_id[ONE_HEX_BYTE+1];
	char sensor_type[ONE_HEX_BYTE+1];
	char current_per_ppm_coefficient_dec[ONE_HEX_BYTE+1];
	char current_per_ppm_coefficient_frac[ONE_HEX_BYTE+1+1];
	char threshold[ONE_HEX_BYTE+1+1];
	char device_turned_on_with_sensor_counter[2*ONE_HEX_BYTE+1];
	char sensor_fired_counter[3*ONE_HEX_BYTE+1];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//+++++++++++++++++++++++++++++++++++++++
//Specify version number!
//+++++++++++++++++++++++++++++++++++++++
#define VERSION_NUM								1									//0-255
//+++++++++++++++++++++++++++++++++++++++
//Specify sensor parameters for wanted type!
//+++++++++++++++++++++++++++++++++++++++
#if defined(SENSOR_CARBON_MONOXIDE)
#define SENSOR_ID 								1									//0-255
#define SENSOR_TYPE								CARBON_MONOXIDE						//enum SENSOR_TYPES
//this coefficient is usually 1-5 nA/ppm -> so f.e. 2.220 will look like: DEC = 2 , FRAC = 220
#define CURRENT_PER_PPM_COEFFICIENT_DEC			1									//depends on the sensor, but ususally >1
#define CURRENT_PER_PPM_COEFFICIENT_FRAC		592									//0-999
#define THRESHOLD								43								//0-4095 for 12b configured ADC
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER	0									//0 when factory-reset (0-65356)
#define SENSOR_FIRED_COUNTER					0								//0 when factory-reset (0-16 777 215)
#elif defined(SENSOR_ETHANOL)
#define SENSOR_ID 								2									//0-255
#define SENSOR_TYPE								ETHANOL						//enum SENSOR_TYPES
//this coefficient is usually 1-5 nA/ppm -> so f.e. 2.220 will look like: DEC = 2 , FRAC = 220
#define CURRENT_PER_PPM_COEFFICIENT_DEC			27									//depends on the sensor, but ususally >1
#define CURRENT_PER_PPM_COEFFICIENT_FRAC		650									//0-999
#define THRESHOLD								10								//0-4095 for 12b configured ADC
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER	0									//0 when factory-reset (0-65356)
#define SENSOR_FIRED_COUNTER					0								//0 when factory-reset (0-16 777 215)
#endif

#define STRUCT_SIZE								8

#define EEPROM_DATA_START						'<'
#define EEPROM_DATA_END							'>'

#define SIZE_OF_SPRINTF_FLAG					4+1

#define EEPROM_ADDRESS								0xA0
#define EEPROM_DATA_START_ADDR						0x00
#define VERSION_NUM_ADDR							0x01		//+2
#define SENSOR_ID_ADDR								0x03		//+2
#define SENSOR_TYPE_ADDR							0x05		//+2
#define CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR		0x07		//+2
#define CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR		0x09		//+3
#define THRESHOLD_ADDR								0x0C		//+3
//another line, because of weird things with this EEPROM's addressing
#define DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR	0x0F		//+4
#define SENSOR_FIRED_COUNTER_ADDR					0x13		//+6
#define EEPROM_DATA_END_ADDR						0x19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void format_to_hex(uint16_t what, uint8_t num_of_signs, char* result);
void create_sensor_info(struct sensor_info* si);
void write_sensor_info_to_eeprom(struct sensor_info* si);
void zero_eeprom_useful_mem();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  struct sensor_info sensor;
  create_sensor_info(&sensor);

  //zero_eeprom_useful_mem();

  write_sensor_info_to_eeprom(&sensor);
  HAL_Delay(1000);
  uint8_t res[128];
  HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x00, 1, res, 128*sizeof(uint8_t), 128);
  HAL_UART_Transmit(&huart2, res, 128*sizeof(uint8_t), 128);
  char msg[2] = "\n\r";
  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x007088FF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void format_to_hex(uint16_t what, uint8_t num_of_signs, char* result)
{
	char flag[SIZE_OF_SPRINTF_FLAG];
	sprintf(flag,"%%0%1dx",num_of_signs);
	sprintf(result,flag,what);
}

void create_sensor_info(struct sensor_info* si)
{
	format_to_hex(VERSION_NUM, ONE_HEX_BYTE, si->version_num);
	format_to_hex(SENSOR_ID, ONE_HEX_BYTE, si->sensor_id);
	SENSOR_TYPES states_holder = SENSOR_TYPE;
	format_to_hex((uint8_t)states_holder, ONE_HEX_BYTE, si->sensor_type);
	format_to_hex(CURRENT_PER_PPM_COEFFICIENT_DEC, ONE_HEX_BYTE, si->current_per_ppm_coefficient_dec);
	format_to_hex(CURRENT_PER_PPM_COEFFICIENT_FRAC, ONE_HEX_BYTE+1,si->current_per_ppm_coefficient_frac);
	format_to_hex(THRESHOLD, ONE_HEX_BYTE+1,si->threshold);
	format_to_hex(DEVICE_TURNED_ON_WITH_SENSOR_COUNTER, 2*ONE_HEX_BYTE,si->device_turned_on_with_sensor_counter);
	format_to_hex(SENSOR_FIRED_COUNTER, 3*ONE_HEX_BYTE,si->sensor_fired_counter);
}

void send_to_eeprom_byte_by_byte(uint16_t where, char* what)
{
	char msg[80];
	for(uint16_t i = 0; i<strlen(what); ++i)
	{
//		HAL_UART_Transmit(&huart2, (uint8_t*)"loop_in_sending_byte_by_byte\n", 29, HAL_MAX_DELAY);
		if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, where+i, 1, (uint8_t*)(&(what[i])), sizeof(what[i]), HAL_MAX_DELAY)!=HAL_OK)
		{
			sprintf(msg,"WRITE OPERATION FAILED at %d index byte in %s.\r\n", i, what);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
		}
		HAL_Delay(500);
	}
}

void send_to_eeprom_begin_and_end_markers()
{
	char begin = EEPROM_DATA_START, end = EEPROM_DATA_END;
	char msg[80];
	HAL_UART_Transmit(&huart2, (uint8_t*)"markers start\n\r", 15, 10);
	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, EEPROM_DATA_START_ADDR, 1, (uint8_t*)&begin, sizeof(begin), HAL_MAX_DELAY)!=HAL_OK)
	{
		sprintf(msg,"WRITE OPERATION FAILED at start eeprom data");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)"markers second\n\r", 16, 10);
	HAL_Delay(1000);

	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, EEPROM_DATA_END_ADDR, 1, (uint8_t*)&end, sizeof(end), HAL_MAX_DELAY)!=HAL_OK)
	{
		sprintf(msg,"WRITE OPERATION FAILED at end eeprom data");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 10);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)"markers end\n\r", 13, 10);
	HAL_Delay(1000);
}

void write_sensor_info_to_eeprom(struct sensor_info* si)
{
	char msg_write[80];
	sprintf(msg_write,"Sending data to EEPROM...\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
	send_to_eeprom_begin_and_end_markers();
	send_to_eeprom_byte_by_byte(VERSION_NUM_ADDR, si->version_num);
	send_to_eeprom_byte_by_byte(SENSOR_ID_ADDR, si->sensor_id);
	send_to_eeprom_byte_by_byte(SENSOR_TYPE_ADDR, si->sensor_type);
	send_to_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR, si->current_per_ppm_coefficient_dec);
	send_to_eeprom_byte_by_byte(CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR, si->current_per_ppm_coefficient_frac);
	send_to_eeprom_byte_by_byte(THRESHOLD_ADDR, si->threshold);
	send_to_eeprom_byte_by_byte(DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR, si->device_turned_on_with_sensor_counter);
	send_to_eeprom_byte_by_byte(SENSOR_FIRED_COUNTER_ADDR, si->sensor_fired_counter);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, VERSION_NUM_ADDR, 1, (uint8_t*)si->version_num, strlen(si->version_num), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at version field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, sizeof(msg_write), 10);
//	}
//	HAL_Delay(1000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, SENSOR_ID_ADDR, 1, (uint8_t*)si->sensor_id, strlen(si->sensor_id), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at sensor_id field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, sizeof(msg_write), 10);
//	}
//	HAL_Delay(1000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, SENSOR_TYPE_ADDR, 1, (uint8_t*)si->sensor_type, strlen(si->sensor_type), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at sensor_type field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
//	HAL_Delay(1000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, CURRENT_PER_PPM_COEFFICIENT_DEC_ADDR, 1,
//			(uint8_t*)si->current_per_ppm_coefficient_dec, strlen(si->current_per_ppm_coefficient_dec), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at current_per_ppm_dec field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
//	HAL_Delay(1000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, CURRENT_PER_PPM_COEFFICIENT_FRAC_ADDR, 1,
//			(uint8_t*)si->current_per_ppm_coefficient_frac, strlen(si->current_per_ppm_coefficient_frac), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at current_per_ppm_frac field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
//	HAL_Delay(1000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, THRESHOLD_ADDR, 1, (uint8_t*)si->threshold, strlen(si->threshold), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at threshold field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
//	HAL_Delay(2000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, DEVICE_TURNED_ON_WITH_SENSOR_COUNTER_ADDR, 1,
//			(uint8_t*)si->device_turned_on_with_sensor_counter, strlen(si->device_turned_on_with_sensor_counter), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at device... field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
//	HAL_Delay(2000);
//	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, SENSOR_FIRED_COUNTER_ADDR, 1,
//			(uint8_t*)si->sensor_fired_counter, strlen(si->sensor_fired_counter), HAL_MAX_DELAY)!=HAL_OK)
//	{
//		sprintf(msg_write,"WRITE OPERATION FAILED at sensor_fired field in struct.\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg_write, strlen(msg_write), 10);
//	}
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_UART_Transmit(&huart2, "yes, this is error", 18, HAL_MAX_DELAY);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

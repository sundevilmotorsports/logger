/*
 * eeprom.c
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 * 	
 *	Edited on: Feb 26, 2025
 *		Author: Alex Rumer
 */
#include "eeprom.h"

HAL_StatusTypeDef eepromWrite(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data) {
	return HAL_I2C_Mem_Write(hi2c, EEPROM_DEV_ID | 0, addr, 2, data, 1, 1000);
}

HAL_StatusTypeDef eepromRead(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, EEPROM_DEV_ID | 0, addr, 2, data, 1, 1000);
	return ret;
}

HAL_StatusTypeDef eepromReadString(I2C_HandleTypeDef* hi2c, uint16_t addr, char* data, uint8_t len){
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, EEPROM_DEV_ID | 0, addr, 2, data, len, 1000);
	return ret;
}

HAL_StatusTypeDef eepromWriteString(I2C_HandleTypeDef* hi2c, uint16_t addr, char* data, uint8_t len){
	return HAL_I2C_Mem_Write(hi2c, EEPROM_DEV_ID | 0, addr, 2, data, len, 1000);
}

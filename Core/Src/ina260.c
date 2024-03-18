/*
 * ina260.c
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */
#include "ina260.h"

static uint8_t buffer[2];

void ina260Init() {
	buffer[0] = 0;
	buffer[1] = 1;
}

uint16_t getCurrent(I2C_HandleTypeDef* hi2c) {
	HAL_I2C_Mem_Read(hi2c, (INA260_DEV_ID << 1 | 1), INA260_REG_CURR, 1, buffer, 2, 1000);
	uint16_t current = buffer[0] << 8 | buffer[1];
	return current;
}

uint16_t getVoltage(I2C_HandleTypeDef* hi2c) {
	HAL_I2C_Mem_Read(hi2c, (INA260_DEV_ID << 1 | 1), INA260_REG_VBUS, 1, buffer, 2, 1000);
	uint16_t volt = buffer[0] << 8 | buffer[1];
	return volt;
}

uint8_t checkMfgID(I2C_HandleTypeDef* hi2c) {
	HAL_I2C_Mem_Read(hi2c, (INA260_DEV_ID << 1 | 1), INA260_REG_MFG, 1, buffer, 2, 1000);
	if(buffer[0] == 0x54 && buffer[1] == 0x49) {
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t checkDieID(I2C_HandleTypeDef* hi2c) {
	HAL_I2C_Mem_Read(hi2c, (INA260_DEV_ID << 1 | 1), INA260_REG_DIE, 1, buffer, 2, 1000);
	if(buffer[0] == 0x22 && buffer[1] == 0x70) {
		return 1;
	}
	else {
		return 0;
	}
}


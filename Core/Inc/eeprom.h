/*
 * eeprom.h
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define EEPROM_DEV_ID 0b1010000 << 1


HAL_StatusTypeDef eepromWrite(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data);
HAL_StatusTypeDef eepromRead(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data);


#endif /* INC_EEPROM_H_ */

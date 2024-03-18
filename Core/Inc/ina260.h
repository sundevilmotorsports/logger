/*
 * ina260.h
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */

#ifndef INC_INA260_H_
#define INC_INA260_H_

#include "main.h"

#define INA260_DEV_ID   0b1000000
#define INA260_REG_CURR 0x01
#define INA260_REG_VBUS 0x02
#define INA260_REG_MFG  0xfe
#define INA260_REG_DIE  0xff

void ina260Init();
uint16_t getCurrent(I2C_HandleTypeDef* hi2c);
uint16_t getVoltage(I2C_HandleTypeDef* hi2c);
uint8_t checkMfgID(I2C_HandleTypeDef* hi2c);
uint8_t checkDieID(I2C_HandleTypeDef* hi2c);

#endif /* INC_INA260_H_ */

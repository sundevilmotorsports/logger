/*
 * logger.h
 *
 *  Created on: Mar 23, 2024
 *      Author: joshl
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

void loggerEmplaceU16(uint8_t* buffer, size_t addr, uint16_t data);
void loggerEmplaceU32(uint8_t* buffer, size_t addr, uint32_t data);
void loggerEmplaceCAN(uint8_t* buffer, size_t addr, uint8_t* msg);
#endif /* INC_LOGGER_H_ */

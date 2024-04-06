/*
 * logger.c
 *
 *  Created on: Mar 23, 2024
 *      Author: joshl
 */
#include "logger.h"

void loggerEmplaceU16(uint8_t* buffer, size_t addr, uint16_t data) {
	buffer[addr] = data >> 8;
	buffer[addr +1] = data & 0xff;
}

void loggerEmplaceU32(uint8_t* buffer, size_t addr, uint32_t data) {
	buffer[addr] = data >> 24;
	buffer[addr + 1] = data >> 16;
	buffer[addr + 2] = data >> 8;
	buffer[addr + 3] = data & 0xff;
}


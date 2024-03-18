/*
 * adc.c
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */
#include "adc.h"

static uint8_t buffer[2];

void adcInit() {
	buffer[0] = 0;
	buffer[1] = 0;
}

void adcEnable() {
	HAL_GPIO_WritePin(ADC_ENABLE);
}

void adcDisable() {
	HAL_GPIO_WritePin(ADC_DISABLE);
}

uint16_t getAnalog(SPI_HandleTypeDef* hspi, uint8_t channel) {

	uint8_t channelInput = channel << 3;
	HAL_SPI_Transmit(hspi, &channelInput, 1, 1000);
	HAL_SPI_Receive(hspi, buffer, 2, 1000);
	uint16_t output = buffer[1] << 8 | buffer[0]; // TODO do some thonking
	return output;
}

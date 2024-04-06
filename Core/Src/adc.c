/*
 * adc.c
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */
#include "adc.h"

static uint8_t buffer[2];
static uint8_t hehe = 0;
static uint16_t adc[8];
static size_t idx = 0;

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

	// request channel input, read don't care
	HAL_SPI_TransmitReceive(hspi, &channelInput, buffer, 1, 1000);
	HAL_SPI_Receive(hspi, (buffer + sizeof(uint8_t)), 1, 1000);

	// request ch0, read requested input
	HAL_SPI_TransmitReceive(hspi, &hehe, buffer, 1, 1000);
	HAL_SPI_Receive(hspi, (buffer + sizeof(uint8_t)), 1, 1000);
	uint16_t output = buffer[0] << 8 | buffer[1]; // TODO do some thonking
	return output;
}

uint16_t eGetAnalog(SPI_HandleTypeDef* hspi, uint8_t channel) {
	// request channel input, read previous request
	uint8_t channelInput = channel << 3;
	HAL_SPI_TransmitReceive(hspi, &channelInput, buffer, 1, 1000);
	HAL_SPI_Receive(hspi, (buffer + sizeof(uint8_t)), 1, 1000);
	adc[idx] = buffer[0] << 8 | buffer[1]; // update previous request
	idx = channel;
	return adc[idx]; // return previous conversion result
}

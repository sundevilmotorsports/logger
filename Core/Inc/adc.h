/*
 * adc.h
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

#define ADC_ENABLE  GPIOE, GPIO_PIN_1, GPIO_PIN_RESET
#define ADC_DISABLE GPIOE, GPIO_PIN_1, GPIO_PIN_SET

#define ADC_FBP 0
#define ADC_RBP 1
#define ADC_STP 2
#define ADC_FRS 3
#define ADC_FLS 4
#define ADC_CH5 5
#define ADC_RLS 6
#define ADC_RRS 7

void adcInit();
void adcEnable();
void adcDisable();
uint16_t getAnalog(SPI_HandleTypeDef* hspi, uint8_t channel);

#endif /* INC_ADC_H_ */

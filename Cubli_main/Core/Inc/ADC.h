#ifndef _ADC_H_
#define _ADC_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint32_t adc_buffer[3];

extern void ADCCalib();
extern void ADCInit();
extern void adc_readAll();

#endif

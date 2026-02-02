#ifndef _ADC_H_
#define _ADC_H_

#include "main.h"

#define ADC_CONV_FACTOR (3.3f / 4095.0f)

#define VSNS_VBAT_DIVIDER 0.124668 // (4.7f/(4.7f + 33.0f))
#define VSNS_12V_DIVIDER 0.124668 // (4.7f/(4.7.0f + 33.0f))

#define MOTOR_VSNS_DIVIDER (10.0f / (33.0f + 10.0f))

#define ISNS_AMP_GAIN 20.0f
#define ISNS_UVW_R 0.001f
#define ISNS_VBAT_R 0.005f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint32_t adc_buffer[3];

extern volatile float isns_u, isns_v, isns_w;
extern volatile float isns_v_offset, isns_w_offset;

extern void ADCCalib();
extern void ADCInit();
extern void adc_readAll();

#endif

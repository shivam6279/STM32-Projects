#ifndef _ADC_H_
#define _ADC_H_

#include "main.h"

#define ADC_CONV_FACTOR (3.3f / 4095.0f)

#define VSNS_VBAT_DIVIDER 8.0212765957444f // ((4.7f + 33.0f)/4.7f)
#define ISNS_VBAT_AMP_GAIN 50.0f
#define ISNS_VBAT_OFFSET 1.175f // (4.7f/(4.7f + 10.0f) * 3.3f

#define MOTOR_VSNS_DIVIDER 4.3f // ((33.0f + 10.0f) / 10.0f)

#define ISNS_AMP_GAIN 20.0f
#define ISNS_UVW_R 0.001f
#define ISNS_VBAT_R 0.005f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint32_t adc_buffer[3];

extern volatile float isns_u, isns_v, isns_w;
extern volatile float isns_v_offset, isns_w_offset;

extern volatile float isns_vbat, vsns_vbat;
extern volatile float vsns_u, vsns_v, vsns_w, vsns_x;

extern void ADCCalib();
extern void ADCInit();
extern void adc_readAll();

#endif

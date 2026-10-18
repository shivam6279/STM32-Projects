#ifndef _ADC_H_
#define _ADC_H_

#include <stdbool.h>
#include <inttypes.h>

// Pressure sensors PRESS0 (PA4, ADC1_INP18) and PRESS1 (PA5, ADC1_INP19).
// XGZP6857A100KPGPN33, -100..100 kPa gauge, 3.3V, 0.2-2.7V out. At ambient both read the
// datasheet's 1.45V zero, so no divider in front of the pin.
// ADC1 runs continuously when started, circular DMA drops each result straight into
// adc_raw[]. Off most of the time: AdcStart() / AdcStop().

#define ADC_CHANNELS	2
#define ADC_VREF		3.3f	// TODO check VREF+ on the schematic

extern volatile uint32_t adc_raw[ADC_CHANNELS];	// 16 bit results, written by DMA

void AdcInit();
void AdcStart();
void AdcStop();
bool AdcRunning();
float AdcVolts(uint8_t ch);
float PressKpa(uint8_t ch);

#endif

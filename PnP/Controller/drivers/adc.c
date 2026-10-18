#include "adc.h"
#include "clock.h"
#include "mpu.h"
#include "stm32h7xx.h"

// Sequence, sampling and oversampling. Timing isn't critical, so make it slow and quiet:
// 810.5 cycle sampling (max, tolerates a high source impedance) and 16x oversampling.
// adc_ker_ck = HSE 25MHz (per_ck) / 2 = 12.5MHz -> ~1ms per channel, ~475Hz per pair.
#define ADC_SMP			7		// 810.5 cycles
#define ADC_OVS_RATIO	16
#define ADC_OVS_SHIFT	4		// 16x sum >> 4 = back to 16 bits

#define ADC_DMA_REQ		9		// LL_DMAMUX1_REQ_ADC1

static const uint8_t channels[ADC_CHANNELS] = { 18, 19 };	// INP18 = PRESS0 (PA4), INP19 = PRESS1 (PA5)

volatile uint32_t adc_raw[ADC_CHANNELS] DMA_BUF;
static bool running = false;

static void Wait(uint32_t ms) {
	uint32_t t = Millis();
	while (Millis() - t <= ms);
}

void AdcInit() {
	uint8_t i;

	// per_ck = HSE, ADC kernel clock = per_ck (LL_RCC_CLKP_CLKSOURCE_HSE, ADCSEL = 2)
	RCC->D1CCIPR = (RCC->D1CCIPR & ~RCC_D1CCIPR_CKPERSEL) | (2 << RCC_D1CCIPR_CKPERSEL_Pos);
	RCC->D3CCIPR = (RCC->D3CCIPR & ~RCC_D3CCIPR_ADCSEL) | (2 << RCC_D3CCIPR_ADCSEL_Pos);
	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN | RCC_AHB1ENR_DMA1EN;
	(void)RCC->AHB1ENR;

	// Async clock / 2 = 12.5MHz
	ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~(ADC_CCR_CKMODE | ADC_CCR_PRESC)) | (1 << ADC_CCR_PRESC_Pos);

	// Out of deep power down, regulator on, 10us to settle (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US)
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	ADC1->CR |= ADC_CR_ADVREGEN;
	Wait(1);

	// BOOST: HAL ADC_ConfigureBoostMode for ADC_VER_V5_V90 uses adc_ker_ck / 2 = 6.25MHz -> 0
	ADC1->CR &= ~ADC_CR_BOOST;

	// Offset + linearity calibration, single ended, ADC disabled
	ADC1->CR |= ADC_CR_ADCALLIN;
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	// 16 bit, continuous, circular DMA, overwrite on overrun, software start
	ADC1->CFGR = (3 << ADC_CFGR_DMNGT_Pos) | ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
	ADC1->CFGR2 = ADC_CFGR2_ROVSE | ((ADC_OVS_RATIO - 1) << ADC_CFGR2_OVSR_Pos) | (ADC_OVS_SHIFT << ADC_CFGR2_OVSS_Pos);

	ADC1->SQR1 = ((ADC_CHANNELS - 1) << ADC_SQR1_L_Pos) | (channels[0] << ADC_SQR1_SQ1_Pos) | (channels[1] << ADC_SQR1_SQ2_Pos);
	ADC1->PCSEL_RES0 = 0;
	for (i = 0; i < ADC_CHANNELS; i++) {
		ADC1->PCSEL_RES0 |= 1 << channels[i];	// channel preselection, needed on H7
		ADC1->SMPR2 = (ADC1->SMPR2 & ~(7 << ((channels[i] - 10) * 3))) | (ADC_SMP << ((channels[i] - 10) * 3));
	}

	// DMA1 Stream1: ADC1->DR -> adc_raw[], 32 bit, circular, no interrupts
	DMA1_Stream1->CR = 0;
	while (DMA1_Stream1->CR & DMA_SxCR_EN);
	DMA1_Stream1->PAR = (uint32_t)&ADC1->DR;
	DMA1_Stream1->M0AR = (uint32_t)adc_raw;
	DMA1_Stream1->FCR = 0;
	DMA1_Stream1->CR = DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_CIRC;	// DIR = 00, periph -> mem
	DMAMUX1_Channel1->CCR = ADC_DMA_REQ;

	for (i = 0; i < ADC_CHANNELS; i++) adc_raw[i] = 0;
	running = false;
}

void AdcStart() {
	if (running) return;

	// Re-arm the DMA at element 0 so the sequence and adc_raw[] can't get out of step
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream1->CR & DMA_SxCR_EN);
	DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
	DMA1_Stream1->NDTR = ADC_CHANNELS;
	DMA1_Stream1->CR |= DMA_SxCR_EN;

	ADC1->ISR = ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;
	running = true;
}

void AdcStop() {
	if (!running) return;

	ADC1->CR |= ADC_CR_ADSTP;
	while (ADC1->CR & ADC_CR_ADSTART);
	ADC1->CR |= ADC_CR_ADDIS;
	while (ADC1->CR & ADC_CR_ADEN);
	running = false;
}

bool AdcRunning() {
	return running;
}

float AdcVolts(uint8_t ch) {
	if (ch >= ADC_CHANNELS) return 0;
	return adc_raw[ch] * ADC_VREF / 65535.0f;
}

// XGZP6857A100KPGPN33 (datasheet V2.8, "Vacuum Pressure to Positive Pressure", 3.3V):
// Pressure = (output - 1.45) / K, K = 0.0125 V/kPa for -100..100 kPa (0.2V..2.7V).
// Output is ratiometric to the sensor supply, so this is exact if the sensor VDD and VREF+
// are the same 3.3V rail. TODO confirm on the schematic.
#define PRESS_ZERO_V	1.45f
#define PRESS_K			0.0125f

float PressKpa(uint8_t ch) {
	if (ch >= ADC_CHANNELS) return 0;
	return (AdcVolts(ch) - PRESS_ZERO_V) / PRESS_K;
}

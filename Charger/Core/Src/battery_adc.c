// 3S cell-tap measurement on PA6/7/8, analog-enable PA11. See battery_adc.h.

#include "battery_adc.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;     // configured by MX_ADC1_Init() in main.c

// Analog-enable line: HIGH makes the tap voltages appear.
#define BATT_EN_PORT     GPIOA
#define BATT_EN_PIN      GPIO_PIN_11

static const uint32_t s_tap_channel[3] = {
	ADC_CHANNEL_6,    // PA6 -> tab1
	ADC_CHANNEL_7,    // PA7 -> tab2
	ADC_CHANNEL_8,    // PA8 -> tab3
};

void BatteryADC_Init(void) {
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	// PA6/PA7/PA8 as analog inputs.
	gpio.Pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	// PA11 analog-enable: push-pull output, start LOW (dividers off).
	HAL_GPIO_WritePin(BATT_EN_PORT, BATT_EN_PIN, GPIO_PIN_RESET);
	gpio.Pin   = BATT_EN_PIN;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BATT_EN_PORT, &gpio);

	// Calibrate the ADC once (must be done while the ADC is disabled).
	HAL_ADCEx_Calibration_Start(&hadc1);
}

void BatteryADC_Sleep(void) {
	// Ensure the dividers are off before standby so they don't drain the pack.
	HAL_GPIO_WritePin(BATT_EN_PORT, BATT_EN_PIN, GPIO_PIN_RESET);
}

// Convert a single tap channel; returns the raw 12-bit count.
static uint16_t read_channel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = {0};

	/* Select exactly this one channel (clear any previously-selected ones;
	 * in this sequencer mode HAL_ADC_ConfigChannel ORs into CHSELR). */
	hadc1.Instance->CHSELR = 0u;
	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	uint16_t v = 0u;
	if (HAL_ADC_Start(&hadc1) == HAL_OK) {
		if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK) {
			v = (uint16_t)HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);
	}
	return v;
}

void BatteryADC_Read(batt_cells_t *out) {
	static const float scaler[3] = { TAB1_SCALER, TAB2_SCALER, TAB3_SCALER };

	// Enable the analog network and let the divider low-pass caps charge.
	HAL_GPIO_WritePin(BATT_EN_PORT, BATT_EN_PIN, GPIO_PIN_SET);
	HAL_Delay(BATT_ADC_SETTLE_MS);

	for (int i = 0; i < 3; i++) {
		uint32_t acc = 0u;
		for (uint32_t n = 0u; n < BATT_ADC_AVG_SAMPLES; n++) {
			acc += read_channel(s_tap_channel[i]);
		}
		out->raw[i]    = (uint16_t)((acc + BATT_ADC_AVG_SAMPLES / 2u)
								  / BATT_ADC_AVG_SAMPLES);
		out->tab_mV[i] = (uint16_t)((float)out->raw[i] * scaler[i] + 0.5f);
	}

	// Done sampling: turn the dividers back off.
	HAL_GPIO_WritePin(BATT_EN_PORT, BATT_EN_PIN, GPIO_PIN_RESET);

	/* Per-cell voltages from the cumulative taps (clamp against noise so a
	 * tiny negative difference doesn't underflow the unsigned result). */
	out->cell_mV[0] = out->tab_mV[0];
	out->cell_mV[1] = (out->tab_mV[1] > out->tab_mV[0])
					? (uint16_t)(out->tab_mV[1] - out->tab_mV[0]) : 0u;
	out->cell_mV[2] = (out->tab_mV[2] > out->tab_mV[1])
					? (uint16_t)(out->tab_mV[2] - out->tab_mV[1]) : 0u;
}

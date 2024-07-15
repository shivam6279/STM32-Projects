#include "tones.h"
#include "stm32.h"
#include "BLDC.h"
#include "stm32g4xx_it.h"
#include <math.h>

#define C3	C4 / 2
#define CS3	CS4 / 2
#define D3	D4 / 2
#define DS3	DS4 / 2
#define E3	E4 / 2
#define F3	F4 / 2
#define FS3	FS4 / 2
#define G3	G4 / 2
#define GS3	GS4 / 2
#define A3	A4 / 2
#define AS3	AS4 / 2
#define B3	B4 / 2

#define C4	261.63
#define CS4	277.18
#define D4	293.66
#define DS4	311.13
#define E4	329.63
#define F4	349.23
#define FS4	369.99
#define G4	392
#define GS4	415.3
#define A4	440
#define AS4	466.16
#define B4	493.88

#define C5	C4 * 2
#define CS5	CS4 * 2
#define D5	D4 * 2
#define DS5	DS4 * 2
#define E5	E4 * 2
#define F5	F4 * 2
#define FS5	FS4 * 2
#define G5	G4 * 2
#define GS5	GS4 * 2
#define A5	A4 * 2
#define AS5	AS4 * 2
#define B5	B4 * 2

#define C6	C5 * 2
#define CS6	CS5 * 2
#define D6	D5 * 2
#define DS6	DS5 * 2
#define E6	E5 * 2
#define F6	F5 * 2
#define FS6	FS5 * 2
#define G6	G5 * 2
#define GS6	GS5 * 2
#define A6	A5 * 2
#define AS6	AS5 * 2
#define B6	B5 * 2

static float music_tones[] = {D5, F5, D5, C5, A4, 0, A4};
static float music_delays[] = {600, 600, 600, 600, 1000, 200, 400, 0};

void TIM4_IRQHandler(void) {
	static uint8_t phase = 1;
	if(TIM4->SR & 0x1){
		TIM4->SR &= ~(0x1);
		BLDC_phase(phase + 1, TONE_POWER);
//		setPhaseVoltage(0.15, phase*90);
		phase ^= 1;
	}
}

void play_tone(float tone) {
	motor_on();
	TIM4_on();
	TIM4_update_freq(tone*2);
}

void play_music() {
	uint16_t i = 0;
	motor_on();
	TIM4_on();
	do {
		if(music_tones[i] < 0.5) {
			tone_off();
		} else {
			play_tone(music_tones[i]);
		}
		delay_ms(music_delays[i]);
		i++;
	}while(music_delays[i] != 0);
	tone_off();
}

void tone_off() {
	TIM4_off();
	motor_off();
}

void TIM4_update_freq(float freq) {
//	uint32_t flag = TIM4->CR1 & 0b10;
//	TIM4->CR1 &= ~(1 << 1);

    float f = (float)(SYSCLK_FREQ/100) / freq;
    // TIM4->ARR |= ((uint32_t)f & 0x000FFFFF);
    TIM4->ARR = (uint32_t)f;
    TIM4->PSC = 100 - 1;

    TIM4->CNT = 0;

//    TIM4->CR1 |= flag << 1;
}

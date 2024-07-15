#include "BLDC.h"
#include "main.h"
#include "stm32.h"
#include "stm32g4xx_it.h"
#include <math.h>

float SVPWM_table[SVPWM_SIZE] = {0, 13.2254525, 26.44989783, 39.6723289, 52.89173877, 66.10712072, 79.31746837, 92.52177568, 105.7190371, 118.9082476, 132.0884028, 145.2584989, 158.4175331, 171.5645031, 184.6984078, 197.8182471, 210.9230217, 224.0117337, 237.0833863, 250.1369841, 263.1715331, 276.1860405, 289.1795152, 302.1509678, 315.0994105, 328.0238571, 340.9233234, 353.7968271, 366.6433878, 379.4620272, 392.2517691, 405.0116395, 417.7406666, 430.4378813, 443.1023164, 455.7330075, 468.3289928, 480.8893131, 493.4130117, 505.8991351, 518.3467323, 530.7548553, 543.1225594, 555.4489025, 567.7329461, 579.9737546, 592.1703958, 604.321941, 616.4274647, 628.4860451, 640.4967638, 652.4587063, 664.3709615, 676.2326222, 688.0427852, 699.8005511, 711.5050245, 723.155314, 734.7505325, 746.2897968, 757.7722283, 761.5612339, 765.2922437, 768.9649736, 772.5791438, 776.134479, 779.6307087, 783.0675664, 786.4447905, 789.7621238, 793.0193137, 796.216112, 799.3522754, 802.4275651, 805.4417468, 808.3945909, 811.2858727, 814.115372, 816.8828732, 819.5881656, 822.2310432, 824.8113047, 827.3287536, 829.7831983, 832.1744518, 834.5023319, 836.7666615, 838.967268, 841.1039839, 843.1766466, 845.185098, 847.1291853, 849.0087605, 850.8236803, 852.5738067, 854.2590062, 855.8791506, 857.4341165, 858.9237855, 860.3480441, 861.7067839, 862.9999013, 864.227298, 865.3888804, 866.4845601, 867.5142537, 868.4778827, 869.3753737, 870.2066584, 870.9716736, 871.6703608, 872.302667, 872.868544, 873.3679486, 873.8008429, 874.1671939, 874.4669736, 874.7001594, 874.8667333, 874.9666827, 875, 874.9666827, 874.8667333, 874.7001594, 874.4669736, 874.1671939, 873.8008429, 873.3679486, 872.868544, 872.302667, 871.6703608, 870.9716736, 870.2066584, 869.3753737, 868.4778827, 867.5142537, 866.4845601, 865.3888804, 864.227298, 862.9999013, 861.7067839, 860.3480441, 858.9237855, 857.4341165, 855.8791506, 854.2590062, 852.5738067, 850.8236803, 849.0087605, 847.1291853, 845.185098, 843.1766466, 841.1039839, 838.967268, 836.7666615, 834.5023319, 832.1744518, 829.7831983, 827.3287536, 824.8113047, 822.2310432, 819.5881656, 816.8828732, 814.115372, 811.2858727, 808.3945909, 805.4417468, 802.4275651, 799.3522754, 796.216112, 793.0193137, 789.7621238, 786.4447905, 783.0675664, 779.6307087, 776.134479, 772.5791438, 768.9649736, 765.2922437, 761.5612339, 757.7722283, 761.5612339, 765.2922437, 768.9649736, 772.5791438, 776.134479, 779.6307087, 783.0675664, 786.4447905, 789.7621238, 793.0193137, 796.216112, 799.3522754, 802.4275651, 805.4417468, 808.3945909, 811.2858727, 814.115372, 816.8828732, 819.5881656, 822.2310432, 824.8113047, 827.3287536, 829.7831983, 832.1744518, 834.5023319, 836.7666615, 838.967268, 841.1039839, 843.1766466, 845.185098, 847.1291853, 849.0087605, 850.8236803, 852.5738067, 854.2590062, 855.8791506, 857.4341165, 858.9237855, 860.3480441, 861.7067839, 862.9999013, 864.227298, 865.3888804, 866.4845601, 867.5142537, 868.4778827, 869.3753737, 870.2066584, 870.9716736, 871.6703608, 872.302667, 872.868544, 873.3679486, 873.8008429, 874.1671939, 874.4669736, 874.7001594, 874.8667333, 874.9666827, 875, 874.9666827, 874.8667333, 874.7001594, 874.4669736, 874.1671939, 873.8008429, 873.3679486, 872.868544, 872.302667, 871.6703608, 870.9716736, 870.2066584, 869.3753737, 868.4778827, 867.5142537, 866.4845601, 865.3888804, 864.227298, 862.9999013, 861.7067839, 860.3480441, 858.9237855, 857.4341165, 855.8791506, 854.2590062, 852.5738067, 850.8236803, 849.0087605, 847.1291853, 845.185098, 843.1766466, 841.1039839, 838.967268, 836.7666615, 834.5023319, 832.1744518, 829.7831983, 827.3287536, 824.8113047, 822.2310432, 819.5881656, 816.8828732, 814.115372, 811.2858727, 808.3945909, 805.4417468, 802.4275651, 799.3522754, 796.216112, 793.0193137, 789.7621238, 786.4447905, 783.0675664, 779.6307087, 776.134479, 772.5791438, 768.9649736, 765.2922437, 761.5612339, 757.7722283, 746.2897968, 734.7505325, 723.155314, 711.5050245, 699.8005511, 688.0427852, 676.2326222, 664.3709615, 652.4587063, 640.4967638, 628.4860451, 616.4274647, 604.321941, 592.1703958, 579.9737546, 567.7329461, 555.4489025, 543.1225594, 530.7548553, 518.3467323, 505.8991351, 493.4130117, 480.8893131, 468.3289928, 455.7330075, 443.1023164, 430.4378813, 417.7406666, 405.0116395, 392.2517691, 379.4620272, 366.6433878, 353.7968271, 340.9233234, 328.0238571, 315.0994105, 302.1509678, 289.1795152, 276.1860405, 263.1715331, 250.1369841, 237.0833863, 224.0117337, 210.9230217, 197.8182471, 184.6984078, 171.5645031, 158.4175331, 145.2584989, 132.0884028, 118.9082476, 105.7190371, 92.52177568, 79.31746837, 66.10712072, 52.89173877, 39.6723289, 26.44989783, 13.2254525, 1.23784E-13, -13.2254525, -26.44989783, -39.6723289, -52.89173877, -66.10712072, -79.31746837, -92.52177568, -105.7190371, -118.9082476, -132.0884028, -145.2584989, -158.4175331, -171.5645031, -184.6984078, -197.8182471, -210.9230217, -224.0117337, -237.0833863, -250.1369841, -263.1715331, -276.1860405, -289.1795152, -302.1509678, -315.0994105, -328.0238571, -340.9233234, -353.7968271, -366.6433878, -379.4620272, -392.2517691, -405.0116395, -417.7406666, -430.4378813, -443.1023164, -455.7330075, -468.3289928, -480.8893131, -493.4130117, -505.8991351, -518.3467323, -530.7548553, -543.1225594, -555.4489025, -567.7329461, -579.9737546, -592.1703958, -604.321941, -616.4274647, -628.4860451, -640.4967638, -652.4587063, -664.3709615, -676.2326222, -688.0427852, -699.8005511, -711.5050245, -723.155314, -734.7505325, -746.2897968, -757.7722283, -761.5612339, -765.2922437, -768.9649736, -772.5791438, -776.134479, -779.6307087, -783.0675664, -786.4447905, -789.7621238, -793.0193137, -796.216112, -799.3522754, -802.4275651, -805.4417468, -808.3945909, -811.2858727, -814.115372, -816.8828732, -819.5881656, -822.2310432, -824.8113047, -827.3287536, -829.7831983, -832.1744518, -834.5023319, -836.7666615, -838.967268, -841.1039839, -843.1766466, -845.185098, -847.1291853, -849.0087605, -850.8236803, -852.5738067, -854.2590062, -855.8791506, -857.4341165, -858.9237855, -860.3480441, -861.7067839, -862.9999013, -864.227298, -865.3888804, -866.4845601, -867.5142537, -868.4778827, -869.3753737, -870.2066584, -870.9716736, -871.6703608, -872.302667, -872.868544, -873.3679486, -873.8008429, -874.1671939, -874.4669736, -874.7001594, -874.8667333, -874.9666827, -875, -874.9666827, -874.8667333, -874.7001594, -874.4669736, -874.1671939, -873.8008429, -873.3679486, -872.868544, -872.302667, -871.6703608, -870.9716736, -870.2066584, -869.3753737, -868.4778827, -867.5142537, -866.4845601, -865.3888804, -864.227298, -862.9999013, -861.7067839, -860.3480441, -858.9237855, -857.4341165, -855.8791506, -854.2590062, -852.5738067, -850.8236803, -849.0087605, -847.1291853, -845.185098, -843.1766466, -841.1039839, -838.967268, -836.7666615, -834.5023319, -832.1744518, -829.7831983, -827.3287536, -824.8113047, -822.2310432, -819.5881656, -816.8828732, -814.115372, -811.2858727, -808.3945909, -805.4417468, -802.4275651, -799.3522754, -796.216112, -793.0193137, -789.7621238, -786.4447905, -783.0675664, -779.6307087, -776.134479, -772.5791438, -768.9649736, -765.2922437, -761.5612339, -757.7722283, -761.5612339, -765.2922437, -768.9649736, -772.5791438, -776.134479, -779.6307087, -783.0675664, -786.4447905, -789.7621238, -793.0193137, -796.216112, -799.3522754, -802.4275651, -805.4417468, -808.3945909, -811.2858727, -814.115372, -816.8828732, -819.5881656, -822.2310432, -824.8113047, -827.3287536, -829.7831983, -832.1744518, -834.5023319, -836.7666615, -838.967268, -841.1039839, -843.1766466, -845.185098, -847.1291853, -849.0087605, -850.8236803, -852.5738067, -854.2590062, -855.8791506, -857.4341165, -858.9237855, -860.3480441, -861.7067839, -862.9999013, -864.227298, -865.3888804, -866.4845601, -867.5142537, -868.4778827, -869.3753737, -870.2066584, -870.9716736, -871.6703608, -872.302667, -872.868544, -873.3679486, -873.8008429, -874.1671939, -874.4669736, -874.7001594, -874.8667333, -874.9666827, -875, -874.9666827, -874.8667333, -874.7001594, -874.4669736, -874.1671939, -873.8008429, -873.3679486, -872.868544, -872.302667, -871.6703608, -870.9716736, -870.2066584, -869.3753737, -868.4778827, -867.5142537, -866.4845601, -865.3888804, -864.227298, -862.9999013, -861.7067839, -860.3480441, -858.9237855, -857.4341165, -855.8791506, -854.2590062, -852.5738067, -850.8236803, -849.0087605, -847.1291853, -845.185098, -843.1766466, -841.1039839, -838.967268, -836.7666615, -834.5023319, -832.1744518, -829.7831983, -827.3287536, -824.8113047, -822.2310432, -819.5881656, -816.8828732, -814.115372, -811.2858727, -808.3945909, -805.4417468, -802.4275651, -799.3522754, -796.216112, -793.0193137, -789.7621238, -786.4447905, -783.0675664, -779.6307087, -776.134479, -772.5791438, -768.9649736, -765.2922437, -761.5612339, -757.7722283, -746.2897968, -734.7505325, -723.155314, -711.5050245, -699.8005511, -688.0427852, -676.2326222, -664.3709615, -652.4587063, -640.4967638, -628.4860451, -616.4274647, -604.321941, -592.1703958, -579.9737546, -567.7329461, -555.4489025, -543.1225594, -530.7548553, -518.3467323, -505.8991351, -493.4130117, -480.8893131, -468.3289928, -455.7330075, -443.1023164, -430.4378813, -417.7406666, -405.0116395, -392.2517691, -379.4620272, -366.6433878, -353.7968271, -340.9233234, -328.0238571, -315.0994105, -302.1509678, -289.1795152, -276.1860405, -263.1715331, -250.1369841, -237.0833863, -224.0117337, -210.9230217, -197.8182471, -184.6984078, -171.5645031, -158.4175331, -145.2584989, -132.0884028, -118.9082476, -105.7190371, -92.52177568, -79.31746837, -66.10712072, -52.89173877, -39.6723289, -26.44989783, -13.2254525};

int32_t motor_zero_offset = 327;
volatile float pre_pos = 0.0, position = 0, rpm = 0, power = 0;
volatile uint8_t motor_mode = 0;

volatile uint8_t current_phase = 1;
volatile float phase_timing = 0;
volatile uint8_t zero_crossing_flag = 0;

//FOC Loop
void TIM3_IRQHandler(void) {
	static int32_t cnt, pre_cnt;
	static float diff;
	if(TIM3->SR & 0x1){
		TIM3->SR &= ~(0x1);

//		GPIOB->BSRR |= 1 << 3;

		pre_cnt = cnt;
		cnt = (int16_t)TIM8->CNT;
		cnt -= motor_zero_offset;
		if(cnt < 0) {
			cnt += ENCODER_RES;
		}
		cnt = cnt % ENCODER_RES;
//		cnt &= 0x7FFF;

		diff = ((float)cnt - (float)pre_cnt) / ENCODER_RES * 20000.0f * 60.0f;
//		if(diff > ENCODER_RES/2) {
//			diff -= ENCODER_RES;
//		} else if(diff < -ENCODER_RES/2) {
//			diff += ENCODER_RES;
//		}

		if(fabs(diff) > 10000) {
			diff = rpm;
		}
		rpm = RPM_LPF * rpm + (1.0-RPM_LPF) * diff;

		position = (float)cnt * 360.0f / ENCODER_RES;
		if(position < 0) {
			position += 360.0;
		}

		if(motor_mode) {
			setPhaseVoltage(power, position * POLE_PAIRS + PHASE_DIFF);
		}

//		GPIOB->BSRR |= 1 << 19;
	}
}

void TIM1_CC_IRQHandler(void) {
	if(TIM1->SR & 1 << 4){
		TIM1->SR &= ~(1 << 4);
		if(get_COMP_value(current_phase) && TIM2->CNT > 25) {
//			phase_timing = TIM2->CNT;
			phase_timing = 0.9 * phase_timing + 0.1 * (float)TIM2->CNT;

			zero_crossing_flag = 1;

			GPIOB->BSRR |= 1 << 3;

			TIM2->CNT = 0;
//			TIM2-> ARR = phase_timing;
//			TIM2->DIER |= 1;

			TIM1->DIER &= ~(1 << 4);
		}
	}
}

void TIM2_IRQHandler(void) {
	if(TIM2->SR & 0x1){
		TIM2->SR &= ~(0x1);

//		current_phase = ((++current_phase - 1) % 6 + 1);
	}
}

inline void setPhaseVoltage(float p, float angle_el) {
    static float pwm_u, pwm_v, pwm_w;

    p = p < -1.0 ? -1.0 : p > 1.0 ? 1.0 : p;
//    p = -p;

	static int index;

	if(p < 0) {
		angle_el += 180;
		p = -p;
	}

	angle_el *= 2;
	index = angle_el;
	index = index % 720;

//	angle_el = fmod(angle_el, 360.0);
//	if(angle_el < 0) {
//		angle_el += 360.0;
//	}
//	angle_el *= 2;
//	index = angle_el;
//	index = (index < 0.0) | (index >= SVPWM_SIZE) ? 0 : index;



	pwm_u = 0.5 * SVPWM_table[index] * p;

	index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
	pwm_v = 0.5 * SVPWM_table[index] * p;

	index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
	pwm_w = 0.5 * SVPWM_table[index] * p;

	static float center;
	center = MAX_PWM / 2.0;
//	static float Umin, Umax;
//	Umin = fmin(pwm_u, fmin(pwm_v, pwm_w));
//	Umax = fmax(pwm_u, fmax(pwm_v, pwm_w));
//	center -= (Umax+Umin) / 2;
	pwm_u += center;
	pwm_v += center;
	pwm_w += center;

	set_motor_pwm(pwm_u, pwm_w, pwm_v);
}

void BLDC_phase(unsigned char phase, float p) {
	if(phase == 1) {
		TIM1->CCER |= TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = p * MAX_PWM;
		TIM1->CCR3 = 0;
	} else if(phase == 2) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER |= TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = p * MAX_PWM;
		TIM1->CCR3 = 0;
	} else if(phase == 3) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER |= TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = p * MAX_PWM;
	} else if(phase == 4) {
 		TIM1->CCER |= TIM_CCER_CC1NP;
 		TIM1->CCER &= ~TIM_CCER_CC2NP;
 		TIM1->CCER &= ~TIM_CCER_CC3NP;
 		TIM1->EGR |= TIM_EGR_COMG;
 		TIM1->CCR1 = 0;
 		TIM1->CCR2 = 0;
 		TIM1->CCR3 = p * MAX_PWM;
	} else if(phase == 5) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER |= TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = p * MAX_PWM;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	} else if(phase == 6) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER |= TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = p * MAX_PWM;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
}

void BEMF_phase(uint8_t phase) {
	COMP1->CSR &= ~1;
	COMP2->CSR &= ~1;

	// W falling
	if(phase == 1) {
		EXTI->IMR1 &= ~(1 << 21);
		EXTI->IMR1 |= 1 << 22;

		EXTI->RTSR1 &= ~(1 << 22);
		EXTI->FTSR1 |= 1 << 22;

		COMP2->CSR |= 0b1;
	}

	//U rising
	else if(phase == 2) {
		EXTI->IMR1 |= 1 << 21;
		EXTI->IMR1 &= ~(1 << 22);

		COMP1->CSR |= (0b111 << 4);

		EXTI->RTSR1 |= 1 << 21;
		EXTI->FTSR1 &= ~(1 << 21);

		COMP1->CSR |= 0b1;
	}

	//V falling
	else if(phase == 3) {
		EXTI->IMR1 |= 1 << 21;
		EXTI->IMR1 &= ~(1 << 22);

		COMP1->CSR |= (0b110 << 4);
		COMP1->CSR &= ~(0b001 << 4);

		EXTI->RTSR1 &= ~(1 << 21);
		EXTI->FTSR1 |= 1 << 21;

		COMP1->CSR |= 0b1;
	}

	//W rising
	else if(phase == 4) {
		EXTI->IMR1 &= ~(1 << 21);
		EXTI->IMR1 |= 1 << 22;

		EXTI->RTSR1 |= 1 << 22;
		EXTI->FTSR1 &= ~(1 << 22);

		COMP2->CSR |= 0b1;
	}

	//U falling
	else if(phase == 5) {
		EXTI->IMR1 |= 1 << 21;
		EXTI->IMR1 &= ~(1 << 22);

		COMP1->CSR |= (0b111 << 4);

		EXTI->RTSR1 &= ~(1 << 21);
		EXTI->FTSR1 |= 1 << 21;

		COMP1->CSR |= 0b1;
	}

	//V rising
	else if(phase == 6) {
		EXTI->IMR1 |= 1 << 21;
		EXTI->IMR1 &= ~(1 << 22);

		COMP1->CSR |= (0b110 << 4);
		COMP1->CSR &= ~(0b001 << 4);

		EXTI->RTSR1 |= 1 << 21;
		EXTI->FTSR1 &= ~(1 << 21);

		COMP1->CSR |= 0b1;
	}
}

uint8_t get_COMP_value(uint8_t phase) {
	uint32_t t;
	if(phase == 1) {
		t = ~COMP2->CSR;
	} else if(phase == 2) {
		t = ~COMP1->CSR;
	} else if(phase == 3) {
		t = COMP1->CSR;
	} else if(phase == 4) {
		t = COMP2->CSR;
	} else if(phase == 5) {
		t = COMP1->CSR;
	} else if(phase == 6) {
		t = ~COMP1->CSR;
	} else {
		return 0;
	}
	return (t >> 30) & 1;
}

void set_motor_pwm(uint16_t u, uint16_t v, uint16_t w) {
	u = u < 0 ? 0 : u;
	u = u > MAX_PWM ? MAX_PWM : u;

	v = v < 0 ? 0 : v;
	v = v > MAX_PWM ? MAX_PWM : v;

	w = w < 0 ? 0 : w;
	w = w > MAX_PWM ? MAX_PWM : w;

	TIM1->CCR1 = u;
	TIM1->CCR2 = v;
	TIM1->CCR3 = w;
}

void motor_on() {
	TIM1->CCER &= ~TIM_CCER_CC1NP;
	TIM1->CCER &= ~TIM_CCER_CC2NP;
	TIM1->CCER &= ~TIM_CCER_CC3NP;
}

void motor_off() {
	TIM1->CCER |= TIM_CCER_CC1NP;
	TIM1->CCER |= TIM_CCER_CC2NP;
	TIM1->CCER |= TIM_CCER_CC3NP;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}
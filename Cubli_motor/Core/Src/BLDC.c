#include "main.h"

#include <inttypes.h>
#include "BLDC.h"
#include "ADC.h"
#include "PWM.h"
#include <math.h>
#include "PID.h"
#include "cogging.h"

#define MOTOR_OV_INITIAL 20.0f // volts

#define _SQRT3_2 0.86602540378443864676372317075294f
#define _1_SQRT3 0.57735026918962576450914878050196f
#define _2_SQRT3 1.1547005383792515290182975610039f

static inline void vel_kf_update(int16_t, float);
static inline void adc_read_motor_isns();
static inline void adc_read_other();
static inline void foc_current_calc(float);
static inline float wave_lut(uint16_t*, float);
static inline float fsignf(float x);

#define DEAD_TIME 25

float motor_pole_pairs = 7;
uint8_t motor_direction = 0; // Motor wiirng: 1 for reverse
uint8_t enc_direction = 0; // Encoder direction: 1 for reverse

// SVPWM
static uint16_t SVPWM_table[LUT_SIZE] = {0, 247, 495, 743, 991, 1239, 1487, 1735, 1983, 2231, 2479, 2727, 2975, 3223, 3471, 3719, 3966, 4214, 4462, 4710, 4958, 5205, 5453, 5701, 5948, 6196, 6444, 6691, 6939, 7186, 7434, 7681, 7929, 8176, 8423, 8671, 8918, 9165, 9412, 9659, 9906, 10153, 10400, 10647, 10894, 11141, 11388, 11634, 11881, 12128, 12374, 12621, 12867, 13113, 13360, 13606, 13852, 14098, 14344, 14590, 14836, 15082, 15328, 15573, 15819, 16064, 16310, 16555, 16801, 17046, 17291, 17536, 17781, 18026, 18271, 18515, 18760, 19004, 19249, 19493, 19737, 19982, 20226, 20470, 20713, 20957, 21201, 21445, 21688, 21931, 22175, 22418, 22661, 22904, 23147, 23389, 23632, 23874, 24117, 24359, 24601, 24843, 25085, 25327, 25569, 25810, 26052, 26293, 26534, 26775, 27016, 27257, 27497, 27738, 27978, 28219, 28459, 28699, 28939, 29178, 29418, 29657, 29897, 30136, 30375, 30614, 30852, 31091, 31329, 31568, 31806, 32044, 32282, 32519, 32757, 32994, 33231, 33468, 33705, 33942, 34179, 34415, 34651, 34887, 35123, 35359, 35594, 35830, 36065, 36300, 36535, 36770, 37004, 37239, 37473, 37707, 37941, 38174, 38408, 38641, 38874, 39107, 39340, 39572, 39804, 40037, 40269, 40500, 40732, 40963, 41194, 41425, 41656, 41887, 42117, 42347, 42577, 42807, 43037, 43266, 43495, 43724, 43953, 44181, 44410, 44638, 44866, 45093, 45321, 45548, 45775, 46002, 46229, 46455, 46681, 46907, 47133, 47358, 47583, 47809, 48033, 48258, 48482, 48706, 48930, 49154, 49377, 49600, 49823, 50046, 50269, 50491, 50713, 50934, 51156, 51377, 51598, 51819, 52039, 52260, 52480, 52700, 52919, 53138, 53357, 53576, 53795, 54013, 54231, 54449, 54666, 54883, 55100, 55317, 55533, 55749, 55965, 56181, 56396, 56611, 56778, 56850, 56921, 56992, 57062, 57132, 57202, 57272, 57342, 57411, 57480, 57548, 57617, 57685, 57753, 57820, 57887, 57954, 58021, 58087, 58154, 58219, 58285, 58350, 58415, 58480, 58545, 58609, 58673, 58736, 58800, 58863, 58926, 58988, 59050, 59112, 59174, 59235, 59297, 59357, 59418, 59478, 59538, 59598, 59657, 59716, 59775, 59834, 59892, 59950, 60008, 60065, 60122, 60179, 60236, 60292, 60348, 60403, 60459, 60514, 60569, 60623, 60678, 60731, 60785, 60838, 60892, 60944, 60997, 61049, 61101, 61153, 61204, 61255, 61306, 61356, 61406, 61456, 61506, 61555, 61604, 61653, 61701, 61749, 61797, 61845, 61892, 61939, 61985, 62032, 62078, 62123, 62169, 62214, 62259, 62303, 62348, 62392, 62435, 62479, 62522, 62564, 62607, 62649, 62691, 62732, 62774, 62815, 62855, 62896, 62936, 62976, 63015, 63054, 63093, 63132, 63170, 63208, 63246, 63283, 63320, 63357, 63393, 63429, 63465, 63501, 63536, 63571, 63606, 63640, 63674, 63708, 63741, 63774, 63807, 63839, 63872, 63904, 63935, 63966, 63997, 64028, 64058, 64088, 64118, 64148, 64177, 64206, 64234, 64262, 64290, 64318, 64345, 64372, 64399, 64425, 64451, 64477, 64503, 64528, 64553, 64577, 64601, 64625, 64649, 64672, 64695, 64718, 64740, 64762, 64784, 64806, 64827, 64848, 64868, 64888, 64908, 64928, 64947, 64966, 64985, 65003, 65021, 65039, 65056, 65073, 65090, 65107, 65123, 65139, 65154, 65169, 65184, 65199, 65213, 65227, 65241, 65254, 65268, 65280, 65293, 65305, 65317, 65328, 65339, 65350, 65361, 65371, 65381, 65391, 65400, 65409, 65418, 65426, 65434, 65442, 65449, 65457, 65463, 65470, 65476, 65482, 65488, 65493, 65498, 65502, 65507, 65511, 65514, 65518, 65521, 65524, 65526, 65528, 65530, 65532, 65533, 65534, 65534, 65535, 65534, 65534, 65533, 65532, 65531, 65529, 65528, 65525, 65523, 65520, 65517, 65513, 65509, 65505, 65501, 65496, 65491, 65486, 65480, 65474, 65468, 65461, 65454, 65447, 65439, 65432, 65423, 65415, 65406, 65397, 65387, 65378, 65368, 65357, 65347, 65336, 65324, 65313, 65301, 65289, 65276, 65263, 65250, 65237, 65223, 65209, 65194, 65179, 65164, 65149, 65133, 65117, 65101, 65085, 65068, 65050, 65033, 65015, 64997, 64979, 64960, 64941, 64921, 64902, 64882, 64861, 64841, 64820, 64798, 64777, 64755, 64733, 64710, 64688, 64665, 64641, 64617, 64593, 64569, 64544, 64519, 64494, 64469, 64443, 64417, 64390, 64363, 64336, 64309, 64281, 64253, 64225, 64196, 64167, 64138, 64108, 64079, 64048, 64018, 63987, 63956, 63925, 63893, 63861, 63829, 63796, 63763, 63730, 63696, 63663, 63628, 63594, 63559, 63524, 63489, 63453, 63417, 63381, 63344, 63308, 63270, 63233, 63195, 63157, 63119, 63080, 63041, 63002, 62962, 62923, 62882, 62842, 62801, 62760, 62719, 62677, 62635, 62593, 62550, 62507, 62464, 62421, 62377, 62333, 62289, 62244, 62199, 62154, 62108, 62062, 62016, 61970, 61923, 61876, 61829, 61781, 61733, 61685, 61637, 61588, 61539, 61489, 61440, 61390, 61339, 61289, 61238, 61187, 61135, 61084, 61032, 60979, 60927, 60874, 60821, 60767, 60714, 60659, 60605, 60551, 60496, 60440, 60385, 60329, 60273, 60217, 60160, 60103, 60046, 59988, 59931, 59873, 59814, 59756, 59697, 59637, 59578, 59518, 59458, 59398, 59337, 59276, 59215, 59154, 59092, 59030, 58967, 58905, 58842, 58779, 58715, 58652, 58588, 58523, 58459, 58394, 58329, 58263, 58198, 58132, 58065, 57999, 57932, 57865, 57798, 57730, 57662, 57594, 57525, 57457, 57388, 57318, 57249, 57179, 57109, 57039, 56968, 56897, 56826, 56754};

static float motor_ov = MOTOR_OV_INITIAL;

// PID control modes
PID pid_angle, pid_rpm, pid_focIq, pid_focId;

// Encoder calibration
float encoder_calib_data[] = {7.91f, 37.08f, 66.35f, 95.71f, 126.4f, 157.23f, 187.73f, 217.17f, 246.35f, 275.8f, 306.47f, 337.23f};
float encoder_LUT[(int)ENCODER_RES];

// Motor constants
motor_t *motor_active;
float motor_kv = 0;
float motor_l = 0;
float motor_r = 0;

// Precomputed per-init constants (avoid recomputing every FOC cycle)
static float motor_flux_linkage = 0.0f;
static float w_e_factor = 0.0f; // 0.10472f * motor_pole_pairs

motor_t motor_list[MOTOR_LIST_SIZE] = {
(struct motor_t){
	.name = "mad3506",
	.polepairs = 7,
	.kv = 400.0f,
	.r_p2p = 240E-3f,
	.l_p2p = 180E-6f,
	.stiction = 0.25f,
	.coulomb = 0.0f,
	.viscous = 0.0f,
},
(struct motor_t){
	.name = "mad4006",
	.polepairs = 12,
	.kv = 740.0f,
	.r_p2p = 51E-3f,
	.l_p2p = 25E-6f,
	.stiction = 0.5f,
	.coulomb = 0.26f,
	.viscous = 0.0f,
},
(struct motor_t){
	.name = "flysky",
	.polepairs = 7,
	.kv = 750.0f,
	.r_p2p = 92E-3f,
	.l_p2p = 35E-6f,
	.stiction = 0.00f,
	.coulomb = 0.0f,
	.viscous = 0.0f,
},
(struct motor_t){
	.name = "tmotor_2806",
	.polepairs = 7,
	.kv = 400.0f,
	.r_p2p = 1.8f,
	.l_p2p = 190E-6f,
	.stiction = 0.00f,
	.coulomb = 0.0f,
	.viscous = 0.0f,
},
(struct motor_t){
	.name = "tmotor_4004",
	.polepairs = 12,
	.kv = 400.0f,
	.r_p2p = 0.4f,//1.8f,
	.l_p2p = 190E-6f,
	.stiction = 0.00f,
	.coulomb = 0.0f,
	.viscous = 0.0f,
},
(struct motor_t){
	.name = "mt2204",
	.polepairs = 7,
	.kv = 2300.0f,
	.r_p2p = 200E-3f,
	.l_p2p = 18.5E-6f,
	.stiction = 0.2f,
	.coulomb = 0.05f,
	.viscous = 0.00005f,
}
};

// FOC
float sin_el, cos_el;
volatile float foc_id = 0.0f, foc_iq = 0.0f;
float i_alpha = 0.0f, i_beta = 0.0f;
float Vq_setpoint = 0;
float torque_setpoint = 0;

// Sensorless
float phase_delay = 0;
uint8_t current_phase = 0;
bool sensorless_flag = false;
uint8_t comp_rshift, bemf_dir;
unsigned char comp_u, comp_v, comp_w, comparator = 0;

// OCP
#define MAX_PHASE_CURRENT 7.0f
float thermal_energy = 0;
float thermal_ilim_2 = MAX_PHASE_CURRENT * MAX_PHASE_CURRENT;
float thermal_limit = 10000.0f;
uint8_t thermal_fault = 0;
float vbat_ilim = 7.0f;

// Motor modes
motor_mode mode = MODE_OFF;
motor_waveform_type waveform_mode = MOTOR_SVPWM;

static float position = 0.0, pos_filt = 0.0, rpm = 0.0, acc = 0.0;
volatile float angle_el = 0.0;

float motor_polarity = 0;
float motor_zero_angle = 0.0;

void TIM4_IRQHandler(void) {
	if(TIM4->SR & 0x1U) {
		TIM4->SR &= ~(0x1U);
	}
}

void ADC1_IRQHandler(void) {
	static uint8_t sample_cnt = 0;
	static int16_t pos_cnt;
	static float angle_el_compensated;
	static motor_mode last_mode = -1;

	float w_e;
	float Uq, Ud, Uq_limit;
	float saturation_error;

	if (ADC1->ISR & ADC_ISR_JEOC) {
		ADC1->ISR = ADC_ISR_JEOC;

		sample_cnt = (sample_cnt + 1) & 0b1;
		if(!sample_cnt) {
			LED0_ON();

			// Read ADCs
			adc_read_motor_isns();
			adc_read_other();

			// Read encoder value
			pos_cnt = ENC_TIM->CNT & ENCODER_RES_MASK;

//          position = encoder_LUT[pos_cnt] - motor_zero_angle ++ (rpm * 6.0f * ENC_LATENCY);
			position = ((float)pos_cnt * 360.0f / ENCODER_RES) - motor_zero_angle + (rpm * 6.0f * ENC_LATENCY);
			// position = ((float)(4095 - spi_angle) * 360.0 / 4095.0) - motor_zero_angle;

			// Wrap angle back to [0, 360]
			position -= 360.0f * (int)(position * 0.002777778f);
			if(position < 0.0f) {
				position += 360.0f;
			} else if(position > 360.0f) {
				position -= 360.0f;
			}

			angle_el = position*motor_pole_pairs;
			angle_el -= 360.0f * (int)(angle_el * 0.002777778f);

			angle_el_compensated = angle_el + rpm * 6.0f * motor_pole_pairs * 0.000015f;
			angle_el_compensated -= 360.0f * (int)(angle_el_compensated * 0.002777778f);

			foc_current_calc(angle_el);

			if(mode != last_mode) {
				PID_reset(&pid_focIq);
				PID_reset(&pid_focId);
				Vq_setpoint = 0;
				torque_setpoint = 0;
			}
			last_mode = mode;

			// pid_focId.setpoint = 0;
			if(mode != MODE_OFF) {
				if(fabsf(pid_focIq.setpoint) < 0.01f && fabsf(rpm) < 10.0f) {
					pid_focId.integral *= 0.95f;
					pid_focIq.integral *= 0.95f;
				}

				w_e = rpm * w_e_factor;
				Uq = 0;
				Ud = 0;
				pid_focIq.setpoint = torque_setpoint;

				if(waveform_mode == MOTOR_FOC_TORQUE) {
				// if(waveform_mode == MOTOR_FOC_TORQUE) {
					// Apply friction_ff compensations
					// pid_focIq.setpoint = 0.06981317008f * motor_kv * torque_setpoint / motor_pole_pairs;

					float friction_ff;
					float cogging_ff;

					#define rpm_deadband 20.0f

					if(fabsf(rpm) < rpm_deadband && fabsf(torque_setpoint) > 0.001f) {
						float blend = fabsf(rpm) / rpm_deadband;
						float stiction_component = motor_active->stiction * (1.0f - blend);
						float coulomb_component  = motor_active->coulomb * tanhf(0.13f * rpm);
						friction_ff = (stiction_component + coulomb_component) * fsignf(torque_setpoint);
					} else {
						friction_ff = motor_active->coulomb * tanhf(0.13f * rpm);
					}

					friction_ff += motor_active->viscous * rpm;

					uint16_t cogging_index = (uint16_t)(position/360.0f * COGGING_LUT_SIZE) % COGGING_LUT_SIZE;
					cogging_ff = 0;//((float)cogging_lut[cogging_index] / 32767.0f) * COGGING_LUT_SCALE;
					pid_focIq.setpoint += cogging_ff + friction_ff;
				}

				if(waveform_mode >= MOTOR_FOC_TORQUE && waveform_mode <= MOTOR_FOC_VQ_ID) {
					// Id/Vd PID

					PID_compute(&pid_focId, foc_id, 0.00002f);

					Ud = pid_focId.output;
					// Inductance feedforward
					Ud += 0.9f * -w_e * motor_l * foc_iq;
				}

				if(waveform_mode == MOTOR_FOC_TORQUE || waveform_mode == MOTOR_FOC_IQ_ID) {
					// Iq/Vq PID

					PID_compute(&pid_focIq, foc_iq, 0.00002f);
					Uq = pid_focIq.output;
					// Resistance feedforward
					Uq += motor_r * pid_focIq.setpoint;
					// BEMF feedforward
					Uq += 0.9f * w_e * motor_flux_linkage;
					// Inductance feedforward
					Uq += 0.9f *  w_e * motor_l * foc_id;

					// Clamp to SVPWM circle
					// Keep Ud, and clamp Uq
					Uq_limit = sqrtf(vsns_vbat*vsns_vbat/3.0f - Ud*Ud);
					Uq_limit = Uq < -Uq_limit ? -Uq_limit : Uq > Uq_limit ? Uq_limit : Uq;

					// De-integrate if Uq is saturated
					saturation_error = Uq - Uq_limit;
					// pid_focIq.integral += saturation_error;
					 if(saturation_error) {
						pid_focIq.integral *= 0.9f;
					 }

					Uq = Uq_limit;

				} else if(waveform_mode == MOTOR_FOC_VQ_ID) {
					Uq = Vq_setpoint;

				} else {
					PID_reset(&pid_focIq);
					PID_reset(&pid_focId);
					Uq = Vq_setpoint;
					Ud = 0;
				}

				setPhaseVoltage(Uq, Ud, angle_el_compensated);
			} else {
				// MODE OFF
				Vq_setpoint = 0;
				PID_reset(&pid_focIq);
				PID_reset(&pid_focId);
			}

			LED0_OFF();
		} else {
			// LED0_ON();
			adc_read_motor_isns();
			// LED0_OFF();
		}   
	}
}

// RPM PID
void TIM5_IRQHandler(void) {
	if(TIM5->SR & 0x1U) {
		TIM5->SR &= ~(0x1U);

		// LED1_ON();

		vel_kf_update((int16_t)ENC_TIM->CNT, 0.0001f);
			
		// Angle PID control
		if (mode == MODE_POS) { 
			pid_angle.derivative = -rpm;
			PID_compute(&pid_angle, pos_filt, 0.0001f);
			// pid_rpm.setpoint = pid_angle.output;
			pid_focIq.setpoint = pid_angle.output;
		}

		// RPM PID control
		if(mode == MODE_RPM) {
			pid_rpm.derivative = -acc/1000.0f;
			PID_compute(&pid_rpm, rpm, 0.0001f);

			// float stiction_ff = 0.0f;
			// if (fabsf(rpm) < 20.0f && fabsf(pid_rpm.output) > 1E-4f) {
			// 	stiction_ff = motor_active->stiction * fsignf(pid_rpm.output);
			// }
			// float friction_ff = stiction_ff + motor_active->coulomb * tanhf(rpm * 0.13f) +  motor_active->viscous * rpm;
			
			pid_focIq.setpoint = pid_rpm.output; // + friction_ff;
			torque_setpoint = pid_rpm.output;
		}

		thermal_energy += (foc_iq*foc_iq + foc_id*foc_id - thermal_ilim_2) * 0.0001f;
		if(thermal_energy < 0) thermal_energy = 0;

		if(thermal_energy > thermal_limit || isns_vbat > vbat_ilim) {
			mode = MODE_OFF;
			MotorOff();
			thermal_fault = 1;
		}

		// LED1_OFF();
	}
}

static void vel_kf_update(int16_t raw_count, float dt) {
	static float x0 = 0.0f;		/* position (counts) */
	static float x1 = 0.0f;		/* velocity (counts/s) */
	static float P00 = 1.0f;
	static float P01 = 0.0f;
	static float P10 = 0.0f;
	static float P11 = 1.0f;
	static int16_t last_raw = 0;

	static int zero_count = 0;

	static const float q = 1e6f;
	static const float R = 1.0f / 12.0f;

	/* --- delta with overflow unwrap --- */
	int16_t delta = raw_count - last_raw;
	last_raw = raw_count;

	if (delta > (int16_t)(ENCODER_RES / 2))  { delta -= (int16_t)ENCODER_RES; }
	if (delta < -(int16_t)(ENCODER_RES / 2)) { delta += (int16_t)ENCODER_RES; }

	if (delta == 0) {
		zero_count++;
		if (zero_count > 100) {
			x1   = 0.0f;
			rpm  = 0.0f;
			P00  = 1.0f;
			P01  = 0.0f;
			P10  = 0.0f;
			P11  = 1.0f;
			zero_count = 0;
		}
	} else {
		zero_count = 0;
	}

	static float meas_pos = 0.0f;
	meas_pos += (float)delta;

	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt3 * dt;

	/* --- Predict --- */
	float xp0 = x0 + x1 * dt;
	float xp1 = x1;

	float Q00 = q * dt4 * 0.25f;
	float Q01 = q * dt3 * 0.5f;
	float Q11 = q * dt2;

	float Pp00 = P00 + dt * (P10 + P01) + dt2 * P11 + Q00;
	float Pp01 = P01 + dt * P11 + Q01;
	float Pp10 = P10 + dt * P11 + Q01;
	float Pp11 = P11 + Q11;

	/* --- Update --- */
	float S_inv	= 1.0f / (Pp00 + R);
	float K0	= Pp00 * S_inv;
	float K1	= Pp10 * S_inv;

	float innov = meas_pos - xp0;

	x0 = xp0 + K0 * innov;
	x1 = xp1 + K1 * innov;

	P00 = (1.0f - K0) * Pp00;
	P01 = (1.0f - K0) * Pp01;
	P10 = Pp10 - K1 * Pp00;
	P11 = Pp11 - K1 * Pp01;

	P00 = fmaxf(P00, 1e-4f);
	P11 = fmaxf(P11, 1e-2f);

	/* --- Outputs --- */
	pos_filt = (float)meas_pos * 360.0f / ENCODER_RES;

	rpm = (x1 / (float)ENCODER_RES) * 60.0f;

	static float prev_rpm = 0.0f;
	static float filt_acc = 0.0f;
	static const float acc_alpha = 0.1f;

	float raw_acc = (rpm - prev_rpm) / dt;   /* RPM/s */
	filt_acc += acc_alpha * (raw_acc - filt_acc);
	prev_rpm = rpm;
	acc = filt_acc;

	float residual = meas_pos - x0;
	if(fabsf(x0) > 1e5f) {
		meas_pos = residual;
		x0 = 0.0f;
	}
}

void setPhaseVoltage(float Uq, float Ud, float angle_el_in) {
	float pwm_u = 0.0f, pwm_v = 0.0f, pwm_w = 0.0f;

	float center;
	float Ualpha, Ubeta;
	float pwm_min, pwm_max;
	float sin, cos;

	float offset = 0.0f;
	
	uint8_t ov_flag = 0;
	uint16_t *PWM_table;
	
	// angle_el_in += motor_polarity;

	angle_el_in -= 360.0f * (int)(angle_el * 0.002777778f);
	if(angle_el_in < 0.0f) {
		angle_el_in += 360.0f;
	} else if(angle_el_in > 360.0f) {
		angle_el_in -= 360.0f;
	}

	int32_t theta_q31;

	if(waveform_mode >= MOTOR_FOC_TORQUE && waveform_mode <= MOTOR_FOC_VQ_ID) {
		theta_q31 = (int32_t)((uint32_t)(angle_el_in * 11930464.711111f));
		CORDIC->WDATA = (uint32_t)theta_q31;

		// Compute inv_vbat while CORDIC calculates sin/cos
		float inv_vbat = 1.0f / vsns_vbat;

		sin = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Sin
		cos = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Cos

		// Inverse Park Transform
		Ualpha = (Ud*cos - Uq*sin) * inv_vbat;
		Ubeta  = (Ud*sin + Uq*cos) * inv_vbat;

		// Inverse Clarke Transform
		pwm_u = Ualpha;
		pwm_v = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
		pwm_w = -0.5f * Ualpha - _SQRT3_2 * Ubeta;
		
//      center = sqrt(Ualpha*Ualpha + Ubeta*Ubeta);
		center = 0.5f;
		
		// SVPWM
		pwm_min = fminf(pwm_u, fminf(pwm_v, pwm_w));
		pwm_max = fmaxf(pwm_u, fmaxf(pwm_v, pwm_w));
		center -= (pwm_max+pwm_min) * 0.5f;

		pwm_u += center;
		pwm_v += center;
		pwm_w += center;
		
	} else if(waveform_mode == MOTOR_SVPWM) {
		PWM_table = SVPWM_table;
		
		if(Uq < 0.0f) {
			angle_el_in += 180.0f;
			Uq = -Uq;
		}
		angle_el_in = -angle_el_in;
		
		pwm_u = (0.5f * wave_lut(PWM_table, angle_el_in)          + 0.5f) * Uq / vsns_vbat;
		pwm_v = (0.5f * wave_lut(PWM_table, angle_el_in + 120.0f) + 0.5f) * Uq / vsns_vbat;
		pwm_w = (0.5f * wave_lut(PWM_table, angle_el_in + 240.0f) + 0.5f) * Uq / vsns_vbat;
		
	} else if(waveform_mode == MOTOR_TRAPEZOID) {
		if(Uq < 0.0f) {
			angle_el_in += 180.0f;
			Uq = -Uq;
		}
		
		angle_el_in = roundf(normalizeAngle(angle_el_in-90.0f) / 60.0f);
		if(vsns_vbat > motor_ov) {
			pwm_u = 0.0f;
			pwm_v = 0.0f;
			pwm_w = 0.0f;
			MotorShort(0.5f);
		} else {
			MotorPhase((signed char)angle_el_in, Uq / vsns_vbat);
		}
	}

	if(waveform_mode != MOTOR_TRAPEZOID) {
		if(vsns_vbat > motor_ov) {
			pwm_u = 0.0f;
			pwm_v = 0.0f;
			pwm_w = 0.0f;
			
			LED1_ON();
			ov_flag = 1;
			
			motor_ov = 15.0f;
			
			MotorShort(0.5f);
		} else {
			if(ov_flag) {
				LED1_OFF();
				ov_flag = 0;
				motor_ov = MOTOR_OV_INITIAL;
			}
			MotorPhasePWM(pwm_u, pwm_v, pwm_w);
		}
	}
}

#define ISNS_LPF 0.5f
#define ISNS_OFFSET_LPF 0.0001f
static inline void adc_read_motor_isns() {
	static float temp_isns_v, temp_isns_w;
	static float isns_v_err, isns_w_err;
	static float amp_gain = ISNS_AMP_GAIN * ISNS_UVW_R;

	// Read injected ADC channels

	if(!motor_direction) {
		// Default case
		temp_isns_w = (float)ADC1->JDR1 * ADC_CONV_FACTOR;
		temp_isns_v = (float)ADC2->JDR1 * ADC_CONV_FACTOR;
		isns_v_err = ISNS_V_GAIN_ERR;
		isns_w_err = ISNS_W_GAIN_ERR;
		
	} else {
		// swap V and W 
		temp_isns_v = (float)ADC1->JDR1 * ADC_CONV_FACTOR;
		temp_isns_w = (float)ADC2->JDR1 * ADC_CONV_FACTOR;
		isns_v_err = ISNS_W_GAIN_ERR;
		isns_w_err = ISNS_V_GAIN_ERR;
	}

	// Isns offset LPF
	isns_v_offset += ISNS_OFFSET_LPF * (temp_isns_v - isns_v_offset);
	isns_w_offset += ISNS_OFFSET_LPF * (temp_isns_w - isns_w_offset);

	// isns_v_offset = 1.665f + 0.008f;
	// isns_w_offset = 1.67f - 0.008f;

	// Clamp offset
	isns_v_offset = isns_v_offset < 1.55f ? 1.55f : isns_v_offset > 1.75f ? 1.75f : isns_v_offset;
	isns_w_offset = isns_w_offset < 1.55f ? 1.55f : isns_w_offset > 1.75f ? 1.75f : isns_w_offset;

	// Isns LPF
	isns_v += ISNS_LPF * (((isns_v_offset - temp_isns_v) * amp_gain * isns_v_err) - isns_v) ;
	isns_w += ISNS_LPF * (((isns_w_offset - temp_isns_w) * amp_gain * isns_w_err) - isns_w) ;

	isns_u = -(isns_v + isns_w);
}

#define VBAT_LPF 0.01f
#define VSNS_LPF 0.01f
static inline void adc_read_other() {
	static uint16_t adc11, adc12, adc13, adc21, adc22, adc23;
	static uint8_t startup = 1;

	// Read other adc
	adc21 = (uint16_t)(adc_buffer[0] >> 16);    // isns_vbat
	adc11 = (uint16_t)(adc_buffer[0] & 0xFFFF); // vsns_w
	adc22 = (uint16_t)(adc_buffer[1] >> 16);    // vsns_vbat
	adc12 = (uint16_t)(adc_buffer[1] & 0xFFFF); // vsns_u
	adc23 = (uint16_t)(adc_buffer[2] >> 16);    // vsns_x
	adc13 = (uint16_t)(adc_buffer[2] & 0xFFFF); // vsns_v

	vsns_u += VSNS_LPF * ((float)adc12*ADC_CONV_FACTOR*MOTOR_VSNS_DIVIDER - vsns_u);
	vsns_v += VSNS_LPF * ((float)adc13*ADC_CONV_FACTOR*MOTOR_VSNS_DIVIDER - vsns_v);
	vsns_w += VSNS_LPF * ((float)adc11*ADC_CONV_FACTOR*MOTOR_VSNS_DIVIDER - vsns_w);
	vsns_x += VSNS_LPF * ((float)adc23*ADC_CONV_FACTOR*MOTOR_VSNS_DIVIDER - vsns_x);

	if(startup) {
		vsns_vbat = (float)adc22*ADC_CONV_FACTOR * VSNS_VBAT_DIVIDER;
		isns_vbat = (float)adc21*ADC_CONV_FACTOR - ISNS_VBAT_OFFSET;
		startup = 0;
	}

	vsns_vbat += VBAT_LPF * ((float)adc22*ADC_CONV_FACTOR * VSNS_VBAT_DIVIDER - vsns_vbat);
	isns_vbat += VBAT_LPF * (((float)adc21*ADC_CONV_FACTOR - ISNS_VBAT_OFFSET) * ISNS_VBAT_AMP_GAIN*ISNS_VBAT_R - isns_vbat);
}

#define FOC_IQ_LPF 0.5f
static inline void foc_current_calc(float angle_el) {
	static int32_t theta_q31;

	theta_q31 = (int32_t)((uint32_t)(angle_el * 11930464.711111f));
	CORDIC->WDATA = (uint32_t)theta_q31;

	// Clarke Transform
	i_alpha = isns_u;
	i_beta = (isns_v - isns_w) * _1_SQRT3;

	// Read sin/cos from CORDIC
	sin_el = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Sin
	cos_el = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Cos

	// Park Transform
	foc_iq += FOC_IQ_LPF * ((-i_alpha * sin_el + i_beta* cos_el) - foc_iq);
	foc_id += FOC_IQ_LPF * (( i_alpha * cos_el + i_beta* sin_el) - foc_id);
}

/*---------------------------------------------------------------------
 |                            Sensorless                              |
 ----------------------------------------------------------------------*/

void SensorlessStart(float p) {
	sensorless_flag = false;
	// SetPower(p);
	current_phase = 0;
	waveform_mode = MODE_SENSORLESS;
	
	comp_rshift = 0;
	bemf_dir = 1;
	
	//TODO: Sensorless implementation
}

void TIM6_IRQHandler(void) {
	if(TIM6->SR & 0x1U) {
		TIM6->SR &= ~(0x1U);
		
		// TODO: Sensorless implementation

		// LATDINV |= 1 << 6;
		// PR8 = 0xFFFFFFFF;
		// T8CONbits.ON = 0;
		
		// if(sensorless_flag) {
		//  sensorless_flag = false;
		//  current_phase = (current_phase + 1) % 6;
		//  MotorPhase(current_phase, power);
		//  TMR8 = 0;
		//  PR8 = 3600;
		
		// } else {
		//  IEC5bits.PWM4IE = 1;
		//  PR8 = 0xFFFFFFFF;
		//  TMR8 = 3600;
		// }
	}
}

bool bemf_flag;
uint8_t pwm_odd = 0;

void TIM7_IRQHandler(void) {
	if(TIM7->SR & 0x1U) {
		TIM7->SR &= ~(0x1U);

		// TODO: PWM synced comparator read
	
		if(pwm_odd == 1) {
//          comparator = (PORTG >> 6) & 0b111;
//          comp_u = (comparator >> 2) & 1;
//          comp_v = (comparator >> 1) & 1;
//          comp_w = comparator & 1;

			// LATDINV |= 1 << 6;
		
			// TMR8 = 0;
			// PR8 = 10;
			// T8CONbits.ON = 1;
		
			// if(waveform_mode == MODE_SENSORLESS) {
			//  if(current_phase == 0) {
			//      if(comparator & 1) {
			//          bemf_flag = true;
			//      }
			//  } else if(current_phase == 1) {
			//      if(!((comparator >> 1) & 1)) {
			//          bemf_flag = true;
			//      }
			//  } else if(current_phase == 2) {
			//      if((comparator >> 2) & 1) {
			//          bemf_flag = true;
			//      }
			//  } else if(current_phase == 3) {
			//      if(!(comparator & 1)) {
			//          bemf_flag = true;
			//      }
			//  } else if(current_phase == 4) {
			//      if((comparator >> 1) & 1) {
			//          bemf_flag = true;
			//      }
			//  } else if(current_phase == 5) {
			//      if(!((comparator >> 2) & 1)) {
			//          bemf_flag = true;
			//      }
			//  }
			//  if(bemf_flag) {
			//      IEC5bits.PWM4IE = 0;
			//      sensorless_flag = true;
			//      phase_delay = (1.0f-LPF_PHASE)*phase_delay + LPF_PHASE*(float)TMR8;
			//      TMR8 = 0;
			//      PR8 = phase_delay;
			//  }
			// }
		}
		pwm_odd = (pwm_odd + 1U) % 2U;
	}
}

bool bemf_phase(unsigned char phase) {
	phase = phase % 6;
	if(phase == 0) {
		return comp_w;
	} else if(phase == 1) {
		return !comp_v;
	} else if(phase == 2) {
		return comp_u;
	} else if(phase == 3) {
		return !comp_w;
	} else if(phase == 4) {
		return comp_v;
	} else if(phase == 5) {
		return !comp_u;
	}
	return false;
}

/*---------------------------------------------------------------------
 |                            Motor PWM                               |
 ----------------------------------------------------------------------*/

void MotorPhase(int8_t num, float val) {
	val = val * (float)PWM_MAX;
	num = num % 6;
	if(num < 0) {
		num += 6;
	}
	if(motor_direction) {
		num = 5 - num;
	}
	switch(num) {
		case 0:
			// U - V+
			MOTOR_TIM->CCER &= ~CCNP_U;
			MOTOR_TIM->CCR_U = (uint32_t)val;

			// V - GND
			MOTOR_TIM->CCER &= ~CCNP_V;
			MOTOR_TIM->CCR_V = 0;

			// W - NC
			MOTOR_TIM->CCER |= CCNP_W;
			MOTOR_TIM->CCR_W = 0;

			break;

		case 1:
			// U - V+
			MOTOR_TIM->CCER &= ~CCNP_U;
			MOTOR_TIM->CCR_U = (uint32_t)val;

			// V - NC
			MOTOR_TIM->CCER |= CCNP_V;
			MOTOR_TIM->CCR_V = 0;

			// W - GND
			MOTOR_TIM->CCER &= ~CCNP_W;
			MOTOR_TIM->CCR_W = 0;

			break;

		case 2:
			// U - NC
			MOTOR_TIM->CCER |= CCNP_U;
			MOTOR_TIM->CCR_U = 0;

			// V - V+
			MOTOR_TIM->CCER &= ~CCNP_V;
			MOTOR_TIM->CCR_V = (uint32_t)val;

			// W - GND
			MOTOR_TIM->CCER &= ~CCNP_W;
			MOTOR_TIM->CCR_W = 0;

			break;

		case 3:
			// U - GND
			MOTOR_TIM->CCER &= ~CCNP_U;
			MOTOR_TIM->CCR_U = 0;

			// V - V+
			MOTOR_TIM->CCER &= ~CCNP_V;
			MOTOR_TIM->CCR_V = (uint32_t)val;

			// W - NC
			MOTOR_TIM->CCER |= CCNP_W;
			MOTOR_TIM->CCR_W = 0;

			break;

		case 4:
			// U - GND
			MOTOR_TIM->CCER &= ~CCNP_U;
			MOTOR_TIM->CCR_U = 0;

			// V - NC
			MOTOR_TIM->CCER |= CCNP_V;
			MOTOR_TIM->CCR_V = 0;

			// W - V+
			MOTOR_TIM->CCER &= ~CCNP_W;
			MOTOR_TIM->CCR_W = (uint32_t)val;

			break;

		case 5:
			// U - NC
			MOTOR_TIM->CCER |= CCNP_U;
			MOTOR_TIM->CCR_U = 0;

			// V - GND
			MOTOR_TIM->CCER &= ~CCNP_V;
			MOTOR_TIM->CCR_V = 0;

			// W - V+
			MOTOR_TIM->CCER &= ~CCNP_W;
			MOTOR_TIM->CCR_W = (uint32_t)val;

			break;

		MOTOR_TIM->CCER |= (CCE_U | CCE_V | CCE_W); // Enable all high channels
	}
}

void MotorOff() {
	// U - NC
	MOTOR_TIM->CCER |= CCNP_U;
	MOTOR_TIM->CCR_U = 0;

	// V - NC
	MOTOR_TIM->CCER |= CCNP_V;
	MOTOR_TIM->CCR_V = 0;

	// W - NC
	MOTOR_TIM->CCER |= CCNP_W;
	MOTOR_TIM->CCR_W = 0;
}

void MotorShort(float p) {
	// p = 0:       float all phases
	// p = 1.0:     short all phases to ground
	// 0 < p < 1:   pwm (short/float) all phases to ground
	
	p = fabsf(p) * PWM_MAX;

	MOTOR_TIM->CCER &= ~(CCE_U | CCE_V | CCE_W);

	MOTOR_TIM->CCER |= CCNP_U;
	MOTOR_TIM->CCER |= CCNP_V;
	MOTOR_TIM->CCER |= CCNP_W;

	MOTOR_TIM->CCR_U = (uint32_t)p;
	MOTOR_TIM->CCR_V = (uint32_t)p;
	MOTOR_TIM->CCR_W = (uint32_t)p;
}

void MotorPhasePWM(float pwm_u, float pwm_v, float pwm_w) {
	// Inputs shoule be within [0, 1.0]
	
	static float temp;

	pwm_u = pwm_u > 1.0f ? 1.0f: pwm_u < 0.0f ? 0.0f: pwm_u;
	pwm_v = pwm_v > 1.0f ? 1.0f: pwm_v < 0.0f ? 0.0f: pwm_v;
	pwm_w = pwm_w > 1.0f ? 1.0f: pwm_w < 0.0f ? 0.0f: pwm_w;
	
	if(motor_direction) {
		// swap V and W
		temp = pwm_v;
		pwm_v = pwm_w;
		pwm_w = temp;
	}
	
	pwm_u *= PWM_MAX;
	pwm_v *= PWM_MAX;
	pwm_w *= PWM_MAX;

	MOTOR_TIM->CCER |= (CCE_U | CCE_V | CCE_W); // Enable all high channels
	MOTOR_TIM->CCER &= ~(CCNP_U | CCNP_V | CCNP_W); // Normal polarity for low channels

	MOTOR_TIM->CCR_U = (uint32_t)pwm_u;
	MOTOR_TIM->CCR_V = (uint32_t)pwm_v;
	MOTOR_TIM->CCR_W = (uint32_t)pwm_w;
}

/*---------------------------------------------------------------------
 |                                PID                                 |
 ----------------------------------------------------------------------*/

uint8_t MotorPIDInit(motor_t *motor) {
	PID_init(&pid_angle);
	PID_init(&pid_rpm);
	PID_init(&pid_focIq);
	PID_init(&pid_focId);

	motor_active = motor;
	motor_pole_pairs = motor_active->polepairs;
	motor_kv = motor_active->kv;
	motor_r = motor_active->r_p2p * 0.5f;
	motor_l = motor_active->l_p2p * 0.5f;

	if(motor_pole_pairs == 0 || motor_kv == 0 || motor_r == 0 || motor_l == 0) {
		return 0;
	}

	motor_flux_linkage = 5.513288954f / (motor_kv * motor_pole_pairs);
	w_e_factor = 0.10472f * motor_pole_pairs;

	float foc_bw = 1000.0f * 6.283185307f;

	// FOC PID Gains
	// Kp = L_phase*bw
	// Ki = R_phase*bw
	// f_bw = 100hz - 5000hz
																		// Input Units	- Output Units
	PID_setGain(&pid_focIq,	foc_bw*motor_l,	foc_bw*motor_r,	0.0		);	// Iq (Amps)	- Volt
	PID_setGain(&pid_focId,	foc_bw*motor_l,	foc_bw*motor_r,	0.0		);	// Id (Amps)	- Volt
	PID_setGain(&pid_rpm,	0.03f,			0.06f,			0.0		);	// RPM			- Iq (Amps)
	PID_setGain(&pid_angle,	0.004f,			0.0f,			0.001f	);	// Degrees		- Iq (Amps)

	// Iq
	// PID_enableErrorConstrain(&pid_focIq);
	// PID_setErrorLimits(&pid_focIq, -0.5, 0.5);
	PID_enableIntegralConstrain(&pid_focIq);
	PID_setIntegralLimits(&pid_focIq, -4, 4);
	PID_enableOutputConstrain(&pid_focIq);
	PID_setOutputLimits(&pid_focIq, -7, 7);
	
	// Id
	// PID_enableErrorConstrain(&pid_focId);
	// PID_setErrorLimits(&pid_focId, -0.8, 0.8);
	PID_enableIntegralConstrain(&pid_focId);
	PID_setIntegralLimits(&pid_focId, -4, 4);
	PID_enableOutputConstrain(&pid_focId);
	PID_setOutputLimits(&pid_focId, -7, 7);

	// RPM
	PID_enableErrorConstrain(&pid_rpm);
	PID_setErrorLimits(&pid_rpm, -500, 500);
	PID_enableIntegralConstrain(&pid_rpm);
	PID_setIntegralLimits(&pid_rpm, -4, 4);
	PID_disableComputeDerivative(&pid_rpm);
	PID_enableOutputConstrain(&pid_rpm);
	PID_setOutputLimits(&pid_rpm, -10, 10);
	
	// Angle
	PID_enableErrorConstrain(&pid_angle);
	PID_setErrorLimits(&pid_angle, -45, 45);
	PID_enableIntegralConstrain(&pid_angle);
	PID_setIntegralLimits(&pid_angle, -7, 7);
	PID_disableComputeDerivative(&pid_angle);
	PID_enableOutputConstrain(&pid_angle);
	PID_setOutputLimits(&pid_angle, -10, 10);

	return 1;
}

inline void ResetMotorPID() {
	PID_reset(&pid_angle);
	PID_reset(&pid_rpm);
}

inline void SetRPM(float rpm) {
	pid_rpm.setpoint = rpm;
}

inline void SetPosition(float pos) {
	pid_angle.setpoint = pos;
}

inline float GetPositionRaw() {
	return position;
}

inline float GetPosition() {
	return pos_filt;
}

inline void ResetPosition() {
	pos_filt = 0.0;
}

inline float GetRPM() {
	return rpm;
}

inline float GetAcc() {
	return acc;
}

inline float normalizeAngle(float angle) {
	angle = fmodf(angle, 360.0f);
	if(angle < 0) {
		angle += 360;
	}
	return angle;
}

void init_encoder_lut() {
	unsigned int i;
	for(i = 0; i < ENCODER_RES; i++) {
		encoder_LUT[i] = (float)i * 360.0f/ENCODER_RES;
	}
}

void interpolate_encoder_lut(float in[], uint16_t len) {
	uint16_t i, j, k;
	float delta = 0;
	float arr[(int)motor_pole_pairs*6][2];
	
	for(i = 0; i < len; i++) {
		arr[i][0] = in[i] * ENCODER_RES / 360.0f;
		arr[i][1] = (float)i / (float)len * ENCODER_RES;
	}
	
	for(i = 0; i < ENCODER_RES; i++) {
		for(j = 0; j < len; j++) {
			k = (uint16_t)(j + 1U) % len;
			if(i >= arr[j][0] && i < arr[k][0]) {
				delta = i - arr[j][0];
				delta = delta / (arr[k][0] - arr[j][0]) * ENCODER_RES / len;//(arr[k][1] - arr[j][1]);
				break;
			} else if((fabsf(arr[k][0] - arr[j][0]) > ENCODER_RES/2.0f) && (i > arr[j][0] || i < arr[k][0])) {
				delta = i - arr[j][0];
				if(delta > ENCODER_RES/2) {
					delta -= ENCODER_RES;
				} else if(delta < -ENCODER_RES/2) {
					delta += ENCODER_RES;
				}
				delta = delta / (arr[k][0] - arr[j][0] + ENCODER_RES) * ENCODER_RES / len;// * (arr[k][1] - arr[j][1]);
				break;
			}
		}
		encoder_LUT[i] = fmodf(arr[j][1] + delta, ENCODER_RES) * 360.0f / ENCODER_RES;
	}
}

// Angle in degrees
// return [-1, 1]
static inline float wave_lut(uint16_t lut[], float angle) {
	float temp;
	angle = normalizeAngle(angle);
	uint16_t index;
	
	index = (uint16_t)(angle * 8.0f);
	if(index >= (LUT_SIZE*4)) {
		index = LUT_SIZE*4 - 1;
	}
	
	if(index < LUT_SIZE) {
		temp = ((float)lut[index] / 65535.0f);
	} else if(index < LUT_SIZE*2) {
		index = LUT_SIZE*2 - index - 1;
		temp = ((float)lut[index] / 65535.0f);
	} else if(index < LUT_SIZE*3) {
		index = index - LUT_SIZE*2;
		temp = ((float)lut[index] / -65535.0f);
	} else {
		index = LUT_SIZE*4 - index - 1;
		temp = ((float)lut[index] / -65535.0f);
	}
	
	temp = temp < -1.0f ? -1.0f : temp > 1.0f ? 1.0f : temp;
	
	return temp;
}

static inline float fsignf(float x) {
	return (float)((x > 0.0f) - (x < 0.0f));
}

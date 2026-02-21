#include "main.h"

#include <inttypes.h>
#include "BLDC.h"
#include "ADC.h"
#include "PWM.h"
#include <math.h>
#include "PID.h"

#define MOTOR_OV_INITIAL 20.0f // volts

#define _SQRT3_2 0.86602540378443864676372317075294f
#define _1_SQRT3 0.57735026918962576450914878050196f
#define _2_SQRT3 1.1547005383792515290182975610039f

static inline void update_motion_observer(int16_t, float);
static inline void adc_read_motor_isns();
static inline void adc_read_other();
static inline void foc_current_calc(float);
static inline float wave_lut(uint16_t*, float);

#define DEAD_TIME 25

float motor_pole_pairs = 7;
float foc_degree_advance = FOC_DEGREE_ADVANCE;
uint8_t motor_direction = 0; // Motor wiring: 1 for reverse
uint8_t enc_direction = 0; // Encoder direction: 1 for reverse

// SVPWM
static uint16_t SVPWM_table[LUT_SIZE] = {0, 247, 495, 743, 991, 1239, 1487, 1735, 1983, 2231, 2479, 2727, 2975, 3223, 3471, 3719, 3966, 4214, 4462, 4710, 4958, 5205, 5453, 5701, 5948, 6196, 6444, 6691, 6939, 7186, 7434, 7681, 7929, 8176, 8423, 8671, 8918, 9165, 9412, 9659, 9906, 10153, 10400, 10647, 10894, 11141, 11388, 11634, 11881, 12128, 12374, 12621, 12867, 13113, 13360, 13606, 13852, 14098, 14344, 14590, 14836, 15082, 15328, 15573, 15819, 16064, 16310, 16555, 16801, 17046, 17291, 17536, 17781, 18026, 18271, 18515, 18760, 19004, 19249, 19493, 19737, 19982, 20226, 20470, 20713, 20957, 21201, 21445, 21688, 21931, 22175, 22418, 22661, 22904, 23147, 23389, 23632, 23874, 24117, 24359, 24601, 24843, 25085, 25327, 25569, 25810, 26052, 26293, 26534, 26775, 27016, 27257, 27497, 27738, 27978, 28219, 28459, 28699, 28939, 29178, 29418, 29657, 29897, 30136, 30375, 30614, 30852, 31091, 31329, 31568, 31806, 32044, 32282, 32519, 32757, 32994, 33231, 33468, 33705, 33942, 34179, 34415, 34651, 34887, 35123, 35359, 35594, 35830, 36065, 36300, 36535, 36770, 37004, 37239, 37473, 37707, 37941, 38174, 38408, 38641, 38874, 39107, 39340, 39572, 39804, 40037, 40269, 40500, 40732, 40963, 41194, 41425, 41656, 41887, 42117, 42347, 42577, 42807, 43037, 43266, 43495, 43724, 43953, 44181, 44410, 44638, 44866, 45093, 45321, 45548, 45775, 46002, 46229, 46455, 46681, 46907, 47133, 47358, 47583, 47809, 48033, 48258, 48482, 48706, 48930, 49154, 49377, 49600, 49823, 50046, 50269, 50491, 50713, 50934, 51156, 51377, 51598, 51819, 52039, 52260, 52480, 52700, 52919, 53138, 53357, 53576, 53795, 54013, 54231, 54449, 54666, 54883, 55100, 55317, 55533, 55749, 55965, 56181, 56396, 56611, 56778, 56850, 56921, 56992, 57062, 57132, 57202, 57272, 57342, 57411, 57480, 57548, 57617, 57685, 57753, 57820, 57887, 57954, 58021, 58087, 58154, 58219, 58285, 58350, 58415, 58480, 58545, 58609, 58673, 58736, 58800, 58863, 58926, 58988, 59050, 59112, 59174, 59235, 59297, 59357, 59418, 59478, 59538, 59598, 59657, 59716, 59775, 59834, 59892, 59950, 60008, 60065, 60122, 60179, 60236, 60292, 60348, 60403, 60459, 60514, 60569, 60623, 60678, 60731, 60785, 60838, 60892, 60944, 60997, 61049, 61101, 61153, 61204, 61255, 61306, 61356, 61406, 61456, 61506, 61555, 61604, 61653, 61701, 61749, 61797, 61845, 61892, 61939, 61985, 62032, 62078, 62123, 62169, 62214, 62259, 62303, 62348, 62392, 62435, 62479, 62522, 62564, 62607, 62649, 62691, 62732, 62774, 62815, 62855, 62896, 62936, 62976, 63015, 63054, 63093, 63132, 63170, 63208, 63246, 63283, 63320, 63357, 63393, 63429, 63465, 63501, 63536, 63571, 63606, 63640, 63674, 63708, 63741, 63774, 63807, 63839, 63872, 63904, 63935, 63966, 63997, 64028, 64058, 64088, 64118, 64148, 64177, 64206, 64234, 64262, 64290, 64318, 64345, 64372, 64399, 64425, 64451, 64477, 64503, 64528, 64553, 64577, 64601, 64625, 64649, 64672, 64695, 64718, 64740, 64762, 64784, 64806, 64827, 64848, 64868, 64888, 64908, 64928, 64947, 64966, 64985, 65003, 65021, 65039, 65056, 65073, 65090, 65107, 65123, 65139, 65154, 65169, 65184, 65199, 65213, 65227, 65241, 65254, 65268, 65280, 65293, 65305, 65317, 65328, 65339, 65350, 65361, 65371, 65381, 65391, 65400, 65409, 65418, 65426, 65434, 65442, 65449, 65457, 65463, 65470, 65476, 65482, 65488, 65493, 65498, 65502, 65507, 65511, 65514, 65518, 65521, 65524, 65526, 65528, 65530, 65532, 65533, 65534, 65534, 65535, 65534, 65534, 65533, 65532, 65531, 65529, 65528, 65525, 65523, 65520, 65517, 65513, 65509, 65505, 65501, 65496, 65491, 65486, 65480, 65474, 65468, 65461, 65454, 65447, 65439, 65432, 65423, 65415, 65406, 65397, 65387, 65378, 65368, 65357, 65347, 65336, 65324, 65313, 65301, 65289, 65276, 65263, 65250, 65237, 65223, 65209, 65194, 65179, 65164, 65149, 65133, 65117, 65101, 65085, 65068, 65050, 65033, 65015, 64997, 64979, 64960, 64941, 64921, 64902, 64882, 64861, 64841, 64820, 64798, 64777, 64755, 64733, 64710, 64688, 64665, 64641, 64617, 64593, 64569, 64544, 64519, 64494, 64469, 64443, 64417, 64390, 64363, 64336, 64309, 64281, 64253, 64225, 64196, 64167, 64138, 64108, 64079, 64048, 64018, 63987, 63956, 63925, 63893, 63861, 63829, 63796, 63763, 63730, 63696, 63663, 63628, 63594, 63559, 63524, 63489, 63453, 63417, 63381, 63344, 63308, 63270, 63233, 63195, 63157, 63119, 63080, 63041, 63002, 62962, 62923, 62882, 62842, 62801, 62760, 62719, 62677, 62635, 62593, 62550, 62507, 62464, 62421, 62377, 62333, 62289, 62244, 62199, 62154, 62108, 62062, 62016, 61970, 61923, 61876, 61829, 61781, 61733, 61685, 61637, 61588, 61539, 61489, 61440, 61390, 61339, 61289, 61238, 61187, 61135, 61084, 61032, 60979, 60927, 60874, 60821, 60767, 60714, 60659, 60605, 60551, 60496, 60440, 60385, 60329, 60273, 60217, 60160, 60103, 60046, 59988, 59931, 59873, 59814, 59756, 59697, 59637, 59578, 59518, 59458, 59398, 59337, 59276, 59215, 59154, 59092, 59030, 58967, 58905, 58842, 58779, 58715, 58652, 58588, 58523, 58459, 58394, 58329, 58263, 58198, 58132, 58065, 57999, 57932, 57865, 57798, 57730, 57662, 57594, 57525, 57457, 57388, 57318, 57249, 57179, 57109, 57039, 56968, 56897, 56826, 56754};
static uint16_t saddle_lut[LUT_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 152, 153, 154, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 164, 165, 165, 166, 167, 168, 169, 170, 171, 171, 172, 173, 174, 175, 175, 176, 177, 178, 179, 179, 180, 181, 182, 183, 183, 184, 185, 186, 186, 187, 188, 189, 189, 190, 191, 192, 192, 193, 194, 195, 195, 196, 197, 197, 198, 199, 199, 200, 201, 202, 202, 203, 204, 204, 205, 206, 206, 207, 207, 208, 209, 209, 210, 211, 211, 212, 213, 213, 214, 214, 215, 216, 216, 217, 217, 218, 218, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 224, 225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 230, 231, 231, 232, 232, 233, 233, 234, 234, 235, 235, 235, 236, 236, 237, 237, 237, 238, 238, 239, 239, 239, 240, 240, 241, 241, 241, 242, 242, 242, 243, 243, 243, 244, 244, 244, 245, 245, 245, 245, 246, 246, 246, 247, 247, 247, 247, 248, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250, 251, 251, 251, 251, 251, 252, 252, 252, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255};
static uint16_t sin_lut[LUT_SIZE] = {0, 143, 286, 429, 572, 715, 859, 1002, 1145, 1288, 1431, 1574, 1717, 1861, 2004, 2147, 2290, 2433, 2576, 2719, 2862, 3005, 3148, 3291, 3434, 3577, 3720, 3863, 4006, 4149, 4292, 4435, 4577, 4720, 4863, 5006, 5148, 5291, 5434, 5577, 5719, 5862, 6004, 6147, 6289, 6432, 6574, 6717, 6859, 7002, 7144, 7286, 7429, 7571, 7713, 7855, 7997, 8139, 8281, 8423, 8565, 8707, 8849, 8991, 9133, 9275, 9416, 9558, 9700, 9841, 9983, 10124, 10266, 10407, 10548, 10690, 10831, 10972, 11113, 11254, 11395, 11536, 11677, 11818, 11959, 12099, 12240, 12381, 12521, 12662, 12802, 12943, 13083, 13223, 13363, 13504, 13644, 13784, 13924, 14063, 14203, 14343, 14483, 14622, 14762, 14901, 15041, 15180, 15319, 15458, 15597, 15737, 15875, 16014, 16153, 16292, 16430, 16569, 16708, 16846, 16984, 17122, 17261, 17399, 17537, 17675, 17812, 17950, 18088, 18225, 18363, 18500, 18638, 18775, 18912, 19049, 19186, 19323, 19460, 19596, 19733, 19869, 20006, 20142, 20278, 20414, 20550, 20686, 20822, 20958, 21093, 21229, 21364, 21499, 21635, 21770, 21905, 22040, 22174, 22309, 22444, 22578, 22713, 22847, 22981, 23115, 23249, 23383, 23516, 23650, 23783, 23917, 24050, 24183, 24316, 24449, 24582, 24714, 24847, 24979, 25112, 25244, 25376, 25508, 25640, 25771, 25903, 26034, 26166, 26297, 26428, 26559, 26690, 26821, 26951, 27082, 27212, 27342, 27472, 27602, 27732, 27861, 27991, 28120, 28250, 28379, 28508, 28637, 28765, 28894, 29022, 29151, 29279, 29407, 29535, 29662, 29790, 29917, 30045, 30172, 30299, 30426, 30553, 30679, 30806, 30932, 31058, 31184, 31310, 31436, 31561, 31687, 31812, 31937, 32062, 32187, 32311, 32436, 32560, 32684, 32808, 32932, 33056, 33179, 33303, 33426, 33549, 33672, 33795, 33917, 34040, 34162, 34284, 34406, 34528, 34649, 34771, 34892, 35013, 35134, 35255, 35376, 35496, 35616, 35736, 35856, 35976, 36096, 36215, 36334, 36453, 36572, 36691, 36810, 36928, 37046, 37164, 37282, 37400, 37517, 37634, 37752, 37868, 37985, 38102, 38218, 38334, 38450, 38566, 38682, 38797, 38913, 39028, 39143, 39258, 39372, 39486, 39601, 39715, 39828, 39942, 40055, 40169, 40282, 40394, 40507, 40620, 40732, 40844, 40956, 41067, 41179, 41290, 41401, 41512, 41623, 41733, 41844, 41954, 42064, 42173, 42283, 42392, 42501, 42610, 42719, 42827, 42935, 43044, 43151, 43259, 43366, 43474, 43581, 43688, 43794, 43901, 44007, 44113, 44219, 44324, 44429, 44535, 44640, 44744, 44849, 44953, 45057, 45161, 45265, 45368, 45471, 45574, 45677, 45780, 45882, 45984, 46086, 46188, 46289, 46390, 46491, 46592, 46693, 46793, 46893, 46993, 47093, 47192, 47291, 47390, 47489, 47588, 47686, 47784, 47882, 47980, 48077, 48174, 48271, 48368, 48464, 48561, 48657, 48753, 48848, 48943, 49039, 49133, 49228, 49322, 49417, 49510, 49604, 49698, 49791, 49884, 49977, 50069, 50161, 50253, 50345, 50437, 50528, 50619, 50710, 50800, 50891, 50981, 51071, 51160, 51250, 51339, 51428, 51516, 51605, 51693, 51781, 51868, 51956, 52043, 52130, 52216, 52303, 52389, 52475, 52560, 52646, 52731, 52816, 52900, 52985, 53069, 53153, 53236, 53320, 53403, 53486, 53568, 53651, 53733, 53815, 53896, 53978, 54059, 54139, 54220, 54300, 54380, 54460, 54540, 54619, 54698, 54776, 54855, 54933, 55011, 55089, 55166, 55243, 55320, 55397, 55473, 55549, 55625, 55701, 55776, 55851, 55926, 56000, 56075, 56149, 56222, 56296, 56369, 56442, 56514, 56587, 56659, 56731, 56802, 56873, 56944, 57015, 57086, 57156, 57226, 57295, 57365, 57434, 57503, 57571, 57639, 57707, 57775, 57843, 57910, 57977, 58043, 58110, 58176, 58241, 58307, 58372, 58437, 58502, 58566, 58630, 58694, 58758, 58821, 58884, 58947, 59009, 59071, 59133, 59195, 59256, 59317, 59378, 59438, 59498, 59558, 59618, 59677, 59736, 59795, 59853, 59911, 59969, 60027, 60084, 60141, 60198, 60254, 60310, 60366, 60422, 60477, 60532, 60587, 60641, 60696, 60749, 60803, 60856, 60909, 60962, 61014, 61066, 61118, 61170, 61221, 61272, 61323, 61373, 61423, 61473, 61522, 61571, 61620, 61669, 61717, 61765, 61813, 61860, 61907, 61954, 62001, 62047, 62093, 62139, 62184, 62229, 62274, 62318, 62362, 62406, 62450, 62493, 62536, 62579, 62621, 62663, 62705, 62746, 62787, 62828, 62869, 62909, 62949, 62989, 63028, 63067, 63106, 63144, 63183, 63220, 63258, 63295, 63332, 63369, 63405, 63441, 63477, 63512, 63548, 63582, 63617, 63651, 63685, 63719, 63752, 63785, 63818, 63850, 63882, 63914, 63946, 63977, 64008, 64038, 64068, 64098, 64128, 64157, 64186, 64215, 64244, 64272, 64300, 64327, 64354, 64381, 64408, 64434, 64460, 64486, 64511, 64536, 64561, 64585, 64609, 64633, 64657, 64680, 64703, 64725, 64748, 64770, 64791, 64813, 64834, 64854, 64875, 64895, 64915, 64934, 64953, 64972, 64991, 65009, 65027, 65045, 65062, 65079, 65096, 65112, 65128, 65144, 65159, 65174, 65189, 65204, 65218, 65232, 65246, 65259, 65272, 65284, 65297, 65309, 65321, 65332, 65343, 65354, 65364, 65374, 65384, 65394, 65403, 65412, 65421, 65429, 65437, 65444, 65452, 65459, 65466, 65472, 65478, 65484, 65489, 65494, 65499, 65504, 65508, 65512, 65516, 65519, 65522, 65524, 65527, 65529, 65531, 65532, 65533, 65534, 65534, 65535};

static float motor_ov = MOTOR_OV_INITIAL;

// PID control modes
PID pid_angle, pid_rpm, pid_focIq, pid_focId;

// Encoder calibration
float encoder_calib_data[] = {7.91, 37.08, 66.35, 95.71, 126.4, 157.23, 187.73, 217.17, 246.35, 275.8, 306.47, 337.23};
float encoder_LUT[(int)ENCODER_RES];

// FOC
volatile float sin_el, cos_el;
volatile float foc_id = 0.0f, foc_iq = 0.0f;
volatile float i_alpha = 0.0f, i_beta = 0.0f;

// Sensorless
volatile float phase_delay = 0;
volatile uint8_t current_phase = 0;
volatile bool sensorless_flag = false;
volatile uint8_t comp_rshift, bemf_dir;
volatile unsigned char comp_u, comp_v, comp_w, comparator = 0;

// Motor modes
volatile motor_mode mode = MODE_OFF;
volatile motor_waveform_type waveform_mode = MOTOR_SVPWM;

static volatile float position = 0.0, pos_filt = 0.0, rpm = 0.0, acc = 0.0, power = 0.0, power_lpf = 0.0;
volatile float angle_el = 0.0;

float motor_polarity = 0;
float motor_zero_angle = 0.0;

void TIM4_IRQHandler(void) {
	if(TIM4->SR & 0x1) {
		TIM4->SR &= ~(0x1);
	}
}

#define POWER_LPF 0.01f
void ADC1_IRQHandler(void) {
	static uint8_t sample_cnt = 0;
	static int16_t pos_cnt;
	static float angle_el_compensated;

	if (ADC1->ISR & ADC_ISR_JEOC) {
		ADC1->ISR = ADC_ISR_JEOC;

		sample_cnt = (sample_cnt + 1) & 0b1;
		if(!sample_cnt) {
			LED1_ON();

			// Read ADCs
			adc_read_motor_isns();
			adc_read_other();

			// Read encoder value
			pos_cnt = ENC_TIM->CNT & ENCODER_RES_MASK;

//          position = encoder_LUT[pos_cnt] - motor_zero_angle + rpm * RPM_ADVANCE_FACTOR;
			position = ((float)pos_cnt * 360.0f / ENCODER_RES) - motor_zero_angle + rpm * RPM_ADVANCE_FACTOR;
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

			angle_el_compensated = angle_el + rpm * 6.0f * motor_pole_pairs * 0.00003f;
			angle_el_compensated -= 360.0f * (int)(angle_el_compensated * 0.002777778f);

			foc_current_calc(angle_el);

			static int32_t theta_q31;
			theta_q31 = (uint32_t)(angle_el_compensated * 11930464.711111f);
			CORDIC->WDATA = theta_q31;

			power += POWER_LPF*(power_lpf - power);

			// Read sin/cos from CORDIC
			sin_el = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Sin
			cos_el = ((float)(int32_t)CORDIC->RDATA) * 4.65661287e-10f; // Cos

			// pid_focId.setpoint = 0;
			if(mode != MODE_OFF) {
				if(fabsf(pid_focIq.setpoint) < 0.01f && fabsf(rpm) < 10.0f) {
					pid_focId.integral *= 0.95f;
					pid_focIq.integral *= 0.95f;
				}
				if(waveform_mode == MOTOR_FOC) {
					PID_compute(&pid_focIq, foc_iq, 0.00004f);
					PID_compute(&pid_focId, foc_id, 0.00004f);
				} else {
					PID_reset(&pid_focIq);
					PID_reset(&pid_focId);
					pid_focIq.output = power * vsns_vbat;
				}
				// if(power) {
				//  PID_compute(&pid_focIq, foc_iq, 0.00004f);
				//  PID_compute(&pid_focId, foc_id, 0.00004f);
				// } else {
				//  PID_reset(&pid_focIq);
				//  PID_reset(&pid_focId);
				// }
				// PID_compute(&pid_focIq, foc_iq, 0.00004f);
				// PID_compute(&pid_focId, foc_id, 0.00004f);

				setPhaseVoltage(pid_focIq.output, pid_focId.output, angle_el_compensated);
			}

			// update_motion_observer((int16_t)ENC_TIM->CNT, 0.00004f);

			LED1_OFF();
		} else {
			LED1_ON();
			adc_read_motor_isns();
			LED1_OFF();
		}   
	}
}

// RPM PID
float Kv = 0.0001;
void TIM5_IRQHandler(void) {
	static float rpm_feedforward;
	static float angle_max_rpm;

	if(TIM5->SR & 0x1) {
		TIM5->SR &= ~(0x1);

		LED1_ON();

		update_motion_observer((int16_t)ENC_TIM->CNT, 0.0001f);
			
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
			rpm_feedforward = 0;//pid_rpm.setpoint * Kv;

			if(pid_rpm.setpoint == 0 && fabs(rpm) < 25) {
				SetPower(0);
				pid_focIq.setpoint = 0;
			} else {
				// SetPower(pid_rpm.output);
				pid_focIq.setpoint = pid_rpm.output + rpm_feedforward;
			}
		}

		LED1_OFF();
	}
}

static float measured_pos = 0.0f, est_pos = 0.0f;

static inline void update_motion_observer(int16_t current_raw_count, float dt) {
	static float L1;
	static float L2;
	static float L3;

	static float est_vel = 0.0f;
	static float est_acc = 0.0f;
	static int16_t last_raw_count = 0;

	static float pos_prediction;
	static float vel_prediction;

	static int16_t delta;
	static float error;

	static float enc_res = ENCODER_RES;
	static float lambda;

	lambda = 30.0f;
	// if (rpm < 200.0f) {
	// 	lambda = 10.0f + (rpm * 0.15f);
	// }
	L1 = 3.0f * lambda;
	L2 = 3.0f * lambda * lambda;
	L3 = lambda * lambda * lambda;

	delta = current_raw_count - last_raw_count;
	if(delta >  ENCODER_RES/2) delta -= ENCODER_RES;
	if(delta < -ENCODER_RES/2) delta += ENCODER_RES;
	if (delta > 100 || delta < -100) {
		delta = 0;
	}

	last_raw_count = current_raw_count;
	
	measured_pos += (float)delta;

	pos_prediction = est_pos + (est_vel * dt) + (0.5f * est_acc * dt * dt);
	vel_prediction = est_vel + (est_acc * dt);

	error = measured_pos - pos_prediction;

	est_pos   = pos_prediction + (L1 * error * dt);
	est_vel   = vel_prediction + (L2 * error * dt);
	est_acc   = est_acc        + (L3 * error * dt);

	pos_filt += 0.5 * (est_pos * 360.0f / enc_res - pos_filt);
	rpm += 0.5 * ((est_vel * 60.0f) / enc_res - rpm); // RPM
	acc += 0.5 * ((est_acc * 60.0f) / enc_res - acc); // RPM/s
}

void reset_motion_observer() {
	measured_pos = 0.0f;
	est_pos = 0.0f;
}


float motor_l = 160E-6f;
float motor_kv = 400;
float motor_resistance = 1.15;

void setPhaseVoltage(float Uq, float Ud, float angle_el) {
	static float pwm_u, pwm_v, pwm_w;

	static float center;
	static float Umag, Uq_limit;
	static float Ualpha, Ubeta;
	static float pwm_min, pwm_max;
	static float w_e;
	static float motor_kv_s;
	static float motor_flux_linkage;

	static float offset = 0.0f;
	
	static uint8_t ov_flag = 0;
	static uint16_t *PWM_table;

	static float p;
	p = Uq / vsns_vbat; // Power percentage for non FOC
	
	p = p < -1.0f ? -1.0f : p > 1.0f ? 1.0f : p;
	
	// angle_el += motor_polarity;
	
	if(waveform_mode == MOTOR_FOC) {

		motor_kv_s = 1.0f/(motor_kv * 0.1047 * motor_pole_pairs);
		motor_flux_linkage = 5.513288954 / (motor_kv * motor_pole_pairs);
		w_e = rpm * 0.10472f * motor_pole_pairs;

		// Bemf feedforward
		Uq += w_e * motor_kv_s;

		// Resistance feedforward
		Uq += motor_resistance * pid_focIq.setpoint;

		// Inductance feedforward
		// Uq +=  w_e * (motor_l * foc_id + motor_flux_linkage);
		// Ud += -w_e * motor_l * foc_iq;

		Umag = sqrt(Uq*Uq + Ud*Ud);
		Uq_limit - sqrt(vsns_vbat*vsns_vbat - Ud*Ud);
		Uq = Uq < -Uq_limit ? -Uq_limit : Uq > Uq_limit ? Uq_limit : Uq;
//		if(Umag > 7) {
//			Uq *= 0.5;
//			Ud *= 0.5;
//		}

		// Inverse Park Transform
		Ualpha = (Ud*cos_el - Uq*sin_el) / vsns_vbat;
		Ubeta  = (Ud*sin_el + Uq*cos_el) / vsns_vbat;

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

//      pwm_u -= pwm_min;
//      pwm_v -= pwm_min;
//      pwm_w -= pwm_min;
		
	} else if(waveform_mode == MOTOR_SIN || waveform_mode == MOTOR_SVPWM || waveform_mode == MOTOR_SADDLE) {
		if(waveform_mode == MOTOR_SIN) {
			PWM_table = sin_lut;
			p *= 0.5f;
			offset = 1.0f;
		} else if(waveform_mode == MOTOR_SADDLE){
			PWM_table = saddle_lut;
			p *= 0.5f;
			offset = 1.0f;
		} else { // SVPWM
			PWM_table = SVPWM_table;
			p *= 0.5f;
			offset = 1.0f;
		}
		
		if(p < 0.0f) {
			angle_el += 180.0f;
			p = -p;
		}
		angle_el = -angle_el;
		
		pwm_u = (wave_lut(PWM_table, angle_el)          + offset) * p;
		pwm_v = (wave_lut(PWM_table, angle_el + 120.0f) + offset) * p;
		pwm_w = (wave_lut(PWM_table, angle_el + 240.0f) + offset) * p;
		
	} else if(waveform_mode == MOTOR_TRAPEZOID) {
		if(p < 0.0f) {
			angle_el += 180.0f;
			p = -p;
		}
		
		angle_el = round(normalizeAngle(angle_el-90.0f) / 60.0f);
		if(vsns_vbat > motor_ov) {
			pwm_u = 0.0f;
			pwm_v = 0.0f;
			pwm_w = 0.0f;
			MotorShort(0.5f);
		} else {
			MotorPhase((signed char)angle_el, p);
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

#define ISNS_LPF 0.7f
#define ISNS_OFFSET_LPF 0.0001f
static inline void adc_read_motor_isns() {
	static float temp_isns_v, temp_isns_w;
	static float isns_v_err, isns_w_err;
	static float amp_gain = ISNS_AMP_GAIN * ISNS_UVW_R;
	static float deadband = 0.007;

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

	// Clamp offset
	isns_v_offset = isns_v_offset < 1.55f ? 1.55f : isns_v_offset > 1.75f ? 1.75f : isns_v_offset;
	isns_w_offset = isns_w_offset < 1.55f ? 1.55f : isns_w_offset > 1.75f ? 1.75f : isns_w_offset;

	// if(temp_isns_v > (isns_v_offset - deadband) && temp_isns_v < (isns_v_offset + deadband)) {
	// 	temp_isns_v = isns_v_offset;
	// }
	// if(temp_isns_w > (isns_w_offset - deadband) && temp_isns_w < (isns_w_offset + deadband)) {
	// 	temp_isns_w = isns_w_offset;
	// }

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

#define FOC_IQ_LPF 0.8f
static inline void foc_current_calc(float angle_el) {
	static int32_t theta_q31;

	theta_q31 = (uint32_t)(angle_el * 11930464.711111f);
	CORDIC->WDATA = theta_q31;

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
	SetPower(p);
	current_phase = 0;
	waveform_mode = MODE_SENSORLESS;
	
	comp_rshift = 0;
	bemf_dir = 1;
	
	//TODO: Sensorless implementation
}

void TIM6_IRQHandler(void) {
	if(TIM6->SR & 0x1) {
		TIM6->SR &= ~(0x1);
		
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

volatile bool bemf_flag;
volatile uint8_t pwm_odd = 0;

void TIM7_IRQHandler(void) {
	if(TIM7->SR & 0x1) {
		TIM7->SR &= ~(0x1);

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
		pwm_odd = (pwm_odd + 1) % 2;
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
			MOTOR_TIM->CCR_U = val;

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
			MOTOR_TIM->CCR_U = val;

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
			MOTOR_TIM->CCR_V = val;

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
			MOTOR_TIM->CCR_V = val;

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
			MOTOR_TIM->CCR_W = val;

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
			MOTOR_TIM->CCR_W = val;

			break;

		MOTOR_TIM->CCER |= (CCE_U | CCE_V | CCE_W); // Enable all high channels

		// TODO: Check if this is needed
		MOTOR_TIM->EGR |= TIM_EGR_COMG;
	}
}

void MotorOff() {
	SetPower(0);

	// U - NC
	MOTOR_TIM->CCER |= CCNP_U;
	MOTOR_TIM->CCR_U = 0;

	// V - NC
	MOTOR_TIM->CCER |= CCNP_V;
	MOTOR_TIM->CCR_V = 0;

	// W - NC
	MOTOR_TIM->CCER |= CCNP_W;
	MOTOR_TIM->CCR_W = 0;

	// TODO: Check if this is needed
	MOTOR_TIM->EGR |= TIM_EGR_COMG;
}

void MotorShort(float p) {
	// p = 0:       float all phases
	// p = 1.0:     short all phases to ground
	// 0 < p < 1:   pwm (short/float) all phases to ground
	
	p = fabs(p) * PWM_MAX;

	MOTOR_TIM->CCER &= ~(CCE_U | CCE_V | CCE_W);

	MOTOR_TIM->CCER |= CCNP_U;
	MOTOR_TIM->CCER |= CCNP_V;
	MOTOR_TIM->CCER |= CCNP_W;

	MOTOR_TIM->CCR_U = p;
	MOTOR_TIM->CCR_V = p;
	MOTOR_TIM->CCR_W = p;

	// TODO: Check if this is needed
	MOTOR_TIM->EGR |= TIM_EGR_COMG;
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

	MOTOR_TIM->CCR_U = pwm_u;
	MOTOR_TIM->CCR_V = pwm_v;
	MOTOR_TIM->CCR_W = pwm_w;

	// TODO: Check if this is needed
	MOTOR_TIM->EGR |= TIM_EGR_COMG;
}

/*---------------------------------------------------------------------
 |                                PID                                 |
 ----------------------------------------------------------------------*/

void MotorPIDInit() {
	PID_init(&pid_angle);
	PID_init(&pid_rpm);
	PID_init(&pid_focIq);
	PID_init(&pid_focId);
														// Input Units	- Output Units
	PID_setGain(&pid_focIq,	0.12,	500.0,	0.0		);	// Iq (Amps)	- Volt
	PID_setGain(&pid_focId,	0.12,	500.0,	0.0		);	// Id (Amps)	- Volt
	PID_setGain(&pid_rpm,	0.01,	0.05,	0.005	);	// RPM			- Iq (Amps)
	PID_setGain(&pid_angle,	0.01,	0.005,	0.005	);	// Degrees		- Iq (Amps)	

	// Iq
	// PID_enableErrorConstrain(&pid_focIq);
	// PID_setErrorLimits(&pid_focIq, -0.5, 0.5);
	PID_enableIntegralConstrain(&pid_focIq);
	PID_setIntegralLimits(&pid_focIq, -7, 7);
	PID_enableOutputConstrain(&pid_focIq);
	PID_setOutputLimits(&pid_focIq, -12, 12);
	
	// Id
	// PID_enableErrorConstrain(&pid_focId);
	// PID_setErrorLimits(&pid_focId, -0.8, 0.8);
	PID_enableIntegralConstrain(&pid_focId);
	PID_setIntegralLimits(&pid_focId, -7, 7);
	PID_enableOutputConstrain(&pid_focId);
	PID_setOutputLimits(&pid_focId, -7, 7);

	// RPM
	PID_enableErrorConstrain(&pid_rpm);
	PID_setErrorLimits(&pid_rpm, -1000, 1000);
	PID_enableIntegralConstrain(&pid_rpm);
	PID_setIntegralLimits(&pid_rpm, -10, 10);
	PID_disableComputeDerivative(&pid_rpm);
	PID_enableOutputConstrain(&pid_rpm);
	PID_setOutputLimits(&pid_rpm, -15, 15);
	
	// Angle
	PID_enableErrorConstrain(&pid_angle);
	PID_setErrorLimits(&pid_rpm, -45, 45);
	PID_enableIntegralConstrain(&pid_angle);
	PID_setIntegralLimits(&pid_angle, -7, 7);
	PID_disableComputeDerivative(&pid_angle);
	PID_enableOutputConstrain(&pid_angle);
	PID_setOutputLimits(&pid_angle, -10, 10);
}

inline void ResetMotorPID() {
	PID_reset(&pid_angle);
	PID_reset(&pid_rpm);
}

inline void SetPower(float p) {
	power_lpf = p;
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

inline float GetPower() {
	return power_lpf;
}

inline void ResetPosition() {
	// TODO: ResetPosition()
	position = 0.0;
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

void interpolate_encoder_lut(float in[], unsigned int len) {
	int i, j, k;
	float delta = 0;
	float arr[(int)motor_pole_pairs*6][2];
	
	for(i = 0; i < len; i++) {
		arr[i][0] = in[i] * ENCODER_RES / 360.0f;
		arr[i][1] = (float)i / len * ENCODER_RES;
	}
	
	for(i = 0; i < ENCODER_RES; i++) {
		for(j = 0; j < len; j++) {
			k = (j + 1) % len;
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
	static float temp;
	angle = normalizeAngle(angle);
	static uint16_t index;
	
	index = (angle * 8.0f);
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

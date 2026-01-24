#include "diags.h"

#include <xc.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <math.h>

#include "pic32.h"
#include "ADC.h"
#include "USART.h"
#include "string_utils.h"
#include "BLDC.h"
#include "PWM.h"
#include "servo.h"
#include "TMP1075.h"
#include "bitbang_I2C.h"
#include "EEPROM.h"
#include "tones.h"

void printDiagsMenu();
char read_rx_char();

bool diags_spinMotor(char*);
bool diags_id(char*);
bool diags_calibrateEncoder(char*);
bool diags_motor(char*);
bool diags_encoder(char*);
bool diags_readTemperature(char*);
bool diags_readADC(char*);
bool diags_i2c(char*);
bool diags_tone(char*);
bool diags_comp(char*);
bool diags_reset(char*);
bool diags_LED(char *cmd);
bool diags_servo(char *cmd);
bool diags_eeprom(char *cmd);
void disp_eeprom_diff();

typedef bool (*diags_function)(char*);

typedef struct diags_menu_item {
	char name[100];
	char cmd[10];
	diags_function func;
} diags_menu_item;

const diags_menu_item diags_list[] = {
	{ .name = "Spin Motor",						.cmd = "spin",	.func = diags_spinMotor			},
	{ .name = "Board ID",						.cmd = "id",	.func = diags_id				},
	{ .name = "Calibrate Encoder",				.cmd = "calib",	.func = diags_calibrateEncoder	},
	{ .name = "Motor Control",					.cmd = "motor",	.func = diags_motor				},
	{ .name = "Rotary Encoder commands",		.cmd = "enc",	.func = diags_encoder			},
	{ .name = "Read Temperature sensors",		.cmd = "temp",	.func = diags_readTemperature	},
	{ .name = "Read ADCs",						.cmd = "adc",	.func = diags_readADC			},
	{ .name = "I2C commands",					.cmd = "i2c",	.func = diags_i2c				},
	{ .name = "Generate tones from the motor",	.cmd = "tone",	.func = diags_tone				},
	{ .name = "Comparator readings",			.cmd = "comp",	.func = diags_comp				},
	{ .name = "Turn LEDs on or off",			.cmd = "led",	.func = diags_LED				},
	{ .name = "Servo controls",					.cmd = "servo",	.func = diags_servo				},
	{ .name = "Save/read from eeprom",			.cmd = "eep",	.func = diags_eeprom			},
	{ .name = "Perform a software reset",		.cmd = "reset",	.func = diags_reset				}
};

unsigned char diags_list_len = sizeof(diags_list) / sizeof(diags_list[0]);

void diagsMenu() {
	unsigned char input[RX_BUFFER_SIZE];
	unsigned char ch = 0;
	unsigned char i;
	unsigned int input_len;
	bool flag;

	MotorOff();
	printDiagsMenu();
	USART3_send_str("[diags]: ");
	
	while(ch != 'x') {
		delay_ms(10);
		if(rx_rdy) {
			for(i = 0; rx_buffer[i] != '\0'; i++) {
				input[i] = rx_buffer[i];
			}
			input[i] = '\0';
			rx_rdy = 0;
			
			ch = input[0];
			
			USART3_send_str(input);
			USART3_send('\n');

			str_removeChar(input, '\r');
			str_removeChar(input, '\n');

			if(str_isEqual(input, "help")) {
				printDiagsMenu();

			} else if(ch == 'x') {
				mode = MODE_OFF;
				MotorOff();
				return;

			} else {
				flag = false;
				for(i = 0; i < diags_list_len; i++) {
					if(str_beginsWith(input, diags_list[i].cmd)) {
						input_len = str_len(input);
						if(input[input_len] == ' ' || input[input_len] == '\0') {
							if(diags_list[i].func(input)) {
								USART3_send_str("OK\n");
							} else {
								USART3_send_str("ERROR\n");
							}
							flag = true;
						}
					}
				}
				if(!flag && input[0] != '\0') {
					USART3_send_str("Incorrect diags command. \"help\" to see list of commands\n");
				}
			}
			USART3_send_str("[diags]: ");
		}
	}
}

char read_rx_char() {
	unsigned char ch = 0;
	if(rx_rdy) {
		ch = rx_buffer[0];
		rx_rdy = 0;
	}
	return ch;
}

void printDiagsMenu() {
	unsigned char i;
	for(i = 0; i < diags_list_len; i++) {
		USART3_send_str(diags_list[i].cmd);
		USART3_send_str(" - ");
		USART3_send_str(diags_list[i].name);
		USART3_send('\n');
	}
	USART3_send_str("x. Exit\n");
}

bool diags_spinMotor(char *cmd) {
	unsigned int i;
	unsigned char ch;
	
	FOC_TIMER_ON = 1; 
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(0.03, 0, (float)i);
			delay_ms(500);
			USART3_write_float(GetPosition(), 2);
			USART3_send('\n');
			delay_ms(250);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					mode = MODE_OFF;
					MotorOff();
					return true;
				}
			}
		}
	}
	
	return true;
}

bool diags_id(char *cmd) {
	unsigned char ch;
	static float diags_power = 0.03;
	static float diags_acceleration = 0.5, set_power, power, acceleration;
	char arg_val[20];
	
	const char help_str[] = "\
Commands to set/display id:\n\
id [x] : Display board id. Optionally set to x.\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	USART3_send_str("Board id: ");
	USART3_write_int(board_id);
	USART3_send('\n');

	if(str_getArgValue(cmd, "id", arg_val)) {
		if(str_isInt(arg_val)) {
			board_id = str_toInt(arg_val);
			USART3_send_str("Board id set to: \n");
			USART3_write_int(board_id);
			USART3_send('\n');
		}
	}
	
	return true;
}

bool diags_motor(char *cmd) {
	unsigned char ch;
	uint8_t i;
	static float diags_power = 0.05;
	static float diags_acceleration = 0.5, set_power, power, acceleration;
	char arg_val[20];
	
	const char help_str[] = "\
Commands to control the BLDC motor with the parameters:\n\
-p [x] : Set motor power to x (-2000, 2000)\n\
ramp [x] (-a [y]): Ramp motor power to x (-2000, 2000) an acceleration y\n\
-e [x] : Move motor to electrical degree x\n\
zero [x] : Move to zero (space vector: 100) x\n\
-s [x] : Move motor to six step phase x\n\
spin : Slowly spin the motor in 6 step commutation\n\
off : Turn motor off (coast)\n\
wave [s] : Display waveform type. Set waveform type to \"foc\", \"svpwm\", \"sin\", \"saddle\", or \"trapezoid\"\n\
polepairs [p] : Display pole pairs. Optionally set to p.\n\
dir [0/1] : Motor wiring direction. 0 for normal, 1 for reverse.\n\
advance [a] : Display foc degree advance. Optionally set to a.\n\
setpower [x] : Set power level for other commands to x (-1.0, 1.0)\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}

	// Set electrical angle
	if(str_getArgValue(cmd, "-p", arg_val)) {
		USART3_send_str("Motor power set to: ");
		USART3_write_float(str_toFloat(arg_val), 4);
		USART3_send('\n');
		SetPower(str_toFloat(arg_val) / 2000.0);

	} else if(str_getArgValue(cmd, "ramp", arg_val)) {
		set_power = str_toFloat(arg_val);
		acceleration = diags_acceleration;
		if(str_getArgValue(cmd, "-a", arg_val)) {
			acceleration = str_toFloat(arg_val);
		}

		USART3_send_str("Ramping motor power to: ");
		USART3_write_float(set_power, 4);
		USART3_send_str("\nAt an acceleration of ");
		USART3_write_float(acceleration, 2);
		USART3_send_str(" per ms");
		USART3_send('\n');

		// SetPower(GetPower() * 2000);
		if(set_power > power) {
			do {
				delay_ms(1);
				power += diags_acceleration;
				SetPower(power / 2000.0);
			} while(power < set_power);
		} else {
			do {
				delay_ms(1);
				power -= diags_acceleration;
				SetPower(power / 2000.0);
			} while(power > set_power);
		}
		USART3_send_str("Done\n");

	} else if(str_getArgValue(cmd, "-e", arg_val)) {
		USART3_send_str("Motor set to electrical angle: ");
		USART3_write_float(str_toFloat(arg_val), 4);
		USART3_send_str("\nAt power: ");
		USART3_write_float(diags_power, 2);
		USART3_send('\n');
		mode = MODE_OFF;
		setPhaseVoltage(diags_power, 0, str_toFloat(arg_val));
		
	} else if(str_getArgValue(cmd, "zero", arg_val)) {
		USART3_send_str("Moved to zero (space vector: 100)");
		USART3_send_str("\nAt power: ");
		USART3_write_float(diags_power, 2);
		USART3_send('\n');
		mode = MODE_OFF;
		setPhaseVoltage(diags_power, 0, str_toFloat(arg_val));
		
		// Set phase voltages to (1, 0, 0)
		MotorPhasePWM(diags_power, 0, 0);
	
	// Set 6 step phase
	} else if(str_getArgValue(cmd, "-s", arg_val)) {
		USART3_send_str("Move motor to six step phase: ");
		USART3_write_int(str_toInt(arg_val));
		USART3_send_str("\nAt power: ");
		USART3_write_float(diags_power, 2);
		USART3_send('\n');
		mode = MODE_OFF;
		MotorPhase(str_toInt(arg_val), diags_power);
		
	// Spin in 6 step
	} else if(str_getArgValue(cmd, "spin", arg_val)) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(diags_power, 0, (float)i);
			delay_ms(500);
			USART3_write_float(GetPosition(), 2);
			USART3_send('\n');
			delay_ms(250);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					mode = MODE_OFF;
					MotorOff();
					return true;
				}
			}
		}

	// Turn off motor
	} else if(str_getArgValue(cmd, "off", arg_val)) {
		USART3_send_str("Motor off\n");
		mode = MODE_OFF;
		MotorOff();

	// Set power
	} else if(str_getArgValue(cmd, "setpower", arg_val)) {
		diags_power = str_toFloat(arg_val);
		USART3_send_str("Power set to: ");
		USART3_write_float(diags_power, 4);
		USART3_send('\n');
	
	// Set motor pole pairs
	} else if(str_getArgValue(cmd, "polepairs", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			motor_pole_pairs = str_toInt(arg_val);
			USART3_send_str("Motor pole pairs set to: \n");
			USART3_write_int(motor_pole_pairs);
			USART3_send('\n');
		} else {
			USART3_send_str("Current motor pole pairs: \n");
			USART3_write_int(motor_pole_pairs);
			USART3_send('\n');
		}
	
	// Set foc degree advance
	} else if(str_getArgValue(cmd, "advance", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			foc_degree_advance = str_toInt(arg_val);
			USART3_send_str("FOC degree advance set to: \n");
			USART3_write_float(foc_degree_advance, 2);
			USART3_send('\n');
		} else {
			USART3_send_str("Current foc degree advance: \n");
			USART3_write_float(foc_degree_advance, 2);
			USART3_send('\n');
		}

	// Set waveform
	} else if(str_getArgValue(cmd, "wave", arg_val)) {
		
		if(str_isEqual(arg_val, "")) {
			USART3_send_str("Current motor waveform type: ");
			if(waveform_mode == MOTOR_FOC) {
				USART3_send_str("FOC\n");
			} else if(waveform_mode == MOTOR_SVPWM) {
				USART3_send_str("SVPWM\n");
			} else if(waveform_mode == MOTOR_SIN) {
				USART3_send_str("SIN\n");
			} else if(waveform_mode == MOTOR_SADDLE) {
				USART3_send_str("SADDLE\n");
			} else if(waveform_mode == MOTOR_TRAPEZOID) {
				USART3_send_str("TRAPEZOID\n");
			}
			
		} else if(str_isEqual(arg_val, "foc")) {
			waveform_mode = MOTOR_FOC;
			USART3_send_str("Waveform type set to: FOC\n");

		} else if(str_isEqual(arg_val, "svpwm")) {
			waveform_mode = MOTOR_SVPWM;
			USART3_send_str("Waveform type set to: SVPWM\n");

		} else if(str_isEqual(arg_val, "sin")) {
			waveform_mode = MOTOR_SIN;
			USART3_send_str("Waveform type set to: SIN\n");
			
		} else if(str_isEqual(arg_val, "saddle")) {
			waveform_mode = MOTOR_SADDLE;
			USART3_send_str("Waveform type set to: SADDLE\n");

		} else if(str_isEqual(arg_val, "trapezoid")) {
			waveform_mode = MOTOR_TRAPEZOID;
			USART3_send_str("Waveform type set to: Trapezoid (6 step)\n");

		} else {
			USART3_send_str("Incorrect arg for wave. Requires:\n");
			USART3_send_str("svpwm\n");
			USART3_send_str("sin\n");
			USART3_send_str("trapezoid\n");
		}
		
	// Set motor direction
	} else if(str_getArgValue(cmd, "dir", arg_val)) {
		if(arg_val[0] == '0' || arg_val[0] == '1') {
			motor_direction = arg_val[0] != '0';
			USART3_send_str("Motor direction set to: \n");
			USART3_write_float(motor_direction, 2);
			USART3_send('\n');
		} else {
			if(arg_val[0] != '\0') {
				USART3_send_str("Incorrect option. Use \"0\" or \"1\" \n");
			}
			USART3_send_str("Current motor direction: \n");
			USART3_write_int(motor_direction);
			USART3_send('\n');
		}

	} else {
		USART3_send_str(help_str);
		return true;
	}
	
	return true;
}

bool diags_encoder(char *cmd) {
	unsigned char ch;
	long int pos_cnt, ind_cnt;
	float pos;
	char arg_val[5];
	
	const char help_str[] = "\
Read rotary encoder data:\n\
-r : Output raw data scaled to degrees. No calibration or offset applied.\n\
zero [x]: Display stored zero offset. Set to x\n\
calib [on/off] : Enable/disable encoder calibration correction\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	// Display raw
	if(str_getArgValue(cmd, "-r", arg_val)) {
		while(1) {
			pos_cnt = POS1CNT;
			ind_cnt = INDX1CNT;
			
			USART3_write_int(pos_cnt);
			USART3_send('\n');
			delay_ms(50);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					return true;
				}
			}
		}

	// Set zero angle
	} else if(str_getArgValue(cmd, "zero", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			motor_zero_angle = str_toFloat(arg_val);
			USART3_send_str("Encoder zero offset set to: \n");
			USART3_write_float(motor_zero_angle, 4);
			USART3_send_str(" degrees\n");
		} else {
			USART3_send_str("Encoder zero offset: \n");
			USART3_write_float(motor_zero_angle, 4);
			USART3_send_str(" degrees\n");
		}
	
	// Encoder calibration correction
	} else if(str_getArgValue(cmd, "calib", arg_val)) {
		if(str_isEqual(arg_val, "on")) {
			interpolate_encoder_lut(encoder_calib_data, 32);
			USART3_send_str("Encoder calibration correction enabled\n");
		} else if(str_isEqual(arg_val, "off")) {
			init_encoder_lut();
			USART3_send_str("Encoder calibration correction disabled\n");
		}

	// Display calibrated encoder data
	} else {
		FOC_TIMER_ON = 1; 
		while(1) {
			USART3_write_float(GetPosition(), 2);
			USART3_send_str(", ");
			USART3_write_float(GetRPM(), 4);
			USART3_send('\n');
			delay_ms(50);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					return true;
				}
			}
		}
	}
	
	return true;
}

bool diags_calibrateEncoder(char *cmd) {
	unsigned char ch;
	unsigned int i, j, arr_indx;
	float power = 0.05;
	float pos = GetPosition(), pre_pos;
	long int pos_cnt, ind_cnt;
	
	pos_cnt = POS1CNT;
	ind_cnt = INDX1CNT;

	FOC_TIMER_ON = 0;
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i <= 360; i += 60) {
			setPhaseVoltage(power, 0, (float)i);
			delay_ms(100);
			
			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					mode = MODE_OFF;
					MotorOff();
					return true;
				}
			}
		}
		pre_pos = pos;
		
		pos_cnt = POS1CNT;
		ind_cnt = INDX1CNT;
		pos = pos_cnt * 360.0f / ENCODER_RES;
		while(pos < 0.0) {
			pos += 360.0;
		}
		while(pos > 360.0) {
			pos -= 360.0;
		}
		
		if(pre_pos - pos > 180) {
			break;
		}
	}

	float output[(int)motor_pole_pairs*6][2];

	arr_indx = 0;
	for(j = 0; j < motor_pole_pairs; j++) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(power, 0, (float)i);
			delay_ms(500);

			pos_cnt = POS1CNT;
			ind_cnt = INDX1CNT;
			pos = pos_cnt * 360.0f / ENCODER_RES;
			while(pos < 0.0) {
				pos += 360.0;
			}
			while(pos > 360.0) {
				pos -= 360.0;
			}

			output[arr_indx][0] = (360.0 * j + i) / (float)motor_pole_pairs;
			output[arr_indx][1] = pos;
			arr_indx++;

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					mode = MODE_OFF;
					MotorOff();
					return true;
				}
			}
		}
	}
	
	float zero_offset = output[0][1];
	
	for(arr_indx = 0; arr_indx < (motor_pole_pairs*6); arr_indx++) {
		output[arr_indx][1] -= zero_offset;
		while(output[arr_indx][1] < 0.0) {
			output[arr_indx][1] += 360.0;
		}
		while(output[arr_indx][1] > 360.0) {
			output[arr_indx][1] -= 360.0;
		}
	}
	
	USART3_write_float(zero_offset, 4);
	USART3_send('\n');

	for(arr_indx = 0; arr_indx < (motor_pole_pairs*6); arr_indx++) {
		USART3_write_float(output[arr_indx][0], 4);
		USART3_send(',');
		USART3_write_float(output[arr_indx][1], 4);
		USART3_send('\n');
	}
	
	mode = MODE_OFF;
	MotorOff();
	
	return true;
}

bool diags_readTemperature(char *cmd) {
	float t1 = 0, t2 = 0;
	int16_t t1_raw = 0, t2_raw = 0;
	char arg_val[5];
	static unsigned int delay = 100;
	float f;
	char ch;
	
	const char help_str[] = "\
Read temp sensor TMP1075:\n\
2 sensors on device near: MCU and FETs\n\
-s : Stream data\n\
-f [x]: Set streaming frequency to f [1, 10000] (Hz).\n\
-r : Output raw data\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 10000.0) {
			USART3_send_str("Invalid frequency\n");
		} else {
			USART3_send_str("Set streaming fequency to ");
			USART3_write_float(f, 4);
			USART3_send_str(" Hz\n");
			delay = 1000 / f;
		}
	}
	
	if(str_getArgValue(cmd, "-s", arg_val)) {
		USART3_send_str("MCU, FETs\n");
		while(1) {
			StartDelaymsCounter();
			TMP1075_getTemp(0, &t1);
			TMP1075_getTemp(1, &t2);
			USART3_write_float(t2, 2);
			USART3_send_str(", "); 
			USART3_write_float(t1, 2);
			USART3_send('\n');
			while(ms_counter() < delay);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					return true;
				}
			}
		}
	} else if(str_getArgValue(cmd, "-r", arg_val)) {
		TMP1075_getRawTemp(0, &t1_raw);
		TMP1075_getRawTemp(1, &t2_raw);

		USART3_send_str("0: "); 
		USART3_write_float(t1_raw, 2);
		USART3_send('\n'); 

		USART3_send_str("1: "); 
		USART3_write_float(t2_raw, 2);
		USART3_send('\n');

	} else {
		TMP1075_getTemp(0, &t1);
		TMP1075_getTemp(1, &t2);

		USART3_send_str("MCU: "); 
		USART3_write_float(t2, 2);
		USART3_send_str(" C\n"); 

		USART3_send_str("FETs: "); 
		USART3_write_float(t1, 2);
		USART3_send_str(" C\n"); 
	}
	
	return true;
}

bool diags_readADC(char *cmd) {
	char ch;
	float f;
	static uint16_t delay = 10;
	float isns_u, isns_v, isns_w, isns_vbat, vsns_vbat, vsns_12v;
	float vsns_u, vsns_v, vsns_w, vsns_x;
	char arg_val[5];
	
	const char help_str[] = "\
Read ADC data\n\
stream -[i/v] -f: Stream all adc data. -i: motor phase currents -v: motor phase voltages -f [x]: Set streaming frequency to f [1, 10000] (Hz)\
-r : Output raw data. No scaling applied.\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}

	adc_readAll();
	
	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 1000.0) {
			USART3_send_str("Invalid frequency\n");
		} else {
			USART3_send_str("Set streaming fequency to ");
			USART3_write_float(f, 4);
			USART3_send_str(" Hz\n");
			delay = 1000000 / f;
		}
	}

	if(str_getArgValue(cmd, "-r", arg_val)) {
		USART3_send_str("ADC0: "); 
		USART3_write_int(adc_data[0]);
		USART3_send('\n');
		
		USART3_send_str("ADC1: "); 
		USART3_write_int(adc_data[1]);
		USART3_send('\n');
		
		USART3_send_str("ADC2: "); 
		USART3_write_int(adc_data[2]);
		USART3_send('\n');

		USART3_send_str("ADC3: "); 
		USART3_write_int(adc_data[3]);
		USART3_send('\n'); 

		USART3_send_str("ADC4: "); 
		USART3_write_int(adc_data[4]);
		USART3_send('\n');
		
		USART3_send_str("ADC5: "); 
		USART3_write_int(adc_data[5]);
		USART3_send('\n');

		USART3_send_str("AN7: "); 
		USART3_write_int(adc_data[7]);
		USART3_send('\n');
		
		USART3_send_str("AN8: "); 
		USART3_write_int(adc_data[8]);
		USART3_send('\n');
		
		USART3_send_str("AN27: "); 
		USART3_write_int(adc_data[27]);
		USART3_send('\n');
	} if(str_getArgValue(cmd, "stream", arg_val)) {
		if(str_getArgValue(cmd, "-i", arg_val)) {
			while(1) {
				StartDelayusCounter();
				isns_u = ((float)adc_buffer[4][0][0] * ADC_CONV_FACTOR - isns_u_offset) / 20.0f / ISNS_UVW_R;
				isns_v = ((float)adc_buffer[1][0][0] * ADC_CONV_FACTOR - isns_v_offset) / 20.0f / ISNS_UVW_R;
				isns_w = -(isns_u + isns_v);
				USART3_write_float(isns_u, 3);
				USART3_send_str(", ");
				USART3_write_float(isns_v, 3);
				USART3_send_str(", ");
				USART3_write_float(isns_w, 3);
				USART3_send('\n');
				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		} else if(str_getArgValue(cmd, "-v", arg_val)) {
			while(1) {
				StartDelayusCounter();
				vsns_u = (float)adc_buffer[2][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_v = (float)adc_buffer[3][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_w = (float)adc_buffer[0][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_x = (float)adc_buffer[5][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				USART3_write_float(vsns_u, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_v, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_w, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_x, 2);
				USART3_send('\n');
				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		} else {
			while(1) {
				StartDelayusCounter();
				isns_u = ((float)adc_buffer[1][0][0] * ADC_CONV_FACTOR - isns_u_offset) / 20.0f / ISNS_UVW_R;
				isns_v = ((float)adc_buffer[4][0][0] * ADC_CONV_FACTOR - isns_v_offset) / 20.0f / ISNS_UVW_R;
				isns_w = -(isns_u + isns_v);
				vsns_u = (float)adc_buffer[2][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_v = (float)adc_buffer[3][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_w = (float)adc_buffer[0][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				vsns_x = (float)adc_buffer[5][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
				USART3_write_float(vsns_u, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_v, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_w, 2);
				USART3_send_str(", ");
				USART3_write_float(vsns_x, 2);
				USART3_send_str(", ");
				USART3_write_float(isns_u, 3);
				USART3_send_str(", ");
				USART3_write_float(isns_v, 3);
				USART3_send_str(", ");
				USART3_write_float(isns_w, 3);
				USART3_send('\n');
				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		}
	} else {
		isns_u = ((float)adc_buffer[1][0][0] * ADC_CONV_FACTOR - isns_u_offset) / 20.0f / ISNS_UVW_R;
		isns_v = ((float)adc_buffer[4][0][0] * ADC_CONV_FACTOR - isns_v_offset) / 20.0f / ISNS_UVW_R;
		isns_w = -(isns_u + isns_v);
		
		vsns_u = (float)adc_data[2] * ADC_CONV_FACTOR;
		vsns_v = (float)adc_data[3] * ADC_CONV_FACTOR;
		vsns_w = (float)adc_data[0] * ADC_CONV_FACTOR;
		vsns_x = (float)adc_data[5] * ADC_CONV_FACTOR;
		
		isns_vbat = ((float)(adc_data[27]) * ADC_CONV_FACTOR - 1.105) / 50.0f / ISNS_VBAT_R;
		vsns_vbat = (float)adc_data[7] * ADC_CONV_FACTOR / VSNS_VBAT_DIVIDER;
		vsns_12v = (float)adc_data[8] * ADC_CONV_FACTOR / VSNS_12V_DIVIDER;
		
		USART3_send_str("VSNS U: "); 
		USART3_write_float(vsns_u, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("VSNS V: "); 
		USART3_write_float(vsns_w, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("VSNS W: "); 
		USART3_write_float(vsns_v, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("VSNS X: "); 
		USART3_write_float(vsns_x, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("ISNS U: "); 
		USART3_write_float(isns_u, 3);
		USART3_send_str(" A\n");

		USART3_send_str("ISNS V: "); 
		USART3_write_float(isns_v, 3);
		USART3_send_str(" A\n");

		USART3_send_str("ISNS W: "); 
		USART3_write_float(isns_w, 3);
		USART3_send_str(" A\n");
		
		USART3_send_str("VSNS 12V: "); 
		USART3_write_float(vsns_12v, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("VSNS VBAT: "); 
		USART3_write_float(vsns_vbat, 2);
		USART3_send_str(" V\n");
		
		USART3_send_str("ISNS VBAT: "); 
		USART3_write_float(isns_vbat, 3);
		USART3_send_str(" A\n");
	}
	
	return true;
}

bool diags_i2c(char *cmd) {
	unsigned int i;
	char arg_val[25];
	char out_str[10];
	
	const char help_str[] = "\
I2C commands\n\
scan : Scan bus and report addresses found\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	// I2C scan
	if(str_getArgValue(cmd, "scan", arg_val)) {
		for(i = 1; i < 0x7F; i++) {
			if(I2C_CheckAddress(i)) {
				hexToStr(out_str, i);
				USART3_send_str(out_str);
				USART3_send('\n');
			}
		}
	} else {
		USART3_send_str(help_str);
	}
	
	return true;
}

bool diags_tone(char *cmd) {
	char arg_val[50];
	
	const char help_str[] = "\
Plays audio through the motor:\n\
note [note] : Note to be played. eg a4, cs5 (sharp), bf2 (flat) etc.\n\
freq [freq] : Frequency to be played in Hz.\n\
song : Play stored song.\n\
wav : Play stored wav file.\n\
rttll [str] : Play rttll formatted string\n\
off : Stop playing tones\n\
power [p]: Display motor power level. Optionally set it to p (0 - 1.0)\n\
amplitude [p]: Display audio amplitude (degrees). Optionally set it to p (0 - 360)\n\
wave [wave] : Set waveform type to [square] or [sin]\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	// Display raw
	if(str_getArgValue(cmd, "note", arg_val)) {
		str_toUpper(arg_val);
		USART3_send_str("Playing note: ");
		USART3_send_str(arg_val);
		USART3_send('\n');
		PlayNote(arg_val);

	} else if(str_getArgValue(cmd, "freq", arg_val)) {
		USART3_send_str("Playing frequency: ");
		USART3_write_float(str_toFloat(arg_val), 4);
		USART3_send_str(" Hz\n");
		PlayTone(str_toFloat(arg_val));

	} else if(str_getArgValue(cmd, "wav", arg_val)) {
		PlayWav();

	} else if(str_getArgValue(cmd, "song", arg_val)) {
		MetroidSaveTheme(1);

	} else if(str_getArgValue(cmd, "off", arg_val)) {
		StopTone();

	} else if(str_getArgValue(cmd, "rttll", arg_val)) {
		USART3_send_str("Playing rttll string\n");
		if(!PlayRTTLL(arg_val)) {
			USART3_send_str("Incorrect rttll string\n");
		}
		StopTone();
		
	} else if(str_getArgValue(cmd, "power", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			tone_power = str_toFloat(arg_val);
			USART3_send_str("Motor audio power level set to: \n");
			USART3_write_float(tone_power, 4);
			USART3_send('\n');
		} else {
			USART3_send_str("Current motor audio power level: \n");
			USART3_write_float(tone_power, 4);
			USART3_send('\n');
		}
		
	}  else if(str_getArgValue(cmd, "amplitude", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			tone_amplitude = str_toFloat(arg_val);
			USART3_send_str("Audio amplitude set to ");
			USART3_write_float(tone_amplitude, 2);
			USART3_send_str(" degrees\n");
		} else {
			USART3_send_str("Current audio amplitude is ");
			USART3_write_float(tone_amplitude, 4);
			USART3_send_str(" degrees\n");
		}
		
	} else if(str_getArgValue(cmd, "wave", arg_val)) {
		if(str_isEqual(arg_val, "square")) {
			tone_waveform = SQUARE;
			USART3_send_str("Set audio waveform type to ");
			USART3_send_str(arg_val);
			USART3_send('\n');

		} else if(str_isEqual(arg_val, "sin")) {
			tone_waveform = SIN;
			USART3_send_str("Set audio waveform type to ");
			USART3_send_str(arg_val);
			USART3_send('\n');
		} else {
			USART3_send_str("Incorrect wave type. Should be \"square\" or \"sin\"\n");
		}

	} else {
		USART3_send_str(help_str);
		return true;
	}

	return true;	
}

bool diags_LED(char *cmd) {
	char arg_val[5];
	
	const char help_str[] = "\
led <led no> <state>:\n\
<led no> : 0: blue led, 1: amber led\n\
<state> : 0: on, 1: off, t: toggle\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}

	if(cmd[4] == '0') {
		if(cmd[6] == '0') {
			USART3_send_str("LED0 turned off\n");
			LED0 = 0;
		} else if(cmd[6] == '1') {
			USART3_send_str("LED0 turned on\n");
			LED0 = 1;
		} else if(cmd[6] == 't') {
			USART3_send_str("LED0 toggled\n");
			LATDINV |= 1 << 6;
		} else {
			USART3_send_str("Incorrect LED state selected\n");
		}

	} else if(cmd[4] == '1') {
		if(cmd[6] == '0') {
			USART3_send_str("LED1 turned off\n");
			LED1 = 0;
		} else if(cmd[6] == '1') {
			USART3_send_str("LED1 turned on\n");
			LED1 = 1;
		} else if(cmd[6] == 't') {
			USART3_send_str("LED1 toggled\n");
			LATAINV |= 1 << 8;
		} else {
			USART3_send_str("Incorrect LED state selected\n");
		}
	} else {
		USART3_send_str("Incorrect LED number selected\n");
		USART3_send_str(help_str);
	}
	
	return true;
}

bool diags_servo(char *cmd) {
	char arg_val[50];
	
	const char help_str[] = "\
Control servo:\n\
us [pulse width] : Length of servo pulse in microseconds (int)\n\
brake : Coast BLDC, then apply the brakes\n\
off : Set servo pin to LOW\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}
	
	if(str_getArgValue(cmd, "us", arg_val)) {
		USART3_send_str("Setting servo pulse width to ");
		USART3_write_int(str_toInt(arg_val));
		USART3_send_str(" microseconds\n");
		setServo_us(str_toInt(arg_val));

	} else if(str_getArgValue(cmd, "brake", arg_val)) {
		USART3_send_str("Braking flywheel\n");
		servoBrake();

	} else if(str_getArgValue(cmd, "off", arg_val)) {
		USART3_send_str("Servo off\n");
		servoOff();

	} else {
		USART3_send_str(help_str);
		return true;
	}

	return true;	
}

bool diags_eeprom(char *cmd) {
	char arg_val[50];
	uint16_t i;
	
	const char help_str[] = "\
write/read from the eeprom:\n\
save : Save all variables\n\
restore : Restore all variables from eeprom\n\
read [x] [b]: Read b bytes at address x\n\
write [x] [b]: Write b bytes at address x\n\
disp : Display all keys stored\n\
diff : Display current vs eeprom diffn";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}

	if(str_getArgValue(cmd, "read", arg_val)) {
		USART3_send_str("Command not implemented yet");
	
	} else if(str_getArgValue(cmd, "write", arg_val)) {
		USART3_send_str("Command not implemented yet");
	
	} else if(str_getArgValue(cmd, "save", arg_val)) {
		disp_eeprom_diff();

		eeprom_board_id = board_id;
		eeprom_zero_offset = motor_zero_angle;
		eeprom_pole_pairs = motor_pole_pairs;
		eeprom_motor_direction = motor_direction;
		for(i = 0; i < 32; i++) {
			eeprom_encoder_calib_data[i] = encoder_calib_data[i];
		}
		eeprom_pid_angle[0] = pid_angle.kp;
		eeprom_pid_angle[1] = pid_angle.ki;
		eeprom_pid_angle[2] = pid_angle.kd;

		eeprom_pid_rpm[0] = pid_rpm.kp;
		eeprom_pid_rpm[1] = pid_rpm.ki;
		eeprom_pid_rpm[2] = pid_rpm.kd;

		eeprom_pid_foc_iq[0] = pid_focIq.kp;
		eeprom_pid_foc_iq[1] = pid_focIq.ki;
		eeprom_pid_foc_iq[2] = pid_focIq.kd;

		eeprom_pid_foc_id[0] = pid_focId.kp;
		eeprom_pid_foc_id[1] = pid_focId.ki;
		eeprom_pid_foc_id[2] = pid_focId.kd;

		EEPROM_writeAll();

	} else if(str_getArgValue(cmd, "restore", arg_val)) {
		disp_eeprom_diff();

		board_id = eeprom_board_id;
		motor_zero_angle = eeprom_zero_offset;
		motor_pole_pairs = eeprom_pole_pairs;
		motor_direction = eeprom_motor_direction;
		for(i = 0; i < 32; i++) {
			encoder_calib_data[i] = eeprom_encoder_calib_data[i];
		}
		pid_angle.kp = eeprom_pid_angle[0];
		pid_angle.ki = eeprom_pid_angle[1];
		pid_angle.kd = eeprom_pid_angle[2];

		pid_rpm.kp = eeprom_pid_rpm[0];
		pid_rpm.ki = eeprom_pid_rpm[1];
		pid_rpm.kd = eeprom_pid_rpm[2];

		pid_focIq.kp = eeprom_pid_foc_iq[0];
		pid_focIq.ki = eeprom_pid_foc_iq[1];
		pid_focIq.kd = eeprom_pid_foc_iq[2];

		pid_focId.kp = eeprom_pid_foc_id[0];
		pid_focId.ki = eeprom_pid_foc_id[1];
		pid_focId.kd = eeprom_pid_foc_id[2];

	} else if(str_getArgValue(cmd, "disp", arg_val)) {
		EEPROM_readAll();

		USART3_send_str("Board ID: ");
		USART3_write_int(eeprom_board_id);

		USART3_send_str("\nZero offset: ");
		USART3_write_float(eeprom_zero_offset, 2);
		USART3_send_str("\nPole Pairs: ");
		USART3_write_int(eeprom_pole_pairs);
		USART3_send_str("\nMotor direction: ");
		USART3_write_int(eeprom_motor_direction);

		USART3_send_str("\nAngle pid gains: ");
		USART3_write_float(eeprom_pid_angle[0], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_angle[1], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_angle[2], 2);

		USART3_send_str("\nRPM pid gains: ");
		USART3_write_float(eeprom_pid_rpm[0], 5);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_rpm[1], 5);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_rpm[2], 5);

		USART3_send_str("\nFOC Iq pid gains: ");
		USART3_write_float(eeprom_pid_foc_iq[0], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_iq[1], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_iq[2], 2);

		USART3_send_str("\nFOC Id pid gains: ");
		USART3_write_float(eeprom_pid_foc_id[0], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_id[1], 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_id[2], 2);

		USART3_send_str("\n");

	} else if(str_getArgValue(cmd, "diff", arg_val)) {
		disp_eeprom_diff();
	} else {
		USART3_send_str(help_str);
		return true;
	}

	return true;	
}

void disp_eeprom_diff() {
	EEPROM_readAll();
	USART3_send_str("Deltas\n");
	USART3_send_str("Keyname: current, EEPROM\n");

	if(board_id != eeprom_board_id) {
		USART3_send_str("Board ID: ");
		USART3_write_int(board_id);
		USART3_send_str(", ");
		USART3_write_int(eeprom_board_id);
		USART3_send_str("\n");
	}
	if(motor_zero_angle != eeprom_zero_offset) {
		USART3_send_str("Zero offset: ");
		USART3_write_float(motor_zero_angle, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_zero_offset, 2);
		USART3_send_str("\n");
	}
	if(motor_pole_pairs != eeprom_pole_pairs) {
		USART3_send_str("Pole pairs: ");
		USART3_write_int(motor_pole_pairs);
		USART3_send_str(", ");
		USART3_write_int(eeprom_pole_pairs);
		USART3_send_str("\n");
	}
	if(motor_direction != eeprom_motor_direction) {
		USART3_send_str("Motor direction: ");
		USART3_write_int(motor_direction);
		USART3_send_str(", ");
		USART3_write_int(eeprom_motor_direction);
		USART3_send_str("\n");
	}
	if(pid_angle.kp != eeprom_pid_angle[0]) {
		USART3_send_str("Angle PID kp: ");
		USART3_write_float(pid_angle.kp, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_angle[0], 2);
		USART3_send_str("\n");
	}
	if(pid_angle.ki != eeprom_pid_angle[1]) {
		USART3_send_str("Angle PID ki: ");
		USART3_write_float(pid_angle.ki, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_angle[1], 2);
		USART3_send_str("\n");
	}
	if(pid_angle.kd != eeprom_pid_angle[2]) {
		USART3_send_str("Angle PID kd: ");
		USART3_write_float(pid_angle.kd, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_angle[2], 2);
		USART3_send_str("\n");
	}

	if(pid_rpm.kp != eeprom_pid_rpm[0]) {
		USART3_send_str("RPM PID kp: ");
		USART3_write_float(pid_rpm.kp, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_rpm[0], 2);
		USART3_send_str("\n");
	}
	if(pid_rpm.ki != eeprom_pid_rpm[1]) {
		USART3_send_str("RPM PID ki: ");
		USART3_write_float(pid_rpm.ki, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_rpm[1], 2);
		USART3_send_str("\n");
	}
	if(pid_rpm.kd != eeprom_pid_rpm[2]) {
		USART3_send_str("RPM PID kd: ");
		USART3_write_float(pid_rpm.kd, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_rpm[2], 2);
		USART3_send_str("\n");
	}

	if(pid_focIq.kp != eeprom_pid_foc_iq[0]) {
		USART3_send_str("FOC Iq PID kp: ");
		USART3_write_float(pid_focIq.kp, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_iq[0], 2);
		USART3_send_str("\n");
	}
	if(pid_focIq.ki != eeprom_pid_foc_iq[1]) {
		USART3_send_str("FOC Iq PID ki: ");
		USART3_write_float(pid_focIq.ki, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_iq[1], 2);
		USART3_send_str("\n");
	}
	if(pid_focIq.kd != eeprom_pid_foc_iq[2]) {
		USART3_send_str("FOC Iq PID kp: ");
		USART3_write_float(pid_focIq.kd, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_iq[2], 2);
		USART3_send_str("\n");
	}

	if(pid_focId.kp != eeprom_pid_foc_id[0]) {
		USART3_send_str("FOC Id PID kp: ");
		USART3_write_float(pid_focId.kp, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_id[0], 2);
		USART3_send_str("\n");
	}
	if(pid_focId.ki != eeprom_pid_foc_id[1]) {
		USART3_send_str("FOC Id PID ki: ");
		USART3_write_float(pid_focId.ki, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_id[1], 2);
		USART3_send_str("\n");
	}
	if(pid_focId.kd != eeprom_pid_foc_id[2]) {
		USART3_send_str("FOC Id PID kd: ");
		USART3_write_float(pid_focId.kd, 2);
		USART3_send_str(", ");
		USART3_write_float(eeprom_pid_foc_id[2], 2);
		USART3_send_str("\n");
	}
}

bool diags_comp(char *cmd) {
	unsigned char comp, ch;
	char arg_val[5];

	float f;
	static unsigned int delay = 100;
	
	const char help_str[] = "\
Comparator output\n\
-s : Stream data until 'x' is entered\n\
-f [x]: Set streaming frequency to f [1, 10000] (Hz).\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		USART3_send_str(help_str);
		return true;
	}

	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 10000.0) {
			USART3_send_str("Invalid frequency\n");
		} else {
			USART3_send_str("Set streaming fequency to ");
			USART3_write_float(f, 4);
			USART3_send_str(" Hz\n");
			delay = 100000 / f;
		}
	}

	if(str_getArgValue(cmd, "-s", arg_val)) {
		USART3_send_str("U, V, W\n");
		while(1) {
			comp = comparator;
			StartDelayusCounter();
			USART3_write_int((comparator >> 2) & 1);
			USART3_send_str(", ");
			USART3_write_int((comparator >> 1) & 1);
			USART3_send_str(", ");
			USART3_write_int(comparator & 1);
			USART3_send('\n');
			while(us_counter() < delay);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					return true;
				}
			}
		}

	} else {
		comp = comparator;
		USART3_send_str("U: ");
		USART3_write_int((comparator >> 2) & 1);
		USART3_send_str("\nV: ");
		USART3_write_int((comparator >> 1) & 1);
		USART3_send_str("\nW: ");
		USART3_write_int(comparator & 1);
		USART3_send('\n');
	}
	
	return true;
}

bool diags_reset(char *cmd) {
	delay_ms(500);
	RSWRSTbits.SWRST = 1;
	RSWRST;
}

#include "diags.h"
#include "main.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "ADC.h"
#include "USART.h"
#include "string_utils.h"
#include "BLDC.h"
#include "PWM.h"
#include "TMP1075.h"
#include "EEPROM.h"
#include "tones.h"

void printDiagsMenu();
char read_rx_char();

uint8_t diags_spinMotor(char*);
uint8_t diags_id(char*);
uint8_t diags_calibrateEncoder(char*);
uint8_t diags_motor(char*);
uint8_t diags_encoder(char*);
uint8_t diags_readTemperature(char*);
uint8_t diags_readADC(char*);
uint8_t diags_i2c(char*);
uint8_t diags_tone(char*);
uint8_t diags_comp(char*);
uint8_t diags_reset(char*);
uint8_t diags_LED(char *cmd);
uint8_t diags_servo(char *cmd);
uint8_t diags_eeprom(char *cmd);
void disp_eeprom_diff();

typedef uint8_t (*diags_function)(char*);

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
	char input[RX_BUFFER_SIZE];
	char ch = 0;
	unsigned char i;
	unsigned int input_len;
	uint8_t flag;

	MotorOff();
	printDiagsMenu();
	printf("[diags]: ");
	
	while(ch != 'x') {
		HAL_Delay(10);
		if(rx_rdy) {
			for(i = 0; rx_buffer[i] != '\0'; i++) {
				input[i] = rx_buffer[i];
			}
			input[i] = '\0';
			rx_rdy = 0;
			
			ch = input[0];
			
			printf("%s\n", input);

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
								printf("OK\n");
							} else {
								printf("ERROR\n");
							}
							flag = true;
						}
					}
				}
				if(!flag && input[0] != '\0') {
					printf("Incorrect diags command. \"help\" to see list of commands\n");
				}
			}
			printf("[diags]: ");
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
		printf("%s - %s\n", diags_list[i].cmd, diags_list[i].name);
	}
	printf("exit. Exit\n");
}

uint8_t diags_spinMotor(char *cmd) {
	unsigned int i;
	unsigned char ch;
	
	FOC_TIMER_ON(); 
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(0.03, 0, (float)i);
			HAL_Delay(500);
			printf("%.2f\n", GetPosition());
			HAL_Delay(250);

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

uint8_t diags_id(char *cmd) {
	char arg_val[20];
	
	const char help_str[] = "\
Commands to set/display id:\n\
id [x] : Display board id. Optionally set to x.\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	
	printf("Board id: %d\n", board_id);

	if(str_getArgValue(cmd, "id", arg_val)) {
		if(str_isInt(arg_val)) {
			board_id = str_toInt(arg_val);
			printf("Board id set to: %d\n", board_id);
		}
	}
	
	return true;
}

uint8_t diags_motor(char *cmd) {
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
		printf(help_str);
		return true;
	}

	// Set electrical angle
	if(str_getArgValue(cmd, "-p", arg_val)) {
		printf("Motor power set to: %.4f\n", str_toFloat(arg_val));
		SetPower(str_toFloat(arg_val) / 2000.0);

	} else if(str_getArgValue(cmd, "ramp", arg_val)) {
		set_power = str_toFloat(arg_val);
		acceleration = diags_acceleration;
		if(str_getArgValue(cmd, "-a", arg_val)) {
			acceleration = str_toFloat(arg_val);
		}

		printf("Ramping motor power to: %.4f\n", set_power);
		printf("\nAt an acceleration of %.2f per ms\n", acceleration);
		// SetPower(GetPower() * 2000);
		if(set_power > power) {
			do {
				HAL_Delay(1);
				power += diags_acceleration;
				SetPower(power / 2000.0);
			} while(power < set_power);
		} else {
			do {
				HAL_Delay(1);
				power -= diags_acceleration;
				SetPower(power / 2000.0);
			} while(power > set_power);
		}
		printf("Done\n");

	} else if(str_getArgValue(cmd, "-e", arg_val)) {
		printf("Motor set to electrical angle: %.4f\n", str_toFloat(arg_val));
		printf("With power = %.2f\n", diags_power);
		mode = MODE_OFF;
		setPhaseVoltage(diags_power, 0, str_toFloat(arg_val));
		
	} else if(str_getArgValue(cmd, "zero", arg_val)) {
		printf("Moved to zero (space vector: 100)\n");
		printf("With power = %.2f\n", diags_power);
		mode = MODE_OFF;
		setPhaseVoltage(diags_power, 0, str_toFloat(arg_val));
		
		// Set phase voltages to (1, 0, 0)
		MotorPhasePWM(diags_power, 0, 0);
	
	// Set 6 step phase
	} else if(str_getArgValue(cmd, "-s", arg_val)) {
		printf("Move motor to six step phase: %d\n", str_toInt(arg_val));
		printf("With power = %.2f\n", diags_power);
		mode = MODE_OFF;
		MotorPhase(str_toInt(arg_val), diags_power);
		
	// Spin in 6 step
	} else if(str_getArgValue(cmd, "spin", arg_val)) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(diags_power, 0, (float)i);
			HAL_Delay(500);
			printf("%.2f\n, ", GetPosition());
			HAL_Delay(250);

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
		printf("Motor off\n");
		mode = MODE_OFF;
		MotorOff();

	// Set power
	} else if(str_getArgValue(cmd, "setpower", arg_val)) {
		diags_power = str_toFloat(arg_val);
		printf("Power set to: %.4f\n", diags_power);
	
	// Set motor pole pairs
	} else if(str_getArgValue(cmd, "polepairs", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			motor_pole_pairs = str_toInt(arg_val);
			printf("Motor pole pairs set to: %.0f\n", motor_pole_pairs);
		} else {
			printf("Current motor pole pairs: %.0f\n", motor_pole_pairs);
		}
	
	// Set foc degree advance
	} else if(str_getArgValue(cmd, "advance", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			foc_degree_advance = str_toInt(arg_val);
			printf("Degree advance set to: %.2f\n", foc_degree_advance);
		} else {
			printf("Current degree advance: %.2f\n", foc_degree_advance);
		}

	// Set waveform
	} else if(str_getArgValue(cmd, "wave", arg_val)) {
		
		if(str_isEqual(arg_val, "")) {
			printf("Current motor waveform type: ");
			if(waveform_mode == MOTOR_FOC) {
				printf("FOC\n");
			} else if(waveform_mode == MOTOR_SVPWM) {
				printf("SVPWM\n");
			} else if(waveform_mode == MOTOR_SIN) {
				printf("SIN\n");
			} else if(waveform_mode == MOTOR_SADDLE) {
				printf("SADDLE\n");
			} else if(waveform_mode == MOTOR_TRAPEZOID) {
				printf("TRAPEZOID\n");
			}
			
		} else if(str_isEqual(arg_val, "foc")) {
			waveform_mode = MOTOR_FOC;
			printf("Waveform type set to: FOC\n");

		} else if(str_isEqual(arg_val, "svpwm")) {
			waveform_mode = MOTOR_SVPWM;
			printf("Waveform type set to: SVPWM\n");

		} else if(str_isEqual(arg_val, "sin")) {
			waveform_mode = MOTOR_SIN;
			printf("Waveform type set to: SIN\n");
			
		} else if(str_isEqual(arg_val, "saddle")) {
			waveform_mode = MOTOR_SADDLE;
			printf("Waveform type set to: SADDLE\n");

		} else if(str_isEqual(arg_val, "trapezoid")) {
			waveform_mode = MOTOR_TRAPEZOID;
			printf("Waveform type set to: Trapezoid (6 step)\n");

		} else {
			printf("Incorrect arg for wave. Requires:\n");
			printf("svpwm\n");
			printf("sin\n");
			printf("trapezoid\n");
		}
		
	// Set motor direction
	} else if(str_getArgValue(cmd, "dir", arg_val)) {
		if(arg_val[0] == '0' || arg_val[0] == '1') {
			motor_direction = arg_val[0] != '0';
			printf("Motor direction set to: %d\n", motor_direction);
		} else {
			if(arg_val[0] != '\0') {
				printf("Incorrect option. Use \"0\" or \"1\" \n");
			}
			printf("Current motor direction: %d\n", motor_direction);
		}

	} else {
		printf(help_str);
		return true;
	}
	
	return true;
}

uint8_t diags_encoder(char *cmd) {
	char ch;
	int16_t pos_cnt;
	char arg_val[5];
	
	const char help_str[] = "\
Read rotary encoder data:\n\
-r : Output raw data scaled to degrees. No calibration or offset applied.\n\
zero [x]: Display stored zero offset. Set to x\n\
calib [on/off] : Enable/disable encoder calibration correction\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	
	// Display raw
	if(str_getArgValue(cmd, "-r", arg_val)) {
		while(1) {
			pos_cnt = ENC_TIM->CNT;
			
			printf("%d\n", pos_cnt);
			HAL_Delay(50);

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
			printf("Encoder zero offset set to: %.4f degrees\n", motor_zero_angle);
		} else {
			printf("Encoder zero offset: %.4f degrees\n", motor_zero_angle);
		}
	
	// Encoder calibration correction
	} else if(str_getArgValue(cmd, "calib", arg_val)) {
		if(str_isEqual(arg_val, "on")) {
			interpolate_encoder_lut(encoder_calib_data, 32);
			printf("Encoder calibration correction enabled\n");
		} else if(str_isEqual(arg_val, "off")) {
			init_encoder_lut();
			printf("Encoder calibration correction disabled\n");
		}

	// Display calibrated encoder data
	} else {
		FOC_TIMER_ON(); 
		while(1) {
			printf("%.2f, %.4f\n, ", GetPosition(), GetRPM());
			HAL_Delay(50);

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

uint8_t diags_calibrateEncoder(char *cmd) {
	unsigned char ch;
	unsigned int i, j, arr_indx;
	float power = 0.05;
	float pos = GetPosition(), pre_pos;
	int16_t pos_cnt;
	
	pos_cnt = ENC_TIM->CNT;

	FOC_TIMER_OFF();
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i <= 360; i += 60) {
			setPhaseVoltage(power, 0, (float)i);
			HAL_Delay(100);
			
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
		
		pos_cnt = ENC_TIM->CNT;
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
			HAL_Delay(500);

			pos_cnt = ENC_TIM->CNT;
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
	
	printf("%.4f\n", zero_offset);

	for(arr_indx = 0; arr_indx < (motor_pole_pairs*6); arr_indx++) {
		printf("%.4f, %.4f\n", output[arr_indx][0], output[arr_indx][1]);
	}
	
	mode = MODE_OFF;
	MotorOff();
	
	return true;
}

uint8_t diags_readTemperature(char *cmd) {
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
		printf(help_str);
		return true;
	}
	
	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 10000.0) {
			printf("Invalid frequency\n");
		} else {
			printf("Set streaming fequency to %.4f Hz\n", f);
			delay = 1000 / f;
		}
	}
	
	if(str_getArgValue(cmd, "-s", arg_val)) {
		printf("MCU, FETs\n");
		while(1) {
			uint32_t current_tick = HAL_GetTick();
			TMP1075_getTemp(0, &t1);
			TMP1075_getTemp(1, &t2);
			printf("%.2f, %.2f\n", t2, t1);
			while((HAL_GetTick() - current_tick) < delay);

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

		printf("0: %d\n", t1_raw);
		printf("1: %d\n", t2_raw);

	} else {
		TMP1075_getTemp(0, &t1);
		TMP1075_getTemp(1, &t2);

		printf("MCU: %.2f C\n", t2);
		printf("FETs: %.2f C\n", t1);
	}
	
	return true;
}

uint8_t diags_readADC(char *cmd) {
	char ch;
	float f;
	static uint16_t delay = 10;
	float isns_vbat, vsns_vbat;
	float vsns_u, vsns_v, vsns_w, vsns_x;
	char arg_val[5];
	
	const char help_str[] = "\
Read ADC data\n\
stream -[i/v] -f: Stream all adc data. -i: motor phase currents -v: motor phase voltages -f [x]: Set streaming frequency to f [1, 10000] (Hz)\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	adc_readAll();
	
	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 1000.0) {
			printf("Invalid frequency\n");
		} else {
			printf("Set streaming fequency to %.4f Hz\n", f);
			delay = 1000000 / f;
		}
	}

	if(str_getArgValue(cmd, "stream", arg_val)) {
		if(str_getArgValue(cmd, "-i", arg_val)) {
			while(1) {
				// TODO: us counter
//				StartDelayusCounter();

				printf("%.3f, %.3f, %.3f\n", isns_u, isns_v, isns_w);

				//TODO: us counter
//				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		} else if(str_getArgValue(cmd, "-v", arg_val)) {
			while(1) {
				//TODO: us counter
//				StartDelayusCounter();
				
				// TODO: VSNS ADC
				vsns_u = 0;
				vsns_v = 0;
				vsns_w = 0;
				vsns_x = 0;

				printf("%.2f, %.2f, %.2f, %.2f\n", vsns_u, vsns_v, vsns_w, vsns_x);

				//TODO: us counter
//				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		} else {
			while(1) {
				//TODO: us counter
//				StartDelayusCounter();

				// TODO: VSNS ADC
				vsns_u = 0;
				vsns_v = 0;
				vsns_w = 0;
				vsns_x = 0;

				printf("%.2f, %.2f, %.2f, %.2f, %.3f, %.3f, %.3f\n", vsns_u, vsns_v, vsns_w, vsns_x, isns_u, isns_v, isns_w);

				//TODO: us counter
//				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		}
	} else {
		// TODO: VSNS ADC
		vsns_u = 0;
		vsns_v = 0;
		vsns_w = 0;
		vsns_x = 0;
		isns_vbat = 0;
		vsns_vbat = 0;
		
		printf("VSNS U: %.2f V\n", vsns_u);		
		printf("VSNS V: %.2f V\n", vsns_w);		
		printf("VSNS W: %.2f V\n", vsns_v);		
		printf("VSNS X: %.2f V\n", vsns_x);		
		printf("ISNS U: %.3f A\n", isns_u);
		printf("ISNS V: %.3f A\n", isns_v);
		printf("ISNS W: %.3f A\n", isns_w);
		printf("VSNS VBAT: %.2f V\n", vsns_vbat);
		printf("ISNS VBAT: %.3f A\n", isns_vbat);
	}
	
	return true;
}

uint8_t diags_i2c(char *cmd) {
	unsigned int i;
	char arg_val[25];
	
	const char help_str[] = "\
I2C commands\n\
scan : Scan bus and report addresses found\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	
	// I2C scan
	if(str_getArgValue(cmd, "scan", arg_val)) {
		for(i = 1; i < 0x7F; i++) {
			if(I2C_CheckAddress(i)) {
				printf("0x%X\n", i);
			}
		}
	} else {
		printf(help_str);
	}
	
	return true;
}

uint8_t diags_tone(char *cmd) {
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
		printf(help_str);
		return true;
	}
	
	// Display raw
	if(str_getArgValue(cmd, "note", arg_val)) {
		str_toUpper(arg_val);
		printf("Playing note: %s\n", arg_val);
		PlayNote(arg_val);

	} else if(str_getArgValue(cmd, "freq", arg_val)) {
		printf("Playing frequency: %.4f Hz\n", str_toFloat(arg_val));
		PlayTone(str_toFloat(arg_val));

	} else if(str_getArgValue(cmd, "wav", arg_val)) {
		PlayWav();

	} else if(str_getArgValue(cmd, "song", arg_val)) {
		MetroidSaveTheme(1);

	} else if(str_getArgValue(cmd, "off", arg_val)) {
		StopTone();

	} else if(str_getArgValue(cmd, "rttll", arg_val)) {
		printf("Playing rttll string\n");
		if(!PlayRTTLL(arg_val)) {
			printf("Incorrect rttll string\n");
		}
		StopTone();
		
	} else if(str_getArgValue(cmd, "power", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			tone_power = str_toFloat(arg_val);
			printf("Motor audio power level set to: %.4f\n", tone_power);
		} else {
			printf("Current motor audio power level: %.4f\n", tone_power);
		}
		
	}  else if(str_getArgValue(cmd, "amplitude", arg_val)) {
		if(char_isDigit(arg_val[0])) {
			tone_amplitude = str_toFloat(arg_val);
			printf("Audio amplitude set to %.2f degrees\n", tone_amplitude);
		} else {
			printf("Current audio amplitude is %.2f degrees\n", tone_amplitude);
		}
		
	} else if(str_getArgValue(cmd, "wave", arg_val)) {
		if(str_isEqual(arg_val, "square")) {
			tone_waveform = SQUARE;
			printf("Set audio waveform type to: SQUARE");

		} else if(str_isEqual(arg_val, "sin")) {
			tone_waveform = SIN;
			printf("Set audio waveform type to: SIN");
		} else {
			printf("Incorrect wave type. Should be \"square\" or \"sin\"\n");
		}

	} else {
		printf(help_str);
		return true;
	}

	return true;	
}

uint8_t diags_LED(char *cmd) {
	char arg_val[5];
	
	const char help_str[] = "\
led <led no> <state>:\n\
<led no> : 0: blue led, 1: amber led\n\
<state> : 0: on, 1: off, t: toggle\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	if(cmd[4] == '0') {
		if(cmd[6] == '0') {
			printf("LED0 turned off\n");
			LED0_OFF();
		} else if(cmd[6] == '1') {
			printf("LED0 turned on\n");
			LED0_ON();
		} else if(cmd[6] == 't') {
			printf("LED0 toggled\n");
			LED0_TOG();
		} else {
			printf("Incorrect LED state selected\n");
		}

	} else if(cmd[4] == '1') {
		if(cmd[6] == '0') {
			printf("LED1 turned off\n");
			LED1_OFF();
		} else if(cmd[6] == '1') {
			printf("LED1 turned on\n");
			LED1_ON();
		} else if(cmd[6] == 't') {
			printf("LED1 toggled\n");
			LED1_TOG();
		} else {
			printf("Incorrect LED state selected\n");
		}
	} else {
		printf("Incorrect LED number selected\n");
		printf(help_str);
	}
	
	return true;
}

uint8_t diags_servo(char *cmd) {
	char arg_val[50];
	
	const char help_str[] = "\
Control servo:\n\
us [pulse width] : Length of servo pulse in microseconds (int)\n\
brake : Coast BLDC, then apply the brakes\n\
off : Set servo pin to LOW\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	
	if(str_getArgValue(cmd, "us", arg_val)) {
		printf("Setting servo pulse width to %d us\n", str_toInt(arg_val));
		setServo_us(str_toInt(arg_val));

	} else if(str_getArgValue(cmd, "brake", arg_val)) {
		printf("Braking flywheel\n");
		// TODO: Servo brake sequence
//		servoBrake();

	} else if(str_getArgValue(cmd, "off", arg_val)) {
		printf("Servo off\n");
		servoOff();

	} else {
		printf(help_str);
		return true;
	}

	return true;	
}

uint8_t diags_eeprom(char *cmd) {
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
		printf(help_str);
		return true;
	}

	if(str_getArgValue(cmd, "read", arg_val)) {
		printf("Command not implemented yet");
	
	} else if(str_getArgValue(cmd, "write", arg_val)) {
		printf("Command not implemented yet");
	
	} else if(str_getArgValue(cmd, "save", arg_val)) {
		disp_eeprom_diff();

		eeprom_data.board_id = board_id;
		eeprom_data.zero_offset = motor_zero_angle;
		eeprom_data.pole_pairs = motor_pole_pairs;
		eeprom_data.motor_direction = motor_direction;
		for(i = 0; i < 32; i++) {
			eeprom_data.enc_calib[i] = encoder_calib_data[i];
		}
		eeprom_data.pid_angle[0] = pid_angle.kp;
		eeprom_data.pid_angle[1] = pid_angle.ki;
		eeprom_data.pid_angle[2] = pid_angle.kd;

		eeprom_data.pid_rpm[0] = pid_rpm.kp;
		eeprom_data.pid_rpm[1] = pid_rpm.ki;
		eeprom_data.pid_rpm[2] = pid_rpm.kd;

		eeprom_data.pid_foc_iq[0] = pid_focIq.kp;
		eeprom_data.pid_foc_iq[1] = pid_focIq.ki;
		eeprom_data.pid_foc_iq[2] = pid_focIq.kd;

		eeprom_data.pid_foc_id[0] = pid_focId.kp;
		eeprom_data.pid_foc_id[1] = pid_focId.ki;
		eeprom_data.pid_foc_id[2] = pid_focId.kd;

		ee_write();

	} else if(str_getArgValue(cmd, "restore", arg_val)) {
		disp_eeprom_diff();

		board_id = eeprom_data.board_id;
		motor_zero_angle = eeprom_data.zero_offset;
		motor_pole_pairs = eeprom_data.pole_pairs;
		motor_direction = eeprom_data.motor_direction;
		for(i = 0; i < 32; i++) {
			encoder_calib_data[i] = eeprom_data.enc_calib[i];
		}
		pid_angle.kp = eeprom_data.pid_angle[0];
		pid_angle.ki = eeprom_data.pid_angle[1];
		pid_angle.kd = eeprom_data.pid_angle[2];

		pid_rpm.kp = eeprom_data.pid_rpm[0];
		pid_rpm.ki = eeprom_data.pid_rpm[1];
		pid_rpm.kd = eeprom_data.pid_rpm[2];

		pid_focIq.kp = eeprom_data.pid_foc_iq[0];
		pid_focIq.ki = eeprom_data.pid_foc_iq[1];
		pid_focIq.kd = eeprom_data.pid_foc_iq[2];

		pid_focId.kp = eeprom_data.pid_foc_id[0];
		pid_focId.ki = eeprom_data.pid_foc_id[1];
		pid_focId.kd = eeprom_data.pid_foc_id[2];

	} else if(str_getArgValue(cmd, "disp", arg_val)) {
		ee_read();

		printf("Board ID: %d\n", eeprom_data.board_id);

		printf("Zero offset: %.2f\n", eeprom_data.zero_offset);
		printf("Pole Pairs: %d\n", eeprom_data.pole_pairs);
		printf("Motor direction: %d\n", eeprom_data.motor_direction);
		printf("Angle pid gains: %.4f, %.4f, %.4f\n", eeprom_data.pid_angle[0], eeprom_data.pid_angle[1], eeprom_data.pid_angle[2]);
		printf("RPM pid gains: %.5f, %.5f, %.5f\n", eeprom_data.pid_rpm[0], eeprom_data.pid_rpm[1], eeprom_data.pid_rpm[2]);
		printf("FOC Iq pid gains: %.4f, %.4f, %.4f\n", eeprom_data.pid_foc_iq[0], eeprom_data.pid_foc_iq[1], eeprom_data.pid_foc_iq[2]);
		printf("FOC Id pid gains: %.4f, %.4f, %.4f\n", eeprom_data.pid_foc_id[0], eeprom_data.pid_foc_id[1], eeprom_data.pid_foc_id[2]);

	} else if(str_getArgValue(cmd, "diff", arg_val)) {
		disp_eeprom_diff();
	} else {
		printf(help_str);
		return true;
	}

	return true;	
}

void disp_eeprom_diff() {
	ee_read();
	printf("Deltas\n");
	printf("Keyname: current, EEPROM\n");

	if(board_id != eeprom_data.board_id) {
		printf("Board ID: %d, %d\n", board_id, eeprom_data.board_id);
	}
	if(motor_zero_angle != eeprom_data.zero_offset) {
		printf("Zero offset: %.4f, %.4f\n", motor_zero_angle, eeprom_data.zero_offset);
	}
	if(motor_pole_pairs != eeprom_data.pole_pairs) {
		printf("Pole pairs: %.0f, %d\n", motor_pole_pairs, eeprom_data.pole_pairs);
	}
	if(motor_direction != eeprom_data.motor_direction) {
		printf("Motor direction: %d, %d\n", motor_direction, eeprom_data.motor_direction);
	}
	if(pid_angle.kp != eeprom_data.pid_angle[0]) {
		printf("Angle PID kp: %.4f, %.4f\n", pid_angle.kp, eeprom_data.pid_angle[0]);
	}
	if(pid_angle.ki != eeprom_data.pid_angle[1]) {
		printf("Angle PID ki: %.4f, %.4f\n", pid_angle.ki, eeprom_data.pid_angle[1]);
	}
	if(pid_angle.kd != eeprom_data.pid_angle[2]) {
		printf("Angle PID kd: %.4f, %.4f\n", pid_angle.kd, eeprom_data.pid_angle[2]);
	}
	if(pid_rpm.kp != eeprom_data.pid_rpm[0]) {
		printf("RPM PID kp: %.4f, %.4f\n", pid_rpm.kp, eeprom_data.pid_rpm[0]);
	}
	if(pid_rpm.ki != eeprom_data.pid_rpm[1]) {
		printf("RPM PID ki: %.4f, %.4f\n", pid_rpm.ki, eeprom_data.pid_rpm[1]);
	}
	if(pid_rpm.kd != eeprom_data.pid_rpm[2]) {
		printf("RPM PID kd: %.4f, %.4f\n", pid_rpm.kd, eeprom_data.pid_rpm[2]);
	}
	if(pid_focIq.kp != eeprom_data.pid_foc_iq[0]) {
		printf("FOC Iq PID kp: %.4f, %.4f\n", pid_focIq.kp, eeprom_data.pid_foc_iq[0]);
	}
	if(pid_focIq.ki != eeprom_data.pid_foc_iq[1]) {
		printf("FOC Iq PID ki: %.4f, %.4f\n", pid_focIq.ki, eeprom_data.pid_foc_iq[1]);
	}
	if(pid_focIq.kd != eeprom_data.pid_foc_iq[2]) {
		printf("FOC Iq PID kp: %.4f, %.4f\n", pid_focIq.kd, eeprom_data.pid_foc_iq[2]);
	}
	if(pid_focId.kp != eeprom_data.pid_foc_id[0]) {
		printf("FOC Id PID kp: %.4f, %.4f\n", pid_focId.kp, eeprom_data.pid_foc_id[0]);
	}
	if(pid_focId.ki != eeprom_data.pid_foc_id[1]) {
		printf("FOC Id PID ki: %.4f, %.4f\n", pid_focId.ki, eeprom_data.pid_foc_id[1]);
	}
	if(pid_focId.kd != eeprom_data.pid_foc_id[2]) {
		printf("FOC Id PID kd: %.4f, %.4f\n", pid_focId.kd, eeprom_data.pid_foc_id[2]);
	}
}

uint8_t diags_comp(char *cmd) {
	uint8_t comp, ch;
	char arg_val[5];

	float f;
	static unsigned int delay = 100;
	
	const char help_str[] = "\
Comparator output\n\
-s : Stream data until 'x' is entered\n\
-f [x]: Set streaming frequency to f [1, 10000] (Hz).\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0 || f > 10000.0) {
			printf("Invalid frequency\n");
		} else {
			printf("Set streaming fequency to %.4f Hz\n", f);
			delay = 100000 / f;
		}
	}

	if(str_getArgValue(cmd, "-s", arg_val)) {
		printf("U, V, W\n");
		while(1) {
			comp = comparator;
			// TODO: us counter
//			StartDelayusCounter();
			printf("%d, %d, %d\n", (comparator >> 2) & 1, (comparator >> 1) & 1, comparator & 1);
			// TODO: us counter
//			while(us_counter() < delay);

			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') {
					return true;
				}
			}
		}

	} else {
		comp = comparator;
		printf("U: %d\n", (comparator >> 2) & 1);
		printf("V: %d\n", (comparator >> 1) & 1);
		printf("W: %d\n", comparator & 1);
	}
	
	return true;
}

uint8_t diags_reset(char *cmd) {
	HAL_Delay(500);
	NVIC_SystemReset();
}

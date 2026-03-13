#include "diags.h"
#include "main.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "ADC.h"
#include "USART.h"
#include "string_utils.h"
#include "EEPROM.h"

#include "ADC.h"
#include "MPU6050.h"
#include "LIS3MDL.h"

void printDiagsMenu();
char read_rx_char();

uint8_t diags_escConnect(char*);
uint8_t diags_euler(char*);
uint8_t diags_rpm(char*);
uint8_t diags_readTemperature(char*);
uint8_t diags_readADC(char*);
uint8_t diags_i2c(char*);
uint8_t diags_reset(char*);
uint8_t diags_LED(char *cmd);
uint8_t diags_eeprom(char *cmd);
void disp_eeprom_diff();

typedef uint8_t (*diags_function)(char*);

typedef struct diags_menu_item {
	char name[100];
	char cmd[10];
	diags_function func;
} diags_menu_item;

const diags_menu_item diags_list[] = {
	{ .name = "Connect to ESCs",			.cmd = "esc",	.func = diags_escConnect		},
	{ .name = "Display euler angles",		.cmd = "euler",	.func = diags_euler				},
	{ .name = "Display motor rpms",			.cmd = "rpm",	.func = diags_rpm				},
	{ .name = "Read Temperature sensors",	.cmd = "temp",	.func = diags_readTemperature	},
	{ .name = "Read ADCs",					.cmd = "adc",	.func = diags_readADC			},
	{ .name = "I2C commands",				.cmd = "i2c",	.func = diags_i2c				},
	{ .name = "Turn LEDs on or off",		.cmd = "led",	.func = diags_LED				},
	{ .name = "Save/read from eeprom",		.cmd = "eep",	.func = diags_eeprom			},
	{ .name = "Perform a software reset",	.cmd = "reset",	.func = diags_reset				}
};

unsigned char diags_list_len = sizeof(diags_list) / sizeof(diags_list[0]);
float diags_power = 0.1f;

void diagsMenu() {
	char input[RX_BUFFER_SIZE];
	char ch = 0;
	unsigned char i;
	unsigned int input_len;
	uint8_t flag;

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

			} else if(str_beginsWith(input, "exit")) {
				printf("Exiting [diags]\n");
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

uint8_t diags_escConnect(char *cmd) {
	char ch;
	char arg_val[10];
	uint16_t i;
	uint16_t node_id;
	char input[RX_BUFFER_SIZE] __attribute__((aligned(32)));
	CanMessage_t can_msg;
	
	const char help_str[] = "\
Connect to ESCs and pipe UART over CAN:\n\
Type \"disconnect\" to leave the connection\n\
con [x] : x = ESC node ID\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	if(str_getArgValue(cmd, "con", arg_val)) {
		if(!str_isInt(arg_val)) {
			printf(help_str);
			return 0;
		}
		node_id = 0x301 + 0x10*(str_toInt(arg_val) - 1);
		while(1) {
			if(rx_rdy) {
				for(i = 0; rx_buffer[i] != '\0' && i < 64; i++) {
					input[i] = rx_buffer[i];
				}
				input[i] = '\0';
				rx_rdy = 0;

				if(str_beginsWith(input, "disconnect")) {
					printf("Disconnecting from ESC: Node id: 0x%x\n", node_id);
					return true;
				} else {
					CAN_send_serial(input, node_id);
				}
			}
			if(can_rxbuffer_available()) {
				pop_can_rxbuffer(&can_msg);
				if(can_msg.Identifier == (node_id - 0x100)) {
					can_msg.Data[63] = '\0';
					printf("%s", can_msg.Data);
				}
			}
		}
	} else {
		printf(help_str);
	}
}

uint8_t diags_euler(char *cmd) {
	char ch;
	char arg_val[10];
	
	const char help_str[] = "\
Display euler angles\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	MPU6050_Data_t imu_data;
	LIS3MDL_Data_t mag_data;
	float roll, pitch, yaw;
	uint32_t tick = 0;
	float dt;
	TIM12->CNT = 0;
	TIM12->CR1 |= 1;
	while(1) {
		if(g_imu.state == MPU6050_STATE_DATA_READY) {
			MPU6050_GetData(&g_imu, &imu_data);
			imu_data.gyro_x = (imu_data.gyro_x - g_imu.gyro_offset_x);
			imu_data.gyro_y = (imu_data.gyro_y - g_imu.gyro_offset_y);
			imu_data.gyro_z = (imu_data.gyro_z - g_imu.gyro_offset_z);

			dt = (float)TIM12->CNT * 0.000001f;

			if(g_mag.state == LIS3MDL_STATE_DATA_READY) {
				LIS3MDL_GetData(&g_mag, &mag_data);
				// TODO: Calibrate compass
				MadgwickQuaternionUpdateGyro(g_q, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, dt);
				MadgwickQuaternionUpdateAcc(g_q, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, dt);
			} else {
				MadgwickQuaternionUpdateGyro(g_q, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, dt);
				MadgwickQuaternionUpdateAcc(g_q, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, dt);
			}
			TIM12->CNT = 0;
		}

		if(HAL_GetTick() - tick > 10) {
			tick = HAL_GetTick();
			QuaternionToEuler(g_q, &roll, &pitch, &yaw);
			printf("%.3f\t%.3f\t%.3f\n", roll, pitch, yaw);
		}

		if(rx_rdy) {
			ch = read_rx_char();
			if(ch == 'x') {
				return true;
			}
		}
	}
}

uint8_t diags_rpm(char *cmd) {
	char ch;
	char arg_val[10];
	
	const char help_str[] = "\
Display motor rpms\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	uint32_t tick = 0;
	while(1) {
		if(HAL_GetTick() - tick > 10) {
			tick = HAL_GetTick();
			printf("%.3f\t%.3f\t%.3f\n", rpm_a, rpm_b, rpm_c);
		}

		if(rx_rdy) {
			ch = read_rx_char();
			if(ch == 'x') {
				return true;
			}
		}
	}
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
		if(f < 1.0f || f > 10000.0f) {
			printf("Invalid frequency\n");
		} else {
			printf("Set streaming fequency to %.4f Hz\n", f);
			delay = 1000.0f / f;
		}
	}
	
	if(str_getArgValue(cmd, "-s", arg_val)) {
		printf("MCU, FETs\n");
		while(1) {
			uint32_t current_tick = HAL_GetTick();
			// TODO: Temp
//			TMP1075_getTemp(0, &t1);
//			TMP1075_getTemp(1, &t2);
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
		// TODO: Temp
//		TMP1075_getRawTemp(0, &t1_raw);
//		TMP1075_getRawTemp(1, &t2_raw);

		printf("0: %d\n", t1_raw);
		printf("1: %d\n", t2_raw);

	} else {
		// TODO: Temp
//		TMP1075_getTemp(0, &t1);
//		TMP1075_getTemp(1, &t2);

		printf("MCU: %.2f C\n", t2);
		printf("FETs: %.2f C\n", t1);
	}
	
	return true;
}

uint8_t diags_readADC(char *cmd) {
	char ch;
	float f;
	static uint16_t delay = 10;
	char arg_val[5];
	
	const char help_str[] = "\
Read ADC data\n\
stream -[i/f/v] -f: Stream all adc data. -i: motor phase currents, -f: quadrature currents, -v: motor phase voltages -f [x]: Set streaming frequency to f [1, 10000] (Hz)\n";
	
	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}
	
	if(str_getArgValue(cmd, "-f", arg_val)) {
		f = str_toFloat(arg_val);
		if(f < 1.0f || f > 1000.0f) {
			printf("Invalid frequency\n");
		} else {
			printf("Set streaming fequency to %.4f Hz\n", f);
			delay = 1000000.0f / f;
		}
	}

	if(str_getArgValue(cmd, "stream", arg_val)) {
		if(str_getArgValue(cmd, "-i", arg_val)) {
			while(1) {
				// TODO: us counter
//				StartDelayusCounter();

				// TODO: ADC

				//TODO: us counter
//				while(us_counter() < delay);

				if(rx_rdy) {
					ch = read_rx_char();
					if(ch == 'x') {
						return true;
					}
				}
			}
		} else if(str_getArgValue(cmd, "-f", arg_val)) {
			while(1) {
				//TODO: us counter
//				StartDelayusCounter();

				// TODO: ADC

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

				// TODO: ADC
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

				// TODO: ADC
				
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
		
		// TODO: ADC
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
	
	// TODO: I2C scan
	if(str_getArgValue(cmd, "scan", arg_val)) {
		for(i = 1; i < 0x7F; i++) {
//			if(I2C_CheckAddress(i)) {
//				printf("0x%X\n", i);
//			}
		}
	} else {
		printf(help_str);
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

	// TODO: LEDs

	if(cmd[4] == '0') {
		if(cmd[6] == '0') {
			printf("LED0 turned off\n");
//			LED0_OFF();
		} else if(cmd[6] == '1') {
			printf("LED0 turned on\n");
//			LED0_ON();
		} else if(cmd[6] == 't') {
			printf("LED0 toggled\n");
//			LED0_TOG();
		} else {
			printf("Incorrect LED state selected\n");
		}

	} else if(cmd[4] == '1') {
		if(cmd[6] == '0') {
			printf("LED1 turned off\n");
//			LED1_OFF();
		} else if(cmd[6] == '1') {
			printf("LED1 turned on\n");
//			LED1_ON();
		} else if(cmd[6] == 't') {
			printf("LED1 toggled\n");
//			LED1_TOG();
		} else {
			printf("Incorrect LED state selected\n");
		}
	} else {
		printf("Incorrect LED number selected\n");
		printf(help_str);
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

		// TODO: Add EEPROM data

		ee_write();

	} else if(str_getArgValue(cmd, "restore", arg_val)) {
		disp_eeprom_diff();

		// TODO: Add EEPROM data

	} else if(str_getArgValue(cmd, "disp", arg_val)) {
		ee_read();

		// TODO: Add EEPROM data

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
	// TODO: Add EEPROM data
}

uint8_t diags_reset(char *cmd) {
	HAL_Delay(500);
	NVIC_SystemReset();
}

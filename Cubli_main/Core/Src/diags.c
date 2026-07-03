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
uint8_t diags_pose(char*);
uint8_t diags_lqr(char*);
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
	{ .name = "Stream euler angles + gyro",	.cmd = "euler",	.func = diags_euler				},
	{ .name = "Display motor rpms",			.cmd = "rpm",	.func = diags_rpm				},
	{ .name = "Detect face/edge/corner",	.cmd = "pose",	.func = diags_pose				},
	{ .name = "LQR bench tools (no motors)",.cmd = "lqr",	.func = diags_lqr				},
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

	// Streams the fused attitude + body rates that the I2C RX ISR maintains in
	// the background (globals roll/pitch/yaw, gyro_x/y/z). The fusion keeps
	// running while we sit here, so this command just prints -- it does NOT run
	// its own filter. Columns: roll pitch yaw (deg)  gx gy gz (deg/s).
	const char help_str[] = "\
Stream fused euler angles + gyro rates (deg, deg/s). Press 'x' to stop.\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	uint32_t tick = 0;
	printf("roll\tpitch\tyaw\tgx\tgy\tgz\n");
	while(1) {
		if(HAL_GetTick() - tick > 10) {
			tick = HAL_GetTick();
			printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
				roll, pitch, yaw, gyro_x, gyro_y, gyro_z);
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

uint8_t diags_pose(char *cmd) {
	char ch;
	char arg_val[10];

	const char help_str[] = "\
Detect which face / edge / corner the cube rests on, from the body-frame\n\
gravity vector with a 20 deg window. Hold the cube roughly still.\n\
Prints e.g. \"Face 4\", \"Edge 5\", \"Corner 7\" (our numbering). 'x' to stop.\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	// A body axis lies in the ground plane (perpendicular to gravity) when its
	// down-vector component is within 20 deg of horizontal: |component| < sin20.
	const float FLAT = 0.342f;   // sin(20 deg)
	uint32_t tick = 0;

	while(1) {
		if(HAL_GetTick() - tick > 200) {
			tick = HAL_GetTick();

			// Down direction in the body frame = -accel (accel reads +up at rest).
			float dx = -acc_x, dy = -acc_y, dz = -acc_z;
			float n = sqrtf(dx*dx + dy*dy + dz*dz);
			if(n < 0.1f) {
				printf("(no gravity signal -- is the cube static?)\n");
			} else {
				dx /= n; dy /= n; dz /= n;

				// side bit per axis: 1 if the down vector points toward +axis
				int bx = (dx > 0.0f) ? 1 : 0;
				int by = (dy > 0.0f) ? 1 : 0;
				int bz = (dz > 0.0f) ? 1 : 0;

				int flat_x = (dx < FLAT && dx > -FLAT);
				int flat_y = (dy < FLAT && dy > -FLAT);
				int flat_z = (dz < FLAT && dz > -FLAT);
				int nflat = flat_x + flat_y + flat_z;

				if(nflat == 2) {
					// FACE: the single non-flat axis is the down face's normal
					int axis = (!flat_x) ? 0 : ((!flat_y) ? 1 : 2);
					int side = (axis == 0) ? bx : (axis == 1) ? by : bz;
					printf("Face   %d", 2*axis + side);
				} else if(nflat == 1) {
					// EDGE: the flat axis is the one the edge runs parallel to
					int group = flat_x ? 0 : (flat_y ? 1 : 2);
					int loax  = (group == 0) ? 1 : 0;   // lower constrained axis
					int hiax  = (group == 2) ? 1 : 2;   // higher constrained axis
					int loS = (loax == 0) ? bx : (loax == 1) ? by : bz;
					int hiS = (hiax == 0) ? bx : (hiax == 1) ? by : bz;
					printf("Edge   %d", 4*group + loS + 2*hiS);
				} else {
					// CORNER: all three axes tilted (nflat == 0)
					printf("Corner %d", bx + 2*by + 4*bz);
				}

				printf("\td = (%+.2f, %+.2f, %+.2f)\n", (double)dx, (double)dy, (double)dz);
			}
		}

		if(rx_rdy) {
			ch = read_rx_char();
			if(ch == 'x') {
				return true;
			}
		}
	}
}

uint8_t diags_lqr(char *cmd) {
	char ch;
	char arg_val[20];

	const char help_str[] = "\
LQR bench tools -- motors are NEVER driven, this only prints:\n\
  lqr gains : compute gains from the physical params and print them\n\
  lqr sign  : for the edge the cube is on, print which motor should spin and\n\
              which way (CW/CCW from outside its face) to restore balance.\n\
              Tilt the cube by hand; the printed direction should OPPOSE the\n\
              tilt. Uses gravity-vector detection + the real edge gains.\n";

	if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
		printf(help_str);
		return true;
	}

	if(str_getArgValue(cmd, "gains", arg_val)) {
		cubli_init();
		printf("Edge gains  K = [angle  rate  wheel]\n");
		printf("  axis X (roll,  wheel B): [%.3f  %.3f  %.4f]\n",
			(double)g_ctrl.edge_gains[0].K[0], (double)g_ctrl.edge_gains[0].K[1], (double)g_ctrl.edge_gains[0].K[2]);
		printf("  axis Y (pitch, wheel A): [%.3f  %.3f  %.4f]\n",
			(double)g_ctrl.edge_gains[1].K[0], (double)g_ctrl.edge_gains[1].K[1], (double)g_ctrl.edge_gains[1].K[2]);
		printf("  axis Z (yaw,   wheel C): [%.3f  %.3f  %.4f]\n",
			(double)g_ctrl.edge_gains[2].K[0], (double)g_ctrl.edge_gains[2].K[1], (double)g_ctrl.edge_gains[2].K[2]);
		printf("initialized: %s\n", g_ctrl.initialized ? "yes" : "no (DARE failed)");
		return true;
	}

	if(str_getArgValue(cmd, "sign", arg_val)) {
		cubli_init();
		if(!g_ctrl.initialized) {
			printf("cubli_init failed -- check physical params\n");
			return true;
		}

		const float FLAT    = 0.342f;        // sin(20 deg): axis "horizontal" cutoff
		const float D2R     = 0.01745329f;   // deg -> rad
		const float RPM2RAD = 0.10471976f;   // rpm -> rad/s
		uint32_t tick = 0;

		printf("Tilt the cube on an edge; the printed spin should oppose the tilt.\n");
		while(1) {
			if(HAL_GetTick() - tick > 200) {
				tick = HAL_GetTick();

				float dx = -acc_x, dy = -acc_y, dz = -acc_z;   // down vector, body frame
				float n = sqrtf(dx*dx + dy*dy + dz*dz);
				if(n < 0.1f) { printf("(no gravity signal)\n"); }
				else {
					dx /= n; dy /= n; dz /= n;

					int flat_x = (dx < FLAT && dx > -FLAT);
					int flat_y = (dy < FLAT && dy > -FLAT);
					int flat_z = (dz < FLAT && dz > -FLAT);
					if(flat_x + flat_y + flat_z != 1) {
						printf("not on an edge (need exactly one axis horizontal)\n");
					} else {
						int group = flat_x ? 0 : (flat_y ? 1 : 2);

						// Signed tilt about the group axis, from the gravity vector.
						// a_cur = current up angle in the plane perpendicular to the
						// group axis; a_ide = ideal (balance) angle from the edge's
						// two down-face signs. Rotation order per axis: X:Y->Z,
						// Y:Z->X, Z:X->Y.
						float a_cur, a_ide, rate, omega;
						const char *motor; int face;
						if(group == 0) {            // parallel X, wheel B
							a_cur = atan2f(-dz, -dy);
							a_ide = atan2f((dz > 0 ? -1.0f : 1.0f), (dy > 0 ? -1.0f : 1.0f));
							rate  = gyro_x * D2R;
							omega = rpm_b * RPM2RAD;
							motor = "B"; face = 0;
						} else if(group == 1) {     // parallel Y, wheel A
							a_cur = atan2f(-dx, -dz);
							a_ide = atan2f((dx > 0 ? -1.0f : 1.0f), (dz > 0 ? -1.0f : 1.0f));
							rate  = gyro_y * D2R;
							omega = rpm_a * RPM2RAD;
							motor = "A"; face = 2;
						} else {                    // parallel Z, wheel C
							a_cur = atan2f(-dy, -dx);
							a_ide = atan2f((dy > 0 ? -1.0f : 1.0f), (dx > 0 ? -1.0f : 1.0f));
							rate  = gyro_z * D2R;
							omega = rpm_c * RPM2RAD;
							motor = "C"; face = 4;
						}

						float err = a_cur - a_ide;
						while(err >  3.14159265f) err -= 6.28318531f;
						while(err < -3.14159265f) err += 6.28318531f;

						float K0 = g_ctrl.edge_gains[group].K[0];
						float K1 = g_ctrl.edge_gains[group].K[1];
						float K2 = g_ctrl.edge_gains[group].K[2];
						float tau = -(K0 * err + K1 * rate + K2 * omega);

						printf("Motor %s  %-3s (outside face %d)   tilt=%+.1f deg   tau=%+.3f\n",
							motor, (tau > 0.0f) ? "CW" : "CCW", face,
							(double)(err / D2R), (double)tau);
					}
				}
			}
			if(rx_rdy) {
				ch = read_rx_char();
				if(ch == 'x') { return true; }
			}
		}
	}

	printf(help_str);
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

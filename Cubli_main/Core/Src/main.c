#include "main.h"
#include <stdio.h>
#include <string.h>
#include "ADC.h"
#include "diags.h"
#include "USART.h"
#include "string_utils.h"
#include "EEPROM.h"
#include "ahrs.h"
#include "cubli_lqr.h"
#include "MPU6050.h"
#include "LIS3MDL.h"
#include "pid.h"

#define PI 3.14159265358979f

DCACHE_HandleTypeDef hdcache1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim13;

FDCAN_TxHeaderTypeDef CAN_TxHeader;
volatile FDCAN_RxHeaderTypeDef RxHeader;
volatile uint8_t CAN_RxData[64];

MPU6050_Handle_t g_imu;
LIS3MDL_Handle_t g_mag;
volatile uint8_t imu_new_data = 0;

// Quaternion
float g_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float roll, pitch, yaw;

static CubliLQR g_ctrl;

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM13_Init(void);
static void TIM5_init(float);
static void TIM6_init(float);
static void TIM12_init(void);
uint32_t get_us();
void set_us(uint32_t);

volatile uint8_t initial_press = 0, initial_delay = 0;

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_3) {
		GPIOA->ODR &= ~(1U << 7U);

		if(initial_press) {
			TIM13->CNT = 0;
			TIM13->SR &= ~(0x1U);
			HAL_TIM_Base_Start_IT(&htim13);
		}
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_3) {
		GPIOA->ODR |= (1 << 7);
		if(initial_delay) {
			initial_press = 1;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM13 && initial_press) {
		GPIOA->ODR &= ~(1U << 2U);
	}
}

CanMessage_t can_rxBuffer[CAN_BUFFER_SIZE];
volatile uint8_t can_buffer_head = 0;
volatile uint8_t can_buffer_tail = 0;
volatile float rpm_a = 0.0f, rpm_b = 0.0f, rpm_c = 0.0f;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	static uint8_t next_head;
	static uint32_t can_float_temp;
	static uint8_t RxData[64];
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		 while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
			if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
				break;
			}
			if(RxHeader.Identifier >= 0x100 && RxHeader.Identifier < 0x200) { // Main rpm message
				can_float_temp = RxData[3] << 24 | RxData[2] << 16 | RxData[1] << 8 | RxData[0];
				if(RxHeader.Identifier == 0x100) {
					rpm_a = *(float*)((uint32_t*)&can_float_temp);
				} else if(RxHeader.Identifier == 0x110) {
					rpm_b = *(float*)((uint32_t*)&can_float_temp);
				} else if(RxHeader.Identifier == 0x120) {
					rpm_c = *(float*)((uint32_t*)&can_float_temp);
				}
			} else { // diags message
				next_head = (can_buffer_head + 1) % CAN_BUFFER_SIZE;
				can_rxBuffer[can_buffer_head].Identifier = RxHeader.Identifier;
				can_rxBuffer[can_buffer_head].DataLength = RxHeader.DataLength;
				memcpy(can_rxBuffer[can_buffer_head].Data, RxData, 64);
				can_buffer_head = next_head;
			}
		 }
	}
}

uint8_t can_rxbuffer_available() {
	return (can_buffer_head + CAN_BUFFER_SIZE - can_buffer_tail) % CAN_BUFFER_SIZE;
}

void pop_can_rxbuffer(CanMessage_t *ret) {
	if(can_buffer_head != can_buffer_tail) {
		HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
		*ret = can_rxBuffer[can_buffer_tail];
		can_buffer_tail = (can_buffer_tail + 1) % CAN_BUFFER_SIZE;
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
}
volatile uint8_t i2c_tx_flag;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c) {
	MPU6050_IRQHandler(&g_imu, hi2c);
	LIS3MDL_IRQHandler(&g_mag, hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c) {
	static uint32_t last_tick = get_us();
	MPU6050_Data_t imu_data;
	LIS3MDL_Data_t mag_data;

	MPU6050_IRQHandler(&g_imu, hi2c);
	LIS3MDL_IRQHandler(&g_mag, hi2c);

	if(g_imu.state == MPU6050_STATE_DATA_READY) {
		MPU6050_GetData(&g_imu, &imu_data);
		imu_data.gyro_x = (imu_data.gyro_x - g_imu.gyro_offset_x);
		imu_data.gyro_y = (imu_data.gyro_y - g_imu.gyro_offset_y);
		imu_data.gyro_z = (imu_data.gyro_z - g_imu.gyro_offset_z);

		dt = ((float)get_us() - last_tick) * 0.000001f;
		last_tick = get_us();

		if(g_mag.state == LIS3MDL_STATE_DATA_READY) {
			LIS3MDL_GetData(&g_mag, &mag_data);
			// TODO: Calibrate compass
			MadgwickQuaternionUpdateGyro(g_q, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, dt);
			MadgwickQuaternionUpdateAcc(g_q, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, dt);
		} else {
			MadgwickQuaternionUpdateGyro(g_q, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, dt);
			MadgwickQuaternionUpdateAcc(g_q, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, dt);
		}
		
		QuaternionToEuler(g_q, &roll, &pitch, &yaw);
		imu_new_data = 1;
	}
}

void CAN_send_motor(uint16_t can_id, char mode, float val) {
	uint8_t CAN_TxData[5];
	FDCAN_TxHeaderTypeDef CAN_TxHeader;

	CAN_TxHeader.Identifier = can_id;
	CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
	CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CAN_TxHeader.DataLength = FDCAN_DLC_BYTES_5;
	CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	CAN_TxHeader.FDFormat = FDCAN_FD_CAN;
	CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CAN_TxHeader.MessageMarker = 0;

	CAN_TxData[0] = mode;
	CAN_TxData[1] = ((uint8_t*)&val)[0];
	CAN_TxData[2] = ((uint8_t*)&val)[1];
	CAN_TxData[3] = ((uint8_t*)&val)[2];
	CAN_TxData[4] = ((uint8_t*)&val)[3];

	HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
	HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader, CAN_TxData);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

void TIM5_IRQHandler(void) {
	static uint16_t imu_cnt = 0, mag_cnt = 5;
	if(TIM5->SR & 0x1) {
		TIM5->SR &= ~(0x1U);

		mag_cnt++;
		imu_cnt++;

		if(mag_cnt >= 1000 && g_imu.state) { // 10Hz
			LIS3MDL_RequestData(&g_mag);
			mag_cnt = 0;
		}
		if(imu_cnt >= 10 && g_mag.state) { // 1Khz
			MPU6050_RequestData(&g_imu);
			imu_cnt = 0;
		}
	}
}

void cubli_control_update(void) {
    float angle_rad[3]   = { roll, pitch, yaw };
    float rate_rad_s[3]  = { gyro_x, gyro_y, gyro_z };
    float wheel_rad_s[3] = { rpm_a, rpm_b, rpm_c };

    float torque[3];
    CubliMode mode;

    if (cubli_lqr_update(&g_ctrl, angle_rad, rate_rad_s, wheel_rad_s, torque, &mode) != 0) {
        CAN_send_motor('P', 0x300, 0.0f);
        CAN_send_motor('P', 0x310, 0.0f);
        CAN_send_motor('P', 0x320, 0.0f);
        return;
    }

    CAN_send_motor('P', 0x300, torque[0]);
    CAN_send_motor('P', 0x310, torque[1]);
    CAN_send_motor('P', 0x320, torque[2]);
}

void cubli_init(void) {

	CubliParams p = {

		.physical = {
			.m        = 0.780f,
			.g        = 9.81f,
			.l_edge   = 0.0525f,    // half side length (m)
			.l_corner = 0.0455f,    // side * sqrt(3)/2 / 2

			// Measure with swing test per axis
			.I_edge_x = 0.00143f,
			.I_edge_y = 0.00143f,
			.I_edge_z = 0.00143f,

			// Measure with trifilar pendulum or CAD
			.Ix = 0.00035f,
			.Iy = 0.00035f,
			.Iz = 0.00035f,

			.Iw = 0.00125f,
		},

		.weights = {
			.edge_q_angle   = 1000.0f,
			.edge_q_rate    =   10.0f,
			.edge_q_wheel   =    1.0f,
			.edge_r_torque  =    1.0f,

			.corner_q_tilt      = 1000.0f,
			.corner_q_tilt_rate =   10.0f,
			.corner_q_yaw       =   50.0f,
			.corner_q_yaw_rate  =    1.0f,
			.corner_q_wheel     =    1.0f,
			.corner_r_torque    =    1.0f,
		},

		.setpoint = {
			.ki_sp    = 0.02f,
			.leak     = 0.05f,
			.deadband = 0.5f * (PI/180.0f),
			.max_offset = 5.0f * (PI/180.0f),
		},

		// ============================================================
		//  EDGE TABLE  (12 entries)
		//
		//  balance_axis:  0=roll, 1=pitch, 2=diagonal
		//  balance_sign:  +1 or -1, flip if motor drives wrong way
		//  wheel_index:   0=motorA, 1=motorB, 2=motorC
		//  gain_axis:     0=X gains, 1=Y gains, 2=Z gains
		//  nominal_sp_rad: fill from measurement
		//
		//  ref_balance_deg:    ~45 or ~-45
		//  ref_nonbalance_deg: multiple of 90 (0, 90, 180, 270)
		// ============================================================
		.edges = {
			// -- X-axis edges (4): wheel A, gain_axis 0 --
			// balance on roll, non-balance is pitch
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=0,   .balance_axis=0, .balance_sign=+1, .wheel_index=0, .gain_axis=0, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=90,  .balance_axis=0, .balance_sign=+1, .wheel_index=0, .gain_axis=0, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=180, .balance_axis=0, .balance_sign=+1, .wheel_index=0, .gain_axis=0, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=270, .balance_axis=0, .balance_sign=+1, .wheel_index=0, .gain_axis=0, .nominal_sp_rad=0 },

			// -- Y-axis edges (4): wheel B, gain_axis 1 --
			// balance on pitch, non-balance is roll
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=0,   .balance_axis=1, .balance_sign=+1, .wheel_index=1, .gain_axis=1, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=90,  .balance_axis=1, .balance_sign=+1, .wheel_index=1, .gain_axis=1, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=180, .balance_axis=1, .balance_sign=+1, .wheel_index=1, .gain_axis=1, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=270, .balance_axis=1, .balance_sign=+1, .wheel_index=1, .gain_axis=1, .nominal_sp_rad=0 },

			// -- Z-axis edges (4): wheel C, gain_axis 2, diagonal --
			// both roll and pitch near 45
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=45,  .balance_axis=2, .balance_sign=+1, .wheel_index=2, .gain_axis=2, .nominal_sp_rad=0 },
			{ .ref_balance_deg=45,  .ref_nonbalance_deg=-45, .balance_axis=2, .balance_sign=+1, .wheel_index=2, .gain_axis=2, .nominal_sp_rad=0 },
			{ .ref_balance_deg=-45, .ref_nonbalance_deg=45,  .balance_axis=2, .balance_sign=-1, .wheel_index=2, .gain_axis=2, .nominal_sp_rad=0 },
			{ .ref_balance_deg=-45, .ref_nonbalance_deg=-45, .balance_axis=2, .balance_sign=-1, .wheel_index=2, .gain_axis=2, .nominal_sp_rad=0 },
		},

		// ============================================================
		//  CORNER TABLE  (8 entries)
		//
		//  Fill ref_roll_deg, ref_pitch_deg from measurement.
		//  Fill nominal setpoints once measured.
		//  Placeholder values shown.
		// ============================================================
		.corners = {
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 }, // top corner (upright)
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 }, // fill remaining 7
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
			{ .ref_roll_deg= 0,  .ref_pitch_deg= 0,  .nominal_sp_alpha=0, .nominal_sp_beta=0, .nominal_sp_gamma=0 },
		},
	};

	if (cubli_lqr_init(&g_ctrl, &p) != 0) {
		Error_Handler();
	}
}

int main(void) {
	MPU_Config();
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DCACHE1_Init();
	MX_FDCAN1_Init();
	MX_I2C1_Init();
	MX_ICACHE_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM13_Init();
	TIM5_init(10000);
	// TIM6_init(1000);
	TIM12_init();

	GPIOC->ODR &= ~(1U << 13U); // CAN S
	HAL_TIM_Base_Stop_IT(&htim13);

	HAL_Delay(250);
	GPIOA->ODR |= 1 << 2;

	MPU6050_Config_t imu_cfg = {
		.accel_fs	= MPU6050_ACCEL_FS_4G,
		.gyro_fs	= MPU6050_GYRO_FS_1000DPS,
		.dlpf		= MPU6050_DLPF_184HZ,
		.sample_rate_div = 0U
	};
	MPU6050_Init(&g_imu, &hi2c1, MPU6050_ADDR_LOW, &imu_cfg);

	LIS3MDL_Config_t mag_cfg = {
		.odr	= LIS3MDL_ODR_10HZ,
		.fs		= LIS3MDL_FS_4GAUSS,
		.om		= LIS3MDL_OM_ULTRA_HIGH,
		.mode	= LIS3MDL_MODE_CONTINUOUS
	};
	LIS3MDL_Init(&g_mag, &hi2c1, LIS3MDL_ADDR_LOW, &mag_cfg);

	HAL_Delay(500);
	initial_delay = 1;

	printf("Start\n");

	uint16_t i;
	char rx_buffer_local[RX_BUFFER_SIZE];

	MPU6050_Data_t imu_data;
	LIS3MDL_Data_t mag_data;
	uint32_t now, an = HAL_GetTick();

	// Gyro offset calibration
	float g_avg_x = 0, g_avg_y = 0, g_avg_z = 0;
	float a_avg_x = 0, a_avg_y = 0, a_avg_z = 0;
	for(i = 0; i < 1000; i++) {
		now = HAL_GetTick();
		MPU6050_RequestData(&g_imu);
		while(g_imu.state != MPU6050_STATE_DATA_READY);
		MPU6050_GetData(&g_imu, &imu_data);
		g_avg_x += imu_data.gyro_x * 0.001f;
		g_avg_y += imu_data.gyro_y * 0.001f;
		g_avg_z += imu_data.gyro_z * 0.001f;

		a_avg_x += imu_data.accel_x * 0.001f;
		a_avg_y += imu_data.accel_y * 0.001f;
		a_avg_z += imu_data.accel_z * 0.001f;

		while(HAL_GetTick() - now < 1);
	}

	g_imu.gyro_offset_x = g_avg_x;
	g_imu.gyro_offset_y = g_avg_y;
	g_imu.gyro_offset_z = g_avg_z;

	QuaternionInit(g_q, a_avg_x, a_avg_y, a_avg_z, 0);

	TIM5->CR1 |= 1; // Turn on IMU timer

	const float RAD2DEG = 57.2957795f;

	PID pid_pitch, pid_roll;
	PID_init(&pid_pitch);
	PID_setGain(&pid_pitch,	1.0f, 0.0f, 0.2f); // Mad3506
//	PID_setGain(&pid_pitch,	2.0f, 0.0f, 0.2f); // Flysky
	PID_disableComputeDerivative(&pid_pitch);
	PID_enableOutputConstrain(&pid_pitch);
	PID_setOutputLimits(&pid_pitch, -25, 25);

	PID_setGain(&pid_roll,	1.5f, 0.0f, 0.2f); // Mad4006
	PID_disableComputeDerivative(&pid_roll);
	PID_enableOutputConstrain(&pid_roll);
	PID_setOutputLimits(&pid_roll, -25, 25);

	pid_pitch.setpoint = 0.0f;

	float ll, ul;
	float temp_output = 0;

	TIM12->CNT = 0;
	TIM12->CR1 |= 1;
	while(1) {

		if(rx_rdy) {
			HAL_NVIC_DisableIRQ(USART1_IRQn);
			for(i = 0; rx_buffer[i] != '\0'; i++) {
				rx_buffer_local[i] = rx_buffer[i];
			}
			rx_rdy = 0;
			HAL_NVIC_EnableIRQ(USART1_IRQn);
			rx_buffer_local[i] = '\0';

			str_removeChar(rx_buffer_local, '\n');
			str_removeChar(rx_buffer_local, '\r');

			if(str_beginsWith(rx_buffer_local, "diags")) {
				diagsMenu();
			}
		}

		if(imu_new_data) {
			imu_new_data = 0;
//			printf("%.2f\t%.2f\n", roll, pitch);
//			printf("%.2f\t%.2f\t%.2f\n", imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

			/*if(pitch > ll && pitch < ul) {
				ll = 32.0f;
				ul = 42.0f;

				#define ks 1.2f
				#define kv 0.00003f

				pid_pitch.derivative += 1.0f * (imu_data.gyro_y - pid_pitch.derivative);
				PID_compute(&pid_pitch, pitch, dt);

				pid_pitch.setpoint += (ks*pid_pitch.error + kv*rpm_c) * dt;

				if(fabsf(rpm_c) < 20) {
					if(fabsf(pid_pitch.output) > 1e-6f) {
						// pid_pitch.output += (pid_pitch.output > 0 ? 1.0f : -1.0f) * 0.02f;
						pid_pitch.output *= 2.0f;
					}
				}

//				CAN_send_motor(0x320, 'P', pid_pitch.output);
				printf("%.3f\t%.3f\n", pitch, pid_pitch.setpoint);
			} else {
				PID_reset(&pid_pitch);
				pid_pitch.setpoint = 37.4f;
				ll = pid_pitch.setpoint - 0.1f;
				ul = pid_pitch.setpoint + 0.1f;
			}*/

			if(roll > ll && roll < ul) {
				ll = 110.0f;
				ul = 160.0f;

				#define ks 0.6f
				#define kv 0.002f
				#define kleak 0.8f
				#define DEADBAND 0.3f

				pid_roll.derivative += 0.8f * (-imu_data.gyro_x - pid_roll.derivative);
				PID_compute(&pid_roll, roll, dt);

				if (fabsf(pid_roll.error) > DEADBAND) {
					pid_roll.setpoint += ks*pid_roll.error * dt;
				}
				pid_roll.setpoint += kv*rpm_b * dt;
//				setpoint += kleak * (45.0f - pid_roll.setpoint) * dt;

				if(fabsf(rpm_b) < 10) {
					if(fabsf(pid_roll.output) > 1e-6f) {
//						 pid_pitch.output += (pid_roll.output > 0 ? 1.0f : -1.0f) * 0.1f;
//						pid_roll.output *= 2.0f;
					}
				}

				temp_output += 0.8 * (pid_roll.output - temp_output);

				CAN_send_motor(0x310, 'P', temp_output);
				printf("%.3f\t%.3f\t%.3f\n", roll, pid_roll.setpoint, rpm_b);
			} else {
				PID_reset(&pid_roll);
				pid_roll.setpoint = 139.5f;
				ll = pid_roll.setpoint - 0.1f;
				ul = pid_roll.setpoint + 0.1f;
				temp_output = 0;
			}

			/*if(pitch > ll && pitch < ul) {
				ll = 20.0f;
				ul = 70.0f;

				#define ks 1.3f
				#define kv 0//  0.00002f
				#define kleak 0.8f
				#define DEADBAND 0.3f

				pid_pitch.derivative += 0.8f * (imu_data.gyro_z - pid_pitch.derivative);
				PID_compute(&pid_pitch, pitch, dt);

				if (fabsf(pid_pitch.error) > DEADBAND) {
					pid_pitch.setpoint += ks*pid_pitch.error * dt;
				}
				pid_pitch.setpoint += kv*rpm_a * dt;
//				setpoint += kleak * (45.0f - pid_pitch.setpoint) * dt;

				if(fabsf(rpm_a) < 10) {
					if(fabsf(pid_pitch.output) > 1e-6f) {
//						 pid_pitch.output += (pid_pitch.output > 0 ? 1.0f : -1.0f) * 0.1f;
						pid_pitch.output *= 2.0f;
					}
				}

				temp_output += 0.8 * (-pid_pitch.output - temp_output);

				CAN_send_motor(0x300, 'P', temp_output);
				printf("%.3f\t%.3f\n", pitch, pid_pitch.setpoint);
			} else {
				PID_reset(&pid_pitch);
				pid_pitch.setpoint = 42.0f;
				ll = pid_pitch.setpoint - 0.1f;
				ul = pid_pitch.setpoint + 0.1f;
				temp_output = 0;
			}*/
		}
	}
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 125;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
															|RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

void TIM5_init(float f) {
	RCC->APB1LENR |= 1 << 3;

	TIM5->CR1 = 0;
	TIM5->CR2 = 0;

	TIM5->PSC = 25; // 10 MHz after prescaler
	TIM5->ARR = (uint32_t)(10000000.0f/f);

	TIM5->CNT = 0;

	TIM5->EGR |= 1;

	TIM5->DIER |= 1;

	NVIC_SetPriority(TIM5_IRQn, 2);
	TIM5->SR &= ~(0x1U);
	NVIC_EnableIRQ(TIM5_IRQn);
}

void TIM6_init(float f) {
	RCC->APB1LENR |= 1 << 4;

	TIM6->CR1 = 0;
	TIM6->CR2 = 0;

	TIM6->PSC = 25; // 10 MHz after prescaler
	TIM6->ARR = (uint32_t)(10000000.0f/f);

	TIM6->CNT = 0;

	TIM6->EGR |= 1;

	TIM6->DIER |= 1;

	NVIC_SetPriority(TIM6_IRQn, 2);
	TIM6->SR &= ~(0x1U);
	NVIC_EnableIRQ(TIM6_IRQn);
}

void TIM12_init() {
	RCC->APB1LENR |= 1 << 6;

	TIM12->CR1 = 0;
	TIM12->CR2 = 0;

	TIM12->PSC = 250-1; // 1 MHz after prescaler
	TIM12->ARR = 0xFFFFFFFF;

	TIM12->CNT = 0;

	TIM12->EGR |= 1;

	TIM12->DIER |= 1;

	TIM12->SR &= ~(0x1U);
}

uint32_t get_us() {
	return TIM12->CNT;
}

void set_us(uint32_t c) {
	TIM12->CNT = c;
}

static void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_DCACHE1_Init(void) {
	hdcache1.Instance = DCACHE1;
	hdcache1.Init.ReadBurstType = DCACHE_READ_BURST_WRAP;
	if (HAL_DCACHE_Init(&hdcache1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_FDCAN1_Init(void) {
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	// 1M - 84%
	hfdcan1.Init.NominalPrescaler = 2;
	hfdcan1.Init.NominalSyncJumpWidth = 30;
	hfdcan1.Init.NominalTimeSeg1 = 94;
	hfdcan1.Init.NominalTimeSeg2 = 30;
	// 5M - 76.9%
	hfdcan1.Init.DataPrescaler = 2;
	hfdcan1.Init.DataSyncJumpWidth = 5; // Max = 16
	hfdcan1.Init.DataTimeSeg1 = 19; // Max = 32
	hfdcan1.Init.DataTimeSeg2 = 5; // Max = 16
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	// Should be less than DataPrescaler * DataTimeSeg1 = 38
	// Should be ~half of data time period = DataPrescaler * (DataTimeSeg1 + DataTimeSeg2 + 1) / 2
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 20, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x100;
	sFilterConfig.FilterID2 = 0x2FF;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2080319C;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_ICACHE_Init(void) {
	ICACHE_RegionConfigTypeDef pRegionConfig = {0};

	pRegionConfig.BaseAddress = 0x0;
	pRegionConfig.RemapAddress = 0x0;
	pRegionConfig.Size = ICACHE_REGIONSIZE_2MB;
	pRegionConfig.TrafficRoute = ICACHE_MASTER1_PORT;
	pRegionConfig.OutputBurstType = ICACHE_OUTPUT_BURST_WRAP;
	if (HAL_ICACHE_EnableRemapRegion(ICACHE_REGION_0, &pRegionConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ICACHE_Enable() != HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM13_Init(void) {
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 25000;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 10000;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = {0};
	MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

	HAL_MPU_Disable();

	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x24000000;
	MPU_InitStruct.LimitAddress = 0x24000040;
	MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
	MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
	MPU_AttributesInit.Attributes = MPU_NOT_CACHEABLE;

	HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
	HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT
/**
	* @brief  Reports the name of the source file and the source line number
	*         where the assert_param error has occurred.
	* @param  file: pointer to the source file name
	* @param  line: assert_param error line source number
	* @retval None
	*/
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#include "mpu6050.h"
#include <string.h>

/* ── Private constants ────────────────────────────────────────────────────── */

#define GRAVITY_MS2      9.80665f
#define I2C_TIMEOUT_MS   10U     /* Used only during blocking Init */

/* Accel LSB/g values for each FS setting */
static const float ACCEL_SENS[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
/* Gyro  LSB/°/s values for each FS setting */
static const float GYRO_SENS[4]  = { 131.0f,   65.5f,  32.8f,  16.4f  };

/* ── Private helpers ──────────────────────────────────────────────────────── */

static MPU6050_Status_t prv_write_reg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };
	if (HAL_I2C_Master_Transmit(hdev->hi2c, hdev->dev_addr, buf, 2, I2C_TIMEOUT_MS) != HAL_OK) {
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}

static MPU6050_Status_t prv_read_reg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t *value) {
	if (HAL_I2C_Master_Transmit(hdev->hi2c, hdev->dev_addr, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
		return MPU6050_ERR;
	}

	if (HAL_I2C_Master_Receive(hdev->hi2c, hdev->dev_addr, value, 1, I2C_TIMEOUT_MS) != HAL_OK) {
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}

static inline int16_t prv_to_int16(uint8_t high, uint8_t low) {
	return (int16_t)((uint16_t)high << 8 | (uint16_t)low);
}

static inline uint8_t prv_accel_idx(MPU6050_AccelFS_t fs) {
	return (uint8_t)((fs >> 3) & 0x03u);
}

static inline uint8_t prv_gyro_idx(MPU6050_GyroFS_t fs) {
	return (uint8_t)((fs >> 3) & 0x03u);
}

MPU6050_Config_t MPU6050_DefaultConfig(void) {
	MPU6050_Config_t cfg = {
		.accel_fs        = MPU6050_ACCEL_FS_2G,
		.gyro_fs         = MPU6050_GYRO_FS_250DPS,
		.dlpf            = MPU6050_DLPF_44HZ,
		.sample_rate_div = 9U,   /* 1 kHz / (1 + 9) = 100 Hz output rate */
	};
	return cfg;
}

/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c, uint8_t address, const MPU6050_Config_t *cfg) {
	if (!hdev || !hi2c) {
		return MPU6050_ERR;
	}

	memset(hdev, 0, sizeof(*hdev));
	hdev->hi2c     = hi2c;
	hdev->dev_addr = address;
	hdev->cfg      = cfg ? *cfg : MPU6050_DefaultConfig();
	hdev->state    = MPU6050_STATE_RESET;

	/* 1. Verify device identity */
	uint8_t who = 0;
	if (prv_read_reg(hdev, MPU6050_REG_WHO_AM_I, &who) != MPU6050_OK) {
		return MPU6050_ERR;
	}
	if (who != MPU6050_WHO_AM_I_VALUE) {
		return MPU6050_ERR;
	}

	/* 2. Wake device, select PLL with X-gyro clock source (recommended) */
	if (prv_write_reg(hdev, MPU6050_REG_PWR_MGMT_1, 0x01U) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	/* 3. Sample-rate divider */
	if (prv_write_reg(hdev, MPU6050_REG_SMPLRT_DIV, hdev->cfg.sample_rate_div) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	/* 4. DLPF */
	if (prv_write_reg(hdev, MPU6050_REG_CONFIG, (uint8_t)hdev->cfg.dlpf) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	/* 5. Gyro full-scale */
	if (prv_write_reg(hdev, MPU6050_REG_GYRO_CONFIG, (uint8_t)hdev->cfg.gyro_fs) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	/* 6. Accel full-scale */
	if (prv_write_reg(hdev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)hdev->cfg.accel_fs) != MPU6050_OK) {
		return MPU6050_ERR;
	}

	/* 7. Precompute scale factors */
	hdev->accel_scale = GRAVITY_MS2 / ACCEL_SENS[prv_accel_idx(hdev->cfg.accel_fs)];
	hdev->gyro_scale  = 1.0f        / GYRO_SENS [prv_gyro_idx (hdev->cfg.gyro_fs) ];

	hdev->state = MPU6050_STATE_IDLE;
	return MPU6050_OK;
}

/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_RequestData(MPU6050_Handle_t *hdev)
{
	if (!hdev || hdev->state == MPU6050_STATE_RESET) {
		return MPU6050_ERR;
	}
	if (hdev->state != MPU6050_STATE_IDLE && hdev->state != MPU6050_STATE_DATA_READY) {
		return MPU6050_BUSY;
	}

	hdev->tx_buf[0] = MPU6050_REG_ACCEL_XOUT_H;
	hdev->state     = MPU6050_STATE_TX_REG;

	if (HAL_I2C_Master_Transmit_IT(hdev->hi2c, hdev->dev_addr, hdev->tx_buf, 1U) != HAL_OK) {
		hdev->state = MPU6050_STATE_ERROR;
		return MPU6050_ERR;
	}

	return MPU6050_OK;
}

/* -------------------------------------------------------------------------- */

void MPU6050_IRQHandler(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c)
{
	if (!hdev || hdev->hi2c != hi2c) {
		return;   /* Not our handle */
	}

	/* Check for HAL error flag first */
	if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
		hdev->state = MPU6050_STATE_ERROR;
		return;
	}

	switch (hdev->state) {
	case MPU6050_STATE_TX_REG:
		/* Register address sent — now read 14 bytes in one burst:
		   ACCEL_X[H:L], ACCEL_Y[H:L], ACCEL_Z[H:L],
		   TEMP[H:L],
		   GYRO_X[H:L],  GYRO_Y[H:L],  GYRO_Z[H:L]  */
		hdev->state = MPU6050_STATE_RX_DATA;
		if (HAL_I2C_Master_Seq_Receive_IT(hdev->hi2c, hdev->dev_addr, hdev->rx_buf, 14U, I2C_LAST_FRAME) != HAL_OK) {
			hdev->state = MPU6050_STATE_ERROR;
		}
		break;

	case MPU6050_STATE_RX_DATA:
		/* All 14 bytes received — data is ready for the application */
		hdev->state = MPU6050_STATE_DATA_READY;
		break;

	default:
		/* Spurious callback — ignore */
		break;
	}
}

/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_GetData(MPU6050_Handle_t *hdev, MPU6050_Data_t *out)
{
	if (!hdev || !out) {
		return MPU6050_ERR;
	}
	if (hdev->state != MPU6050_STATE_DATA_READY) {
		return MPU6050_BUSY;
	}

	/* Parse raw big-endian burst (indices 0-13):
	   [0-1]  AX   [2-3]  AY   [4-5]  AZ
	   [6-7]  TEMP
	   [8-9]  GX   [10-11] GY  [12-13] GZ  */
	const uint8_t *b = hdev->rx_buf;

	int16_t raw_ax   = prv_to_int16(b[0],  b[1]);
	int16_t raw_ay   = prv_to_int16(b[2],  b[3]);
	int16_t raw_az   = prv_to_int16(b[4],  b[5]);
	int16_t raw_temp = prv_to_int16(b[6],  b[7]);
	int16_t raw_gx   = prv_to_int16(b[8],  b[9]);
	int16_t raw_gy   = prv_to_int16(b[10], b[11]);
	int16_t raw_gz   = prv_to_int16(b[12], b[13]);

	out->accel_x = (float)raw_ax * hdev->accel_scale;
	out->accel_y = (float)raw_ay * hdev->accel_scale;
	out->accel_z = (float)raw_az * hdev->accel_scale;

	/* Temperature formula from datasheet: Temp[°C] = raw/340 + 36.53 */
	out->temp_c  = (float)raw_temp / 340.0f + 36.53f;

	out->gyro_x  = (float)raw_gx * hdev->gyro_scale;
	out->gyro_y  = (float)raw_gy * hdev->gyro_scale;
	out->gyro_z  = (float)raw_gz * hdev->gyro_scale;

	/* Transition back to IDLE so the next request can be issued */
	hdev->state = MPU6050_STATE_IDLE;
	return MPU6050_OK;
}

/* -------------------------------------------------------------------------- */

void MPU6050_Reset(MPU6050_Handle_t *hdev)
{
	if (hdev) {
		hdev->state = MPU6050_STATE_IDLE;
	}
}

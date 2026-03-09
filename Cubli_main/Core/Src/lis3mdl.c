/**
 * @file    lis3mdl.c
 * @brief   Non-blocking I2C driver for LIS3MDL on STM32H5
 *
 * Transaction flow (all bus activity after Init is interrupt-driven):
 *
 *   LIS3MDL_RequestData()
 *       │
 *       ▼  HAL_I2C_Master_Transmit_IT (1 byte: register addr 0xA8)
 *   STATE_TX_REG          (0x28 | 0x80 auto-increment flag)
 *       │  TxCplt IRQ fires → LIS3MDL_IRQHandler()
 *       ▼  HAL_I2C_Master_Receive_IT  (6 bytes: X_L X_H Y_L Y_H Z_L Z_H)
 *   STATE_RX_DATA
 *       │  RxCplt IRQ fires → LIS3MDL_IRQHandler()
 *       ▼
 *   STATE_DATA_READY  ← caller may now call LIS3MDL_GetData()
 */

#include "lis3mdl.h"
#include <string.h>

/* ── Private constants ────────────────────────────────────────────────────── */

#define I2C_TIMEOUT_MS  10U     /* Used only during blocking Init */

/*
 * Sensitivity values in LSB/gauss from the datasheet (Table 3).
 * Index matches LIS3MDL_FS_t >> 5: 0=4G, 1=8G, 2=12G, 3=16G
 */
static const float MAG_SENS[4] = { 6842.0f, 3421.0f, 2281.0f, 1711.0f };

/* ── Private helpers ──────────────────────────────────────────────────────── */

static LIS3MDL_Status_t prv_write_reg(LIS3MDL_Handle_t *hdev, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };
	if (HAL_I2C_Master_Transmit(hdev->hi2c, hdev->dev_addr, buf, 2, I2C_TIMEOUT_MS) != HAL_OK) {
		return LIS3MDL_ERR;
	}
	return LIS3MDL_OK;
}

static LIS3MDL_Status_t prv_read_reg(LIS3MDL_Handle_t *hdev, uint8_t reg, uint8_t *value) {
	if (HAL_I2C_Master_Transmit(hdev->hi2c, hdev->dev_addr, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
		return LIS3MDL_ERR;
	}
	if (HAL_I2C_Master_Receive(hdev->hi2c, hdev->dev_addr, value, 1, I2C_TIMEOUT_MS) != HAL_OK) {
		return LIS3MDL_ERR;
	}
	return LIS3MDL_OK;
}

/** LIS3MDL is little-endian: low byte first */
static inline int16_t prv_to_int16(uint8_t low, uint8_t high) {
	return (int16_t)((uint16_t)high << 8 | (uint16_t)low);
}

/** Map FS enum to sensitivity table index */
static inline uint8_t prv_fs_idx(LIS3MDL_FS_t fs) {
	return (uint8_t)((fs >> 5) & 0x03u);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

LIS3MDL_Config_t LIS3MDL_DefaultConfig(void) {
	LIS3MDL_Config_t cfg = {
		.odr  = LIS3MDL_ODR_10HZ,
		.fs   = LIS3MDL_FS_4GAUSS,
		.om   = LIS3MDL_OM_HIGH,
		.mode = LIS3MDL_MODE_CONTINUOUS,
	};
	return cfg;
}

/* -------------------------------------------------------------------------- */

LIS3MDL_Status_t LIS3MDL_Init(LIS3MDL_Handle_t 			*hdev,
								I2C_HandleTypeDef 		*hi2c,
								uint8_t					address,
								const LIS3MDL_Config_t	*cfg) {
	if (!hdev || !hi2c) {
		return LIS3MDL_ERR;
	}

	memset(hdev, 0, sizeof(*hdev));
	hdev->hi2c     = hi2c;
	hdev->dev_addr = address;
	hdev->cfg      = cfg ? *cfg : LIS3MDL_DefaultConfig();
	hdev->state    = LIS3MDL_STATE_RESET;

	/* 1. Verify device identity */
	uint8_t who = 0;
	if (prv_read_reg(hdev, LIS3MDL_REG_WHO_AM_I, &who) != LIS3MDL_OK) {
		return LIS3MDL_ERR;
	}
	if (who != LIS3MDL_WHO_AM_I_VALUE) {
		return LIS3MDL_ERR;
	}

	/*
	 * 2. CTRL_REG1: TEMP_EN=0, XY operative mode, ODR, FAST_ODR=0, ST=0
	 *
	 *    [7]   TEMP_EN  = 0 (temperature sensor off; not needed for AHRS)
	 *    [6:5] OM[1:0]  = operative mode for X and Y axes
	 *    [4:2] DO[2:0]  = output data rate
	 *    [1]   FAST_ODR = 0
	 *    [0]   ST       = 0 (self-test off)
	 */
	uint8_t ctrl1 = (uint8_t)(hdev->cfg.om | hdev->cfg.odr);
	if (prv_write_reg(hdev, LIS3MDL_REG_CTRL_REG1, ctrl1) != LIS3MDL_OK) {
		return LIS3MDL_ERR;
	}

	/*
	 * 3. CTRL_REG2: full-scale range, REBOOT=0, SOFT_RST=0
	 *
	 *    [6:5] FS[1:0]  = full-scale range
	 *    others = 0
	 */
	uint8_t ctrl2 = (uint8_t)hdev->cfg.fs;
	if (prv_write_reg(hdev, LIS3MDL_REG_CTRL_REG2, ctrl2) != LIS3MDL_OK) {
		return LIS3MDL_ERR;
	}

	/*
	 * 4. CTRL_REG3: operating mode
	 *
	 *    [1:0] MD[1:0] = system mode (continuous / single / power-down)
	 */
	uint8_t ctrl3 = (uint8_t)hdev->cfg.mode;
	if (prv_write_reg(hdev, LIS3MDL_REG_CTRL_REG3, ctrl3) != LIS3MDL_OK) {
		return LIS3MDL_ERR;
	}

	/*
	 * 5. CTRL_REG4: Z-axis operative mode, big/little endian select
	 *
	 *    [3:2] OMZ[1:0] = operative mode for Z axis (match XY mode)
	 *    [1]   BLE      = 0 (little-endian, low byte at lower address)
	 */
	uint8_t ctrl4 = (uint8_t)((hdev->cfg.om >> 3) & 0x0Cu);
	if (prv_write_reg(hdev, LIS3MDL_REG_CTRL_REG4, ctrl4) != LIS3MDL_OK) {
		return LIS3MDL_ERR;
	}

	/* 6. Precompute scale factor */
	hdev->mag_scale = 1.0f / MAG_SENS[prv_fs_idx(hdev->cfg.fs)];

	hdev->state = LIS3MDL_STATE_IDLE;
	return LIS3MDL_OK;
}

/* -------------------------------------------------------------------------- */

LIS3MDL_Status_t LIS3MDL_RequestData(LIS3MDL_Handle_t *hdev) {
	if (!hdev || hdev->state == LIS3MDL_STATE_RESET) {
		return LIS3MDL_ERR;
	}
	if (hdev->state != LIS3MDL_STATE_IDLE && hdev->state != LIS3MDL_STATE_DATA_READY) {
		return LIS3MDL_BUSY;
	}

	/*
	 * Send OUT_X_L register address with the auto-increment flag set (bit 7).
	 * This allows the subsequent 6-byte read to walk through
	 * OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H in one burst.
	 */
	hdev->tx_buf[0] = LIS3MDL_REG_OUT_X_L | LIS3MDL_ADDR_AUTO_INC;
	hdev->state     = LIS3MDL_STATE_TX_REG;

	if (HAL_I2C_Master_Transmit_IT(hdev->hi2c, hdev->dev_addr, hdev->tx_buf, 1U) != HAL_OK) {
		hdev->state = LIS3MDL_STATE_ERROR;
		return LIS3MDL_ERR;
	}

	return LIS3MDL_OK;
}

/* -------------------------------------------------------------------------- */

void LIS3MDL_IRQHandler(LIS3MDL_Handle_t *hdev, I2C_HandleTypeDef *hi2c) {
	if (!hdev || hdev->hi2c != hi2c) {
		return;   /* Not our handle */
	}

	if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
		hdev->state = LIS3MDL_STATE_ERROR;
		return;
	}

	switch (hdev->state) {

	case LIS3MDL_STATE_TX_REG:
		/* Register pointer sent — read 6 bytes: X_L X_H Y_L Y_H Z_L Z_H */
		hdev->state = LIS3MDL_STATE_RX_DATA;
		if (HAL_I2C_Master_Receive_IT(hdev->hi2c, hdev->dev_addr, hdev->rx_buf, 6U) != HAL_OK) {
			hdev->state = LIS3MDL_STATE_ERROR;
		}
		break;

	case LIS3MDL_STATE_RX_DATA:
		hdev->state = LIS3MDL_STATE_DATA_READY;
		break;

	default:
		/* Spurious callback — ignore */
		break;
	}
}

/* -------------------------------------------------------------------------- */

LIS3MDL_Status_t LIS3MDL_GetData(LIS3MDL_Handle_t *hdev, LIS3MDL_Data_t *out) {
	if (!hdev || !out) {
		return LIS3MDL_ERR;
	}
	if (hdev->state != LIS3MDL_STATE_DATA_READY) {
		return LIS3MDL_BUSY;
	}

	/*
	 * Parse raw little-endian burst (indices 0-5):
	 *   [0-1] X   [2-3] Y   [4-5] Z
	 */
	const uint8_t *b = hdev->rx_buf;

	int16_t raw_x = prv_to_int16(b[0], b[1]);
	int16_t raw_y = prv_to_int16(b[2], b[3]);
	int16_t raw_z = prv_to_int16(b[4], b[5]);

	out->mag_x =  (float)raw_x * hdev->mag_scale;
	out->mag_y = -(float)raw_y * hdev->mag_scale;
	out->mag_z = -(float)raw_z * hdev->mag_scale;

	hdev->state = LIS3MDL_STATE_IDLE;
	return LIS3MDL_OK;
}

/* -------------------------------------------------------------------------- */

void LIS3MDL_Reset(LIS3MDL_Handle_t *hdev) {
	if (hdev) {
		hdev->state = LIS3MDL_STATE_IDLE;
	}
}

/**
 * @file    lis3mdl.h
 * @brief   Non-blocking I2C driver for LIS3MDL magnetometer on STM32H5
 *
 * Uses HAL_I2C_Master_Transmit_IT / Receive_IT (interrupt-driven) so the
 * CPU is never stalled waiting for the bus.  A lightweight state machine
 * manages the two-step register-pointer-then-read transaction.
 *
 * Typical usage
 * -------------
 *   LIS3MDL_Handle_t mag;
 *   LIS3MDL_Init(&mag, &hi2c1, LIS3MDL_ADDR_LOW, NULL);
 *
 *   // In main loop — only call when the I2C bus is free:
 *   LIS3MDL_RequestData(&mag);
 *
 *   // Called from HAL_I2C_MasterTxCpltCallback,
 *   //             HAL_I2C_MasterRxCpltCallback,
 *   //             HAL_I2C_ErrorCallback:
 *   LIS3MDL_IRQHandler(&mag, hi2c);
 *
 *   // When mag.state == LIS3MDL_STATE_DATA_READY:
 *   LIS3MDL_Data_t data;
 *   LIS3MDL_GetData(&mag, &data);
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── I2C addresses ────────────────────────────────────────────────────────── */
#define LIS3MDL_ADDR_LOW   (0x1CU << 1)   /**< SA1 pin = GND */
#define LIS3MDL_ADDR_HIGH  (0x1EU << 1)   /**< SA1 pin = VCC */

/* ── Register map ─────────────────────────────────────────────────────────── */
#define LIS3MDL_REG_WHO_AM_I    0x0FU
#define LIS3MDL_REG_CTRL_REG1   0x20U
#define LIS3MDL_REG_CTRL_REG2   0x21U
#define LIS3MDL_REG_CTRL_REG3   0x22U
#define LIS3MDL_REG_CTRL_REG4   0x23U
#define LIS3MDL_REG_CTRL_REG5   0x24U
#define LIS3MDL_REG_OUT_X_L     0x28U   /**< First of 6 consecutive output bytes */
#define LIS3MDL_WHO_AM_I_VALUE  0x3DU

/* Auto-increment bit: set bit 7 of the register address for multi-byte reads */
#define LIS3MDL_ADDR_AUTO_INC   0x80U

/* ── Configuration enumerations ───────────────────────────────────────────── */

/**
 * @brief Output data rate.
 *        Applied to XY axes via CTRL_REG1[OM] and to Z via CTRL_REG4[OMZ].
 *        FAST_ODR must be enabled separately for rates above 80 Hz.
 */
typedef enum {
    LIS3MDL_ODR_0_625HZ = 0x00,
    LIS3MDL_ODR_1_25HZ  = 0x04,
    LIS3MDL_ODR_2_5HZ   = 0x08,
    LIS3MDL_ODR_5HZ     = 0x0C,
    LIS3MDL_ODR_10HZ    = 0x10,
    LIS3MDL_ODR_20HZ    = 0x14,
    LIS3MDL_ODR_40HZ    = 0x18,
    LIS3MDL_ODR_80HZ    = 0x1C,
} LIS3MDL_ODR_t;

/** Full-scale range */
typedef enum {
    LIS3MDL_FS_4GAUSS  = 0x00,   /**< ±4  gauss, 6842 LSB/gauss */
    LIS3MDL_FS_8GAUSS  = 0x20,   /**< ±8  gauss, 3421 LSB/gauss */
    LIS3MDL_FS_12GAUSS = 0x40,   /**< ±12 gauss, 2281 LSB/gauss */
    LIS3MDL_FS_16GAUSS = 0x60,   /**< ±16 gauss, 1711 LSB/gauss */
} LIS3MDL_FS_t;

/** XY-axis operative mode (performance vs. power) */
typedef enum {
    LIS3MDL_OM_LOW_POWER    = 0x00,
    LIS3MDL_OM_MEDIUM       = 0x20,
    LIS3MDL_OM_HIGH         = 0x40,
    LIS3MDL_OM_ULTRA_HIGH   = 0x60,
} LIS3MDL_OperativeMode_t;

/** System operating mode (CTRL_REG3) */
typedef enum {
    LIS3MDL_MODE_CONTINUOUS = 0x00,
    LIS3MDL_MODE_SINGLE     = 0x01,
    LIS3MDL_MODE_POWER_DOWN = 0x03,
} LIS3MDL_Mode_t;

/* ── Driver state machine ─────────────────────────────────────────────────── */
typedef enum {
    LIS3MDL_STATE_RESET = 0,    /**< Not initialised               */
    LIS3MDL_STATE_IDLE,         /**< Ready to start a transaction  */
    LIS3MDL_STATE_TX_REG,       /**< Sending register address byte */
    LIS3MDL_STATE_RX_DATA,      /**< Receiving 6-byte XYZ payload  */
    LIS3MDL_STATE_DATA_READY,   /**< Fresh data available          */
    LIS3MDL_STATE_ERROR,        /**< Bus or device error           */
} LIS3MDL_State_t;

/* ── Scaled sensor data ───────────────────────────────────────────────────── */
typedef struct {
    float mag_x;   /**< Magnetic field X [gauss] */
    float mag_y;   /**< Magnetic field Y [gauss] */
    float mag_z;   /**< Magnetic field Z [gauss] */
} LIS3MDL_Data_t;

/* ── Driver configuration ─────────────────────────────────────────────────── */
typedef struct {
    LIS3MDL_ODR_t           odr;    /**< Output data rate   (default 10 Hz)        */
    LIS3MDL_FS_t            fs;     /**< Full-scale range   (default ±4 gauss)     */
    LIS3MDL_OperativeMode_t om;     /**< XY operative mode  (default High perf.)   */
    LIS3MDL_Mode_t          mode;   /**< System mode        (default Continuous)   */
} LIS3MDL_Config_t;

/* ── Main handle ──────────────────────────────────────────────────────────── */
typedef struct {
    /* --- user-supplied --- */
    I2C_HandleTypeDef *hi2c;
    uint8_t            dev_addr;     /**< 7-bit address shifted left by 1 */
    LIS3MDL_Config_t   cfg;

    /* --- internal --- */
    volatile LIS3MDL_State_t state;
    uint8_t  tx_buf[1];              /**< Register address to send        */
    uint8_t  rx_buf[6];              /**< Raw 6-byte XYZ burst            */

    /* --- scale factor (precomputed at init) --- */
    float mag_scale;                 /**< LSB → gauss */
} LIS3MDL_Handle_t;

/* ── Return codes ─────────────────────────────────────────────────────────── */
typedef enum {
    LIS3MDL_OK   =  0,
    LIS3MDL_ERR  = -1,
    LIS3MDL_BUSY = -2,
} LIS3MDL_Status_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise driver, verify WHO_AM_I, configure device registers.
 *         BLOCKING — uses HAL polling.  Call once at startup before enabling
 *         I2C interrupts.
 *
 * @param  hdev     Pointer to the driver handle (caller-allocated).
 * @param  hi2c     Pointer to the HAL I2C handle (already initialised).
 * @param  address  LIS3MDL_ADDR_LOW or LIS3MDL_ADDR_HIGH.
 * @param  cfg      Desired configuration, or NULL for defaults.
 * @return LIS3MDL_OK on success, LIS3MDL_ERR on failure.
 */
LIS3MDL_Status_t LIS3MDL_Init(LIS3MDL_Handle_t       *hdev,
                                I2C_HandleTypeDef      *hi2c,
                                uint8_t                 address,
                                const LIS3MDL_Config_t *cfg);

/**
 * @brief  Kick off a non-blocking 6-byte XYZ read.
 *         Returns LIS3MDL_BUSY if a transaction is already in progress.
 *         Only call this when the I2C bus is free (i.e. no other sensor
 *         is currently communicating).
 */
LIS3MDL_Status_t LIS3MDL_RequestData(LIS3MDL_Handle_t *hdev);

/**
 * @brief  Single callback to be called from all three HAL I2C callbacks:
 *           HAL_I2C_MasterTxCpltCallback()
 *           HAL_I2C_MasterRxCpltCallback()
 *           HAL_I2C_ErrorCallback()
 *         Checks handle ownership internally; safe to call with any hi2c.
 */
void LIS3MDL_IRQHandler(LIS3MDL_Handle_t *hdev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Copy the latest scaled data out of the handle.
 *         Returns LIS3MDL_BUSY if data is not yet ready.
 *         On success, state returns to IDLE so LIS3MDL_RequestData() can
 *         be called again.
 */
LIS3MDL_Status_t LIS3MDL_GetData(LIS3MDL_Handle_t *hdev, LIS3MDL_Data_t *out);

/**
 * @brief  Reset the driver state machine to IDLE (e.g. after a bus error).
 */
void LIS3MDL_Reset(LIS3MDL_Handle_t *hdev);

/**
 * @brief  Return the default configuration.
 */
LIS3MDL_Config_t LIS3MDL_DefaultConfig(void);

#ifdef __cplusplus
}
#endif
#endif /* LIS3MDL_H */

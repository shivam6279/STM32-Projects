#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR_LOW   (0x68U << 1)
#define MPU6050_ADDR_HIGH  (0x69U << 1)

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_WHO_AM_I_VALUE    0x68U

/** Accelerometer full-scale range */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,
    MPU6050_ACCEL_FS_4G  = 0x08,
    MPU6050_ACCEL_FS_8G  = 0x10,
    MPU6050_ACCEL_FS_16G = 0x18,
} MPU6050_AccelFS_t;

/** Gyroscope full-scale range */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0x00,
    MPU6050_GYRO_FS_500DPS  = 0x08,
    MPU6050_GYRO_FS_1000DPS = 0x10,
    MPU6050_GYRO_FS_2000DPS = 0x18,
} MPU6050_GyroFS_t;

/** Digital Low-Pass Filter bandwidth */
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} MPU6050_DLPF_t;

/* ── Driver state machine ─────────────────────────────────────────────────── */
typedef enum {
    MPU6050_STATE_RESET = 0,    /**< Not initialised               */
    MPU6050_STATE_IDLE,         /**< Ready to start a transaction  */
    MPU6050_STATE_TX_REG,       /**< Sending register address byte */
    MPU6050_STATE_RX_DATA,      /**< Receiving sensor payload      */
    MPU6050_STATE_DATA_READY,   /**< Fresh data available          */
    MPU6050_STATE_ERROR,        /**< Bus or device error           */
} MPU6050_State_t;

/* ── Scaled sensor data ───────────────────────────────────────────────────── */
typedef struct {
    float accel_x;   /**< Acceleration X  [m/s²] */
    float accel_y;   /**< Acceleration Y  [m/s²] */
    float accel_z;   /**< Acceleration Z  [m/s²] */
    float gyro_x;    /**< Angular rate X  [°/s]  */
    float gyro_y;    /**< Angular rate Y  [°/s]  */
    float gyro_z;    /**< Angular rate Z  [°/s]  */
    float temp_c;    /**< Die temperature [°C]   */
} MPU6050_Data_t;

/* ── Driver configuration ─────────────────────────────────────────────────── */
typedef struct {
    MPU6050_AccelFS_t accel_fs;    /**< Accel full-scale (default ±2 g)       */
    MPU6050_GyroFS_t  gyro_fs;     /**< Gyro  full-scale (default ±250 °/s)   */
    MPU6050_DLPF_t    dlpf;        /**< DLPF bandwidth   (default 44 Hz)      */
    uint8_t           sample_rate_div; /**< Output rate = 1 kHz / (1 + div)   */
} MPU6050_Config_t;

/* ── Main handle ──────────────────────────────────────────────────────────── */
typedef struct {
    /* --- user-supplied ---*/
    I2C_HandleTypeDef *hi2c;
    uint8_t            dev_addr;     /**< 7-bit address shifted left by 1     */
    MPU6050_Config_t   cfg;

    /* --- internal ---*/
    volatile MPU6050_State_t state;
    uint8_t  tx_buf[1];              /**< Register address to send            */
    uint8_t  rx_buf[14];             /**< Raw 14-byte sensor burst            */

    /* --- scale factors (precomputed at init) ---*/
    float accel_scale;               /**< LSB → m/s²  */
    float gyro_scale;                /**< LSB → °/s   */
} MPU6050_Handle_t;

/* ── Return codes ─────────────────────────────────────────────────────────── */
typedef enum {
    MPU6050_OK    =  0,
    MPU6050_ERR   = -1,
    MPU6050_BUSY  = -2,
} MPU6050_Status_t;

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t   *hdev,
                               I2C_HandleTypeDef  *hi2c,
                               uint8_t             address,
                               const MPU6050_Config_t *cfg);

/**
 * @brief  Kick off a non-blocking sensor data read.
 *         Returns MPU6050_BUSY if a transaction is already in progress.
 *         The caller should poll hdev->state or wait for the data-ready
 *         callback to fire.
 */
MPU6050_Status_t MPU6050_RequestData(MPU6050_Handle_t *hdev);

/**
 * @brief  Must be called from both:
 *           HAL_I2C_MasterTxCpltCallback()
 *           HAL_I2C_MasterRxCpltCallback()
 *           HAL_I2C_ErrorCallback()
 *         Pass all handles; the function checks ownership internally.
 */
void MPU6050_IRQHandler(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Copy the latest scaled sensor data out of the handle.
 *         Returns MPU6050_BUSY if data is not yet ready.
 *         After a successful call, state transitions back to IDLE so a new
 *         MPU6050_RequestData() can be issued.
 */
MPU6050_Status_t MPU6050_GetData(MPU6050_Handle_t *hdev, MPU6050_Data_t *out);

/**
 * @brief  Reset the driver state machine (e.g. after a bus error).
 */
void MPU6050_Reset(MPU6050_Handle_t *hdev);

MPU6050_Config_t MPU6050_DefaultConfig(void);

#ifdef __cplusplus
}
#endif
#endif

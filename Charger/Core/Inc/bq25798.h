// TI BQ25798 charger driver. I2C1 on PB6/PB7, addr 0x6B (needs external pull-ups).
// Register map per datasheet SLUSDV2B.
#ifndef BQ25798_H
#define BQ25798_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ---- Device ---------------------------------------------------------------
#define BQ25798_I2C_ADDR        0x6Bu   // 7-bit address
#define BQ25798_PART_NUMBER     0x03u   // REG48 PN[5:3] = 011b for BQ25798

// ---- Register addresses (datasheet section 9.5) ---------------------------
#define BQ_REG_MIN_SYS_V        0x00u   // Minimal System Voltage
#define BQ_REG_CHG_V_LIM        0x01u   // Charge Voltage Limit   (16-bit)
#define BQ_REG_CHG_I_LIM        0x03u   // Charge Current Limit   (16-bit)
#define BQ_REG_IN_V_LIM         0x05u   // Input Voltage Limit (VINDPM, 8-bit)
#define BQ_REG_IN_I_LIM         0x06u   // Input Current Limit (IINDPM,16-bit)
#define BQ_REG_PRECHG_CTRL      0x08u   // Precharge Control
#define BQ_REG_TERM_CTRL        0x09u   // Termination Control (REG_RST=bit6)
#define BQ_REG_RECHG_CTRL       0x0Au   // Re-charge Control
#define BQ_REG_VOTG             0x0Bu   // VOTG regulation        (16-bit)
#define BQ_REG_IOTG             0x0Du   // IOTG regulation
#define BQ_REG_TIMER_CTRL       0x0Eu   // Timer Control
#define BQ_REG_CHG_CTRL0        0x0Fu   // Charger Control 0 (EN_CHG=bit5)
#define BQ_REG_CHG_CTRL1        0x10u   // Charger Control 1 (WD/WD_RST)
#define BQ_REG_CHG_CTRL2        0x11u   // Charger Control 2
#define BQ_REG_CHG_CTRL3        0x12u   // Charger Control 3 (EN_OTG=bit6)
#define BQ_REG_CHG_CTRL4        0x13u   // Charger Control 4
#define BQ_REG_CHG_CTRL5        0x14u   // Charger Control 5
#define BQ_REG_MPPT_CTRL        0x15u   // MPPT Control (EN_MPPT=bit0)
#define BQ_REG_TEMP_CTRL        0x16u   // Temperature Control
#define BQ_REG_CHG_STAT0        0x1Bu   // Charger Status 0
#define BQ_REG_CHG_STAT1        0x1Cu   // Charger Status 1 (CHG_STAT[7:5])
#define BQ_REG_CHG_STAT2        0x1Du   // Charger Status 2
#define BQ_REG_CHG_STAT3        0x1Eu   // Charger Status 3 (ADC_DONE=bit5)
#define BQ_REG_CHG_STAT4        0x1Fu   // Charger Status 4
#define BQ_REG_FAULT_STAT0      0x20u   // FAULT Status 0
#define BQ_REG_FAULT_STAT1      0x21u   // FAULT Status 1
#define BQ_REG_ADC_CTRL         0x2Eu   // ADC Control (ADC_EN=bit7)
#define BQ_REG_ADC_FN_DIS0      0x2Fu   // ADC Function Disable 0
#define BQ_REG_ADC_FN_DIS1      0x30u   // ADC Function Disable 1
#define BQ_REG_IBUS_ADC         0x31u   // IBUS ADC (16-bit, signed, 1 mA)
#define BQ_REG_IBAT_ADC         0x33u   // IBAT ADC (16-bit, signed, 1 mA)
#define BQ_REG_VBUS_ADC         0x35u   // VBUS ADC (16-bit, 1 mV)
#define BQ_REG_VBAT_ADC         0x3Bu   // VBAT ADC (16-bit, 1 mV)
#define BQ_REG_VSYS_ADC         0x3Du   // VSYS ADC (16-bit, 1 mV)
#define BQ_REG_TS_ADC           0x3Fu   // TS ADC (16-bit, 0.0976563 %)
#define BQ_REG_TDIE_ADC         0x41u   // TDIE ADC (16-bit, signed, 0.5 C)
#define BQ_REG_PART_INFO        0x48u   // Part Information (PN[5:3])

// ---- Key bit masks --------------------------------------------------------
#define BQ_TERM_CTRL_REG_RST    (1u << 6)   // REG09: software reset
#define BQ_CTRL0_EN_CHG         (1u << 5)   // REG0F: charge enable
#define BQ_CTRL0_EN_HIZ         (1u << 2)   // REG0F: high-impedance input
#define BQ_CTRL0_EN_TERM        (1u << 1)   // REG0F: charge termination
#define BQ_CTRL1_WD_RST         (1u << 3)   // REG10: kick watchdog
#define BQ_CTRL1_WATCHDOG_Msk   (0x7u << 0) // REG10: watchdog timer select
#define BQ_CTRL3_EN_OTG         (1u << 6)   // REG12: OTG (boost) enable
#define BQ_ADC_CTRL_EN          (1u << 7)   // REG2E: ADC enable
#define BQ_ADC_CTRL_ONESHOT     (1u << 6)   // REG2E: 1 = one-shot, 0 = cont.
#define BQ_ADC_FN_DIS0_TS       (1u << 2)   // REG2F: disable TS ADC channel

// Watchdog timer settings (REG10 WATCHDOG[2:0]).
typedef enum {
	BQ_WD_DISABLE = 0x0u,
	BQ_WD_0_5S    = 0x1u,
	BQ_WD_1S      = 0x2u,
	BQ_WD_2S      = 0x3u,
	BQ_WD_20S     = 0x4u,
	BQ_WD_40S     = 0x5u,   // POR default
	BQ_WD_80S     = 0x6u,
	BQ_WD_160S    = 0x7u,
} bq_watchdog_t;

// CHG_STAT field (REG1C[7:5]).
typedef enum {
	BQ_CHG_NOT_CHARGING = 0x0u,
	BQ_CHG_TRICKLE      = 0x1u,
	BQ_CHG_PRECHARGE    = 0x2u,
	BQ_CHG_FAST_CC      = 0x3u,
	BQ_CHG_TAPER_CV     = 0x4u,
	BQ_CHG_TOPOFF       = 0x6u,
	BQ_CHG_DONE         = 0x7u,
} bq_chg_state_t;

// Driver return codes.
typedef enum {
	BQ_OK = 0,
	BQ_ERR,        // generic / parameter error
	BQ_TIMEOUT,    // I2C transfer timed out
	BQ_NACK,       // device did not acknowledge
	BQ_BUSY,       // bus stuck busy
	BQ_BADID,      // part number mismatch on Init
} bq_status_t;

// Decoded ADC telemetry (see BQ25798_ReadADC).
typedef struct {
	uint16_t vbus_mV;     // bus voltage
	uint16_t vbat_mV;     // battery voltage
	uint16_t vsys_mV;     // system voltage
	int16_t  ibus_mA;     // + = into VBUS->PMID, - = reverse
	int16_t  ibat_mA;     // + = charging, - = discharging
	int16_t  tdie_C_x10;  // die temperature in 0.1 C steps
} bq_adc_t;

// ---- Low-level register access --------------------------------------------
bq_status_t BQ25798_Init(void);   // I2C1 + PB6/PB7 + verify part number
bq_status_t BQ_BusRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t len);  // any device on I2C1
bq_status_t BQ_BusWrite(uint8_t addr7, uint8_t reg, const uint8_t *data, uint8_t len);
bq_status_t BQ25798_ReadReg8(uint8_t reg, uint8_t *val);
bq_status_t BQ25798_WriteReg8(uint8_t reg, uint8_t val);
bq_status_t BQ25798_ReadReg16(uint8_t reg, uint16_t *val);   // MSB at lower addr
bq_status_t BQ25798_WriteReg16(uint8_t reg, uint16_t val);
bq_status_t BQ25798_UpdateBits(uint8_t reg, uint8_t mask, uint8_t val);

// Aggregate charger configuration applied by BQ25798_Configure().
typedef struct {
	uint8_t  cell_count;      // 1..4 cells in series
	uint16_t charge_mV;       // pack charge voltage / VREG (e.g. 12600=3S)
	uint16_t charge_mA;       // fast-charge current (ICHG)
	uint16_t iprechg_mA;      // pre-charge current
	uint16_t iterm_mA;        // termination current
	uint16_t input_ilim_mA;   // input current limit (IINDPM)
	uint16_t input_vlim_mV;   // input voltage limit (VINDPM)
	bq_watchdog_t watchdog;   // BQ_WD_DISABLE keeps settings persistent
} bq_config_t;

// ---- Configuration helpers ------------------------------------------------
bq_status_t BQ25798_Configure(const bq_config_t *cfg);      // apply a full setup
bq_status_t BQ25798_Reset(void);                            // REG_RST
bq_status_t BQ25798_SetChargeVoltage_mV(uint16_t mv);       // 3000..18800, 10 mV
bq_status_t BQ25798_SetChargeCurrent_mA(uint16_t ma);       // 50..5000,   10 mA
bq_status_t BQ25798_SetInputCurrentLimit_mA(uint16_t ma);   // 100..3300,  10 mA
bq_status_t BQ25798_SetInputVoltageLimit_mV(uint16_t mv);   // 3600..22000,100 mV
bq_status_t BQ25798_EnableCharging(uint8_t enable);
bq_status_t BQ25798_EnableHIZ(uint8_t enable);
bq_status_t BQ25798_SetWatchdog(bq_watchdog_t sel);
bq_status_t BQ25798_KickWatchdog(void);                     // WD_RST = 1
bq_status_t BQ25798_EnableADC(uint8_t enable, uint8_t one_shot);

// ---- Telemetry / status ---------------------------------------------------
bq_status_t BQ25798_ReadADC(bq_adc_t *out);
bq_status_t BQ25798_GetChargeState(bq_chg_state_t *state); // from REG1C
bq_status_t BQ25798_ReadStatus(uint8_t status[5]);         // REG1B..REG1F
bq_status_t BQ25798_ReadFaults(uint8_t faults[2]);         // REG20..REG21

#ifdef __cplusplus
}
#endif
#endif // BQ25798_H

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include "MPU6050.h"
#include "LIS3MDL.h"

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct CanMessage_t {
	uint32_t Identifier;
	uint32_t DataLength;
	uint8_t Data[64];
} CanMessage_t;

#define CAN_BUFFER_SIZE 16U

uint8_t can_rxbuffer_available();
void pop_can_rxbuffer(CanMessage_t*);

extern MPU6050_Handle_t g_imu;
extern LIS3MDL_Handle_t g_mag;
extern float g_q[4];

extern volatile float rpm_a, rpm_b, rpm_c;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif

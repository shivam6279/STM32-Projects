#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct CanMessage_t {
	uint32_t Identifier;
	uint32_t DataLength;
	uint8_t Data[64];
} CanMessage_t;

#define CAN_BUFFER_SIZE 16

extern uint8_t can_rxbuffer_available();
extern CanMessage_t* pop_can_rxbuffer();

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif

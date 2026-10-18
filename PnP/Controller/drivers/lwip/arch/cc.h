#ifndef _LWIP_ARCH_CC_H_
#define _LWIP_ARCH_CC_H_

#include <stdio.h>

// lwIP port: arm-none-eabi-gcc, little endian, no RTOS

void NetAssert(const char *msg, const char *file, int line);

#define LWIP_PLATFORM_DIAG(x)			do { printf x; } while (0)
#define LWIP_PLATFORM_ASSERT(x)			NetAssert(x, __FILE__, __LINE__)
#define LWIP_RAND()						((u32_t)NetRand())

unsigned long NetRand();

#endif

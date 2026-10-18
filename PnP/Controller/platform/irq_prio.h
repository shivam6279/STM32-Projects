#ifndef _IRQ_PRIO_H_
#define _IRQ_PRIO_H_

// All NVIC priorities live here. 0 = highest, 15 = lowest. Motion > comms.

#define IRQ_PRIO_ESTOP			0	// not wired yet
#define IRQ_PRIO_STEPGEN_DMA	1
#define IRQ_PRIO_DIAG			2	// TMC2209 DIAG pins: stop stepping on a driver fault
// 2-7 motion / timebase
// 8-15 comms
#define IRQ_PRIO_BUTTONS		8	// endstops may move up to 2-7 once they're defined
#define IRQ_PRIO_CAN			9
#define IRQ_PRIO_USB			10
#define IRQ_PRIO_ETH			10	// only wakes the main loop
#define IRQ_PRIO_UART			11
#define IRQ_PRIO_SYSTICK		12

#endif

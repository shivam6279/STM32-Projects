#include "stepgen.h"
#include "board.h"
#include "clock.h"
#include "irq_prio.h"
#include "mpu.h"
#include "resources.h"
#include "stm32h7xx.h"

#include <string.h>

// TIM6 -> DMAMUX1 ch0 (req 69, TIM6_UP) -> DMA1 stream 0 -> GPIOE->BSRR
// TODO: RM0468 bus matrix ref for DMA1 -> AHB4 GPIO, ES0491 errata check for TIM6/DMA

#define STEP_MUX_REQ	69	// LL_DMAMUX1_REQ_TIM6_UP
#define STEP_ARR		(CLOCK_TIM_APB1_HZ / STEP_TICK_HZ - 1)

_Static_assert(CLOCK_TIM_APB1_HZ % STEP_TICK_HZ == 0, "tick must divide timer clock");
_Static_assert(STEP_ARR <= 0xFFFF, "TIM6 is 16 bit");

#define ON_PORT_E(id) (PIN_##id##_PORT_IDX == 4)
_Static_assert(ON_PORT_E(STEP1) && ON_PORT_E(STEP2) && ON_PORT_E(STEP3) && ON_PORT_E(STEP4)
	&& ON_PORT_E(STEP5) && ON_PORT_E(STEP6) && ON_PORT_E(STEP7) && ON_PORT_E(STEP8)
	&& ON_PORT_E(DIR1) && ON_PORT_E(DIR2) && ON_PORT_E(DIR3) && ON_PORT_E(DIR4)
	&& ON_PORT_E(DIR5) && ON_PORT_E(DIR6) && ON_PORT_E(DIR7) && ON_PORT_E(DIR8),
	"all STEP/DIR pins need to be on GPIOE");

#define S0_FLAGS	(DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0)
#define STEP_ALL	(PIN_STEP1_MASK | PIN_STEP2_MASK | PIN_STEP3_MASK | PIN_STEP4_MASK \
	| PIN_STEP5_MASK | PIN_STEP6_MASK | PIN_STEP7_MASK | PIN_STEP8_MASK)

static const uint32_t step_masks[STEP_AXES] = {
	PIN_STEP1_MASK, PIN_STEP2_MASK, PIN_STEP3_MASK, PIN_STEP4_MASK,
	PIN_STEP5_MASK, PIN_STEP6_MASK, PIN_STEP7_MASK, PIN_STEP8_MASK,
};
static const uint32_t dir_masks[STEP_AXES] = {
	PIN_DIR1_MASK, PIN_DIR2_MASK, PIN_DIR3_MASK, PIN_DIR4_MASK,
	PIN_DIR5_MASK, PIN_DIR6_MASK, PIN_DIR7_MASK, PIN_DIR8_MASK,
};

static uint32_t step_buf[2 * STEP_HALF] DMA_BUF;
// The fill does a read-modify-write per edge. Doing that on the uncached DMA buffer is slow,
// so fill here in DTCM (zero wait) and copy the finished half over in one go.
static uint32_t scratch[STEP_HALF];
static StepGen sg;
static volatile bool running;
static volatile uint8_t fault;
static volatile uint32_t dry_sink DMA_BUF;	// DMA target in dry run, instead of GPIOE->BSRR (DMA can't reach DTCM)
static volatile uint32_t isr_max;		// worst refill time, CPU cycles

void StepInit() {
	StepFillInit(&sg, step_masks, dir_masks, STEP_DIR_SETUP);

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
	(void)RCC->APB1LENR;

	// TIM6 free running, update -> DMA request
	TIM6->CR1 = 0;
	TIM6->PSC = 0;
	TIM6->ARR = STEP_ARR;
	TIM6->CR1 = TIM_CR1_ARPE | TIM_CR1_URS; // URS so UG doesn't fire a DMA request
	TIM6->EGR = TIM_EGR_UG;
	TIM6->SR = 0;

	// Memory -> GPIOE->BSRR, 32 bit, circular, very high priority, direct mode
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream0->CR & DMA_SxCR_EN);
	DMA1->LIFCR = S0_FLAGS;
	DMA1_Stream0->PAR = (uint32_t)&GPIOE->BSRR;
	DMA1_Stream0->M0AR = (uint32_t)step_buf;
	DMA1_Stream0->NDTR = 2 * STEP_HALF;
	DMA1_Stream0->FCR = 0;
	DMA1_Stream0->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1
		| DMA_SxCR_CIRC | (3 << DMA_SxCR_PL_Pos)
		| DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
	DMAMUX1_Channel0->CCR = STEP_MUX_REQ;

	// Cycle counter, to time the refill ISR
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	NVIC_SetPriority(DMA1_Stream0_IRQn, IRQ_PRIO_STEPGEN_DMA);
	NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	fault = STEP_OK;
}

static void Restart(uint32_t target) {
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->DIER = 0;
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream0->CR & DMA_SxCR_EN);
	DMA1->LIFCR = S0_FLAGS;
	DMA1_Stream0->PAR = target;
	DMA1_Stream0->M0AR = (uint32_t)step_buf;
	DMA1_Stream0->NDTR = 2 * STEP_HALF;
	GPIOE->BSRR = STEP_ALL << 16;
	StepStart();
}

bool StepRunning() {
	return running;
}

// After an abort or fault: whatever was queued is thrown away (the planner re-syncs from the
// step positions), then the timer and DMA start again. Pins or dry run stay as they were.
void StepRecover() {
	if (running) return;
	StepFillFlush(&sg);
	Restart(DMA1_Stream0->PAR);
}

// Point the DMA at the pins or at a dummy word. Only while idle: buffered words are dropped.
bool StepDryRun(bool on) {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		if (!StepAxisIdle(i)) return false;
	}
	Restart(on ? (uint32_t)&dry_sink : (uint32_t)&GPIOE->BSRR);
	return true;
}

bool StepDry() {
	return DMA1_Stream0->PAR == (uint32_t)&dry_sink;
}

uint32_t StepIsrMaxUs() {
	return isr_max / (CLOCK_SYSCLK_HZ / 1000000);
}

void StepIsrReset() {
	isr_max = 0;
}

void StepStart() {
	// Fill both halves before the first request
	StepFillWords(&sg, &step_buf[0], STEP_HALF);
	StepFillWords(&sg, &step_buf[STEP_HALF], STEP_HALF);
	__DSB();

	fault = STEP_OK;	// explicit (re)start clears an old fault
	running = true;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
	TIM6->DIER = TIM_DIER_UDE;
	TIM6->CR1 |= TIM_CR1_CEN;
}

bool StepQueue(uint32_t axis, const StepSegment *seg) {
	if (!running) return false;
	return StepFillQueue(&sg, axis, seg);
}

uint32_t StepQueueFree(uint32_t axis) {
	return axis < STEP_AXES ? StepFillQueueFree(&sg, axis) : 0;
}

// 64 bit values written by the DMA ISR can tear, read until stable
static uint64_t Read64(volatile const void *p) {
	uint64_t a, b;
	do {
		a = *(volatile const uint64_t *)p;
		b = *(volatile const uint64_t *)p;
	} while (a != b);
	return a;
}

uint64_t StepNow() {
	// sg.tick is the next tick the ISR will fill. Anything starting from there is in time.
	return Read64(&sg.tick);
}

int64_t StepPosition(uint32_t axis) {
	return axis < STEP_AXES ? (int64_t)Read64(&sg.axis[axis].position) : 0;
}

uint32_t StepLate(uint32_t axis) {
	return axis < STEP_AXES ? sg.axis[axis].late : 0;
}

bool StepAxisIdle(uint32_t axis) {
	return StepFillAxisIdle(&sg, axis);
}

uint64_t StepCount(uint32_t axis) {
	return axis < STEP_AXES ? Read64(&sg.axis[axis].emitted) : 0;
}

// Flags first: the DMA interrupt can cut in while this runs (it's higher priority than the
// DIAG pins) and must see we're stopped, not report its own fault on top
static void Stop(uint8_t why, bool disable) {
	if (fault == STEP_OK) fault = why;
	running = false;
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->DIER = 0;
	if (disable) BoardStepperEnable(false);
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream0->CR & DMA_SxCR_EN); // wait for the last transfer so nothing lands after the reset below
	GPIOE->BSRR = STEP_ALL << 16;
}

void StepAbort() {
	Stop(STEP_FAULT_ABORT, true);
}

// A driver's DIAG went high: no more steps, but leave the drivers enabled so the fault flags can
// still be read (ENN high clears the short flags). tmc.c turns them off after
void StepDriverFault() {
	Stop(STEP_FAULT_DRIVER, false);
}

uint8_t StepFault() {
	return fault;
}

void DMA1_Stream0_IRQHandler() {
	uint32_t flags = DMA1->LISR & S0_FLAGS;
	DMA1->LIFCR = flags;

	if (!running) return;

	if (flags & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0)) {
		Stop(STEP_FAULT_DMA, true);
		return;
	}
	// Both halves done since last time = one got replayed stale
	if ((flags & (DMA_LISR_HTIF0 | DMA_LISR_TCIF0)) == (DMA_LISR_HTIF0 | DMA_LISR_TCIF0)) {
		Stop(STEP_FAULT_UNDERRUN, true);
		return;
	}

	uint32_t *half, next;
	if (flags & DMA_LISR_HTIF0) {
		half = &step_buf[0]; // DMA is in the top half now
		next = DMA_LISR_TCIF0;
	} else if (flags & DMA_LISR_TCIF0) {
		half = &step_buf[STEP_HALF];
		next = DMA_LISR_HTIF0;
	} else {
		return;
	}
	uint32_t t0 = DWT->CYCCNT;
	StepFillWords(&sg, scratch, STEP_HALF);
	memcpy(half, scratch, sizeof(scratch));
	__DSB();
	t0 = DWT->CYCCNT - t0;
	if (t0 > isr_max) isr_max = t0;

	// DMA already got through the other half, so it's back in the one we just wrote
	if (DMA1->LISR & next) Stop(STEP_FAULT_UNDERRUN, true);
}

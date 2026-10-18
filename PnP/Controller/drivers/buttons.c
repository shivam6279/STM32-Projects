#include "buttons.h"
#include "board.h"
#include "clock.h"
#include "irq_prio.h"
#include "stm32h7xx.h"
#include "tmc.h"

#define BTN_PORT	3		// EXTICR value for port D
#define BTN_LINES	0xFF	// EXTI lines 0-7

// Button n = net BTN(n+1). The order on PD0-PD7 comes from the pin sheet.
static const uint8_t btn_pin[BUTTONS] = {
	PIN_BTN1_NUM, PIN_BTN2_NUM, PIN_BTN3_NUM, PIN_BTN4_NUM,
	PIN_BTN5_NUM, PIN_BTN6_NUM, PIN_BTN7_NUM, PIN_BTN8_NUM,
};

// All on port D and on pins 0-7, so EXTI lines 0-7 (one per pin, the sheet can't repeat a pin)
#define ON_PD0_7(n) (PIN_BTN##n##_PORT_IDX == BTN_PORT && PIN_BTN##n##_NUM < 8)
_Static_assert(ON_PD0_7(1) && ON_PD0_7(2) && ON_PD0_7(3) && ON_PD0_7(4) && ON_PD0_7(5) && ON_PD0_7(6) && ON_PD0_7(7) && ON_PD0_7(8),
	"buttons expected on PD0-PD7");

static volatile uint32_t edges[BUTTONS];
static volatile uint32_t edge_ms[BUTTONS];
static uint8_t line_btn[8];	// EXTI line -> button
static uint8_t stable;
static uint8_t changed;

static const IRQn_Type irqs[] = { EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn };

uint8_t ButtonsRaw() {
	uint32_t idr = PIN_BTN1_PORT->IDR;
	uint8_t i, v = 0;
	for (i = 0; i < BUTTONS; i++) v |= ((idr >> btn_pin[i]) & 1) << i;
	return v;
}

void ButtonsInit() {
	uint8_t i;

	// SYSCFG clock is already on (BoardIoInit). Lines 0-7 -> port D
	for (i = 0; i < 8; i++) {
		SYSCFG->EXTICR[i / 4] = (SYSCFG->EXTICR[i / 4] & ~(0xF << ((i % 4) * 4))) | (BTN_PORT << ((i % 4) * 4));
	}
	for (i = 0; i < BUTTONS; i++) line_btn[btn_pin[i]] = i;

	EXTI->RTSR1 |= BTN_LINES;
	EXTI->FTSR1 |= BTN_LINES;
	EXTI_D1->PR1 = BTN_LINES;
	EXTI_D1->IMR1 |= BTN_LINES;

	stable = ButtonsRaw();
	changed = 0;

	for (i = 0; i < sizeof(irqs) / sizeof(irqs[0]); i++) {
		// EXTI9_5 is shared with DIAG1-2 (lines 8-9), so it runs at their priority
		NVIC_SetPriority(irqs[i], irqs[i] == EXTI9_5_IRQn ? IRQ_PRIO_DIAG : IRQ_PRIO_BUTTONS);
		NVIC_EnableIRQ(irqs[i]);
	}
}

void ButtonsTask() {
	uint8_t i, raw = ButtonsRaw(), bit;
	uint32_t now = Millis();

	for (i = 0; i < BUTTONS; i++) {
		bit = 1 << i;
		// Different from what we have, and no edges for a while = settled
		if ((raw ^ stable) & bit && now - edge_ms[i] >= BUTTON_DEBOUNCE_MS) {
			stable ^= bit;
			changed |= bit;
		}
	}
}

uint8_t ButtonsState() {
	return stable;
}

uint8_t ButtonsChanged() {
	uint8_t c = changed;
	changed = 0;
	return c;
}

uint32_t ButtonsEdges(uint8_t n) {
	return n < BUTTONS ? edges[n] : 0;
}

void ButtonsIsr() {
	uint32_t pr = EXTI_D1->PR1 & BTN_LINES;
	uint32_t now = Millis();
	uint8_t line, b;

	EXTI_D1->PR1 = pr;
	for (line = 0; line < 8; line++) {
		if (pr & (1 << line)) {
			b = line_btn[line];
			edges[b]++;
			edge_ms[b] = now;
		}
	}
}

void EXTI0_IRQHandler() { ButtonsIsr(); }
void EXTI1_IRQHandler() { ButtonsIsr(); }
void EXTI2_IRQHandler() { ButtonsIsr(); }
void EXTI3_IRQHandler() { ButtonsIsr(); }
void EXTI4_IRQHandler() { ButtonsIsr(); }
void EXTI9_5_IRQHandler() {
	TmcDiagIsr(); // lines 8-9 = DIAG1-2, first: it stops the steppers
	ButtonsIsr();
}

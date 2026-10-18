#include "board.h"

static void PinConfig(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t af, uint32_t otype, uint32_t speed, bool high) {
	uint32_t moder;

	port->OTYPER = (port->OTYPER & ~(1 << pin)) | (otype << pin);
	port->OSPEEDR = (port->OSPEEDR & ~(3 << (pin * 2))) | (speed << (pin * 2));
	port->PUPDR &= ~(3 << (pin * 2)); // no pulls anywhere in the sheet

	if (mode == PIN_MODE_AF) {
		port->AFR[pin >> 3] = (port->AFR[pin >> 3] & ~(0xF << ((pin & 7) * 4))) | (af << ((pin & 7) * 4));
	}

	switch (mode) {
		case PIN_MODE_IN:
			moder = 0;
			break;
		case PIN_MODE_OUT:
		case PIN_MODE_PARK_LOW:
			port->BSRR = high ? (1 << pin) : (1 << (pin + 16)); // set level before it becomes an output
			moder = 1;
			break;
		case PIN_MODE_AF:
			moder = 2;
			break;
		default:
			moder = 3;
			break;
	}
	port->MODER = (port->MODER & ~(3 << (pin * 2))) | (moder << (pin * 2));
}

void BoardIoInit() {
	// Keep the debugger working while the core sleeps in WFI
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEPD1 | DBGMCU_CR_DBG_STOPD1 | DBGMCU_CR_DBG_STANDBYD1 | DBGMCU_CR_DBG_CKD1EN | DBGMCU_CR_DBG_CKD3EN;

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	(void)RCC->APB4ENR;

	// FLASH2 is on PC3_C, close the analog switch so the GPIO gets to the pad
	// TODO verify in RM0468 SYSCFG_PMCR
	SYSCFG->PMCR &= ~SYSCFG_PMCR_PC3SO;

	// Outputs start low, except STEPPER_EN starts disabled
#define IS_STEPPER_EN(id) (PIN_##id##_PORT_IDX == PIN_STEPPER_EN_PORT_IDX && PIN_##id##_NUM == PIN_STEPPER_EN_NUM)
#define PIN_INIT(id, port, pin, mode, af, otype, speed) \
	PinConfig(GPIO##port, pin, mode, af, otype, speed, IS_STEPPER_EN(id) ? STEPPER_EN_ACTIVE_LOW : false);
	PINS_TABLE(PIN_INIT)
#undef PIN_INIT
#undef IS_STEPPER_EN
}

void BoardLed(uint8_t led, bool on) {
	bool high = DEBUG_LED_ACTIVE_HIGH ? on : !on;
	if (led == 0) PIN_DEBUG_LED0_PORT->BSRR = high ? PIN_DEBUG_LED0_MASK : (PIN_DEBUG_LED0_MASK << 16);
	else PIN_DEBUG_LED1_PORT->BSRR = high ? PIN_DEBUG_LED1_MASK : (PIN_DEBUG_LED1_MASK << 16);
}

bool BoardStepperEnabled() {
	bool high = (PIN_STEPPER_EN_PORT->ODR & PIN_STEPPER_EN_MASK) != 0;
	return STEPPER_EN_ACTIVE_LOW ? !high : high;
}

void BoardStepperEnable(bool enable) {
	bool high = STEPPER_EN_ACTIVE_LOW ? !enable : enable;
	PIN_STEPPER_EN_PORT->BSRR = high ? PIN_STEPPER_EN_MASK : (PIN_STEPPER_EN_MASK << 16);
}

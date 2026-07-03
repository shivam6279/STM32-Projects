// Standby + wake on PA2 (touch) / PA4 (USB 3V3 rail). See lowpower.h.

#include "lowpower.h"
#include "main.h"
#include "rgb_pwm.h"

// Wake lines (active HIGH): PA2 = PWR_WKUP4, PA4 = PWR_WKUP2.
#define WAKE_PORT     GPIOA
#define WAKE_A_PIN    GPIO_PIN_2     // PWR_WKUP4
#define WAKE_B_PIN    GPIO_PIN_4     // PWR_WKUP2

static uint32_t s_last_active_ms;

uint8_t LowPower_WokeFromStandby(void) {
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB)) {
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		return 1u;
	}
	return 0u;
}

void LowPower_Init(void) {
	GPIO_InitTypeDef in = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* PA2 (touch): no pull. The AT42QT1010 OUT is push-pull and actively
	 * drives the line low when un-touched, so an internal pull is redundant
	 * (and the spec wants this pin left floating). */
	in.Pin  = WAKE_A_PIN;
	in.Mode = GPIO_MODE_INPUT;
	in.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WAKE_PORT, &in);

	/* PA4 (REGN 3V3 rail): NO pull -- it's driven by the rail through a series
	 * impedance, so an internal pull-down would fight it and read low even with
	 * USB present. With no pull it reflects the rail: high = USB up, low = down. */
	in.Pin  = WAKE_B_PIN;
	in.Mode = GPIO_MODE_INPUT;
	in.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WAKE_PORT, &in);

	s_last_active_ms = HAL_GetTick();
}

void LowPower_NoteActivity(void) {
	s_last_active_ms = HAL_GetTick();   // defer standby (e.g. while charging)
}

static uint8_t wake_line_high(void) {
	return (HAL_GPIO_ReadPin(WAKE_PORT, WAKE_A_PIN) == GPIO_PIN_SET) ||
		   (HAL_GPIO_ReadPin(WAKE_PORT, WAKE_B_PIN) == GPIO_PIN_SET);
}

uint8_t LowPower_UsbPresent(void) {
	// PA4 = the REGN-powered 3V3 rail: high only while USB/REGN is up.
	return (HAL_GPIO_ReadPin(WAKE_PORT, WAKE_B_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

static void enter_standby(void) {
	RGB_SetRGB(0, 0, 0);                            // cosmetic; standby kills it anyway

	/* In Standby the GPIO output drivers are off, so the LED pins float. These
	 * are active-LOW LEDs (OFF = pin high), so hold PA12/13/14 high with the
	 * internal pull-up through standby to keep the LED dark. This relies on the
	 * external BOOT0 pulldown on PA14 having been removed -- the ~40k internal
	 * pull-up cannot overcome a ~10k external pulldown. Boot still works because
	 * nBOOT_SEL=1 makes the BOOT0 pin irrelevant (boot taken from nBOOT0). */
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A,
							   PWR_GPIO_BIT_12 | PWR_GPIO_BIT_13 | PWR_GPIO_BIT_14);
	// PA2 (touch, WKUP4) left floating: the AT42QT1010 OUT is push-pull and
	// holds the line low until a real touch, so no standby pull is needed.
	// PA11 (cell-tap analog enable) pulled low: in Standby the output
	// driver is off, and a floating enable could leave the dividers bleeding.
	// PA3 + PC14/PC15 (cell-balance FET gates) likewise: a floating gate
	// could leave a ~50R bleed across a cell for the whole sleep.
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_3 | PWR_GPIO_BIT_11);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_14 | PWR_GPIO_BIT_15);
	HAL_PWREx_EnablePullUpPullDownConfig();

	/* Arm wake-on-high for both lines, clear any pending wake flags, sleep.
	 * Both lines are low at this point (that is why we are sleeping), so there
	 * is no immediate re-wake; the next rising edge wakes via system reset. */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);  // PA4
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);  // PA2
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);

	HAL_PWR_EnterSTANDBYMode();                      // wake = system reset
}

void LowPower_EnterStandby(void) {
	enter_standby();                                 // on-demand standby (wake = reset)
}

void LowPower_Task(void) {
	if (wake_line_high()) {
		s_last_active_ms = HAL_GetTick();           // activity: restart idle timer
		return;
	}
	if ((uint32_t)(HAL_GetTick() - s_last_active_ms) >= LOWPOWER_IDLE_MS) {
		enter_standby();
	}
}

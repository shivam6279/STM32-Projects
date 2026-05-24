/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lowpower.c
  * @brief   Auto-standby with wake on PA2/PA4. See lowpower.h.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "lowpower.h"
#include "main.h"
#include "rgb_pwm.h"

/* Wake lines (active HIGH): PA2 = PWR_WKUP4, PA4 = PWR_WKUP2. */
#define WAKE_PORT     GPIOA
#define WAKE_A_PIN    GPIO_PIN_2     /* PWR_WKUP4 */
#define WAKE_B_PIN    GPIO_PIN_4     /* PWR_WKUP2 */

static uint32_t s_last_active_ms;

uint8_t LowPower_WokeFromStandby(void)
{
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB))
    {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
        return 1u;
    }
    return 0u;
}

void LowPower_Init(void)
{
    GPIO_InitTypeDef in = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Read the wake lines in run mode. Pull-down so a floating/released line
     * reads low (idle); an external source pulling high still reads high. */
    in.Pin  = WAKE_A_PIN | WAKE_B_PIN;
    in.Mode = GPIO_MODE_INPUT;
    in.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WAKE_PORT, &in);

    s_last_active_ms = HAL_GetTick();
}

static uint8_t wake_line_high(void)
{
    return (HAL_GPIO_ReadPin(WAKE_PORT, WAKE_A_PIN) == GPIO_PIN_SET) ||
           (HAL_GPIO_ReadPin(WAKE_PORT, WAKE_B_PIN) == GPIO_PIN_SET);
}

static void enter_standby(void)
{
    RGB_SetRGB(0, 0, 0);                            /* cosmetic; standby kills it anyway */

    /* Arm wake-on-high for both lines, clear any pending wake flags, sleep.
     * Both lines are low at this point (that is why we are sleeping), so there
     * is no immediate re-wake; the next rising edge wakes via system reset. */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);  /* PA4 */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);  /* PA2 */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);

    HAL_PWR_EnterSTANDBYMode();                      /* wake = system reset */
}

void LowPower_Task(void)
{
    if (wake_line_high())
    {
        s_last_active_ms = HAL_GetTick();           /* activity: restart idle timer */
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_last_active_ms) >= LOWPOWER_IDLE_MS)
    {
        enter_standby();
    }
}

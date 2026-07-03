#include "main.h"

#include "rgb_pwm.h"
#include "led_anim.h"
#include "lowpower.h"
#include "bq25798.h"
#include "debug_uart.h"
#include "battery_adc.h"
#include "balancer.h"
#include <stdio.h>

// SoC colour: avg cell = tab3 / count; <=RED red, >=GREEN green, gradient between.
#define BATT_CELL_COUNT      3u
#define BATT_GREEN_MV        4100u
#define BATT_RED_MV          3800u
#define BATT_DISPLAY_MS      3000u   // touch-display duration
#define BATT_CHG_START_MA    64      // IBAT to consider charging started
// Charging-progress hue: CC ramps red->yellow (capped below green), CV ramps
// yellow->green as IBAT tapers ICHG->ITERM. Keep these in sync with cfg.
#define BATT_ICHG_MA         1200    // fast-charge current (cfg.charge_mA)
#define BATT_ITERM_MA        120     // termination current (cfg.iterm_mA)
// Breath / morph / fade timing now live in led_anim.h (the animation engine).
// Debug: hold charging off and print LIVE cell readings every second
// (balancer disabled). Set to 0 to restore normal operation.
#define DEBUG_NO_CHARGE      0
// Current-floor test: when 1, force EVERY GPIO to analog (true Hi-Z, input
// buffer off), drop all pulls, kill the debug-clock keep-alive, then enter
// Standby forever with NO wake source armed. Isolates the bare MCU + LDO draw
// from any firmware/pin load. Recover by reflashing under reset. Set back to 0
// for normal operation.
#define LOWPOWER_FLOOR_TEST  0
/* USER CODE END PD */

// Private macro -------------------------------------------------------------
/* USER CODE BEGIN PM */

/* USER CODE END PM */

// Private variables ---------------------------------------------------------
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

// Private function prototypes -----------------------------------------------
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

// Private user code ---------------------------------------------------------
/* USER CODE BEGIN 0 */
#if LOWPOWER_FLOOR_TEST
// Force every pin to analog Hi-Z, then Standby forever -- bare MCU + LDO floor.
static void LowPowerFloorTest(void) {
  // DBGMCU keeps the clocks alive through Standby (mA-level drain) -- clear it.
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  DBG->CR = 0u;
  __HAL_RCC_DBGMCU_CLK_DISABLE();

  // Clock every GPIO bank, then set ALL pins analog (MODER all 1s) with no pull
  // (PUPDR = 0). Analog mode disconnects the Schmitt input, so a mid-rail
  // floating pin can't burn crossbar current. This is the truest Hi-Z state.
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN
               | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIOFEN;
  (void)RCC->IOPENR;
  GPIO_TypeDef *const banks[] = { GPIOA, GPIOB, GPIOC, GPIOF };
  for (unsigned i = 0; i < sizeof(banks) / sizeof(banks[0]); i++) {
    banks[i]->MODER = 0xFFFFFFFFu;   // all analog
    banks[i]->PUPDR = 0x00000000u;   // no pull-up / pull-down
  }

  // Arm no wake source and clear stale flags so Standby is actually entered
  // (a pending wake flag would bounce us straight back out).
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

  // No GPIO pull-through-standby config enabled => every pin stays true Hi-Z.
  HAL_PWR_EnterSTANDBYMode();         // wake = reset; never returns
  while (1) { }                       // belt-and-suspenders
}
#endif // LOWPOWER_FLOOR_TEST

static const char *bq_chg_state_name(bq_chg_state_t s) {
  switch (s) {
	case BQ_CHG_NOT_CHARGING: return "idle";
	case BQ_CHG_TRICKLE:      return "trickle";
	case BQ_CHG_PRECHARGE:    return "precharge";
	case BQ_CHG_FAST_CC:      return "fast-cc";
	case BQ_CHG_TAPER_CV:     return "taper-cv";
	case BQ_CHG_TOPOFF:       return "topoff";
	case BQ_CHG_DONE:         return "done";
	default:                  return "?";
  }
}

// avg cell mV -> hue 0 (red) .. 120 (green)
static uint16_t soc_hue(uint16_t avg_cell_mV) {
  int32_t v = (int32_t)avg_cell_mV;
  if (v <= (int32_t)BATT_RED_MV)   { return 0u;   }
  if (v >= (int32_t)BATT_GREEN_MV) { return 120u; }
  return (uint16_t)(((v - (int32_t)BATT_RED_MV) * 120)
					/ (int32_t)(BATT_GREEN_MV - BATT_RED_MV));
}

// Touch-display: a static SoC colour snapshot (driven through the LED engine so
// it stays the sole owner of the LED).
static void show_battery_color(uint16_t avg_cell_mV) {
  LED_ShowStatic(soc_hue(avg_cell_mV), 255u);     // 0=red .. 120=green
}

// Charge-progress hue (0=red .. 120=green) from the charger's phase + current,
// NOT loaded VBAT (which is inflated while charging). precharge/trickle = red;
// CC bulk = red->yellow (capped below green); CV = yellow->green as IBAT tapers
// ICHG->ITERM; done/topoff = green.
static uint16_t charge_progress_hue(bq_chg_state_t cs, uint16_t vbat_mV, int16_t ibat_mA) {
  switch (cs) {
	case BQ_CHG_TRICKLE:
	case BQ_CHG_PRECHARGE:
	  return 0u;                                       // red: conditioning a low pack
	case BQ_CHG_TAPER_CV: {
	  int32_t num = (int32_t)BATT_ICHG_MA - (int32_t)ibat_mA;
	  int32_t den = (int32_t)BATT_ICHG_MA - (int32_t)BATT_ITERM_MA;
	  if (den < 1)   den = 1;
	  if (num < 0)   num = 0;
	  if (num > den) num = den;
	  return (uint16_t)(60 + (num * 60) / den);        // yellow -> green
	}
	case BQ_CHG_TOPOFF:
	case BQ_CHG_DONE:
	  return 120u;                                     // green
	case BQ_CHG_FAST_CC:
	default:
	  // bulk: red->yellow over RED_MV..GREEN_MV, capped at yellow (never green)
	  return (uint16_t)(soc_hue((uint16_t)(vbat_mV / BATT_CELL_COUNT)) / 2u);
  }
}

int main(void) {
  // RGB LED (PA12/13/14, active-low): drive high (off) at boot before any init
  // so there's no glow on wake. Claims PA13/14 (SWD) immediately.
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  (void)RCC->IOPENR;
  GPIOA->BSRR  = (1U << 12) | (1U << 13) | (1U << 14);
  GPIOA->MODER = (GPIOA->MODER & ~((3U << 24) | (3U << 26) | (3U << 28)))
               | ((1U << 24) | (1U << 26) | (1U << 28));

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  /* A debug session sets DBG_STOP/DBG_STANDBY, which keep the clocks running
   * through Standby (mA-level drain). They survive everything except a true
   * POR -- and VBAT keeps VSYS up, so a POR may never happen. Clear them. */
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  DBG->CR = 0u;
  __HAL_RCC_DBGMCU_CLK_DISABLE();

#if LOWPOWER_FLOOR_TEST
  LowPowerFloorTest();   // all pins analog Hi-Z + Standby forever; does not return
#endif

  // Configure the system clock
  SystemClock_Config();

  // Initialize all configured peripherals
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* PA13/PA14 are the SWD pins. They are intentionally left in their SWD reset
   * state by MX_GPIO_Init; wait 1 s here so a debugger can attach via a normal
   * connect, then hand PA12/PA13/PA14 to the RGB driver (SWD is lost after this).
   * Skip the wait when we just woke from standby so the rainbow resumes at once. */
  uint8_t woke_from_standby = LowPower_WokeFromStandby();
  if (!woke_from_standby) {
	HAL_Delay(1000);
  }
  RGB_PWM_Init();
  LED_Init();                  // start the interrupt-driven LED animation engine
  LowPower_Init();

  // Debug console on PA0/PA1 @ 115200 8N1 (printf retargeted here).
  DebugUART_Init();
  printf("\n=== Charger boot ===  woke=%u PA4=%u\n",
		 woke_from_standby, LowPower_UsbPresent());

  // Cell-tap ADC on PA6/PA7/PA8, analog-enable on PA11.
  BatteryADC_Init();

  // Cell-balance bleeds on PA3/PC14/PC15: outputs, all off.
  Balancer_Init();

  // Charger answers on I2C => USB present => charge; else touch wake => show SoC.
  bq_status_t bq_status = BQ25798_Init();
  if (bq_status != BQ_OK) {
	bq_status = BQ25798_Init();   // one retry in case the bus was mid-glitch
  }
  printf("BQ25798_Init() -> %d\n", (int)bq_status);

  if (bq_status == BQ_OK) {
	// ---- Charging mode: record charger status, then configure ----
	uint8_t st[5];
	if (BQ25798_ReadStatus(st) == BQ_OK) {
	  printf("wake charger status REG1B-1F: %02X %02X %02X %02X %02X\n",
			 st[0], st[1], st[2], st[3], st[4]);
	}

	// 3S Li-ion, ~2400 mAh pack.
	static const bq_config_t cfg = {
	  .cell_count    = 3u,
	  .charge_mV     = 12600u,   // 3 * 4.2 V
	  .charge_mA     = 1200u,    // 0.5C; raise toward 2400 if cells allow
	  .iprechg_mA    = 240u,     // ~0.1C
	  .iterm_mA      = 120u,     // ~0.05C
	  .input_ilim_mA = 2000u,    // ceiling; ICO/BC1.2 may lower it
	  .input_vlim_mV = 4600u,    // hold a 5 V source up
	  .watchdog      = BQ_WD_DISABLE,
	};
	printf("BQ25798_Configure() -> %d\n", (int)BQ25798_Configure(&cfg));
#if DEBUG_NO_CHARGE
	printf("DEBUG: charging disabled -> %d\n", (int)BQ25798_EnableCharging(0u));
#endif
	BQ25798_EnableADC(1, 0);     // continuous ADC for telemetry
	// disable TS ADC so REGN follows VBUS (not held up in battery-only mode)
	BQ25798_UpdateBits(BQ_REG_ADC_FN_DIS0, BQ_ADC_FN_DIS0_TS, BQ_ADC_FN_DIS0_TS);
  } else {
	// ---- Touch wake (charger down): show pack SoC colour, then sleep ----
	batt_cells_t cells;
	BatteryADC_Read(&cells);
	uint16_t avg_cell_mV = (uint16_t)(cells.tab_mV[2] / BATT_CELL_COUNT);
	printf("touch wake: TAB=%u/%u/%u mV  CELL=%u/%u/%u mV  avg=%u mV\n",
		   cells.tab_mV[0], cells.tab_mV[1], cells.tab_mV[2],
		   cells.cell_mV[0], cells.cell_mV[1], cells.cell_mV[2], avg_cell_mV);

#if DEBUG_NO_CHARGE
	// Debug: never sleep. Keep the dividers enabled and stream live reads so
	// the PA6/7/8 nodes can be probed with USB in/out. LED kept dark so its
	// software PWM can't couple into the tap nodes.
	RGB_SetRGB(0u, 0u, 0u);
	printf("DEBUG: no-sleep loop, PA11 held high\n");
	while (1) {
	  batt_cells_t live;
	  BatteryADC_Read(&live);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  printf("LIVE TAB=%u/%u/%u mV  CELL=%u/%u/%u mV  PA4=%u\n",
			 live.tab_mV[0], live.tab_mV[1], live.tab_mV[2],
			 live.cell_mV[0], live.cell_mV[1], live.cell_mV[2],
			 LowPower_UsbPresent());
	  HAL_Delay(1000);
	}
#else
	show_battery_color(avg_cell_mV);
	HAL_Delay(BATT_DISPLAY_MS);

	printf("display done -> standby (PA4=%u)\n", LowPower_UsbPresent());
	Balancer_AllOff();           // bleed FETs off
	BatteryADC_Sleep();          // analog-enable LOW
	LowPower_EnterStandby();     // wake = reset; does not return
#endif
  }

  uint16_t led_hue     = 0u;   // progress hue (set from charger phase)
  uint8_t  led_started = 0u;   // LED stays off until charging starts

  while (1) {
	// The LED is driven entirely by the interrupt-driven animation engine
	// (led_anim): the main loop only posts targets via the setters below, so
	// the breath keeps running smoothly even while this loop blocks.

	// Sleep only when PA4 (REGN 3V3 rail) is low CONTINUOUSLY for ~200 ms.
	// A real unplug holds it low; switching-noise glitches don't survive this.
#if !DEBUG_NO_CHARGE
	if (!LowPower_UsbPresent()) {
	  uint8_t real_unplug = 1u;
	  for (int i = 0; i < 40; i++) {        // 40 * 5 ms = 200 ms
		if (LowPower_UsbPresent()) { real_unplug = 0u; break; }
		HAL_Delay(5);
	  }
	  if (real_unplug) {
		// Cross-check + diagnose: is the charger really gone, or did PA4 lie?
		uint8_t s0 = 0u, fl[2] = { 0u, 0u };
		bq_status_t r = BQ25798_ReadReg8(BQ_REG_CHG_STAT0, &s0);
		BQ25798_ReadFaults(fl);
		printf("PA4 low 200ms: charger r=%d REG1B=%02X FAULT=%02X %02X\n",
			   (int)r, s0, fl[0], fl[1]);
		if (led_started) {
		  // breathe out to dark, then sleep; the engine animates it from the
		  // TIM14 ISR while we wait (bounded so a stuck flag can't hang here).
		  LED_FadeOut();
		  uint32_t t0 = HAL_GetTick();
		  while (LED_Busy() && (uint32_t)(HAL_GetTick() - t0) < BATT_FADE_MS + 100u) {
		  }
		}
		/* ADC off so the BQ drops to its ~20 uA battery-only state; settings
		 * persist on VBAT (watchdog disabled), so EN_ADC would stay set. */
		BQ25798_EnableADC(0, 0);
		Balancer_AllOff();
		BatteryADC_Sleep();
		LowPower_EnterStandby();   // wake = reset; does not return
	  }
	}
#endif // !DEBUG_NO_CHARGE

	// Once per second: report charger telemetry/status.
	static uint32_t last_poll = 0u;
	if ((uint32_t)(HAL_GetTick() - last_poll) >= 1000u) {
	  last_poll = HAL_GetTick();

	  uint8_t        status[5];
	  bq_adc_t       m;
	  bq_chg_state_t cs = BQ_CHG_NOT_CHARGING;
	  uint8_t        vbus_present = 0u;

	  if (BQ25798_ReadStatus(status) == BQ_OK) {
		vbus_present = (uint8_t)(status[0] & 0x01u);   // REG1B VBUS_PRESENT_STAT
		uint8_t pg   = (uint8_t)((status[0] >> 3) & 0x01u); // PG_STAT
		(void)BQ25798_GetChargeState(&cs);

		uint8_t faults[2] = { 0u, 0u };
		BQ25798_ReadFaults(faults);

		if (BQ25798_ReadADC(&m) == BQ_OK) {
		  printf("VBUS=%umV VBAT=%umV VSYS=%umV IBUS=%dmA IBAT=%dmA "
				 "T=%dC chg=%s vbus=%u pg=%u PA4=%u FAULT=%02X %02X\n",
				 m.vbus_mV, m.vbat_mV, m.vsys_mV, m.ibus_mA, m.ibat_mA,
				 (int)(m.tdie_C_x10 / 10), bq_chg_state_name(cs),
				 vbus_present, pg, LowPower_UsbPresent(), faults[0], faults[1]);

		  // Update the charging LED: hue from charge phase + current (not VBAT).
		  led_hue = charge_progress_hue(cs, m.vbat_mV, m.ibat_mA);
		  uint8_t fault = (uint8_t)(faults[0] != 0u || faults[1] != 0u);
		  // FULL only once balancing is done too: keep breathing (green) while
		  // the post-charge top-balance runs, so unplugging early is visibly
		  // "not finished yet".
		  uint8_t full  = (uint8_t)((cs == BQ_CHG_DONE || cs == BQ_CHG_TOPOFF)
								  && Balancer_Balanced());

		  // show the LED once current flows, the charger reports done/topoff
		  // (covers a full-on-plug-in pack that goes straight to the post-full
		  // top-balance with IBAT=0), or full/fault. The first LED_Breathe after
		  // this anchors the breath at its trough (OFF->breathe entry).
		  if (!led_started &&
			  (m.ibat_mA >= BATT_CHG_START_MA ||
			   cs == BQ_CHG_DONE || cs == BQ_CHG_TOPOFF || full || fault)) {
			led_started = 1u;
		  }

		  if (led_started) {
			if (fault)     { LED_Fault(); }
			else if (full) { LED_Full(); }
			else           { LED_Breathe(led_hue); }
		  }
		}
	  }
	  else {
		printf("BQ25798 not responding\n");
	  }

#if DEBUG_NO_CHARGE
	  // Live tap read every second, charging held off, no balancing.
	  batt_cells_t live;
	  BatteryADC_Read(&live);
	  printf("LIVE TAB=%u/%u/%u mV  CELL=%u/%u/%u mV\n",
			 live.tab_mV[0], live.tab_mV[1], live.tab_mV[2],
			 live.cell_mV[0], live.cell_mV[1], live.cell_mV[2]);
	  // Hold the dividers enabled between reads so the PA6/7/8 nodes can be
	  // probed with a meter (BatteryADC_Read drops the enable at the end).
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
#else
	  // Balance check (self-timed): pause-measure-decide on clean readings.
	  Balancer_Task(cs);

	  // Per-cell voltages: the balancer's clean snapshot (charge + bleeds
	  // paused), NOT a live read -- that would be inflated by the charge I*R.
	  const batt_cells_t *cells = Balancer_CleanCells();
	  if (cells != NULL) {
		printf("TAB=%u/%u/%u mV  CELL=%u/%u/%u mV (clean)\n",
			   cells->tab_mV[0], cells->tab_mV[1], cells->tab_mV[2],
			   cells->cell_mV[0], cells->cell_mV[1], cells->cell_mV[2]);
	  }

	  // Which balance bleeds are currently on (silent when none).
	  uint8_t bleed = Balancer_BleedMask();
	  if (bleed != 0u) {
		printf("BLEED: cell1=%s cell2=%s cell3=%s\n",
			   (bleed & 1u) ? "ON" : "off",
			   (bleed & 2u) ? "ON" : "off",
			   (bleed & 4u) ? "ON" : "off");
	  }
#endif
	  (void)vbus_present;
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
	Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5; // long: high-Z dividers
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
	Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
	Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|GPIO_PIN_14, GPIO_PIN_RESET);

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_11, GPIO_PIN_RESET);

  // Configure GPIO pins : PC15 PC14
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure GPIO pins : PA0 PA1
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA11  (PA12/PA13/PA14 deferred to RGB_PWM_Init so
	PA13=SWDIO / PA14=SWCLK stay usable as SWD for ~1 s after reset) */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pins : PB6 PB7
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  // User can add his own implementation to report the HAL error return state
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif // USE_FULL_ASSERT

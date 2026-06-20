# Cubli Motor — FOC Controller Review & TODO

Review of the raw motor-control / FOC stack (`BLDC.c`, `PWM.c`, `ADC.c`, `pid.c`).

## Verdict

The **core FOC is correct** — transforms, modulation, and voltage-limiting are all done right:

- **Clarke** (`foc_current_calc`, BLDC.c:633): `i_alpha = i_u`, `i_beta = (i_v - i_w)/sqrt(3)` — amplitude-invariant, U as reference phase.
- **Park / inverse Park** (BLDC.c:641, 481): signs correct (`id = ia*cos + ib*sin`, `iq = -ia*sin + ib*cos`).
- **Min/max zero-sequence injection** (BLDC.c:493-499) — standard SVPWM-equivalent centering.
- **Voltage clamp** `Uq_limit = sqrt(Vbat^2/3 - Ud^2)` (BLDC.c:267) — correct SVPWM inscribed-circle limit, prioritizes Vd (right for high-speed / field-weakening headroom).
- **Latency phase-advance** `angle_el_compensated` (BLDC.c:192) — demodulate current at sample angle, apply voltage at predicted angle. Correct.
- Cascade (Iq/Id -> RPM -> angle), feedforwards (R*Iq, BEMF, L*w decoupling), I2t thermal model, OV hysteresis — all the right ingredients.

The gap to "professional grade" is **not the FOC math** — it's correctness bugs in mode plumbing, current-sense offset handling, and the safety/robustness layer.

---

## Real bugs (fix first)

### 1. Position mode is clobbered — BLDC.c:215 vs :320  [CRITICAL]
In the ADC ISR, `pid_focIq.setpoint = torque_setpoint;` runs unconditionally every current-loop cycle (~50 kHz). The angle loop in `TIM5_IRQHandler` (BLDC.c:320) feeds its output via `pid_focIq.setpoint = pid_angle.output;` but **never updates `torque_setpoint`**. RPM mode sets both (BLDC.c:334-335); POS mode sets only one. So the current loop overwrites the angle command with stale `torque_setpoint` (=0) ~5x between angle updates -> **POS mode commands ~zero torque.**
- **Fix:** have the angle loop write `torque_setpoint = pid_angle.output` like the RPM loop does.
- Directly breaks Cubli balancing. Confirm intended cascade.

### 2. Current-offset auto-tracking eats stall torque — BLDC.c:579  [CRITICAL for balance]
`isns_v_offset += ISNS_OFFSET_LPF * (meas - offset)` (tau ~= 0.2 s) continuously drags "zero" toward measured current. Fine while spinning (phase current is AC), but **at standstill holding torque the phase currents are DC** and get absorbed as "offset." The `[1.55, 1.75]` clamp bounds error to ~+/-1 A out of a 7 A system — significant for precise balance-holding.
- **Fix:** calibrate offset once with the bridge disabled (zero current), then **freeze it**. `ADCCalib()` (ADC.c:17) is the right idea but is uncalled and assigns wrong vars (`isns_v_offset = isns_u`).

### 3. Dead code in `MotorPhase` — BLDC.c:868
`MOTOR_TIM->CCER |= (CCE_U|CCE_V|CCE_W)` sits after `case 5`'s `break`, inside no case — unreachable. Harmless today (enables set at init / in `MotorPhasePWM`) but means trapezoid mode never manages its own channel enables.
- **Fix:** clean up when revisiting trapezoid/sensorless.

### Minor
- `if(saturation_error)` (BLDC.c:273) is float-truthiness — use `fabsf(...) > eps`.
- `mode` / `waveform_mode` are read in ISRs but not `volatile`.
- Control-loop `dt` constants (`0.00002f`, `0.0001f`) are hard-coded and decoupled from `MX_TIM1_Init(100000)` — change PWM freq and gains/KF silently go wrong. Derive `dt` from timer config.

---

## Professional-grade gaps (what's left to do)

### Safety layer (biggest gap)

**Decision: no hardware break on this board — software-only protection.**
The TIM1 break input (`TIM_BREAK_DISABLE`, PWM.c:80) stays disabled; this board has no
hardware fault path. Accepted tradeoff: SW protection covers all *operational* faults
(overload, stall, regen/OV, thermal) but **cannot** catch a catastrophic short /
shoot-through / failed FET — current there rises at di/dt = V/L (~1.8 A/us at 16 V into
the ~9 uH phase, far more for a true bridge short), destroying the FET long before the
next sample. That risk is mitigated only by adequate dead-time (3 units in BDTR — verify
against the FET turn-off/turn-on), layout, and staying inside ratings. Named as a
deliberate choice, not an oversight.

Make the SW protection as strong as possible (in order):
1. **Instantaneous phase-current trip in the ADC ISR.** Add at the *top* of `ADC1_IRQHandler`,
   runs every interrupt (~100 kHz — current is already refreshed every ISR via
   `adc_read_motor_isns()` in both branches, BLDC.c:300). Compare **raw, unfiltered**
   `|isns_u|/|isns_v|/|isns_w|` to a hard limit (above the I2t limit, below FET/board abs max)
   -> `MotorOff()` + latch. Single biggest improvement; ~10x faster than the current TIM5 check.
2. **Trip on raw current, not `foc_iq`/`foc_id`** — those are LPF'd (a=0.5) and dq-rotated, adding lag.
3. **Latch the fault**; require explicit re-enable, bridge stays off until deliberately cleared
   (extend existing `thermal_fault`).
4. **Sensor-sanity trip:** if a current channel reads railed (~0 or full-scale) for N cycles -> fault.
   Catches a dead shunt/amp, which otherwise blinds all SW protection.
5. **Freeze the current-sense offset (see bug #2)** — a drifting offset drifts the trip threshold;
   the instantaneous trip is only trustworthy once offset is fixed. Coupled items.
6. Keep the 10 kHz I2t loop (BLDC.c:341) as the slow thermal layer beneath the fast trip — complementary.

- **No independent watchdog (IWDG).** Standard for anything spinning a flywheel.
- **No sensor fault detection** — encoder loss, overspeed, NaN guards on control state.

### Accuracy / smoothness (matters for balance)
- **Encoder calibration not wired in.** `interpolate_encoder_lut` + `encoder_calib_data` exist, but the live path uses a plain linear map (BLDC.c:178; LUT line commented out). Eccentricity/nonlinearity -> torque ripple -> balance jitter. Also need an automated **electrical-zero alignment** routine to produce `motor_zero_angle` (commissioning should be a routine, not hand-tuned EEPROM).
- **Velocity estimator.** The 2-state KF + "force to zero after 100 idle counts" (BLDC.c:372) gives a ~3 rpm dead floor and a discontinuity right near zero speed where a balancing controller lives. A **PLL / tracking observer** locked to the encoder angle gives smoother low-speed velocity + clean predicted angle.
- **Deadtime compensation.** None present (deadtime = 3 in BDTR). At low duty it distorts current and adds torque ripple.
- **Anti-windup** is an ad-hoc `integral *= 0.9` leak (BLDC.c:274). Back-calculation (`integral += Kaw*(Uq_sat - Uq)`) is cleaner and predictable.

### Optional
- Field weakening (Id<0) for more top speed from the 2300 kv motor; the Vd-priority clamp already makes this straightforward.
- ICACHE 2-way is set in `MX_ICACHE_Init` (generated code) but not in the `.ioc` — a CubeMX regen reverts it. Set the ICACHE to "2-ways set associative" in the `.ioc` to persist.
- Place the ADC ISR + FOC in `.RamFunc` to cut instruction-fetch jitter (determinism, not throughput).

---

## Suggested order
1. POS-mode clobber fix (#1) + current-offset freeze (#2) — small, high-impact, directly fix Cubli balancing.
2. SW safety layer — instantaneous phase-current trip + latch + sensor-sanity (Safety items 1-5).
3. Encoder calibration + observer — smoothness.

---

## Refactoring / cleanliness

Pure-cleanup pass — no behavior change. From the 2026-06-20 audit. Remaining items
(the implemented ones have been removed).

### Top priority (structural)
1. **`main()` superloop carries app logic that belongs in modules.** CAN/UART RX handling + setpoint mapping + debug-capture in one ~200-line loop; the mode→setpoint mapping is duplicated in the CAN and UART branches. Extract `ApplySetpoint(mode, value)`; split `Process_CanRx()` / `Process_UartRx()`.
2. **FDCAN IRQ disable/enable guard duplicated 5×** (main.c several sites, USART.c:62). Two `static inline` helpers (`FdcanIrqDisable/Enable`) or a critical-section wrapper.

### Medium priority
3. **Magic `0.577350269f` / `1/√3` unnamed, in multiple files** (main.c setpoint mapping, BLDC voltage scaling). Name once (`#define INV_SQRT3`) in a shared header.
4. **`tones.h` note table inconsistent** — `NOTE_C0` has 34 decimals, `NOTE_C1+` have 2. Standardize precision or compute via `NoteFreq(semitone)`.
5. **Timer-init functions in `main.c` are near-identical register sequences** (`TIM4/5/6_init`). One parameterized `InitBasicTimer(...)`.

### Low priority (hygiene)
- **Dead stubs:** `SPI.c` (`SPI1_init`/`SPI1_write16` empty), `TMP1075Init()` empty, `adc_readAll()` declared but undefined (ADC.h:42), commented-out `MX_SPI1_Init()`/`TIM4_init()` in main.
- **Header hygiene:** `diags.h` reserved guard `_DIAGS_H`; `string_utils.h` `str_toIntHex()` declared with empty params; `str_contains()` missing from header.
- **Naming drift:** `static uint8_t i;` loop counter in `CAN_send_serial` (USART.c:43) shouldn't be `static`.
- **`BLDC.h` exposes lots of internal state** (`sin_el`, `cos_el`, `foc_id/iq`, `thermal_energy`, …) as extern globals — narrow what's truly public.

### Flagged but out of scope (look like *bugs*, not cleanliness — verify separately)
- `str_isHex` boolean logic in `string_utils.c` (`str[1] != 'x' || str[1] != 'X'` is always true).
- `ADCCalib` may assign the wrong phase offsets (`isns_u`→`isns_v_offset`).

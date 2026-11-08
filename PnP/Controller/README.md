# PnP controller - STM32H723VET6

Pins come from `PNP_CTRL_H723VET6.xlsx` -> `tools/gen_pins.py` -> `bsp/pnp_h723/pins.h`. Don't edit pins.h by hand.

## Build
Uses the ST VS Code bundle tools in `%LOCALAPPDATA%\stm32cube\bundles`:
```
B=$LOCALAPPDATA/stm32cube/bundles
$B/cmake/4.0.1+st.3/bin/cmake -S . -B build -G Ninja \
  -DCMAKE_MAKE_PROGRAM=$B/ninja/1.13.1+st.1/bin/ninja.exe \
  -DARM_GCC_BIN=$B/gnu-tools-for-stm32/14.3.1+st.2/bin
$B/cmake/4.0.1+st.3/bin/cmake --build build
```
`-DSTEPGEN_SELFTEST=ON` for the analyzer test, `--target pins` to regenerate pins.h.

## Flash layout / flashing
- `pnp_boot` (boot/): bootloader, sector 0 (0x08000000). Shows up as the PnP-Boot COM port (PID 0x4002), LED0 blinks fast.
- `pnp_ctrl` (app/): app, sectors 1-2 (0x08020000). PnP-Diags COM port (PID 0x4001).
- Settings: sector 3 (0x08060000), app/settings.c. Firmware updates leave it alone; `python tools/flash_usb.py --erase-settings` wipes it through the bootloader.

GUI: double click `tools/pnp_tool.pyw`. Terminal on any COM port or WiFi (ESP32 console), and buttons for
every flash below plus ESP32 OTA. It lets go of the port while flashing and reconnects after.

Normal flashing is over USB: `python tools/flash_usb.py` (default `build/pnp_ctrl.bin`).
It sends `M997` (or `diags` + `boot`) on PnP-Diags (or on the ST-Link UART if that's busy, or `--port COMx`), erases, writes,
CRC checks and restarts the app. About 2s.
The app's vector table is written last, after the CRC matches, so an interrupted flash leaves the
bootloader running and you just run the script again.

Over WiFi: `python tools/flash_usb.py --wifi` (`--wifi <ip>` if mDNS doesn't resolve). The ESP32
sends `!boot` to the app (ignored while moving or jogging), then pipes TCP port 3233 to the
bootloader on UART6 at 921600. About 8s with the controller's BLE on (4s without).

If a change touches the H7 <-> ESP link (DLE framing, pad packet), flash the H7 first (USB is
safest), then the ESP. An old H7 can't read a new ESP's traffic, so `!boot` gets lost and the WiFi
flash fails.

Protocol in boot/loader.h: CRC32 on every packet, bad ones get resent, and the whole image CRC
once at the end of a flash. A normal boot does almost nothing: STP_EN_L driven high (ENN has no
pull-up), the vector table and the size / magic word after the image checked, jump. No clocks,
no CRC, a few us.

SWD (ST-Link) is only needed for the bootloader itself (or if both are dead). The CLI isn't on
PATH: `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`.
`mode=UR` connects under reset, so it works whatever the firmware is doing.
- Both: `STM32_Programmer_CLI -c port=SWD mode=UR -w build/pnp_boot.elf -v -w build/pnp_ctrl.elf -v -rst`
- App only (bootloader left alone): `STM32_Programmer_CLI -c port=SWD mode=UR -w build/pnp_ctrl.elf -v -rst`

ST's ROM DFU bootloader doesn't work reliably on this board (it usually keeps USB disconnected), so it isn't used.

## Commands (G-code / diags)
Three ports take commands: the PnP-Diags USB COM port, the ST-Link UART (115200) and WiFi
(raw TCP to `pnp-esp.local:3234`, see ESP32 below). Each port is on its own and starts in G-code
mode. Replies only go back to the port that sent the command.

G-code mode speaks Marlin's dialect, so Marlin hosts (OpenPnP's GcodeDriver, Pronterface, ...)
work: no echo, `start` at reset, every line answered with `ok` once it's done. A move that doesn't
fit in the step queues yet, M400, G4 and M18 hold their `ok`, which is the flow control.
Line numbers and checksums (`N.. *cs`, M110, `Resend:`) work like Marlin. M112 runs even while a
command is still waiting.

| code | what |
|---|---|
| G0 / G1 | move. Axes X Y Z A B C U V (I J K = A B C), F in mm/min, modal. F is along X Y Z only (A B just finish with them), deg/min for a rotary only move |
| G1 .. E(mm/s) | exit speed: end at E (mm/s, not modal) if the next G1 carries straight on (within ~1 deg), and that one starts at it. Queued ending at rest anyway; the end is only swapped when the next G1 arrives before the move's decel has started, else it stops like any move. Never above either move's F / axis limits, lower if a move is too short. No E = rest to rest as before |
| G4 P(ms) / S(s) | wait for motion to finish, then dwell |
| G20 / G21 | inches (error) / mm |
| G28 [axes] | homing (StallGuard or limit switch per axis, `home` in diags), order Z, Y, X. Axes not set up for it are set to 0 |
| G38.2 / G38.3 Z.. [F P S N A D R W V U T] | vacuum probe (app/probe.h): waits for motion, blanks D ms after the start / last pump-valve change, R ms reference (>= V kPa vacuum, steady), then Z toward the target at F; ref - filtered(A) > S on N fresh ADC samples in a row = Z halts dead and stays. W = rolling reference ms (0 = fixed), U = vacuum-lost rise (0 = off), T = timeout (0 = auto). G38.2: target reached = error, G38.3: result 0. Reports `[PRB:Z<z>:<1/0/E> ref: at: filt: min: n: t: hz: stop: samples: why:]` |
| G90 / G91 | absolute / relative |
| G92 | set position |
| M17 / M18 / M84 | drivers on / wait for idle, drivers off |
| M92 M201 M203 M205 | steps/mm, accel, max feed, jerk per axis. No args = report. M205 is jerk in mm/s^3 (Marlin's is a speed) |
| M204 S/P/T | path acceleration limit, 0 = axis limits only |
| M105 | `ok T:0.00 /0.00` (no heaters, hosts poll it) |
| M106 P(ch) S(0-255) / M107 P(ch) | coil driver PWM 0-7 on / off, after the moves before it finish. A channel with a strike / hold profile is just on / off (S ignored) |
| M110 | set line number |
| M112 | emergency stop: step DMA stopped, drivers + PWM off. Moves get an error until M999 |
| M114 | position: planned mm, then `Count` = steps put out |
| M115 | firmware info + UID |
| M119 | buttons |
| M400 | wait for motion to finish |
| M500 / M501 | save all settings to flash (sector 3, not while moving) / reload the last save. Boot loads the newest save |
| M502 / M503 | defaults / report settings |
| M790 P(ch) S(strike %) T(strike ms) H(hold %) | strike / hold profile, words left out unchanged, no P = report. Default: PWM 1 + 2 (valves) 50% 100ms then 25%. S0 T0 H0 = none. Saved with M500 |
| M800 | pressure sensors, kPa |
| M821 [F P S N A D R W V U T L H] | probe defaults, same letters as G38.x (F mm/min), L..H = allowed targets (L >= H = any). No letters = report. Saved with M500 |
| M820 [R] | G1 E junctions: `asked` (moves with E decided) = `full` (at E) + `limited` (slower, or stopped for speed limits / too short) + `late` (next G1 came after the decel started, or no G1 came next) + `corner` (next move turns). `last:used/asked` mm/s, `from:` re-planned from the last move's start / cruise / decel. R = reset the counters |
| M997 | reboot to the USB bootloader |
| M999 | resume after M112 or a step fault |
| T.. | ignored, `ok` |

Typing `diags` switches that port to the diags menu (echo, `[PnP]: ` prompt); `exit` goes back.

## ESP32
ESP32-C3-MINI-1 (U3) on UART6 (PC6/PC7). EN and BOOT (IO9) on the IO expander (U1, I2C3, 0x20)
P0_0 / P0_1.

Firmware in `../ESP32` (PlatformIO, Arduino), WiFi + OTA as `pnp-esp.local`. 2.4GHz only, WiFi
in `ESP32/src/secrets.h`.
- `pio run -e ota -t upload`: over WiFi
- `pio run -e usb -t upload`: through the H7 (first time / recovery), uses `tools/flash_esp.py`

WiFi console: raw TCP on port 3234 is the same G-code / diags console as the USB COM port
(greeting on connect, one client, a new one takes over). PuTTY: Raw, local echo and line editing
forced off. pyserial / hosts that take a URL: `socket://pnp-esp.local:3234`. It shares UART6 with
the ESP's own traffic, DLE framed (drivers/esp_link.c), and is off while `esp bridge` is open.

Xbox controller (BLE, on the ESP): `esp pad pair`, then hold the controller's pair button until
the logo flashes fast. The ESP keeps that one (NVS) and reconnects on the Xbox button. It sends the
state to the H7 as a `DLE P` frame on every change and every 20ms; `pad` / `pad -s` show it.
Nothing for 150ms = let go (sticks centred, no buttons).

`esp bridge` in diags wires the port straight to the ESP, 1s quiet + `+++` + 1s quiet to exit.
`esp wifi`, `esp wifi -s`, `esp scan` ask the ESP app.

## Clocks
25MHz HSE -> 520MHz SYSCLK (VOS0, LDO). HCLK 260, APBs 130, APB1 timers 260.

## Steppers / motion
See docs/motion.md for the full walkthrough. In short: TIM6 + DMA1 S0 write one word per us into
GPIOE->BSRR for all 8 STEP/DIR pins. The planner (app/motion.c) turns a move in mm into a 7 phase
S-curve, every axis on the same curve, and queues constant-jerk segments; the fill
(drivers/stepgen_fill.c) solves the exact time of every step. Limits: 200kHz per axis, 500k
steps/s total (measured with `step bench`).

Tests: `make -C test/host run` (whole motion stack on the PC), `step bench` and `step dry on` on
the board.

Homing (app/home.c) is per axis either sensorless (`home set <axis> mode soft`) or a limit switch
(`mode switch`). Sensorless: TMC2209 StallGuard in StealthChop. The axis jogs
into its hard stop; once at speed TCOOLTHRS turns StallGuard on, and the DIAG interrupt halts
just that driver's STEP output on a stall (StepHalt), so a Y1 + Y2 gantry squares itself on the
stop. Outside homing TCOOLTHRS is 0 and DIAG still only means a fault. An axis homes once
`home set` has its end, sgthrs and travel. Tuning per axis:
1. `home sg <axis> <mm>` in free air: SG_RESULT while cruising at the homing speed, and an sgthrs to start from
2. `home set <axis> sgthrs <n>`, then `home <axis>` a few times: it prints how far off the last home it stopped
3. Stops early: lower sgthrs (or raise speed). Grinds without stopping: raise it. Lower `current` is gentler and more sensitive

Limit switch: a switch on a button input (`btn`, 0-7 = BTN1-8, `level` = pin level while hit). The
edge halts the axis' drivers from the EXTI interrupt (ButtonsLimitArm). `pre` only if it starts on the switch, then a fast touch at `speed`,
3 mm back, slow touch at 20%, then backoff and the position. One switch stops both Y drivers
together, so the gantry doesn't square itself in this mode. Y on btn 0, X on btn 1; Z's goes on
btn 2 but isn't connected, so Z stays soft. Check `level` with `btn mon` first: if it's the wrong
way round homing stops with "switch still hit" before it moves towards the end.

## TODO
- RM0468 / ES0491 / DS13313 refs. Couldn't get them from st.com, so values were checked against the ST CMSIS/LL headers.
- Board: STEPPER_EN polarity, LED polarity, LDO supply, PHY_RST direction (left as input).
- STEP_DIR_SETUP from the driver datasheet.
- PWM0-7 held low until there's a PWM driver.

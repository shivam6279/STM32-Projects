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
- `pnp_ctrl` (app/): app, sectors 1-3 (0x08020000). PnP-Diags COM port (PID 0x4001).

Normal flashing is over USB: `python tools/flash_usb.py` (default `build/pnp_ctrl.bin`).
It sends `M997` (or `diags` + `boot`) on PnP-Diags (or on the ST-Link UART if that's busy, or `--port COMx`), erases, writes,
CRC checks and restarts the app. About 2s.
The app's vector table is written last, after the CRC matches, so an interrupted flash leaves the
bootloader running and you just run the script again.

SWD is only needed for the bootloader itself (or if both are dead):
`STM32_Programmer_CLI -c port=SWD mode=UR -w build/pnp_boot.elf -v -w build/pnp_ctrl.elf -v -rst`

ST's ROM DFU bootloader doesn't work reliably on this board (it usually keeps USB disconnected), so it isn't used.

## Commands (G-code / diags)
Both ports take commands: the PnP-Diags USB COM port and the ST-Link UART (115200). Each port is
on its own and starts in G-code mode. Replies only go back to the port that sent the command.

G-code mode speaks Marlin's dialect, so Marlin hosts (OpenPnP's GcodeDriver, Pronterface, ...)
work: no echo, `start` at reset, every line answered with `ok` once it's done. A move that doesn't
fit in the step queues yet, M400, G4 and M18 hold their `ok`, which is the flow control.
Line numbers and checksums (`N.. *cs`, M110, `Resend:`) work like Marlin. M112 runs even while a
command is still waiting.

| code | what |
|---|---|
| G0 / G1 | move. Axes X Y Z A B C U V (I J K = A B C), F in mm/min, modal. E ignored |
| G4 P(ms) / S(s) | wait for motion to finish, then dwell |
| G20 / G21 | inches (error) / mm |
| G28 [axes] | no endstops yet: waits for idle and sets the position to 0 |
| G90 / G91 | absolute / relative |
| G92 | set position |
| M17 / M18 / M84 | drivers on / wait for idle, drivers off |
| M92 M201 M203 M205 | steps/mm, accel, max feed, jerk per axis. No args = report. M205 is jerk in mm/s^3 (Marlin's is a speed) |
| M204 S/P/T | path acceleration limit, 0 = axis limits only |
| M105 | `ok T:0.00 /0.00` (no heaters, hosts poll it) |
| M106 P(ch) S(0-255) / M107 P(ch) | coil driver PWM 0-7 |
| M110 | set line number |
| M112 | emergency stop: step DMA stopped, drivers + PWM off. Moves get an error until M999 |
| M114 | position: planned mm, then `Count` = steps put out |
| M115 | firmware info + UID |
| M119 | buttons |
| M400 | wait for motion to finish |
| M500 / M501 | no storage yet (FRAM TODO) |
| M502 / M503 | defaults / report settings |
| M800 | pressure sensors, kPa |
| M997 | reboot to the USB bootloader |
| M999 | resume after M112 or a step fault |
| T.. | ignored, `ok` |

Typing `diags` switches that port to the diags menu (echo, `[PnP]: ` prompt); `exit` goes back.

## ESP32
ESP32-C3-MINI-1 (U3) on UART6 (PC6/PC7). EN and BOOT (IO9) on the IO expander (U1, I2C3, 0x20)
P0_0 / P0_1.

`python tools/flash_esp.py write-flash 0x0 app.bin` flashes it through the H7 (needs esptool).
`esp bridge` in diags wires the port straight to the ESP, 1s quiet + `+++` + 1s quiet to exit.

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

## TODO
- RM0468 / ES0491 / DS13313 refs. Couldn't get them from st.com, so values were checked against the ST CMSIS/LL headers.
- Board: STEPPER_EN polarity, LED polarity, LDO supply, PHY_RST direction (left as input).
- STEP_DIR_SETUP from the driver datasheet.
- PWM0-7 held low until there's a PWM driver.

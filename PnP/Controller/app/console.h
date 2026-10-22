#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include <stdbool.h>
#include <inttypes.h>

// The command ports: USB COM (PnP-Diags), the debug UART (ST-Link VCP), the ESP32's TCP console
// (pnp-esp.local:3234, carried on UART6 next to the ESP's own traffic, see esp_link.c) and the
// ethernet TCP console (10.77.0.2:3234, net.c).
// Each one is independently in one of two modes:
//   G-code (default)  Marlin style: no echo, every line answered with "ok"
//   diags             typed "diags": the menu, echo and [PnP]: prompt. "exit" goes back
//   ESP bridge        "esp bridge" in diags: straight to the ESP32 UART. 1s quiet, +++, 1s quiet to exit
// printf goes to whichever port is being served, so a reply never shows up on the other one.

#define CON_USB		0
#define CON_UART	1
#define CON_ESP		2
#define CON_NET		3
#define CON_PORTS	4

void ConsoleTask();					// main loop
void ConsoleOut(uint8_t mask);		// where printf goes, bit per port
uint8_t ConsoleOutMask();
int ConsoleReadChar();				// from the port(s) being served, -1 if nothing (for diags streaming)
bool ConsoleBridge();				// port being served -> ESP bridge

#endif

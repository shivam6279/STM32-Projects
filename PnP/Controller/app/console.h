#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include <stdbool.h>
#include <inttypes.h>

// The two command ports: USB COM (PnP-Diags) and the debug UART (ST-Link VCP).
// Each one is independently in one of two modes:
//   G-code (default)  Marlin style: no echo, every line answered with "ok"
//   diags             typed "diags": the menu, echo and [PnP]: prompt. "exit" goes back
// printf goes to whichever port is being served, so a reply never shows up on the other one.

#define CON_USB		0
#define CON_UART	1
#define CON_PORTS	2

void ConsoleTask();					// main loop
void ConsoleOut(uint8_t mask);		// where printf goes, bit per port
uint8_t ConsoleOutMask();
int ConsoleReadChar();				// from the port(s) being served, -1 if nothing (for diags streaming)

#endif

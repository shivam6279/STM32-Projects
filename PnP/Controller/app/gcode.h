#ifndef _GCODE_H_
#define _GCODE_H_

#include <stdbool.h>
#include <inttypes.h>

// Marlin dialect G-code. Every line gets "ok" when it's done, like Marlin, so a host that waits
// for "ok" before sending the next line gets flow control for free: a move that doesn't fit in
// the queue yet, M400, G4, M18 just don't answer until they can.
//
// Supported: G0 G1 G4 G20 G21 G28 G90 G91 G92
//            M17 M18 M84 M92 M105 M106 M107 M110 M112 M114 M115 M119 M201 M203 M204 M205
//            M400 M500 M501 M502 M503 M800 (pressure) M997 (bootloader) M999, T (ignored)
// Line numbers + checksums (N.. *cs) as Marlin does them, with Resend on a mismatch.
// Axes: X Y Z A B C U (see motion.c for which drivers). Marlin's I J K are accepted for A B C.

void GcodeLine(uint8_t port, char *line);	// start a command (ConsoleTask)
bool GcodeBusy(uint8_t port);				// still running, no "ok" yet
void GcodeResume(uint8_t port);				// main loop: carry on with a running command
bool GcodeEmergency(const char *line);		// M112: runs even while a command is busy

#endif

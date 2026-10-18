#pragma once

// Xbox controller over BLE (Series X|S, or an Xbox One / Elite 2 on the BLE firmware).
// Locked to one controller, its address saved in NVS. Pairing mode takes any controller that's
// in pairing mode (hold its pair button until the Xbox logo flashes fast). No saved controller
// = pairing mode from boot.

void PadSetup();
void PadLoop();
bool PadConnected();
bool PadPairing();
bool PadCmd(const char *c);	// "pad", "pad pair", "pad state". false = not a pad command

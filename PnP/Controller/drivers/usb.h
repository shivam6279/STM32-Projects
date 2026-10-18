#ifndef _USB_H_
#define _USB_H_

#include <stdbool.h>
#include <inttypes.h>

// USB FS device on PA11/PA12 (OTG_HS + internal PHY), no VBUS pin.
// CDC virtual COM port for debug.

bool UsbInit();			// false if VDD33USB never came up
void UsbTask();			// call from the main loop
bool UsbConnected();	// terminal open (DTR set)
uint32_t UsbWrite(const char *buf, uint32_t len);
int UsbReadChar();		// -1 if nothing
uint32_t UsbRead(char *buf, uint32_t len);	// returns bytes read, doesn't wait
void UsbDetach();		// drop off the bus (pull-up off)

#endif

#ifndef _LOADER_H_
#define _LOADER_H_

#include <stdbool.h>

// Flashing protocol over the USB COM port. Every packet is
//   [cmd 1][len 2, little endian][payload len]
// and every reply is 'K' (+ data) or 'E' + error code.
//
//   'I'  -> K + "PNPB" + version u16 + app base u32 + app size u32
//   'E'  sector u32                  erase one app sector (1-3)
//   'W'  offset u32 + data           write, offset/len multiples of 32
//   'V'  size u32 + crc32 u32        check CRC of the image, then write the first 32 bytes
//   'R'                              reset into the app
//
// The first 32 bytes (vector table) are held back and only written once the CRC matches,
// so an interrupted flash leaves no valid app and the bootloader stays.

void LoaderTask();
bool AppValid();

#endif

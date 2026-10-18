#ifndef _LOADER_H_
#define _LOADER_H_

#include <stdbool.h>

// Flashing over USB or UART6 (ESP32, 921600)
// Packet: [cmd 1][len 2][payload][crc32 4 of all of it]
// Reply: 'K' (+ data) or 'E' + error code. Bad packet = resend it, E/W/V can be repeated
//
//   'I'                     -> K + "PNPB" + version u16 + app base u32 + app size u32
//   'E'  sector u32         erase app sector 1-3
//   'W'  offset u32 + data  write, multiples of 32
//   'V'  size u32 + crc32   check the CRC, write the info word + vector table
//   'R'                     run the app
//
// Vector table goes in last so a half done flash never boots.
// Vector 7 = image size, info word (size + CRC) right after the image, checked every boot.
// Erase up to size + 32.

void LoaderInit();
void LoaderTask();
bool AppValid(bool crc);

#endif

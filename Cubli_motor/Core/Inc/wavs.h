#ifndef WAVS_H
#define WAVS_H

#include <stdint.h>

// Available sounds. Each lives in its own Core/Src/wav_*.c. The linker
// (--gc-sections) strips any sound that isn't referenced, so only the
// selected one costs flash.
#define WAV_ZELDA_PUZZLE	0
#define WAV_METROID_SAVE	1
#define WAV_METROID_ITEM	2

// Select which sound is compiled in (override with -DWAV_SELECTION=... if desired).
#ifndef WAV_SELECTION
#define WAV_SELECTION		WAV_ZELDA_PUZZLE
#endif

extern const uint8_t  wav_zelda_puzzle[];
extern const uint32_t wav_zelda_puzzle_size;
extern const uint8_t  wav_metroid_save[];
extern const uint32_t wav_metroid_save_size;
extern const uint8_t  wav_metroid_item[];
extern const uint32_t wav_metroid_item_size;

#if   WAV_SELECTION == WAV_ZELDA_PUZZLE
	#define wav			wav_zelda_puzzle
	#define wav_size	wav_zelda_puzzle_size
#elif WAV_SELECTION == WAV_METROID_SAVE
	#define wav			wav_metroid_save
	#define wav_size	wav_metroid_save_size
#elif WAV_SELECTION == WAV_METROID_ITEM
	#define wav			wav_metroid_item
	#define wav_size	wav_metroid_item_size
#else
	#error "Invalid WAV_SELECTION"
#endif

#endif // WAVS_H

#pragma once

#include "hal.h"

// Lamp ASM implementation. A crude streaming bytecode interpreter to perform animation synchronized with music.
// Commands are 1 + 8 bytes long. First byte is opcode:
//   0 - end program.
//   1 - wait until. 32 bit le integer - milliseconds 
//   2, 3 - pixie lights. 4 x 16 bit le integer brightness values, one for each channel.
//     2 - channels 0-3
//     3 - channels 4-7
//   4, 5, 6, 7, 8 - RGB lights. 2 x [R, G, B] bytes.

// Interprets the HEX encoded program from the stream.
void execute_lamp_asm(BaseSequentialStream *chp);

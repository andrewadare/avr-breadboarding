#ifndef __HT16K33_H__
#define __HT16K33_H__

#include "i2c_utils.h"

// 7 segment driver code for AVR microcontrollers
// Based on code by Bruce Hall w8bh.net

// ---------------------------------------------------------------------------
// 7-SEGMENT BACKPACK (HT16K33) ROUTINES
//
// The HT16K33 driver contains 16 bytes of display memory, mapped to 16 row x 8 column output
// Each column can drive an individual 7-segment display; only 0-4 are used for this device.
// Each row drives a segment of the display; only rows 0-6 are used.
//
//     0            For example, to display the number 7, we need to light up
//  -------         segments 0, 1, 2, 3, and 6. This would be binary 0100.1111
// 5|     | 1       or 0x4F.
//  |  6  |         Mapping to the display address memory:
//  -------         0x00 Digit 0 (left most digit)
// 4|     | 2       0x02 Digit 1
//  |  3  |         0x04 colon ":" on bit1
//  -------         0x06 Digit 2
//                  0x08 Digit 4 (right-most digit)
//
#define HT16K33            0xE0 // I2C bus address for Ht16K33 backpack
#define HT16K33_ON         0x21 // turn device oscillator on
#define HT16K33_STANDBY    0x20 // turn device oscillator off
#define HT16K33_DISPLAYON  0x81 // turn on output pins
#define HT16K33_DISPLAYOFF 0x80 // turn off output pins
#define HT16K33_BLINKON    0x85 // blink rate 1 Hz (-2 for 2 Hz)
#define HT16K33_BLINKOFF   0x81 // same as display on
#define HT16K33_DIM        0xE0 // add level (15=max) to byte

static const uint8_t char_segments[] = // Array of character segments
{
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, // 0 - 9
  0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, // A, b, C, d, E, F
  0x00, // blank
};

void ht16k33_init();
void ht16k33_print_(uint8_t place, uint8_t segs);
void ht16k33_print(uint8_t place, uint8_t character);
void ht16k33_clear(uint8_t place);
void ht16k33_colon(uint8_t state);
void ht16k33_digits(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t colon);
void ht16k33_integer(int value, uint8_t radix);

#endif /* __HT16K33_H__ */

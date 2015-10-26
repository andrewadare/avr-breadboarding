#include "HT16K33.h"
#include <string.h>
#include <stdlib.h>

void ht16k33_init()
{
  i2c_write_byte(HT16K33, HT16K33_ON);        // turn on device oscillator
  i2c_write_byte(HT16K33, HT16K33_DISPLAYON); // turn on display, no blink
  i2c_write_byte(HT16K33, HT16K33_DIM + 15);  // set max brightness
}

// Light up the pattern specified by segs in place 0-3, L to R.
void ht16k33_print_(uint8_t place, uint8_t segs)
{
  if (place > 3)
    return;

  // Offset to skip colon at position 2
  if (place > 1)
    place++;

  place <<= 1;

  i2c_write_byte_to_register(HT16K33, place, segs);
}

void ht16k33_print(uint8_t place, uint8_t character)
{
  // Stay within maximum array index of 16
  if (character > 0x10)
    return;

  ht16k33_print_(place, char_segments[character]);
}

// Darken requested digit
void ht16k33_clear(uint8_t place)
{
  ht16k33_print(place, 0x00);
}

// State 0,1 turns colon off/on
void ht16k33_colon(uint8_t state)
{
  i2c_write_byte_to_register(HT16K33, 0x04, state << 1);
}

void ht16k33_digits(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t colon)
{
  ht16k33_print(0, a);
  ht16k33_print(1, b);
  ht16k33_print(2, c);
  ht16k33_print(3, d);
  ht16k33_colon(colon);
}

void ht16k33_integer(int value, uint8_t radix)
{
  char s[5] = "";
  itoa(value, s, radix);
  uint8_t len = strlen(s);

  if (len > 4)
    return;

  // Light up positions 0-3
  for (uint8_t i=0; i<4; i++)
  {
    uint8_t nblanks = 4 - len;

    // Right justify
    if (i < nblanks)
      ht16k33_clear(i);

    else
    {
      char ch = s[i - nblanks];
      if (ch>='a')
        ch -= 87; // correct for hex digits
      else
        ch -= '0'; // ascii -> numeric value

      ht16k33_print(i, ch);
    }
  }
}

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


////////////////////////////////

// void SS_SetDigitRaw(byte digit, byte data) // digits (L-to-R) are 0,1,2,3
// // Send segment-data to specified digit (0-3) on LED display
// {
//   if (digit>4) return; // only digits 0-4
//   if (digit>1) digit++; // skip over colon @ position 2
//   digit <<= 1; // multiply by 2
//   I2C_WriteRegister(HT16K33,digit,data); // send segment-data to display
// }

// void SS_BlankDigit(byte digit)
// // Blanks out specified digit (0-3) on LED display
// {
//   SS_SetDigitRaw(digit,0x00); // turn off all segments on specified digit
// }

// void SS_SetDigit(byte digit, byte data)
// // display data value (0-F) on specified digit (0-3) of LED display
// {
//   if (data>0x10) return; // only values <=16
//   SS_SetDigitRaw(digit,numberTable[data]); // show value on display
// }

// void SS_SetColon(byte data) // 0=off, 1=on
// // the colon is represented by bit1 at address 0x04. There are three other single LED
// // "decimal points" on the display, which are at the following bit positions
// // bit2=top left, bit3=bottom left, bit4=top right
// {
//   I2C_WriteRegister(HT16K33,0x04,data<<1);
// }

// void SS_SetDigits(byte d0, byte d1, byte d2, byte d3, byte colon)
// {
//   SS_SetDigit(0,d0);
//   SS_SetDigit(1,d1);
//   SS_SetDigit(2,d2);
//   SS_SetDigit(3,d3);
//   SS_SetColon(colon);
// }

// void SS_Integer(int data, byte base)
// {
//   char st[5]="";
//   itoa(data,st,base); // convert to string
//   byte len = strlen(st);
//   if (len>4) return;
//   for (byte digit=0; digit<4; digit++) // for all 4 digits
//   {
//     byte blanks = 4-len; // number of blanks
//     if (digit<blanks) // right-justify display
//       SS_SetDigit(digit,0x10); // padding with blanks
//     else
//     {
//       char ch = st[digit-blanks]; // get char for this digit
//       if (ch>='a') ch-=87; // correct for hex digits
//       else ch-='0'; // ascii -> numeric value
//       SS_SetDigit(digit,ch); // display digit
//     }
//   }
// }

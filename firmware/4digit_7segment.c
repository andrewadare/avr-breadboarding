#include "HT16K33.h"
#include <util/delay.h>

void count_to(uint16_t number, uint8_t radix)
{
  for (int i=0; i<number; i++)
  {
    ht16k33_integer(i, radix);
    _delay_ms(10);
  }
}

void beef() // displays pulsating 'beeF' message
{
  ht16k33_digits(0x0b,0x0e,0x0e,0x0f,0); // write 'beeF'
  for (uint8_t count=0; count<2; count++)
  {

    // Fade in/out LED brightness
    for (uint8_t j=15; j>0; j--)
    {
      i2c_write_byte(HT16K33, HT16K33_DIM + j);
      _delay_ms(100);
    }

    for (uint8_t j=0; j<16; j++)
    {
      i2c_write_byte(HT16K33, HT16K33_DIM + j);
      _delay_ms(100);
    }
  }
}

void circles() // show rotating circle on each digit
{
  for (uint8_t count=0; count<10; count++)
  {
    for (uint8_t i=0; i<6; i++) // display each segment in turn
    {
      ht16k33_print_(0, 1 << i);
      ht16k33_print_(1, 1 << i);
      ht16k33_print_(2, 1 << i);
      ht16k33_print_(3, 1 << i);

      _delay_ms(100);
    }
  }
}

int main(void)
{
  i2c_init();
  ht16k33_init();

  while (1)
  {
    circles();
    beef();
    circles();
    count_to(255, 10);
  }

  return 0;
}

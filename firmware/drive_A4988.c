#include <avr/io.h>
#include <util/delay.h>

#define STEP PB0 // 8
#define DIR PB1  // 9

int main()
{
  DDRB &= ((1 << STEP) | (1 << DIR));

  while (1)
  {

    // Step one way
    PORTB |= (1 << DIR);
    for (uint16_t i=0; i<400; i++)
    {
      PORTB |= (1 << STEP);
      _delay_us(16);
      PORTB &= ~(1 << STEP);
      _delay_ms(160);
    }

    // // Step the other way
    // PORTB &= ~(1 << DIR);
    // for (uint16_t i=0; i<4000; i++)
    // {
    //   PORTB |= (1 << STEP);
    //   _delay_us(16);
    //   PORTB &= ~(1 << STEP);
    //   _delay_ms(16);
    // }

    _delay_ms(16000);
  }

  return 0;
}

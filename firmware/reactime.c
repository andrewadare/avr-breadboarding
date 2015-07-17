
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"

// Default timer mode, counting on timer 1.
static inline void init_timer1()
{
  // CPU clock is 1 MHz by default, and timer clocks default to the CPU rate. 
  // Here, prescale timer 1 to F_CPU/64 so it runs at 15.625 kHz.
  TCCR1B |= (1 << CS11) | (1 << CS10);
}

// Create an arbitrary delay between 1 and 3.5 sec
void arb_delay()
{
  uint8_t arbtime;
  _delay_ms(1000);                              /* wait at least 1 sec */

  // Downcast to keep only the 8 LSBs, which change too fast to predict.
  arbtime = (uint8_t) TCNT1;

  // arbtime is somewhere from 0-255 --> delay 0-2.5 sec
  while (--arbtime)
  {
    _delay_ms(10);
  }
}

int main()
{
  uint16_t dt; // Reaction time

  initUSART();
  init_timer1();

  DDRB = 0xff;         // All LED pins to output
  PORTD |= (1 << PD2); // Pullup for button

  printString("\r\nReaction timer: press any key to start.\r\n");

  while (1)
  {
    receiveByte();

    arb_delay();

    // Light all LEDs
    printString("\r\nGo!\r\n");
    PORTB = 0xff;

    // Reset counter
    TCNT1 = 0;

    if (bit_is_set(PIND, PD2))
    {
      loop_until_bit_is_clear(PIND, PD2);

      // Divide timer result by 16 to roughly be in ms, since clock is ~16 kHz
      dt = TCNT1 >> 4;

      // Send result as a zero-padded 5 digit string
      printWord(dt);
    }
    else
      printString("\r\nFalse start, try again.\r\n"); // Pressed too early

    // Reset LEDs
    PORTB = 0;
    printString("\r\nPress any key to try again.\r\n");

  }

  return 0;
}
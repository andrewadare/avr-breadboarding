// The Hello World of AVR C: blink an LED on one pin. (PB5, or arduino pin 13).

#include <avr/io.h>
#include <util/delay.h>

enum
{
  BLINK_DELAY_MS = 500, // ms
};

int main(void)
{
  // Set pin 5 in data direction register B high to enable output.
  // All other pins in DDRB are set low (which was their default).
  DDRB |= (1 << DDB5);

  while (1)
  {
    // Set PORTB5 high
    PORTB |= (1 << PB5);  // or PORTB |= _BV(PORTB5);
    _delay_ms(BLINK_DELAY_MS);

    // Set PORTB5 low
    PORTB &= ~(1 << PB5); // or PORTB &= ~_BV(PORTB5);
    _delay_ms(BLINK_DELAY_MS);
  }

  return 0;
}

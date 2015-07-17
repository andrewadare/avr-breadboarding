// Pushbutton control
// Holding down a button at PD2 enables a blinking LED at PB5.

#include <avr/io.h>
#include <util/delay.h>

enum
{
  BLINK_DELAY_MS = 100, // ms
};

int main(void)
{
  // Set pin 5 in data direction register B high to enable output.
  // All other pins in DDRB are set low (which was their default).
  DDRB |= (1 << DDB5);

  // Since the DDRs are zeroed by default, all PORTD pins are in input mode.
  // Enable the internal pullup resistor at PD2.
  // This stabilizes the AVR-facing contact of the pushbutton.
  PORTD |= (1 << PD2);

  while (1)
  {
    // Check PIND for the state of PD2. If low, the button has been pressed. 
    if (!(PIND & (1 << PD2)))
    {
      // Set PORTB5 high and wait
      PORTB |= (1 << PB5);
      _delay_ms(BLINK_DELAY_MS);

      // Set PORTB5 low and wait
      PORTB &= ~(1 << PB5);
      _delay_ms(BLINK_DELAY_MS);
    }
    else
    {
      PORTB &= ~(1 << PB5);
    }
  }

  return 0;
}

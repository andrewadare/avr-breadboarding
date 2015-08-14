// Control code for H-bridge driver chip like the L293 or SN754410.
// Direction is controlled by a toggle switch. No speed control.
// Setup:
// - PB1 for driver enable pin
// - PD2 switch to toggle motor direction
// - PD3,PD4 for driver pins 1A,2A (motor logic pins 1,2)

#include <avr/io.h>
#include <util/delay.h>

int main()
{
  // Ensure input mode on switch pin (default, so not strictly necessary).
  DDRD &= ~(1 << PD2);

  // Set the driver logic pins to output mode
  DDRD |= ((1 << PD3) | (1 << PD4));

  // Set driver EN1,2 pin to output mode, then high to enable motor
  DDRB |= (1 << PB1);
  PORTB |= (1 << PB1);

  while (1)
  {
    // Turn one way if PD2 reads high, or the other if low.
    if (PIND & (1 << PD2))
    {
      PORTD &= ~(1 << PD3);
      PORTD |= (1 << PD4);
    }
    else
    {
      PORTD |= (1 << PD3);
      PORTD &= ~(1 << PD4);
    }
  }

  return 0;
}

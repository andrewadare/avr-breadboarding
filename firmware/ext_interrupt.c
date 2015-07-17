// Example showing use of external interrupt INT0 on PD2.
// Holding a button on PD2 interrupts the main PB0 blink loop to un-light PB1.
// Besides RESET, INT0 is the highest-priority interrupt on the chip.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Interrupt Service Routine macro is called automatically on PD2 change
ISR(INT0_vect)
{
  if (PIND & (1 << PD2))
    PORTB |= (1 << PB1);
  else
    PORTB &= ~(1 << PB1);
}

// Must set both external interrupt INT0 and the global interrupt enable flag
void initINT0()
{
  EIMSK |= (1 << INT0);  // Enable INT0
  EICRA |= (1 << ISC00); // Trigger interrupt when button changes
  sei();                 // Global Set Enable Interrupt call also needed
}

int main()
{
  DDRB = 0xff;           // Set LED pins to output mode
  PORTD |= (1 << PD2);   // Enable pullup on button pin
  initINT0();

  // Blink PB0 as the main (interruptable) task
  while (1)
  {
    _delay_ms(200);
    PORTB ^= (1 << PB0);
  }

  return 0;
}

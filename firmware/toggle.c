// Pushbutton toggle control
// I put an LED at PB4 and an LED at PB5.
// Pushing a button at PD2 toggles their state.

#include <avr/io.h>
#include <util/delay.h>

// Check button_pin_register for the state of button_pin. 
// If low, the button appears pressed.
// But check twice, with a short delay, to "debounce" physical button glitching. 
uint8_t pressed(uint8_t button_pin_register, uint8_t button_pin)
{
  if (!(button_pin_register & (1 << button_pin)))
  {
    _delay_ms(5);
    if (!(button_pin_register & (1 << button_pin)))
      return 1;
  }
  return 0;
}

int main(void)
{
  // To toggle a pushbutton, we must track its state.
  // Treat it like an up/down pen click (even if it physically is not).
  uint8_t buttondown = 0;

  // Set all DDRB pins high for output
  DDRB |= 0xff;

  // Enable the pullup on the button pin
  PORTD |= (1 << PD2);

  // Start the LEDs out with PB4 low and PB5 high
  PORTB &= ~(1 << PB4);
  PORTB |= (1 << PB5);

  while (1)
  {
    if (pressed(PIND, PD2))
    {
      // Toggle PB4 and PB5 on a button push
      if (!buttondown)
      {
        PORTB ^= ((1 << PB4) | (1 << PB5));
      }
      buttondown = 1;
    }
    else
    {
      buttondown = 0;
    }
  }

  return 0;
}

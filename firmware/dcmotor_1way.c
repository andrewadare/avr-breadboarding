// Unidirectional PWM motor speed control.
// Output a PWM signal on PD5 (a.k.a OC0B, the timer 0 output pin B).
// LEDs on PB0/PB1 will light during ramp up/down.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"

#define RAMP_DELAY 2 // ms

int main()
{
  uint8_t target_speed; // From user input

  // Configure timer 0 (8 bits) for PWM to control motor speed
  TCCR0A |= ((1 << WGM00) | (1 << WGM01)); // Fast PWM. Table 15-8 mode 3
  TCCR0A |= (1 << COM0B1);                 // Output to OC0B/PD5. Table 15-6
  TCCR0B |= (1 << CS02);                   // Clock to CPU/256. Table 15-9
  // TCCR0B |= ((1 << CS02) | (1 << CS00));   // CPU/1024. Makes motor pulse

  // Controls PWM duty cycle. Continuously compared with TCNT0
  OCR0B = 0;

  // Enable output on the PWM line and on rampup/rampdown LED indicator pins
  DDRD |= (1 << PD5);
  DDRB = ((1 << PB0) | (1 << PB1));

  initUSART();

  printString("DC Motor PWM speed controller\r\n");
  printString("Enter a speed [000-255]:\r\n");
  while (1)
  {
    target_speed = getNumber(); // From USART (blocking)

    // Ramp up to target
    while (OCR0B < target_speed)
    {
      OCR0B++;
      PORTB |= (1 << PB0);
      _delay_ms(RAMP_DELAY);
    }

    // Ramp down to target
    while (OCR0B > target_speed)
    {
      OCR0B--;
      PORTB |= (1 << PB1);
      _delay_ms(RAMP_DELAY);
    }

    PORTB = 0; // Indicator LEDs off

  }

  return 0;
}

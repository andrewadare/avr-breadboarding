// Sawtooth LED fade example using only timers. Timer 1 sets the duty cycle
// for the timer 2 PWM signal (and hence the sawtooth period).
// Hardware setup: an LED on PB3.

#include <avr/io.h>
#include <util/delay.h>

static inline void init_timers()
{
  // Timer/Counter Control Register configuration

  // Use timer 1 to set the fade in/out frequency. Use CTC mode.
  // Timer 1 is a bit more complicated than timers 0 or 2 since it is 16 bits.
  // WGM10 and 11 belong to TCCR1A, while WGM12 and 13 belong to TCCR1B.
  // (see sec 16.11.1,2). For some modes, both registers need bits set.
  // See table 16-4 in the data sheet.
  // In this case, setting to CTC mode only requires this:
  TCCR1B |= (1 << WGM12);

  // Set timer 1 clock source. Use prescaler to set to F_CPU/1024 (table 16-5)
  // At (default) 1 MHz CPU clock speed, timer 1 will run at 976 Hz.
  TCCR1B |= ((1 << CS12) | (1 << CS10));

  // Use OCR1A (16 bits) to set counter TOP value
  TCCR1A |= (1 << COM1A1);
  OCR1A = 1000;

  // Timer 2 - PWM to set brightness based on OCR1A value. Fast PWM, table 18-8.
  TCCR2A |= ((1 << WGM21) | (1 << WGM20));

  // Set timer 2 clock source. Table 18-9.
  TCCR2B |= (1 << CS20);                                     // CPU/1
  // TCCR2B |= (1 << CS21);                                  // CPU/8
  // TCCR2B |= ((1 << CS22) | (1 << CS21));                  // CPU/256
  // TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20));    // CPU/1024

  // Use OCR2A to set PWM counter TOP value. Table 18-3.
  TCCR2A |= (1 << COM2A1);
}

int main()
{
  DDRB |= (1 << PB3);
  init_timers();

  while (1)
  {
    // Set PWM duty cycle on PB3 from timer/counter 1.
    OCR2A = TCNT1 >> 4;
  }

  return 0;
}

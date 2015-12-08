#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ONE_REV      400 // Steps
#define STEPPER_PORT PORTB
#define STEPPER_DDR  DDRB
#define STEP_PIN     PB0 // Arduino 8
#define DIR_PIN      PB1 // Arduino 9

#define MAX(a,b) ((a) > (b) ? a : b)

volatile uint16_t steps_taken = 0;

void setup()
{
  STEPPER_DDR |= ((1 << STEP_PIN) | (1 << DIR_PIN));

  // Configure timer 0 to tick with a 4us period. Interrupt is enabled by TIMSK0
  // and is triggered every OCR0A*4 us. One pulse period is 2*OCR0A*4 us.
  TCCR0A |= (1 << WGM01);                  // Timer 0 to CTC mode
  TCCR0B |= ((1 << CS01) | (1 << CS00));   // Set to 16MHz/64
  OCR0A = 255;                             // Init to max (~1ms) pulse width
  sei();
}

void cw()
{
  STEPPER_PORT |= (1 << DIR_PIN);
}

void ccw()
{
  STEPPER_PORT &= ~(1 << DIR_PIN);
}

// Take n steps with a pulse width (half-period) t, where t has 4us increments
void step(uint16_t n, uint16_t t)
{
  steps_taken = 0;

  // Set toggle rate
  OCR0A = MAX(1,t);

  // Stepping handled by Timer 0 interrupt. Enable, wait, disable.
  TIMSK0 |= (1 << OCIE0A);
  while (!(steps_taken == n)) {;}
  TIMSK0 &= ~(1 << OCIE0A);
}

int main()
{
  setup();

  while (1)
  {
    cw();
    step(ONE_REV, 100);
    ccw();
    step(ONE_REV, 100);
  }

  return 0;
}

// Produce a square wave by toggling STEP_PIN at a rate set by OCR0A
ISR(TIMER0_COMPA_vect)
{
  STEPPER_PORT ^= (1 << STEP_PIN);

  // Count steps on high pulses
  if (STEPPER_PORT & (1 << STEP_PIN))
    steps_taken++;
}

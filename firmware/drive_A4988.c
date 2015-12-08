// Code to control stepper drivers using the Allegro A4988 or equivalent.
// This amounts to manipulating two pins: direction and step.
// Blocking delays are avoided by using Timer 0 interrupts to toggle STEP_PIN
// at the requested rate. Configuration based on 16 MHz CPU clock.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ONE_REV      400    // Steps/rev - adjust for your motor
#define STEPPER_PORT PORTB
#define STEPPER_DDR  DDRB
#define STEP_PIN     PB0    // Arduino 8
#define DIR_PIN      PB1    // Arduino 9

#define MAX(a,b) ((a) > (b) ? a : b)

volatile uint16_t step_counter = 0;

void setup()
{
  STEPPER_DDR |= ((1 << STEP_PIN) | (1 << DIR_PIN));

  // Configure timer 0 to tick with a 4us period. Interrupt is enabled by TIMSK0
  // and is triggered every OCR0A*4 us. One high/low pulse cycle is 2x that.
  TCCR0A |= (1 << WGM01);                  // Timer 0 to CTC mode
  TCCR0B |= ((1 << CS01) | (1 << CS00));   // Prescale to F_CPU/64
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

// Take n steps with a pulse width (half-period) t
void step(uint16_t n, uint16_t t)
{
  step_counter = 0;

  // Set pin toggle rate
  OCR0A = MAX(1,t);

  // Stepping handled by Timer 0 interrupt. Enable, wait, disable.
  TIMSK0 |= (1 << OCIE0A);
  while (!(step_counter == n)) {;}
  TIMSK0 &= ~(1 << OCIE0A);
}

void move(uint16_t n)
{
  // Parameters for ramp schedule
  uint8_t t     = 255;             // Pulse width, initialized to slowest value
  uint8_t tmin  = 75;              // Pulse width at max speed
  uint8_t nramp = 18;              // Number of steps in ramp interval
  uint8_t dt = (t - tmin) / nramp;

  // Other, slightly less aggressive options for t,tmin,nramp:
  // 240,80,16; 250,100,15

  uint16_t steps_taken = 0;

  // Trapezoid-like ramp schedule
  if (n > 2*nramp)
  {
    // Ramp up
    while (steps_taken < nramp)
    {
      step(1, t);
      t -= dt;
      steps_taken++;
    }

    // Constant (max) speed
    t = tmin;
    step(n - 2*nramp, t);
    steps_taken += n - 2*nramp;

    // Ramp down
    while (steps_taken < n)
    {
      step(1, t);
      t += dt;
      steps_taken++;
    }
  }
  // Truncated (pyramid-like) ramp schedule for short moves
  else
  {
    // Ramp up
    while (steps_taken < n/2)
    {
      step(1, t);
      t -= dt;
      steps_taken++;
    }

    // Ramp down
    t += dt;
    while (steps_taken < n)
    {
      step(1, t);
      t += dt;
      steps_taken++;
    }
  }
}

int main()
{
  setup();

  while (1)
  {
    cw();
    move(10*ONE_REV);
    ccw();
    move(10*ONE_REV);
  }

  return 0;
}

// Produce a square wave by toggling STEP_PIN at a rate set by OCR0A
ISR(TIMER0_COMPA_vect)
{
  STEPPER_PORT ^= (1 << STEP_PIN);

  // Count steps on high pulses
  if (STEPPER_PORT & (1 << STEP_PIN))
    step_counter++;
}

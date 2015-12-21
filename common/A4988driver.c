#include "A4988driver.h"

volatile uint16_t step_counter = 0;
volatile int16_t position = 0;

void stepper_setup()
{
  // Enable the three control pins for output, but leave them low.
  STEPPER_DDR |= ((1 << STEP_PIN) | (1 << DIR_PIN) | (1 << STEPPER_ENABLE_PIN));

  // Configure timer x to tick with a 4us period, where x is 0 or 2. Interrupt
  // is enabled by TIMSKx and is triggered every OCRxA*4 us.
  // Square waves are generated, so the pulse period is twice the pulse width.
#ifdef USE_TIMER_0_FOR_STEPPER
  TCCR0A |= (1 << WGM01);                  // Timer 0 to CTC mode
  TCCR0B |= ((1 << CS01) | (1 << CS00));   // Prescale to F_CPU/64
  OCR0A = 255;                             // Init to max (~1ms) pulse width
#endif

#ifdef USE_TIMER_2_FOR_STEPPER
  TCCR2A |= (1 << WGM21);                  // Timer 2 to CTC mode
  TCCR2B |= (1 << CS22);                   // Prescale to F_CPU/64
  OCR2A = 255;                             // Init to max (~1ms) pulse width
#endif

  sei();
}

void enable_stepper()
{
  STEPPER_PORT &= ~(1 << STEPPER_ENABLE_PIN);
}

void disable_stepper()
{
  STEPPER_PORT |= (1 << STEPPER_ENABLE_PIN);
}

void set_stepper_position(int16_t p)
{
  position = p;
}

int16_t stepper_position()
{
  return position;
}

void setdir_cw()
{
  STEPPER_PORT |= (1 << DIR_PIN);
}

void setdir_ccw()
{
  STEPPER_PORT &= ~(1 << DIR_PIN);
}

// Take n steps with a pulse width (half-period) t
void step(uint16_t n, uint8_t t)
{
  step_counter = 0;

  // Set pin toggle rate to max(t,1).
  // Stepping is executed by timer interrupt (when enabled).
#ifdef USE_TIMER_0_FOR_STEPPER
  OCR0A = t > 1 ? t : 1;
  TIMSK0 |= (1 << OCIE0A);
  while (!(step_counter == n)) {;}
  TIMSK0 &= ~(1 << OCIE0A);
#endif

#ifdef USE_TIMER_2_FOR_STEPPER
  OCR2A = t > 1 ? t : 1;
  TIMSK2 |= (1 << OCIE2A);
  while (!(step_counter == n)) {;}
  TIMSK2 &= ~(1 << OCIE2A);
#endif
}

void ramp_move(
  uint16_t n,     // Number of steps
  uint8_t tmax,   // Pulse width at start and end of ramp (thus maximum)
  uint8_t tmin,   // Pulse width at max speed (thus minimum)
  uint8_t nramp)  // Number of steps in ramp interval
{
  uint8_t t = tmax;
  uint8_t dt = (t - tmin) / nramp;
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

// Produce a square wave by toggling STEP_PIN at a rate set by OCRxA
ISR(STEPPER_VECT)
{
  STEPPER_PORT ^= (1 << STEP_PIN);

  // Count steps on high pulses
  if (STEPPER_PORT & (1 << STEP_PIN))
  {
    step_counter++;

    // Keep track of overall (signed) position
    if (STEPPER_PORT & (1 << DIR_PIN))
    {
      position++;
    }
    else
    {
      position--;
    }
  }
}

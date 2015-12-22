#include "stepdriver.h"

volatile int16_t position = 0;

void stepper_setup()
{
  // Enable the control pins for output, but leave them low.
  STEPPER_DDR |= ((1 << DIR_PIN) | (1 << STEPPER_ENABLE_PIN));

  // Configure timer x to tick at 250 kHz, where x is 0 or 2.
  // Toggling occurs every OCRxA*4 us.
  // Square waves are generated, so the pulse period is twice the pulse width.
#ifdef USE_TIMER_0_FOR_STEPPER
  TCCR0A |= (1 << WGM01);                  // Timer 0 to CTC mode
  TCCR0B |= ((1 << CS01) | (1 << CS00));   // Prescale to F_CPU/64
  TCCR0A |= (1 << COM0B0);  // Toggle OC0B/PD5 when TCNT0 == OCR0B. Table 15-5
  TCCR0A |= (1 << COM0A0);  // Toggle OC0A/PD6 when TCNT0 == OCR0A. Table 15-2
#endif

#ifdef USE_TIMER_2_FOR_STEPPER
  TCCR2A |= (1 << WGM21);                  // Timer 2 to CTC mode
  TCCR2B |= (1 << CS22);                   // Prescale to F_CPU/64
  // TCCR2A |= (1 << COM2A0);  // Toggle OC2A/PB3 when TCNT2 == OCR2A. Table 18-2
  TCCR2A |= (1 << COM2B0);  // Toggle OC2B/PD3 when TCNT2 == OCR2B. Table 18-5
#endif

  // Initialize output compare value to max (~1ms) pulse width
  STEPPER_OCR = 255;
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
  uint16_t step_counter = 0;
  int8_t one_step = (bit_is_set(STEPPER_PORT, DIR_PIN)) ? 1 : -1;

  // Set pin toggle rate to max(t,1).
  STEPPER_OCR = t > 1 ? t : 1;

  // Stepping is handled by timer output to STEP_PIN (OCxA).
  // All that is needed here is to enable output, count steps, and disable.
  STEPPER_DDR |= (1 << STEP_PIN);
  while (step_counter < n)
  {
    loop_until_bit_is_set(STEPPER_PIN_REG, STEP_PIN);
    step_counter++;
    position += one_step;
    loop_until_bit_is_clear(STEPPER_PIN_REG, STEP_PIN);
  }
  STEPPER_DDR &= ~(1 << STEP_PIN);
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

// Example code to control stepper drivers using the Allegro A4988 or 
// equivalent. This amounts to manipulating two pins: direction and step.
// Blocking delays are avoided by using timer interrupts to toggle STEP_PIN
// at the requested rate. Configuration based on 16 MHz CPU clock.
// Note that Timer0/PORTB or Timer2/PORTD are selected in stepper_config.h, 
// along with other important parameters (like which pins to wire up!).

#include <util/delay.h>
#include "stepdriver.h"

#define ONE_REV 400        // Steps/rev - 0.9 deg / step

int main()
{
  stepper_setup();

  // Stepper ramping parameters (pulse widths and ramp length)
  // uint8_t tmax = 240, tmin = 160, nramp = 8; // Slow
  // uint8_t tmax = 240, tmin = 80, nramp = 16; // Faster
  uint8_t tmax = 255, tmin = 55, nramp = 20;    // Really fast!

  while (1)
  {
    setdir_cw();
    ramp_move(10*ONE_REV, tmax, tmin, nramp);
    setdir_ccw();
    ramp_move(10*ONE_REV, tmax, tmin, nramp);
  }

  return 0;
}

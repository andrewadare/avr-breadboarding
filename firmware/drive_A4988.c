// Example code to control stepper drivers using the Allegro A4988 or 
// equivalent. This amounts to manipulating two pins: direction and step.
// Blocking delays are avoided by using timer interrupts to toggle STEP_PIN
// at the requested rate. Configuration based on 16 MHz CPU clock.
// Note that Timer0/PORTB or Timer2/PORTD are selected in stepper_config.h, 
// along with other important parameters (like which pins to wire up!).

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "A4988driver.h"

int main()
{
  stepper_setup();

  // Stepper ramping parameters (pulse widths and ramp length)
  uint8_t tmax = 240, tmin = 80, nramp = 16;

  setdir_cw(); // right
  ramp_move(ONE_REV, tmax, tmin, nramp);
  setdir_ccw(); // left
  ramp_move(ONE_REV, tmax, tmin, nramp);
  // while (1)
  // {
  //   setdir_cw();
  //   ramp_move(ONE_REV, tmax, tmin, nramp);
  //   setdir_ccw();
  //   _delay_ms(500);
  //   ramp_move(ONE_REV, tmax, tmin, nramp);
  //   _delay_ms(500);
  // }

  return 0;
}

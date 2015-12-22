// stepdriver.h
// A mini-library for controlling stepper drivers such as the Allegro A4988,
// TI DRV8825, or equivalent. This involves controlling 3 pins: direction, step,
// and (optionally) enable. The step pin is pulsed directly by the timer
// hardware, which is easy on the CPU and doesn't require interrupts. The
// tradeoff is inflexibility: only a few pins are designed for direct timer
// output.

// TODO: Select COMxxx pins in setup routine instead of toggle-by-commenting.

#ifndef __STEPDRIVER_H__
#define __STEPDRIVER_H__
#include <avr/io.h>

/*
// Timer 0 can output pulses to PD5 and/or PD6. Thus only PORTD can be used.
#define USE_TIMER_0_FOR_STEPPER 1
#define STEPPER_PORT            PORTD      // D only
#define STEPPER_DDR             DDRD       //  "
#define STEPPER_PIN_REG         PIND       //  "
#define STEP_PIN                5          // Must be 5 or 6
#define DIR_PIN                 4
#define STEPPER_ENABLE_PIN      3
*/

// Timer 2 can pulse to PB3 and/or PD3. Thus either
// PORTB or PORTD can be used, but STEP_PIN must remain 3.
#define USE_TIMER_2_FOR_STEPPER 1
#define STEPPER_PORT            PORTD
#define STEPPER_DDR             DDRD
#define STEPPER_PIN_REG         PIND
#define STEP_PIN                3          // Must be 3
#define DIR_PIN                 4
#define STEPPER_ENABLE_PIN      5

void stepper_setup();
void enable_stepper();
void disable_stepper();
void setdir_cw();
void setdir_ccw();
void step(uint16_t n, uint8_t t);
void set_stepper_position(int16_t p);
int16_t stepper_position();

// Take n steps at a speed controlled by the provided ramp params (esp. tmin).
// Some options for tmax,tmin,nramp triplets, from fastest to slowest:
// 255,75,18; 240,80,16; 250,100,15
// Currently using 240,80,16
void ramp_move(
  uint16_t n,     // Number of steps
  uint8_t tmax,   // Pulse width at start and end of ramp (thus maximum)
  uint8_t tmin,   // Pulse width at max speed (thus minimum)
  uint8_t nramp); // Number of steps in ramp interval

#endif  /* __STEPDRIVER_H__ */

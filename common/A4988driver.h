// Code to control stepper drivers using the Allegro A4988 or equivalent.
// This amounts to manipulating two pins: direction and step.
// Blocking delays are avoided by using Timer 0 interrupts to toggle STEP_PIN
// at the requested rate. Configuration based on 16 MHz CPU clock.

#ifndef __A4988DRIVER_H__
#define __A4988DRIVER_H__
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "stepper_config.h"

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

#endif  /* __A4988DRIVER_H__ */

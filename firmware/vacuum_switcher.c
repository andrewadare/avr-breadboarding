// ATMega328P firmware for vacuum switch controller
//
// Currently, the hardware setup is about the simplest thing possible: one pin,
// PB0, controls a MOSFET switch. That's it (besides USART serial TX/RX).
// The main loop just waits around for a serial byte and sets the PORTB pins.
// The logic to shut off PB0 after VALVE_TIMEOUT milliseconds is all
// implemented in two interrupt service routines (ISRs), as explained inline.
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"

#define VALVE_TIMEOUT 3000 // ms

// Count milliseconds since valve opened. Hey compiler! Don't optimize this out.
volatile uint16_t ms_counter = 0;

static inline void init_timer1()
{
  // Set Timer 1 to CTC (Clear Timer on Compare match) mode
  TCCR1B |= (1 << WGM12);

  // Set Timer 1 clock source to F_CPU/1 (table 16-5 in data sheet)
  // At F_CPU = 1 MHz (the default), count in 1 us ticks.
  TCCR1B |= (1 << CS10);

  // Use OCR1A (16 bits) to set counter TOP value so it resets every 1 ms.
  OCR1A = 1000;

  // The timer 1 output compare interrupt enable (OCIE) bit is set conditionally 
  // in the pin-change ISR.
}

// If the pin change triggering this ISR was a rise (i.e. PB0 went on),
// enable the timer interrupt and start the millisecond timer.
// If not, don't let timer 1 generate output compare interrupts, so no counting.
ISR(PCINT0_vect)
{
  if (PORTB & (1 << PB0))
  {
    TIMSK1 |= (1 << OCIE1A);
    ms_counter = 0;
  }
  else
  {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}

// Increment millisecond counter when TCNT1 reaches OCR1A value.
// Shut off valve at timeout!
ISR(TIMER1_COMPA_vect)
{
  ms_counter++;

  if (ms_counter > VALVE_TIMEOUT)
  {
    PORTB &= ~(1 << PB0);
  }
}

int main(void)
{
  // Single character for serial TX/RX
  char a;

  // Enable output on all 8 bits in DDRB (although currently only PB0 is used)
  DDRB = 0xff;

  // Enable pin change interrupt for the B pins, but only check PB0.
  sei();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PB0);

  init_timer1();
  initUSART();

  while (1)
  {
    a = receiveByte();
    transmitByte(a);
    PORTB = a;
  }
  return 0;
}

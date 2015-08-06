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
// #include <string.h>

#define VALVE_TIMEOUT 3000 // ms

// Globals used in ISRs are declared volatile or compiler optimizes them out.
// Count milliseconds since each valve opened.
volatile uint16_t ms_counter[2];
// Track on/off state of each valve.
volatile uint8_t valve_state[2];
// Alias for PB0, PB1, to enable looping.
volatile uint8_t pb[2];

static inline void init_timer1()
{
  // Set Timer 1 to CTC (Clear Timer on Compare match) mode
  TCCR1B |= (1 << WGM12);

  // Set Timer 1 clock source to F_CPU/1 (table 16-5 in data sheet)
  // At F_CPU = 1 MHz (the default), count in 1 us ticks.
  TCCR1B |= (1 << CS10);

  // Use OCR1A (16 bits) to set counter TOP value so it resets every 1 ms.
  OCR1A = 1000;

  // Enable Timer 1 interrupt by setting the output compare interrupt enable
  // (OCIE) bit. Now TIMER1_COMPA_vect will trigger ISR once per millisecond.
  TIMSK1 |= (1 << OCIE1A);
}

// ISR for enabled PORTB pins (the high bits in PCMSK0).
// Only updates state variables; counting and switching are done in Timer 1 ISR.
ISR(PCINT0_vect)
{
  for (uint8_t i=0; i<2; i++)
  {
    // If pin went high, set valve_state to ON and reset counter.
    if ((PORTB & (1 << pb[i])) && valve_state[i] == 0)
    {
      valve_state[i] = 1;
      ms_counter[i] = 0;
    }

    // If pin went low, set valve_state to OFF.
    if (!(PORTB & (1 << pb[i])) && valve_state[i] == 1)
      valve_state[i] = 0;
  }
}

// ISR for Timer 1
// Increments selected millisecond counters when TCNT1 reaches OCR1A value.
ISR(TIMER1_COMPA_vect)
{
  for (uint8_t i=0; i<2; i++)
  {
    // Only count time if valve is on
    if (valve_state[i])
    {
      ms_counter[i]++;
    }

    // Shut off valve at timeout!
    if (ms_counter[i] > VALVE_TIMEOUT)
    {
      PORTB &= ~(1 << pb[i]);
      valve_state[i] = 0;
    }
  }

}

int main(void)
{
  // Single character for serial TX/RX
  char a;

  // Enable output on all 8 bits in DDRB (but only PB0 and PB1 are used)
  DDRB = 0xff;

  // Enable pin change interrupt for the B pins, but only check PB0 and PB1.
  sei();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= ((1 << PB0) | (1 << PB1));

  init_timer1();
  initUSART();

  pb[0] = PB0;
  pb[1] = PB1;
  while (1)
  {

    // for (uint8_t i=0; i<255; i++)
    //   cmd[i] = 0;

    // char cmd[255];
    // readString(cmd, 255);

    // if (strcmp(cmd, "V0_ON") == 0)
    // {
    //   printString("\r\nYou wrote: ");
    //   printString(cmd);
    //   printString("\r\n");
    //   PORTB |= (1 << PB0);
    // }

    // if (strcmp(cmd, "V1_ON"))
    //   PORTB |= (1 << PB1);
    // if (strcmp(cmd, "V0_OFF"))
    //   PORTB &= ~(1 << PB0);
    // if (strcmp(cmd, "V1_OFF"))
    //   PORTB &= ~(1 << PB1);

    // if (cmd == "V0_ON") set_bit(PORTB, PB0);
    // PORTB |= (1 << PB0);
    a = receiveByte();
    transmitByte(a);

    if (a == '0')
      PORTB &= ~(1 << PB0);
    if (a == '1')
      PORTB |= (1 << PB0);
    if (a == '2')
      PORTB &= ~(1 << PB1);
    if (a == '3')
      PORTB |= (1 << PB1);

    // PORTB = a;
  }
  return 0;
}

// Takes in a character at a time and sends it right back out,
// displaying the ASCII value on the LEDs.
// Uses interrupt system instead of a blocking read.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"

// Single character for serial TX/RX and "byte received" flag
volatile char a;
volatile uint8_t newbyte;

// Run when USART receives a new byte.
// Reading from and writing to UDR0 trigger critical side-effects.
// UDR0 *must* be read inside this ISR.
// Otherwise, RXC0 will not be cleared, and another interrupt will follow.
// See 20.7.3 in the datasheet.
ISR(USART_RX_vect)
{
  a = UDR0; // Assigning to an lvalue also clears RXC0
  UDR0 = a; // Writes to TX shift register (echoes a back out)
  newbyte = 1;
}

int main(void)
{
  initUSART();

  // Set the RX Complete Interrupt Enable 0 bit
  UCSR0B |= (1 << RXCIE0);
  sei();

  // All DDRB pins to output mode
  DDRB = 0xff;

  newbyte = 0;
  while (1)
  {
    // Light up PORTB in the ASCII binary representation of a
    if (newbyte)
    {
      PORTB = a;
      newbyte = 0;
    }
  }

  return 0;
}

// Takes in a character at a time and sends it right back out, 
// displaying the ASCII value on the LEDs.

#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"

int main(void)
{
  // Single character for serial TX/RX
  char a;

  // Set all 8 bits high in DDRB register to enable output
  DDRB = 0xff;

  initUSART();

  while (1)
  {
    a = receiveByte();
    transmitByte(a);
    PORTB = a;
  }
  return 0;
}

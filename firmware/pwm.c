// PWM LED example
// Compare LED brightness side-by-side on 3 adjacent LEDs
//
// This program reads a number over the UART serial bus (0-255, terminated by 
// returning) and sets LED brightness on PB1. 0 is darkest, 255 is brightest.
// As more numbers are entered, the old values are queued to PB2, then PB3.
// Hardware setup: 3 LEDs on PB1-3, plus TX,RX on the USART pins.
#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"

static inline void init_timers()
{
  // Timer/Counter Control Register configuration
  
  // Timer 1 
  // This is a 16 bit timer, but set it here to 8 bit mode for consistency 
  // with timer 2.
  TCCR1A |= (1 << WGM10);   // Set waveform generator mode to 8 bit fast PWM
  TCCR1B |= (1 << WGM12);   // This is mode 5 in table 16-4.
  TCCR1B |= (1 << CS11);    // Prescale PWM clock to F_CPU/8/256
  TCCR1A |= (1 << COM1A1);  // Use OCR1A to set counter TOP value (OC1A/PB1)
  TCCR1A |= (1 << COM1B1);  // Use OCR1B to set counter TOP value (OC1B/PB2)

  // Timer 2
  TCCR2A |= (1 << WGM20);   // Fast PWM mode,
  TCCR2A |= (1 << WGM21);   // which is mode 3 in table 18-8.
  TCCR2B |= (1 << CS21);    // PWM freq = F_CPU/8/256 = 488.3 Hz for 1 MHz CPU
  TCCR2A |= (1 << COM2A1);  // Use OCR2A to set counter TOP value (OC2A/PB3)
}

int main()
{
  uint8_t brightness;

  init_timers();
  initUSART();
  printString("LED PWM example\r\n");

  // Enable PB1-3 for output. 
  // There are 3 other PWM pins: PD3 (the other timer 2 pin), and timer 0 pins 
  // PD5 and PD6. But they are not used here.
  DDRB = 0b00001110;

  while (1)
  {
    printString("\r\nEnter a duty cycle value in the range 0-255: ");
    brightness = getNumber();

    OCR2A = OCR1B;
    OCR1B = OCR1A;
    OCR1A = brightness;
  }

  return 0;
}
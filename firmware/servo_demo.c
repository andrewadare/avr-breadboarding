// Servo controller example for ATMega*8 at 1MHz.
// Servo positions are controlled by (integrated) logic pulses.
// 20 ms is a typical servo PWM period, and PWM duty cycles are ~1/20-2/20.
// Hardware setup: servo control wire on PB1.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "USART.h"

// Pulse widths in microseconds. Precise values are servo-dependent.
#define PULSE_MIN 1000
#define PULSE_MID 1500
#define PULSE_MAX 2000

static inline void init_timer1_servo()
{
  // Fast PWM (mode 14, table 16-4). TOP (i.e. the PWM counter period) will be
  // controlled by ICR1, which we set to 20000 (20 ms with CPU/1 prescale).
  TCCR1A |= (1 << WGM11);
  TCCR1B |= ((1 << WGM12) | (1 << WGM13));
  ICR1 = 20000;

  // Clock/1 prescaling (table 16-5). 1 MHz --> 1 counter tick = 1 us.
  TCCR1B |= (1 << CS10);

  // Send timer output to OC1A, overtaking PB1 (table 16-2, non-inverting mode).
  // In other words, the PWM pulses will be physically output to PB1, and the
  // pulse width is controlled by OCR1A.
  TCCR1A |= (1 << COM1A1);

  // Enable PB1 for output
  DDRB |= (1 << PB1);
}

static inline void exercise_servo(void)
{
  printString("Center\r\n");
  OCR1A = PULSE_MID;
  _delay_ms(1500);

  printString("Clockwise Max\r\n");
  OCR1A = PULSE_MIN;
  _delay_ms(1500);

  printString("Counterclockwise Max\r\n");
  OCR1A = PULSE_MAX;
  _delay_ms(1500);

  printString("Center\r\n");
  OCR1A = PULSE_MID;
  _delay_ms(1500);
}

static inline uint16_t serial_atoi16()
{
  // Get a PWM value from the serial port.
  // Read a char string and convert to a 16 bit unsigned integer.
  char thousands = '0';
  char hundreds = '0';
  char tens = '0';
  char ones = '0';
  char byte = '0';

  do
  {
    thousands = hundreds;
    hundreds = tens;
    tens = ones;
    ones = byte;
    byte = receiveByte();

    // Echo what was just read
    transmitByte(byte);
  }
  while (byte != '\r');

  transmitByte('\n');
  return (1000 * (thousands - '0') + 100 * (hundreds - '0') +
          10 * (tens - '0') + ones - '0');
}

int main()
{

#if (F_CPU == 16000000UL)
  clock_prescale_set(clock_div_16);
#endif

  uint16_t pulse_width;

  // Home the servo
  OCR1A = PULSE_MID;

  init_timer1_servo();
  initUSART();

  printString("\r\nServo Demo\r\n----------\r\n");
  exercise_servo();

  while (1)
  {
    printString("\r\nEnter a four-digit pulse width (1000-2000 us):\r\n");
    pulse_width = serial_atoi16();

    printString("Slewing....\r\n");
    OCR1A = pulse_width;

    // Enable PB1 for output during high part of PWM cycle
    DDRB |= (1 << PB1);

    _delay_ms(1000);

    printString("Releasing...\r\n");

    // Delay while pulse is high, + margin
    while (TCNT1 < 3000) {;}

    // Disable PB1 during low part of PWM cycle.
    DDRB &= ~(1 << PB1);
  }

  return 0;
}

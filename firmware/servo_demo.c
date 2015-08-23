// Servo controller example for ATMega*8 at 1MHz or 16 MHz.
// Servo positions are controlled by (integrated) logic pulses.
// 20 ms is a typical servo PWM period, and PWM duty cycles are ~ 0.6-2.1 / 20.
// Hardware setup: servo control wire on PB1.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "USART.h"

// Pulse widths in microseconds. Precise values are servo-dependent.
#if F_CPU == 16000000UL
#define PULSE_MIN 1200
#define PULSE_MID 2850
#define PULSE_MAX 4600
#define PWM_MAX   6000
#else // 1 MHz
#define PULSE_MIN 600
#define PULSE_MID 1500
#define PULSE_MAX 2100
#define PWM_MAX   3000
#endif

static inline void init_timer1_servo()
{
  // Fast PWM (mode 14, table 16-4). TOP (i.e. the PWM counter period) will be
  // controlled by ICR1, which we set based on F_CPU to be equivalent to 20 ms.
  TCCR1A |= (1 << WGM11);
  TCCR1B |= ((1 << WGM12) | (1 << WGM13));

#if F_CPU == 16000000UL
  // Use clock/8 prescaling for a 16 MHz CPU (table 16-5).
  // Each counter tick = 0.5 us.
  // (/16 prescale not available, Ideally that's what I'd use.)
  TCCR1B |= (1 << CS11);
  ICR1 = 40000;
#else
  // Assume CPU is running at 1 MHz; prescale to clock/1.
  // Then 1 counter tick = 1 us.
  TCCR1B |= (1 << CS10);
  ICR1 = 20000;
#endif

  // Send timer output to OC1A, overtaking PB1 (table 16-2, non-inverting mode).
  // In other words, the PWM pulses will be physically output to PB1, and the
  // pulse width is controlled by OCR1A.
  TCCR1A |= (1 << COM1A1);
}

static inline void exercise_servo(void)
{
  // Enable PB1 for output
  DDRB |= (1 << PB1);

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

  // Disable PB1 until a new value is entered
  DDRB &= ~(1 << PB1);
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

    // Enable PB1 for output
    DDRB |= (1 << PB1);

    _delay_ms(1000);

    // Delay while pulse is high, + margin
    while (TCNT1 < PWM_MAX) {;}

    // Disable PB1 until a new value is entered
    DDRB &= ~(1 << PB1);
  }

  return 0;
}

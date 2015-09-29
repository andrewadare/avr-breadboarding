// Quadrature encoder readout using interrupt handler
// Hardware setup:
// - encoder channels A and B on ENC_PORT pins ENC_A and ENC_B
// - forward/backward motion indicator LEDs on LEFT_LED and RIGHT_LED

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> // For itoa()
#include "slcd.h"

// Encoder readout
#define ENC_PORT    PORTB       // Encoder port write  
#define ENC_PIN     PINB        // Encoder port read
#define ENC_VECT    PCINT0_vect // Pin-change interrupt vector
#define ENC_BANK    PCIE0       // Pin group for PCICR
#define ENC_PCIMSK  PCMSK0      // PCI mask register
#define ENC_A       1           // ENC_PORT pin for Ch A
#define ENC_B       0           // ENC_PORT pin for Ch B

// Cart motion indicator LEDs
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LEFT_LED    4
#define RIGHT_LED   5
#define BLINK_DELAY 50 // ms

// Motor control via TI SN754410 driver
#define MOTOR_DDR   DDRD
#define MOTOR_PORT  PORTD
#define MOTOR_EN    4
#define MOTOR_1A    5
#define MOTOR_2A    6
#define RAMP_DELAY  2 // ms

#define backlit 1
volatile int16_t xcart = 0;  // Horizontal cart position in encoder ticks

// Lookup table to increment position (+1), decrement it (-1), or leave it
// unchanged (0) given a transition in the encoder Q state (qprev, qcurr).
// Note: this assumes ENC_A,B = 1,0.
const volatile int8_t lut[4][4] =
{
  // Current Q:
  //0   1   2   3   // Prev Q:
  { 0, +1, -1,  0}, // 0
  {-1,  0,  0, +1}, // 1
  {+1,  0,  0, -1}, // 2
  { 0, -1, +1,  0}  // 3
};

// Set a pin in LED_PORT high for BLINK_DELAY ms
void flash_pin(uint8_t pin)
{
  LED_PORT |= (1 << pin);
  _delay_ms(BLINK_DELAY);
  LED_PORT &= ~(1 << pin);
}

// Pin-change ISR for encoder channels A and B
ISR(ENC_VECT)
{
  // Previous and current two-channel encoder states
  static uint8_t qprev = 0, qcurr = 0;

  // Short-term accumulation of encoder displacements
  static int8_t encval = 0;

  // Store previous Q; read current Q; update encval from the transition.
  qprev = qcurr;
  qcurr = ((ENC_PIN & (1 << ENC_A)) | (ENC_PIN & (1 << ENC_B)));
  encval += lut[qprev][qcurr];

  // Flash indicator LEDs to show motion.
  if (encval > 3)
  {
    xcart++;
    encval = 0;
    flash_pin(LEFT_LED);
  }
  else if (encval < -3)
  {
    xcart--;
    encval = 0;
    flash_pin(RIGHT_LED);
  }
}

int main()
{
  init_lcd();

  // Configure encoder pins and interrupt system
  ENC_PORT   |= ((1 << ENC_A) | (1 << ENC_B)); // Enable internal pullups
  PCICR      |= (1 << ENC_BANK);               // Select interrupt pin group
  ENC_PCIMSK |= ((1 << ENC_A) | (1 << ENC_B)); // Select interrupt pins
  sei();

  // Indicator LEDs to output mode
  LED_DDR |= ((1 << LEFT_LED) | (1 << RIGHT_LED));

  // Configure timer 0 (8 bits) for PWM to control motor speed.
  TCCR0A |= ((1 << WGM00) | (1 << WGM01)); // Fast PWM. Table 15-8 mode 3
  TCCR0A |= (1 << COM0A1);                 // Output to OC0A/PD6. Table 15-3
  TCCR0A |= (1 << COM0B1);                 // Output to OC0B/PD5. Table 15-6
  TCCR0B |= ((1 << CS02) | (1 << CS00));   // Clock to CPU/1024. Table 15-9

  // The PWM duty cycle can now be controlled by varying OCR0A and OCR0B, which
  // get continuously compared with TCNT0
  OCR0A = OCR0B = 0;

  // Enable motor!
  MOTOR_DDR  |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));
  MOTOR_PORT |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));

  uint8_t target_speed = 0;
  int16_t oldpos = 0;
  char val[6]; // ASCII representation of signed 16 bit integer
  while (1)
  {
    if (xcart != oldpos)
    {
      oldpos = xcart;

      target_speed = (xcart < -10 || xcart > 10) ? 255 : 25*abs(xcart);

      itoa(xcart, val, 10); // radix = 10
      lcd_clrscr();
      lcd_goto(0,0);
      lcd_puts(val, backlit);
    }

    // For now, a test...
    // Ramp motor to a speed proportional to xcart in -10..+10
    if (xcart > 0)
    {
      OCR0A = 0;

      // Ramp up to target
      while (OCR0B < target_speed)
      {
        OCR0B++;
        _delay_ms(RAMP_DELAY);
      }
      // Ramp down to target
      while (OCR0B > target_speed)
      {
        OCR0B--;
        _delay_ms(RAMP_DELAY);
      }
    }
    else if (xcart < 0)
    {
      OCR0B = 0;

      // Ramp up to target
      while (OCR0A < target_speed)
      {
        OCR0A++;
        _delay_ms(RAMP_DELAY);
      }
      // Ramp down to target
      while (OCR0A > target_speed)
      {
        OCR0A--;
        _delay_ms(RAMP_DELAY);
      }
    }
    else
    {
      OCR0A = OCR0B = 0;
    }
  }

  return 0;
}

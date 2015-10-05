
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h> // For itoa()
#include "slcd.h"

// Cart encoder readout - x coordinate
#define X_ENC_PORT    PORTB       // Encoder port write  
#define X_ENC_PIN     PINB        // Encoder port read
#define X_ENC_VECT    PCINT0_vect // Pin-change interrupt vector
#define X_ENC_BANK    PCIE0       // Pin group for PCICR
#define X_ENC_PCIMSK  PCMSK0      // PCI mask register
#define X_ENC_A       1           // X_ENC_PORT pin for Ch A
#define X_ENC_B       0           // X_ENC_PORT pin for Ch B

// Encoder for pendulum angle - theta coordinate
#define T_ENC_PORT    PORTC       // Encoder port write  
#define T_ENC_PIN     PINC        // Encoder port read
#define T_ENC_VECT    PCINT1_vect // Pin-change interrupt vector
#define T_ENC_BANK    PCIE1       // Pin group for PCICR
#define T_ENC_PCIMSK  PCMSK1      // PCI mask register
#define T_ENC_A       1           // T_ENC_PORT pin for Ch A
#define T_ENC_B       0           // T_ENC_PORT pin for Ch B
#define T_MAX         1199        // Encoder period (pulses/rev) - 1

// Motor control via TI SN754410 driver
#define MOTOR_DDR   DDRD
#define MOTOR_PORT  PORTD
#define MOTOR_EN    4
#define MOTOR_1A    5
#define MOTOR_2A    6
#define RAMP_DELAY  2 // ms

#define backlit 1

volatile uint16_t xcart = 0;  // Horizontal cart position in encoder ticks
volatile uint16_t theta = 3*(T_MAX + 1)/4;  // Pendulum angle in encoder ticks
char xstr[6]; // ASCII representation of x for LCD display
char tstr[6]; // ASCII representation of theta

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

// Pin-change ISR for x (cart) encoder
ISR(X_ENC_VECT)
{
  // Previous and current two-channel encoder states
  static uint8_t qprev = 0, qcurr = 0;

  // Short-term accumulation of encoder displacements
  static int8_t encval = 0;

  // Store previous Q; read current Q; update encval from the transition.
  qprev = qcurr;
  qcurr = ((X_ENC_PIN & (1 << X_ENC_A)) | (X_ENC_PIN & (1 << X_ENC_B)));
  encval += lut[qprev][qcurr];

  // Flash indicator LEDs to show motion.
  if (encval > 3)
  {
    xcart++;
    encval = 0;
  }
  else if (encval < -3)
  {
    xcart--;
    encval = 0;
  }
}

// Pin-change ISR for theta (pendulum) encoder
ISR(T_ENC_VECT)
{
  // Previous and current two-channel encoder states
  static uint8_t qprev = 0, qcurr = 0;

  // Short-term accumulation of encoder displacements
  static int8_t encval = 0;

  // Store previous Q; read current Q; update encval from the transition.
  qprev = qcurr;
  qcurr = ((T_ENC_PIN & (1 << T_ENC_A)) | (T_ENC_PIN & (1 << T_ENC_B)));
  encval += lut[qprev][qcurr];

  // Flash indicator LEDs to show motion.
  if (encval > 3)
  {
    theta = (theta < T_MAX) ? theta + 1 : 0;
    encval = 0;
  }
  else if (encval < -3)
  {
    theta = (theta > 0) ? theta - 1 : T_MAX;
    encval = 0;
  }
}

int main()
{
  init_lcd();

  // Configure encoder pins and interrupt system
  sei();
  X_ENC_PORT   |= ((1 << X_ENC_A) | (1 << X_ENC_B)); // Enable internal pullups
  T_ENC_PORT   |= ((1 << T_ENC_A) | (1 << T_ENC_B));
  PCICR        |= (1 << X_ENC_BANK);               // Select interrupt pin group
  PCICR        |= (1 << T_ENC_BANK);
  X_ENC_PCIMSK |= ((1 << X_ENC_A) | (1 << X_ENC_B)); // Select interrupt pins
  T_ENC_PCIMSK |= ((1 << T_ENC_A) | (1 << T_ENC_B));

  // Configure timer 0 (8 bits) for PWM to control motor speed.
  TCCR0A |= ((1 << WGM00) | (1 << WGM01)); // Fast PWM. Table 15-8 mode 3
  TCCR0A |= (1 << COM0A1);                 // Output to OC0A/PD6. Table 15-3
  TCCR0A |= (1 << COM0B1);                 // Output to OC0B/PD5. Table 15-6
  TCCR0B |= ((1 << CS02) | (1 << CS00));   // Clock to CPU/1024. Table 15-9

  // The PWM duty cycle can now be controlled by varying OCR0A and OCR0B, which
  // get continuously compared with TCNT0
  OCR0A = OCR0B = 0;

  // Run timer 1 at CPU/64. Then TCNT1 resets at 16MHz/64/65535 = 3.8 Hz.
  TCCR1B |= ((1 << CS11) | (1 << CS10));   // Table 16-5.
  // TCCR1B |= (1 << CS12);   // CPU/256

  // Enable motor!
  MOTOR_DDR  |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));
  MOTOR_PORT |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));

  // uint8_t target_speed = 0;
  // int16_t oldx = 0;

  while (1)
  {
    // if (xcart != oldx)
    // {
    //   oldx = xcart;
    //   target_speed = (xcart < -10 || xcart > 10) ? 255 : 25*abs(xcart);
    // }


    if (TCNT1 == 0)
    {
      lcd_clrscr();
      lcd_goto(0,0);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        itoa(xcart, xstr, 10); // radix = 10
      }
      lcd_puts(xstr, backlit);
      lcd_goto(1,0);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        itoa(theta, tstr, 10);
      }
      lcd_puts(tstr, backlit);
    }


    /*
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
        */
  }

  return 0;
}

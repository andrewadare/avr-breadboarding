#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include "A4988driver.h"
#include "slcd.h"

#define MIN(a,b) ((a) < (b) ? a : b)
#define MAX(a,b) ((a) > (b) ? a : b)

// Definitions for x limit stops
#define LIM_DDR     DDRB        // x limit data direction register  
#define LIM_PORT    PORTB       // x limit port write  
#define LIM_PIN     PINB        // x limit port read
#define LIM_VECT    PCINT0_vect // Pin-change interrupt vector
#define LIM_BANK    PCIE0       // Pin group for PCICR
#define LIM_PCIMSK  PCMSK0      // PCI mask register
#define LIM_L       0           // LIM_PORT pin for left stop (Arduino 8)
#define LIM_R       1           // LIM_PORT pin for right stop (9)
#define LED_L       2           // LED indicator for left stop (10)
#define LED_R       3           // LED indicator for right stop (11)
#define X_MIN       0
#define X_MAX       1400

// Encoder for pendulum angle - theta coordinate
#define T_ENC_PORT   PORTC        // Encoder port write  
#define T_ENC_PIN    PINC         // Encoder port read
#define T_ENC_VECT   PCINT1_vect  // Pin-change interrupt vector
#define T_ENC_BANK   PCIE1        // Pin group for PCICR
#define T_ENC_PCIMSK PCMSK1       // PCI mask register
#define T_ENC_A      0            // T_ENC_PORT pin for Ch A (Arduino A0)
#define T_ENC_B      1            // T_ENC_PORT pin for Ch B (Arduino A1)
#define T_MAX        1199         // Encoder period (pulses/rev) - 1
#define T_REF        300          // Vertical position (PID target value)
#define T_INIT       900          // Starting position (pendulum at rest)

#define CURR_ROT 0        // Bit 0 for current rotation direction
#define PREV_ROT 1        // Bit 1 for previous rotation direction
#define CW 0              // Next 4 definitions encode prev + current
#define CCW 1             // rotation directions.
#define CW_TO_CCW 2
#define CCW_TO_CW 3

// Stepper ramping parameters (pulse widths and ramp length)
#define TMAX 240
#define TMIN 80
#define NRAMP 16

volatile uint16_t theta = T_INIT;  // Pendulum angle in encoder ticks
volatile  int16_t error = T_REF - T_INIT;  // Theta error for PID (P)
volatile  int16_t errsum = 0;      // Accumulated error for PID (I)
volatile  int16_t omega = 0;       // Angular velocity of pendulum (D)
volatile uint8_t rotreg = 0;  // Register for storing pendulum rotation state

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

void setup()
{
  // Stepper pins and parameters are defined in stepper_config.h

  // Limit stop configuration
  LIM_DDR     = ((1 << LED_L) | (1 << LED_R)); // LED pins to output mode
  LIM_PORT    = ((1 << LIM_L) | (1 << LIM_R)); // Enable pullups on input pins
  PCICR      |= (1 << LIM_BANK);               // Select interrupt pin group
  LIM_PCIMSK |= ((1 << LIM_L) | (1 << LIM_R)); // Select interrupt pins

  // Pendulum angle encoder configuration
  T_ENC_PORT   |= ((1 << T_ENC_A) | (1 << T_ENC_B));
  PCICR        |= (1 << T_ENC_BANK);
  T_ENC_PCIMSK |= ((1 << T_ENC_A) | (1 << T_ENC_B));

  // Timer 1 configuration - use as timebase for angular velocity
  TCCR1B |= (1 << WGM12);  // CTC mode (table 16-4)
  TCCR1B |= (1 << CS12);   // Prescale to CPU/256 (table 16-5) --> 62.5 kHz
  TIMSK1 |= (1 << OCIE1A); // Enable output compare interrupt
  OCR1A = 6250;            // Interrupt every 100 ms

  stepper_setup();
  init_lcd();
  sei();
}

uint8_t rotation_state()
{
  uint8_t rs = rotreg & ((1 << PREV_ROT) | (1 << CURR_ROT));

  if (rs == ((CW << PREV_ROT) | (CW << CURR_ROT)))
    return CW;

  if (rs == ((CCW << PREV_ROT) | (CCW << CURR_ROT)))
    return CCW;

  if (rs == ((CW << PREV_ROT) | (CCW << CURR_ROT)))
    return CW_TO_CCW;

  if (rs == ((CCW << PREV_ROT) | (CW << CURR_ROT)))
    return CCW_TO_CW;

  return 99;
}


void go(int16_t n)
{
  int16_t newx = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    newx = n + stepper_position();
  }

  if (newx < 0 || newx > X_MAX)
    return;

  uint16_t nsteps = n;
  if (n >= 0)
    setdir_cw(); // right
  else
  {
    setdir_ccw(); // left
    nsteps = -n;
  }
  ramp_move(nsteps, TMAX, TMIN, NRAMP);
}

void home_cart()
{
  lcd_goto(1, 0);
  lcd_puts("Homing...");

  while bit_is_clear(LIM_PORT, LED_L)
    go(-1);
  while bit_is_set(LIM_PORT, LED_L)
    go(ONE_REV/8);
  LIM_PORT &= ~(1 << LED_L);

  set_stepper_position(0);
}

void swing()
{
  if (abs(error) > 50)
  {
    if (rotation_state() == CW_TO_CCW)
    {
      if (abs(error) < 100)
        go(ONE_REV);
      else if (abs(error) < 200)
        go(3*ONE_REV/4);
      else
        go(ONE_REV/2);
    }
    if (rotation_state() == CCW_TO_CW)
    {
      if (abs(error) < 100)
        go(-ONE_REV);
      else if (abs(error) < 200)
        go(-3*ONE_REV/4);
      else
        go(-ONE_REV/2);
    }
  }
}

void check_limits()
{
  while bit_is_set(LIM_PORT, LED_L)
    go(ONE_REV/8);
  LIM_PORT &= ~(1 << LED_L);

  while bit_is_set(LIM_PORT, LED_R)
    go(-ONE_REV/8);
  LIM_PORT &= ~(1 << LED_R);
}

void write_x()
{
  char s[6] = "";
  lcd_goto(1, 0);
  lcd_puts("xcart: ");
  itoa(stepper_position(), s, 10);
  for (uint8_t i = 6 - strlen(s); i > 0; i--)
    lcd_puts(" ");
  lcd_puts(s);
}

void write_theta()
{
  char s[6] = "";
  lcd_goto(2, 0);
  lcd_puts("theta: ");
  itoa(theta, s, 10);
  for (uint8_t i = 6 - strlen(s); i > 0; i--)
    lcd_puts(" ");
  lcd_puts(s);
}

void write_omega()
{
  char s[6] = "";
  lcd_goto(3, 0);
  lcd_puts("omega: ");
  itoa(omega, s, 10);
  for (uint8_t i = 6 - strlen(s); i > 0; i--)
    lcd_puts(" ");
  lcd_puts(s);
}

int main()
{
  setup();
  lcd_clrscr();
  lcd_goto(0, 0);
  lcd_puts(" INVERTED PENDULUM ");
  // home_cart();

  while (1)
  {
    check_limits();
    write_x();
    write_theta();
    write_omega();
  }

  return 0;
}

// Interrupt handlers are listed from highest to lowest priority, according to
// current definitions.

// Pin-change ISR for limit stops.
ISR(LIM_VECT)
{
  // Left limt
  if bit_is_clear(LIM_PIN, LIM_L)
  {
    _delay_us(500);
    if bit_is_clear(LIM_PIN, LIM_L)
    {
      LIM_PORT |= (1 << LED_L);
    }
  }
  else
    LIM_PORT &= ~(1 << LED_L);

  // Right limit
  if bit_is_clear(LIM_PIN, LIM_R)
  {
    _delay_us(500);
    if bit_is_clear(LIM_PIN, LIM_R)
    {
      LIM_PORT |= (1 << LED_R);
    }
  }
  else
    LIM_PORT &= ~(1 << LED_R);
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

  if (encval > 3) // Theta increasing (CCW rotation)
  {
    // Store prev direction and set CURR_ROT bit high
    rotreg <<= 1;
    rotreg |= (1 << CURR_ROT);

    theta = (theta < T_MAX) ? theta + 1 : 0;
    encval = 0;
    error = T_REF - theta;
  }
  else if (encval < -3) // Theta decreasing (CW rotation)
  {
    // Store prev direction and keep CURR_ROT bit low
    rotreg <<= 1;

    theta = (theta > 0) ? theta - 1 : T_MAX;
    encval = 0;
    error = T_REF - theta;
  }
}

// ISR(TIMER2_COMPA_vect) in the A4988driver library is used for stepper pulsing

// Update angular velocity at a rate controlled by Timer 1 and OCR1A
ISR(TIMER1_COMPA_vect)
{
  static uint16_t prev_theta = T_INIT;
  omega = theta - prev_theta;
  prev_theta = theta;

  if (abs(error) < 50)
  {
    errsum += error;
    errsum = MIN(error, 100);
  }
  else
    errsum = 0;
}

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "stepdriver.h"
// #include "slcd.h"

#define MIN(a,b) ((a) < (b) ? a : b)
#define MAX(a,b) ((a) > (b) ? a : b)

#define ONE_REV 400               // Stepper detents/rev (0.9 deg/step)

// Definitions for x limit stops
#define LIM_DDR     DDRB          // x limit data direction register  
#define LIM_PORT    PORTB         // x limit port write  
#define LIM_PIN     PINB          // x limit port read
#define LIM_VECT    PCINT0_vect   // Pin-change interrupt vector
#define LIM_BANK    PCIE0         // Pin group for PCICR
#define LIM_PCIMSK  PCMSK0        // PCI mask register
#define LIM_L       0             // LIM_PORT pin for left stop (Arduino 8)
#define LIM_R       1             // LIM_PORT pin for right stop (9)
#define LED_L       2             // LED indicator for left stop (10)
#define LED_R       3             // LED indicator for right stop (11)

// Stepper ramping parameters (pulse widths and ramp length)
#define TMAX 240
#define TMIN 60
#define NRAMP 18
// #define TMAX 240
// #define TMIN 80
// #define NRAMP 16

// Encoder for pendulum angle - theta coordinate.
// The increments of theta are encoder ticks, where theta = 0 is the vertical
// (target) position, increasing CCW. The pendulum begins hanging at rest, at
// position T_INIT. The angular interval is thus defined as -T_INIT+1:T_INIT.
// T_INIT is half the number of encoder counts/revolution, and an integer-valued
// proxy for pi.
#define T_INIT       900          // Starting angle (pendulum hanging at rest)
#define T_ENC_PORT   PORTC        // Encoder port write  
#define T_ENC_PIN    PINC         // Encoder port read
#define T_ENC_VECT   PCINT1_vect  // Pin-change interrupt vector
#define T_ENC_BANK   PCIE1        // Pin group for PCICR
#define T_ENC_PCIMSK PCMSK1       // PCI mask register
#define T_ENC_A      0            // T_ENC_PORT pin for Ch A (Arduino A0)
#define T_ENC_B      1            // T_ENC_PORT pin for Ch B (Arduino A1)

#define CURR_ROT 0                // Bit 0 for current rotation direction
#define PREV_ROT 1                // Bit 1 for previous rotation direction
#define CW 0                      // Decreasing theta (omega < 0)
#define CCW 1                     // Increasing theta (omega > 0)
#define CW_TO_CCW 2               // Max left swing (omega = 0)
#define CCW_TO_CW 3               // Max right swing (omega = 0)

volatile  int16_t theta = T_INIT; // Pendulum angle in encoder ticks
volatile  int16_t errsum = 0;     // Accumulated error for PID
volatile  int16_t omega = 0;      // Angular velocity (actually d(theta))
volatile uint8_t rotreg = 0;      // Register to store pendulum rotation state
volatile  int16_t x_max = 12345;  // Upper x limit - set during orientation
volatile uint8_t timer1_tick = 0;

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
  // Stepper pins and parameters are defined in stepdriver.h. At the moment:
  // STEP_PIN is PD3 (Uno 3) and DIR_PIN is PD4 (Uno pins 3,4)

  // Limit stop configuration
  LIM_DDR     = ((1 << LED_L) | (1 << LED_R)); // LED pins to output mode
  LIM_PORT    = ((1 << LIM_L) | (1 << LIM_R)); // Enable pullups on input pins
  PCICR      |= (1 << LIM_BANK);               // Select interrupt pin group
  LIM_PCIMSK |= ((1 << LIM_L) | (1 << LIM_R)); // Select interrupt pins

  // Pendulum angle encoder configuration
  T_ENC_PORT   |= ((1 << T_ENC_A) | (1 << T_ENC_B));
  PCICR        |= (1 << T_ENC_BANK);
  T_ENC_PCIMSK |= ((1 << T_ENC_A) | (1 << T_ENC_B));

  // Timer 1 configuration - controls sample rate for angular velocity
  TCCR1B |= (1 << WGM12);  // CTC mode (table 16-4)
  TCCR1B |= ((1 << CS11) | (1 << CS10));   // CPU/64 (table 16-5) --> 250 kHz
  TIMSK1 |= (1 << OCIE1A); // Enable output compare interrupt
  // OCR1A = 2500; // every 10 ms (100 Hz)
  // OCR1A = 6250; // Sample every 25 ms (40 Hz)
  OCR1A = 8000; // Sample every 40 ms (25 Hz)
  // OCR1A = 12500; // every 50 ms (20 Hz)
  // OCR1A = 25000; // every 100 ms (10 Hz)

  stepper_setup();
  // init_lcd();
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

// Travel n steps left (n<0) or right (n>0).
void go_unchecked(int16_t n)
{
  uint16_t nsteps = n;
  if (n >= 0)
    setdir_cw(); // right
  else
  {
    setdir_ccw(); // left
    nsteps = -n;
  }
  // enable_stepper();
  // for (uint8_t i=0; i<255; i++) {;}
  ramp_move(nsteps, TMAX, TMIN, NRAMP);
  // disable_stepper();
}

// Travel n steps if destination is in bounds.
void go(int16_t n)
{
  int16_t newx = n + stepper_position();

  if (newx < 0 || newx > x_max)
    return;

  go_unchecked(n);
}

void go_to(int16_t x)
{
  go(x - stepper_position());
}

// Find x boundaries using limit stops and center cart
void initialize_cart()
{
  // lcd_goto(1, 0);
  // lcd_puts("Finding left edge");
  while bit_is_clear(LIM_PORT, LED_L) // Move cautiously to left limit stop
    go_unchecked(-1);
  while bit_is_set(LIM_PORT, LED_L)   // Rebound to create a margin
    go_unchecked(ONE_REV/8);
  LIM_PORT &= ~(1 << LED_L);
  set_stepper_position(0);            // Zero out stepper counter

  // lcd_goto(1, 0);
  // lcd_puts("Finding right edge");
  while bit_is_clear(LIM_PORT, LED_R)
    go_unchecked(1);
  while bit_is_set(LIM_PORT, LED_R)
    go_unchecked(-ONE_REV/8);
  LIM_PORT &= ~(1 << LED_R);
  x_max = stepper_position();

  // lcd_goto(1, 0);
  // lcd_puts("                  ");
  // char s[6] = "";
  // itoa(x_max, s, 10);
  // lcd_goto(1, 0);
  // lcd_puts("xmin = 0");
  // lcd_goto(2, 0);
  // lcd_puts("xmax = ");
  // lcd_puts(s);

  // Set up for swing routine
  _delay_ms(500);
  go_to(x_max/4);
}

void swing()
{
  if (abs(theta) < 50)
    return;

  if (rotation_state() == CW_TO_CCW)
  {
    if (theta < T_INIT/6)
      go(x_max/2);
    else if (theta < T_INIT/3)
      go(x_max/3);
    else
      go(x_max/4);
  }
  if (rotation_state() == CCW_TO_CW)
  {
    if (theta > -T_INIT/6)
      go(-x_max/2);
    else if (theta > -T_INIT/3)
      go(-x_max/3);
    else
      go(-x_max/4);
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
/*
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
*/

int main()
{
  setup();
  // lcd_clrscr();
  // lcd_goto(0, 0);
  // lcd_puts(" INVERTED PENDULUM ");

  initialize_cart();

  while (1)
  {
    while (abs(theta) > 0 && abs(theta) < 135)
    {
      // 25Hz
      go(-1.8*theta - 1.75*omega - 1.5*errsum);

      // With 20 Hz sample rate
      // go(-5.6*theta - 4.8*omega - 7*errsum);
      // go(-3.6*theta - 2.4*omega - 3.9*errsum);
      // go(-2.4*theta - 2.0*omega - 3.0*errsum);
      // go(-1.5*theta - 1.5*omega - 1.9*errsum);
      // go(-1.25*theta - 1.5*omega - 1.1*errsum);
      // go(-1.75*theta - 1.5*omega - 1.25*errsum);
      // go(-4*theta - 3*omega - 3.5*errsum); // With timer 1 sampling at 10 Hz
      check_limits();
    }

    // if (timer1_tick)
    // {
    //   write_x();
    //   write_theta();
    //   write_omega();
    //   timer1_tick = 0;
    // }

    swing();
    check_limits();
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

  if (encval < -3) // Theta increasing (CCW rotation)
  {
    // Store prev direction and set CURR_ROT bit high
    rotreg <<= 1;
    rotreg |= (1 << CURR_ROT);
    theta = (theta == T_INIT) ? -T_INIT + 1 : theta + 1;
    encval = 0;
  }
  else if (encval > 3) // Theta decreasing (CW rotation)
  {
    // Store prev direction and keep CURR_ROT bit low
    rotreg <<= 1;
    theta = (theta == -T_INIT + 1) ? T_INIT : theta - 1;
    encval = 0;
  }
}

// Update angular velocity at a rate controlled by Timer 1 and OCR1A
ISR(TIMER1_COMPA_vect)
{
  static uint16_t prev_theta = T_INIT;

  // Compute angular difference for this time step, respecting periodicity
  omega = theta - prev_theta;

  if (omega > T_INIT)
    omega -= 2*T_INIT;
  else if (omega < -T_INIT)
    omega += 2*T_INIT;

  prev_theta = theta;

  if (abs(theta) < 50)
    errsum += theta;
  else
    errsum = 0;

  timer1_tick = 1;
}

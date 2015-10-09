
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>

// #define ROUNDED_DIV(a, b) ((((a) * 10) / (b)) + 5) / 10)

// Cart encoder readout - x coordinate
#define X_ENC_PORT    PORTB       // Encoder port write  
#define X_ENC_PIN     PINB        // Encoder port read
#define X_ENC_VECT    PCINT0_vect // Pin-change interrupt vector
#define X_ENC_BANK    PCIE0       // Pin group for PCICR
#define X_ENC_PCIMSK  PCMSK0      // PCI mask register
#define X_ENC_A       1           // X_ENC_PORT pin for Ch A
#define X_ENC_B       0           // X_ENC_PORT pin for Ch B
#define X_MAX         5000        // Upper limit on x position

// Encoder for pendulum angle - theta coordinate
#define T_ENC_PORT   PORTC        // Encoder port write  
#define T_ENC_PIN    PINC         // Encoder port read
#define T_ENC_VECT   PCINT1_vect  // Pin-change interrupt vector
#define T_ENC_BANK   PCIE1        // Pin group for PCICR
#define T_ENC_PCIMSK PCMSK1       // PCI mask register
#define T_ENC_A      1            // T_ENC_PORT pin for Ch A
#define T_ENC_B      0            // T_ENC_PORT pin for Ch B
#define T_MAX        1199         // Encoder period (pulses/rev) - 1
#define T_REF        T_MAX/4      // Vertical position (PID target value)
#define T_INIT       3*T_MAX/4    // Starting position (pendulum at rest)

// Motor control via TI SN754410 driver
#define MOTOR_DDR   DDRD
#define MOTOR_PORT  PORTD
#define MOTOR_EN    4
#define MOTOR_1A    5
#define MOTOR_2A    6
#define RAMP_DELAY  2 // ms

// Register for storing pendulum rotation state.
volatile uint8_t rotreg = 0;
#define CURR_ROT 0        // Bit 0 for current rotation direction
#define PREV_ROT 1        // Bit 1 for previous rotation direction
#define CW 0              // Next 4 definitions encode prev + current
#define CCW 1             // rotation directions.
#define CW_TO_CCW 2
#define CCW_TO_CW 3

volatile uint16_t xcart = 2000;    // Horizontal cart position in encoder ticks
volatile uint16_t theta = T_INIT;  // Pendulum angle in encoder ticks
volatile int16_t  omega = 0;       // Angular velocity
volatile int16_t  error = T_REF - T_INIT; // Theta error for PID
volatile int16_t  err10 = 0;       // (10*) the weighted moving average error

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

void move(int16_t dx)
{
  int16_t newx = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    newx = xcart + dx;
  }

  if (newx < 0 || newx > X_MAX)
    return;

  OCR0A = OCR0B = 0;

  if (dx > 0) // Move right
  {
    while (xcart < newx)
    {
      OCR0A = 255;
    }
    OCR0A = 0;
  }
  if (dx < 0) // Move left
  {
    while (xcart > newx)
    {
      OCR0B = 255;
    }
    OCR0B = 0;
  }
}

// Update angular velocity at a rate controlled by Timer 1 and OCR1A
ISR(TIMER1_COMPA_vect)
{
  static uint16_t prev_theta = T_INIT;
  omega = theta - prev_theta;
  prev_theta = theta;
  // PORTB ^= (1 << 4);

  // Exponentially weighted moving average. Update err10 to include this error.
  // The weighting is 1/10*(current error) + 9/10*(running average):
  //    y[t] = x[t]/10 + 9*y[t-1]/10
  // Integer division error is avoided by working with 10y as follows:
  // 10*y[t] = x[t] + 9*y[t-1]
  //         = x[t] + 10*y[t-1] - (10*y[t-1] - 10/2)/10
  // Remember to use err10/10 as the actual EWMA!
  err10 = error + err10 - (err10 - 5)/10;
}

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

int main()
{
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

  // Timer 1 configuration - use as timebase for angular velocity
  TCCR1B |= (1 << WGM12);  // CTC mode (table 16-4)
  TCCR1B |= (1 << CS12);   // Prescale to CPU/256 (table 16-5) --> 62.5 kHz
  TIMSK1 |= (1 << OCIE1A); // Enable output compare interrupt
  // OCR1A = 12500;            // Trigger interrupt every 200 ms
  OCR1A = 6250;            // Trigger interrupt every 100 ms

  // Enable motor!
  MOTOR_DDR  |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));
  MOTOR_PORT |= ((1 << MOTOR_EN) | (1 << MOTOR_1A) | (1 << MOTOR_2A));

  // Raise PB2-5 to output mode for indicator LEDs
  DDRB |= 0b00111100;

  while (1)
  {
    // if (rotation_state() == CW_TO_CCW && abs(error) > 50)
    // {
    //   move(+1300);
    // }
    // if (rotation_state() == CCW_TO_CW && abs(error) > 50)
    // {
    //   move(-1300);
    // }

    if (rotation_state() == CW_TO_CCW && abs(error) > 50)
    {
      if (theta > T_REF && theta < (T_MAX + 1)/2)
        move(+1500);
      else
        move(+1300);
    }
    if (rotation_state() == CCW_TO_CW && abs(error) > 50)
    {
      if (theta < T_REF)
        move(-1500);
      else
        move(-1300);
    }

    if (abs(error) < 50)
      PORTB |= (1 << 2);
    else
      PORTB &= ~(1 << 2);

    if (abs(error) < 10)
      PORTB |= (1 << 3);
    else
      PORTB &= ~(1 << 3);


    // PID loop
    while (abs(error) < 40)
    {
      PORTB |= (1 << 4);
      move(19*error + -27*omega + err10/6);
    }
    PORTB &= ~(1 << 4);

    if (xcart > 10000)
      PORTB |= (1 << 5);
  }

  return 0;
}

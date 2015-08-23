// Quadrature encoder readout using interrupt handler
// Hardware setup:
// - encoder channels A and B on ENC_PORT pins ENC_A and ENC_B
// - forward/backward motion indicator LEDs on LEFT_LED and RIGHT_LED

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Definitions related to encoder readout
#define ENC_PORT    PORTB       // Encoder port write  
#define ENC_PIN     PINB        // Encoder port read
#define ENC_VECT    PCINT0_vect // Pin-change interrupt vector
#define ENC_BANK    PCIE0       // Pin group for PCICR
#define ENC_PCIMSK  PCMSK0      // PCI mask register
#define ENC_A       1           // ENC_PORT pin for Ch A
#define ENC_B       0           // ENC_PORT pin for Ch B

// Definitions for indicator LEDs
#define LED_DDR     DDRD
#define LED_PORT    PORTD
#define LEFT_LED    PD2
#define RIGHT_LED   PD3
#define BLINK_DELAY 50 // ms

volatile int16_t position = 0;  // Position in encoder ticks (currently unused)

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
    position += encval;
    encval = 0;
    flash_pin(LEFT_LED);
  }
  else if (encval < -3)
  {
    position += encval;
    encval = 0;
    flash_pin(RIGHT_LED);
  }
}

int main()
{
  // Configure encoder pins and interrupt system
  ENC_PORT   |= ((1 << ENC_A) | (1 << ENC_B)); // Enable internal pullups
  PCICR      |= (1 << ENC_BANK);               // Select interrupt pin group
  ENC_PCIMSK |= ((1 << ENC_A) | (1 << ENC_B)); // Select interrupt pins
  sei();

  // Indicator LEDs to output mode
  LED_DDR |= ((1 << LEFT_LED) | (1 << RIGHT_LED));

  while (1)
  {
  }

  return 0;
}

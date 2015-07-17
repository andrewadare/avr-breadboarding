// Transmit some beeps over an AM carrier.
// Clock the CPU to its native speed of 8 MHz (ATMegas are underclocked to 1 MHz
// from the factory). This can be done in software (without touching fuses) by:
// 1. Setting F_CPU either here or in the Makefile
// 2. Calling clock_prescale_set(clock_div_1); from power.h
//
// Use timer 0 for the carrier, also clocked to 8 MHz. Timer
// 1 is clocked to 1 MHz and used for generating rudimentary audio square-wave
// tones.
// Hardware setup: simply put a long antenna wire on PD5, and tune a radio
// receiver to the carrier frequency set in init_timer0.

#include <avr/io.h>

// Redefine the CPU clock speed over the setting in the Makefile.
#undef  F_CPU // Prevent compiler warning about redefinition
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// A few musical pitches from Elliot Williams' scale16.h file
#define  G2     1669
#define  C3     1250
#define  Cx3    1180
#define  D3     1114
#define  Dx3    1051
#define  E3      992
#define  F3      936
#define  Fx3     884
#define  G3      834
#define  Gx3     787
#define  A3      743
#define  Ax3     702
#define  B3      662

// OCR0A sets the carrier frequency: f = f_cpu / (2N(1 + OCRnx)) where N
// is the prescale factor. Below, N = 1 for timer 0 (CS00).
// Division by two since a full carrier cycle takes two loops.
// Pick a value from 2-6 that's clear on an AM receiver.
// 8Mhz / (2*(1 + 2)) = 1333 kHz
// 8Mhz / (2*(1 + 3)) = 1000 kHz
// 8Mhz / (2*(1 + 4)) = 800 kHz
// 8Mhz / (2*(1 + 5)) = 670 kHz
// 8Mhz / (2*(1 + 6)) = 570 kHz
// 8Mhz / (2*(1 + 7)) = 500 kHz
static inline void init_timer0()
{
  TCCR0A |= (1 << WGM01);  // Put timer 0 in CTC mode
  TCCR0A |= (1 << COM0B0); // Toggle antenna pin on each loop pass
  TCCR0B |= (1 << CS00);   // Prescale timer 0 clock to CPU/1
  OCR0A = 3;               // See comments above.
}

static inline void init_timer1()
{
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS11);   // Prescale to CPU/8
  TIMSK1 |= (1 << OCIE1A); // Enable output compare interrupt
}

// Toggle antenna at timer1 frequency to make square waves
ISR(TIMER1_COMPA_vect)
{
  DDRD ^= (1 << PD5);
}

static inline void beep(uint8_t pitch, uint16_t duration)
{
  OCR1A = pitch; // Set pitch for timer 1

  sei();
  while (duration > 0)
  {
    _delay_ms(1);
    duration--;
  }
  cli();

  DDRD |= (1 << PD5);
}

int main()
{
  clock_prescale_set(clock_div_1);
  init_timer0();
  init_timer1();

  while (1)
  {
    uint16_t hold = 200; // ms

    beep(E3, hold);
    _delay_ms(50);
    beep(E3, hold);
    _delay_ms(100);
    beep(E3, hold);
    _delay_ms(100);
    beep(C3, hold);
    beep(E3, hold);
    _delay_ms(200);
    beep(G3, 2*hold);
    _delay_ms(400);
    beep(G2, 2*hold);

    _delay_ms(2500);    
  }

  return 0;
}

// Free-running (i.e. auto-triggered) ADC example.
// Same circuit setup as for lightmeter.c, with TX serial line connected to RX
// on computer.
// The accompanying program read_adc displays the ADC value in real time.
#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"

#define SAMPLE_DELAY 20 // ms

static inline void init_adc_freerun()
{
  ADMUX  |= (1 << REFS0);                // Set AREF to AVCC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // Prescale ADC clock by 1/8
  ADCSRA |= (1 << ADEN);                 // AD enable
  ADCSRA |= (1 << ADATE);                // AD auto-trigger enable (free run)
  ADCSRA |= (1 << ADSC);                 // AD start conversion. Kickoff loop.
}

// Transmit the uint16_t ADC result over USART. 
// A 16 bit word requires up to 5 decimal digits, but since our range is 0-1023,
// only 4 characters are needed.
void send_10bit_word(uint16_t word)
{
  transmitByte('0' + ((word / 1000) % 10));               /* Thousands */
  transmitByte('0' + ((word / 100) % 10));                 /* Hundreds */
  transmitByte('0' + ((word / 10) % 10));                      /* Tens */
  transmitByte('0' + (word % 10));                             /* Ones */
}

int main()
{
  initUSART();
  init_adc_freerun();

  while (1)
  {
    send_10bit_word(ADC);

    // Throttle the transmission rate, and therefore the display refresh rate.
    _delay_ms(SAMPLE_DELAY);
  }
}

// Using AVR's built in ADC
// Read a voltage divider with a photoresistor on PC0 (arduino A0)
// 8 LEDs on PORTB

#include <avr/io.h>
#include <util/delay.h>

static inline void initADC0()
{
  // The ADC samples from PC0 by default, so no need to configure the multiplexer.

  // Set the voltage reference, AREF, to the the ADC power source (AVCC).
  // This assumes the light sensor voltage divider is outputting in the same
  // 0-5 V range as the chip's operating voltage.
  ADMUX |= (1 << REFS0);

  // Set the ADC clock prescale value.
  // This choice prescales the internal 1 MHz clock down by 8, to 125 kHz.
  // The recommended range is 50-200 kHz.
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);

  // Enable the ADC by setting the ADEN flag
  ADCSRA |= (1 << ADEN);

  // No triggering here, so nothing is done with ADCSRB.
}

int main()
{
  uint8_t ledval = 0;
  uint8_t i = 0;

  initADC0();

  DDRB = 0xff;

  while (1)
  {
    // Tell ADC Status Register A to begin conversion
    // (ADSC = "A-D Start Conversion")
    ADCSRA |= (1 << ADSC);

    // Wait for ADC to finish.
    // This function is defined in sfr_defs.h
    loop_until_bit_is_clear(ADCSRA, ADSC);

    // The 10-bit result is stored across two registers, ADCL and ADCH (the 
    // most significant 6 bits in ADCH are zero-padded).
    // Even simpler, the result is also in a uint16_t labeled ADC.
    // Here, scale the ADC value (0-1023) down by 2^7 to a one-byte range, so 
    // the result can be displayed on 8 LEDs.
    ledval = (ADC >> 7);

    // Light up the bar graph with a value from 0-7
    PORTB = 0;
    for (i=0; i<ledval; i++)
      PORTB |= (1 << i);

    // Keep the LEDs lit for a short time
    _delay_ms(50);
  }

  return 0;
}

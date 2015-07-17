#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include "USART.h"

#define PIEZO PC2 // ADC2
#define SWITCH PB7
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED0 PB0
#define LED1 PB1

#define MIN_NOISE      16 // ADC units. Raising reduces sensitivity
#define SWITCH_TIMEOUT 2000 // ms
#define LOOP_DELAY     10 // ms

void initADC()
{
  // ADC pin is selected in readADC()

  // Select AVCC as reference voltage. Table 24-3.
  ADMUX |= (1 << REFS0);

  // Set input clock to CPU/64. Table 24-5.
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));

  // Enable ADC
  ADCSRA |= (1 << ADEN);
}

uint16_t readADC(uint8_t channel)
{
  // Clear old input channel selections, then set to this channel
  ADMUX = (0b11110000 & ADMUX) | channel;

  // Start conversion
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return ADC;
}

// 16 bit exponentially weighted moving average. Update xavg to include this x.
void ewma(uint16_t x, uint16_t *xavg)
{
  *xavg = x + *xavg - ((*xavg - 8) >> 4);
}

int main()
{
  uint16_t adcNow = 0, adcLow = 500, adcMid = 510, adcHigh = 520, adcNoise = 0;
  uint16_t countdown = 0;

  initADC();
  initUSART();

  LED_DDR = ((1 << LED0) | (1 << LED1) | (1 << SWITCH));

  while (1)
  {
    adcNow = readADC(PIEZO);

    // Maintain the EWMA for the central, positive, and negative measurements.
    ewma(adcNow, &adcMid);
    if (adcNow > (adcMid >> 4))
      ewma(adcNow, &adcHigh);
    if (adcNow < (adcMid >> 4))
      ewma(adcNow, &adcLow);

    adcNoise = adcHigh - adcLow + MIN_NOISE;

    // If signal is large: turn on switch, blip indicators, and start timer.
    // If within noise: turn off the indicator LEDs and keep counting down.
    if (adcNow < ((adcMid - adcNoise) >> 4))
    {
      LED_PORT = (1 << LED0) | (1 << SWITCH);
      countdown = SWITCH_TIMEOUT / LOOP_DELAY;
    }
    else if (adcNow > ((adcMid + adcNoise) >> 4))
    {
      LED_PORT = (1 << LED1) | (1 << SWITCH);
      countdown = SWITCH_TIMEOUT / LOOP_DELAY;
    }
    else
    {
      LED_PORT &= ~((1 << LED0) | (1 << LED1));
      if (countdown)
        countdown--;
      else
        LED_PORT &= ~(1 << SWITCH);
    }

    // Report ADC values rescaled and centered about 127
    transmitByte(adcNow - 512 + 127);
    transmitByte((adcLow >> 4) - 512 + 127);
    transmitByte((adcHigh >> 4) - 512 + 127);
    _delay_ms(LOOP_DELAY);
  }

  return 0;
}

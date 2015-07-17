// Voltmeter using ADC peripheral
// Continuously measures voltage and writes over USART.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include "USART.h"

// This voltmeter is only as accurate as your reference voltage.
// If you want four digits of accuracy, need to measure your AVCC well.
// Measure either AVCC or AREF and enter it here.
#define REF_VCC 5.053
// Measured division by voltage divider
#define VOLTAGE_DIV_FACTOR  3.114


void initADC()
{
  // Configure multiplexer to read from ADC5/PC5 (only). See table 24-4.
  ADMUX |= (0b00001111 & PC5);

  // Select AVCC as reference voltage. Table 24-3.
  ADMUX |= (1 << REFS0);

  // Set input clock to CPU/64. Table 24-5.
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));

  // Enable ADC
  ADCSRA |= (1 << ADEN);
}

// Dummy ISR needed for SLEEP_MODE_ADC, just provides a safe landing point 
// for ADC_vect.
EMPTY_INTERRUPT(ADC_vect);

// Return a running sum of the previous 16 10-bit measurements, divided by 4.
// This effectively provides 12 bit resolution from a 10 bit device (at the
// cost of a small time lag).
uint16_t oversample()
{
  uint16_t val = 0;
  for (uint8_t i=0; i<16; i++)
  {
    // In SLEEP_MODE_ADC, the ADC does its conversion while the CPU sleeps,
    // then triggers an interrupt to wake up when done. Takes 13-25 cycles.
    sleep_mode();
    val += ADC;
  }

  return (val >> 2);
}

int main()
{
  // TODO: test this code on 16MHz system
#if (F_CPU == 16000000UL)
  clock_prescale_set(clock_div_16);
#endif

  float voltage;

  initUSART();
  initADC();

  // Configure AVR so chip goes to sleep while ADC is working, then wakes chip
  // when done. Uses the interrupt system; requires an empty interrupt handler.
  set_sleep_mode(SLEEP_MODE_ADC);

  // Enable ADC interrupt
  ADCSRA |= (1 << ADIE);
  sei();

  printString("\r\nDigital Voltmeter\r\n\r\n");

  while (1)
  {
    voltage = oversample() * VOLTAGE_DIV_FACTOR * REF_VCC / 4096;
    printFloat(voltage);
    _delay_ms(500);
  }

  return 0;
}

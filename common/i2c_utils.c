#include <compat/twi.h>
#include "i2c_utils.h"

void i2c_init(void)
{
  // Initial ATMega328P TWI/I2C Peripheral
  TWSR = 0x00;         // Select Prescaler of 1
  // SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
  TWBR = 0x30;        // 48 Decimal
}

unsigned char i2c_transmit(unsigned char type)
{
  switch (type)
  {
  case I2C_START:    // Send Start Condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    break;
  case I2C_DATA:     // Send Data with No-Acknowledge
    TWCR = (1 << TWINT) | (1 << TWEN);
    break;
  case I2C_DATA_ACK: // Send Data with Acknowledge
    TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
    break;
  case I2C_STOP:     // Send Stop Condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    return 0;
  }
  // Wait for TWINT flag set on Register TWCR
  while (!(TWCR & (1 << TWINT)));
  // Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
  return (TWSR & 0xF8);
}

char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type)
{
  unsigned char twi_status;

  for (uint8_t n = 0; n < I2C_MAX_TRIES; n++)
  {
    // Transmit Start Condition
    twi_status = i2c_transmit(I2C_START);

    // Check TWI Status
    if (twi_status == TW_MT_ARB_LOST)
      continue;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
      return -1;

    // Send slave address (SLA_W)
    TWDR = (dev_id & 0xF0) | (dev_addr & 0x0E) | rw_type;

    // Transmit I2C Data
    twi_status=i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST))
      continue;
    if (twi_status != TW_MT_SLA_ACK)
      return -1;
  }

  return 0;
}

void i2c_stop(void)
{
  i2c_transmit(I2C_STOP);
}

char i2c_write(char data)
{
  // Send the Data to I2C Bus
  TWDR = data;

  // Transmit I2C data and check TWSR status
  if (i2c_transmit(I2C_DATA) != TW_MT_DATA_ACK)
    return -1;

  return 0;
}

char i2c_read(char *data, char ack_type)
{
  unsigned char twi_status;

  if (ack_type)
  {
    // Read I2C Data and Send Acknowledge
    twi_status = i2c_transmit(I2C_DATA_ACK);
    if (twi_status != TW_MR_DATA_ACK)
      return -1;
  }
  else
  {
    // Read I2C Data and Send No Acknowledge
    twi_status = i2c_transmit(I2C_DATA);
    if (twi_status != TW_MR_DATA_NACK)
      return -1;
  }

  // Get the Data
  *data = TWDR;

  return 0;
}

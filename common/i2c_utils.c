#include "i2c_utils.h"

void i2c_init(void)
{
  // Set I2C clock prescale to 1 (no prescaling)
  TWSR = 0x00;

  // Set SCL frequency in TWI bit register. If no prescaling,
  //      I2C_SCL = F_CPU/(16 + 2*TWBR).
  // See ATMega*8 datasheet section 22.5.2.
  // If SCL = 100 kHz and F_CPU = 16 MHz, then TWBR = 72.
  TWBR = ((F_CPU/I2C_SCL) - 16)/2;
}

uint8_t i2c_transmit(uint8_t type)
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

char i2c_start(uint8_t dev_addr, uint8_t rw_type)
{
  uint8_t twi_status;

  for (uint8_t n = 0; n < I2C_MAX_TRIES; n++)
  {
    // Transmit Start Condition
    twi_status = i2c_transmit(I2C_START);

    // Check TWI Status
    if (twi_status == TW_MT_ARB_LOST)
      continue;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
      return -1;

    // Send 7-bit slave address (SLA_W) and R/W direction
    // http://www.avrbeginners.net/architecture/twi/twi.html#addressing
    TWDR = (dev_addr & 0xFE) | rw_type;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

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

void i2c_write_byte(uint8_t dev_addr, char data)
{
  // Start the I2C write transmission
  i2c_start(dev_addr, TW_WRITE);

  // Write data
  i2c_write(data);

  // Stop I2C Transmission
  i2c_stop();
}

void i2c_write_byte_to_register(uint8_t dev_addr, uint8_t reg_addr, char data)
{
  // Start the I2C write transmission
  i2c_start(dev_addr, TW_WRITE);

  // Write register address
  i2c_write(reg_addr);

  // Write data to register
  i2c_write(data);

  // Stop I2C Transmission
  i2c_stop();
}

char i2c_read(char *data, char ack_type)
{
  uint8_t twi_status;

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

uint8_t i2c_read_byte_from_register(uint8_t dev_addr, uint8_t reg_addr)
{
  char data;

  // Start the I2C write transmission
  i2c_start(dev_addr, TW_WRITE);

  // Then send register address
  i2c_write(reg_addr);

  // Stop I2C Transmission
  i2c_stop();

  // Re-Start the I2C Read Transmission
  i2c_start(dev_addr, TW_READ);
  i2c_read(&data, 0); // 0 for NACK

  // Stop I2C Transmission
  i2c_stop();

  return data;
}

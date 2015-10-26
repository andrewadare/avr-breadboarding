#include "MCP23008.h"

void write_mcp23008(uint8_t reg_addr, uint8_t data)
{
  // Start the I2C write transmission
  i2c_start(MCP23008, TW_WRITE);

  // Then send register address
  i2c_write(reg_addr);

  // Write data to MCP23008 Register
  i2c_write(data);

  // Stop I2C Transmission
  i2c_stop();
}

uint8_t read_mcp23008(uint8_t reg_addr)
{
  char data;

  // Start the I2C write transmission
  i2c_start(MCP23008, TW_WRITE);

  // Then send register address
  i2c_write(reg_addr);

  // Stop I2C Transmission
  i2c_stop();

  // Re-Start the I2C Read Transmission
  i2c_start(MCP23008, TW_READ);
  i2c_read(&data, 0); // 0 for NACK

  // Stop I2C Transmission
  i2c_stop();

  return data;
}

#include "MCP23008.h"

void write_mcp23008(unsigned char reg_addr,unsigned char data)
{
  // Start the I2C Write Transmission
  i2c_start(MCP23008_ID, MCP23008_ADDR, TW_WRITE);
  // Sending the Register Address
  i2c_write(reg_addr);
  // Write data to MCP23008 Register
  i2c_write(data);
  // Stop I2C Transmission
  i2c_stop();
}

unsigned char read_mcp23008(unsigned char reg_addr)
{
  char data;
  // Start the I2C Write Transmission
  i2c_start(MCP23008_ID, MCP23008_ADDR, TW_WRITE);
  // Read data from MCP23008 Register Address
  i2c_write(reg_addr);
  // Stop I2C Transmission
  i2c_stop();

  // Re-Start the I2C Read Transmission
  i2c_start(MCP23008_ID, MCP23008_ADDR, TW_READ);
  i2c_read(&data, 0); // 0 for NACK

  // Stop I2C Transmission
  i2c_stop();

  return data;
}

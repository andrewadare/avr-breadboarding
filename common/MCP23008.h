#ifndef __MCP23008_H__
#define __MCP23008_H__

#include <compat/twi.h>
#include "i2c_utils.h"

#define MCP23008_ID    0x40  // MCP23008 I2C Device Identifier (0100, fixed)
#define MCP23008_ADDR  0x00  // MCP23008 I2C Address (000-111 in bits 3..1)
#define MCP23008_IODIR 0x00  // MCP23008 I/O Direction Register
#define MCP23008_GPIO  0x09  // MCP23008 General Purpose I/O Register
#define MCP23008_OLAT  0x0A  // MCP23008 Output Latch Register

void write_mcp23008(unsigned char reg_addr,unsigned char data);
unsigned char read_mcp23008(unsigned char reg_addr);

#endif /*__MCP23008_H__*/

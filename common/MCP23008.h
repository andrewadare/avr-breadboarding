// I/O functions for MCP23008 port expander using I2C/TWI protocol
// Derived from blog post at http://www.ermicro.com/blog/?p=1239

#ifndef __MCP23008_H__
#define __MCP23008_H__

#include <compat/twi.h>
#include "i2c_utils.h"

// About the 7-bit I2C slave address MCP23008:
//  - The high bits 7..4 are fixed at 0100.
//  - Bits 3..1 are for further addressing (e.g. eeprom page, jumpered pads)
//  - Bit 0 is disregarded (the R/W direction is handled separately).
// See http://www.avrbeginners.net/architecture/twi/twi.html#addressing

#define MCP23008       0x40  // MCP23008 I2C device address
#define MCP23008_IODIR 0x00  // MCP23008 I/O Direction Register
#define MCP23008_GPIO  0x09  // MCP23008 General Purpose I/O Register
#define MCP23008_OLAT  0x0A  // MCP23008 Output Latch Register

void write_mcp23008(uint8_t reg_addr, uint8_t data);
uint8_t read_mcp23008(uint8_t reg_addr);

#endif /*__MCP23008_H__*/

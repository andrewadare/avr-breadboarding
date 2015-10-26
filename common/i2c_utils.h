// Derived from blog post at http://www.ermicro.com/blog/?p=1239

#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#include <compat/twi.h>

#define I2C_MAX_TRIES 50     // Connection attempts (0 < tries < 255)
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define I2C_SCL 100000L      // Set I2C clock to 100 kHz 

void i2c_init(void);
uint8_t i2c_transmit(uint8_t type);
char i2c_start(uint8_t dev_addr, uint8_t rw_type);
void i2c_stop(void);
char i2c_write(char data);
char i2c_read(char *data, char ack_type);

// Higher-level convenience functions composed from functions above.
void    i2c_write_byte(uint8_t dev_addr, char data);
void    i2c_write_byte_to_register(uint8_t dev_addr, uint8_t reg, char data);
uint8_t i2c_read_byte_from_register(uint8_t dev_addr, uint8_t reg_addr);

#endif  /*__I2C_UTILS_H__*/

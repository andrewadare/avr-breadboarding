// Derived from blog post at http://www.ermicro.com/blog/?p=1239

#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#define I2C_MAX_TRIES 50     // Connection attempts (0 < tries < 255)
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3

void i2c_init(void);
unsigned char i2c_transmit(unsigned char type);
char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type);
void i2c_stop(void);
char i2c_write(char data);
char i2c_read(char *data, char ack_type);

#endif  /*__I2C_UTILS_H__*/

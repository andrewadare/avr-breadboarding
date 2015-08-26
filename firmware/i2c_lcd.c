// I2C code for AVR master and mcp23008 port expander slave
// Based on blog post at http://www.ermicro.com/blog/?p=1239

// For use with the Adafruit I2C/SPI backpack for LCDs using the Hitachi
// HD44780 driver or equivalent. Uses 4 bit mode. Only I2C implemented here.

#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>

#define I2C_MAX_TRIES 50     // Connection attempts (0 < tries < 255)
#define MCP23008_ID    0x40  // MCP23008 I2C Device Identifier (0100, fixed)
#define MCP23008_ADDR  0x00  // MCP23008 I2C Address (000-111 in bits 3..1)
#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPIO  0x09           // MCP23008 General Purpose I/O Register
#define OLAT  0x0A           // MCP23008 Output Latch Register
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define ACK 1
#define NACK 0
#define DATASIZE 13

// Bits in byte written to HD44780 via MCP23008 GP1-GP7 pins (GP0 not connected)
#define LCD_RS   1 // GP1:   Register Select (0: cmd, 1: data)
#define LCD_E    2 // GP2:   Enable bit
#define LCD_DB4  3 // GP3-6: Highest 4 data bits
#define LCD_DB5  4 //        DB0-3 not used in 4-bit mode.
#define LCD_DB6  5 //        Instead, a byte is written to DB4-7 in 2 nibbles. 
#define LCD_DB7  6 //        
#define LCD_LITE 7 // GP7:   LCD backlight on/off (only on 16 pin LCDs)

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

void Write_MCP23008(unsigned char reg_addr,unsigned char data)
{
  // Start the I2C Write Transmission
  i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
  // Sending the Register Address
  i2c_write(reg_addr);
  // Write data to MCP23008 Register
  i2c_write(data);
  // Stop I2C Transmission
  i2c_stop();
}

unsigned char Read_MCP23008(unsigned char reg_addr)
{
  char data;
  // Start the I2C Write Transmission
  i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
  // Read data from MCP23008 Register Address
  i2c_write(reg_addr);
  // Stop I2C Transmission
  i2c_stop();

  // Re-Start the I2C Read Transmission
  i2c_start(MCP23008_ID,MCP23008_ADDR,TW_READ);
  i2c_read(&data,NACK);

  // Stop I2C Transmission
  i2c_stop();

  return data;
}

void i2c_init(void)
{
  // Initial ATMega328P TWI/I2C Peripheral
  TWSR = 0x00;         // Select Prescaler of 1
  // SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
  TWBR = 0x30;        // 48 Decimal
}

void send_nibble(uint8_t rs, uint8_t nibble, uint8_t backlit)
{
  uint8_t byte = 0;

  if (rs)
    byte |= (1 << LCD_RS);
  if (backlit)
    byte |= (1 << LCD_LITE);

  byte |= (nibble << LCD_DB4);

  // Set enable bit high and write
  byte |= (1 << LCD_E);
  Write_MCP23008(GPIO, byte);
  _delay_ms(1);

  // Set enable bit low and write the same data again
  byte &= ~(1 << LCD_E);
  Write_MCP23008(GPIO, byte);
  _delay_ms(1);
}

// Using LCD in 4 bit mode requires writing the 4 highest and 4 lowest data
// bits in two separate, sequential bytes.
void send_byte(uint8_t rs, uint8_t data, uint8_t backlit)
{
  send_nibble(rs, (data & 0xf0) >> 4, backlit);
  send_nibble(rs, (data & 0x0f), backlit);
}

void init_lcd()
{
  // Bit patterns come from HD44780 data sheet tables 4 and 6.

  _delay_ms(15);    // Need long delay between power-up and initialization
  send_nibble(0, 0b0010, 0);   // Set to 4 bit operation (1 nibble operation)
  _delay_ms(5);

  send_byte(0, 0b00101000, 0); // 4 bit mode, 2 display lines, 5x8 dot font
  send_byte(0, 0b00001111, 0); // Display on, cursor on, cursor blinking
  send_byte(0, 1, 0);          // Clear display

  // See "Entry mode set" in HD44780 data sheet, table 6.
  // Increment cursor position, No display shift
  // TODO: put this in its own function (?)
  send_byte(0, 0b00000110, 0);
}

void lcd_puts(const char *s, uint8_t backlit)
{
  char c;

  while ((c = *s++))
    send_byte(1, c, backlit);
}

int main()
{
  i2c_init();

  // Set MCP23008 GP0-GP7 to output, then clear
  Write_MCP23008(IODIR,0b00000000);
  Write_MCP23008(GPIO,0b00000000);

  init_lcd();

  uint8_t backlit = 1;
  lcd_puts("Hello World!", backlit);

  return 0;
}

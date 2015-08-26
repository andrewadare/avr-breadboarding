// I2C code for AVR master and mcp23008 port expander slave
// Based on blog post at http://www.ermicro.com/blog/?p=1239

// For use with the Adafruit I2C/SPI backpack for LCDs using the Hitachi
// HD44780 driver or equivalent. Uses 4 bit mode. Only I2C implemented here.

#include <avr/io.h>
#include <util/delay.h>
#include "MCP23008.h"

// Bits in byte written to HD44780 via MCP23008 GP1-GP7 pins (GP0 not connected)
#define LCD_RS   1 // GP1:   Register Select (0: cmd, 1: data)
#define LCD_E    2 // GP2:   Enable bit
#define LCD_DB4  3 // GP3-6: Highest 4 data bits
#define LCD_DB5  4 //        DB0-3 not used in 4-bit mode.
#define LCD_DB6  5 //        Instead, a byte is written to DB4-7 in 2 nibbles. 
#define LCD_DB7  6 //        
#define LCD_LITE 7 // GP7:   LCD backlight on/off (only on 16 pin LCDs)

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
  write_mcp23008(MCP23008_GPIO, byte);
  _delay_ms(1);

  // Set enable bit low and write the same data again
  byte &= ~(1 << LCD_E);
  write_mcp23008(MCP23008_GPIO, byte);
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

  // Set MCP23008 GP0-GP7 pins to output, then clear GPIO register
  write_mcp23008(MCP23008_IODIR, 0);
  write_mcp23008(MCP23008_GPIO, 0);

  init_lcd();

  uint8_t backlit = 1;
  lcd_puts("Hello World!", backlit);

  return 0;
}

#include <avr/io.h>
#include <util/delay.h>
#include "slcd.h"

void send_nibble(uint8_t rs, uint8_t nibble, uint8_t backlit)
{
  // The HD44780 LCD driver reads on the falling edge of LCD_E.
  // To send data or a command to the display:
  //   1. Set Enable to high
  //   2. Set RS and D0-D7 desired values
  //   3. Set Enable to low
  // I don't (yet) know how to write to only one pin over I2C, so I just write
  // the byte twice, once with LCD_E high and once low.
  // In any case two delay-separated I2C transactions are needed, so this is
  // not too bad.

  uint8_t byte = 0;

  if (rs)
    byte |= (1 << LCD_RS);
  if (backlit)
    byte |= (1 << LCD_LITE);

  byte |= (nibble << LCD_DB4);

  // Set enable bit high and write
  byte |= (1 << LCD_E);
  write_mcp23008(MCP23008_GPIO, byte);
  _delay_us(200);

  // Set enable bit low and write the same data again
  byte &= ~(1 << LCD_E);
  write_mcp23008(MCP23008_GPIO, byte);
  _delay_us(200);
}

// Using LCD in 4 bit mode requires writing the high and low nibbles
// in two separate, sequential bytes.
void send_byte(uint8_t rs, uint8_t data, uint8_t backlit)
{
  send_nibble(rs, (data & 0xf0) >> 4, backlit);
  send_nibble(rs, (data & 0x0f), backlit);
}

void lcd_write(uint8_t data, uint8_t backlit)
{
  send_byte(1, data, backlit);
}

void lcd_command(uint8_t cmd)
{
  send_byte(0, cmd, 0);
}

void init_lcd()
{
  i2c_init();

  // Set MCP23008 GP0-GP7 pins to output, then clear GPIO register
  write_mcp23008(MCP23008_IODIR, 0);
  write_mcp23008(MCP23008_GPIO, 0);

  // Bit patterns come from HD44780 data sheet tables 4 and 6.
  _delay_ms(15);               // Long delay from power-up to initialization
  send_nibble(0, 0b0010, 0);   // Set to 4 bit operation (1 nibble operation)
  _delay_ms(5);

  lcd_command(0b00101000); // 4 bit mode, 2 display lines, 5x8 dot font
  // lcd_command(0b00111000); // 4 bit mode, 1 display line, 5x8 dot font
  lcd_command(0b00001111); // Display on, cursor on, cursor blinking

  lcd_command(1);          // Clear display (similar to lcd_clrscr())

  // See "Entry mode set" in HD44780 data sheet, table 6.
  // Increment cursor position, No display shift
  // TODO: put this in its own function (?)
  lcd_command(0b00000110);
}

void lcd_puts(const char *s, uint8_t backlit)
{
  char c;

  while ((c = *s++))
    lcd_write(c, backlit);
}

void lcd_goto(uint8_t pos)
{
  // This implements the "Set DDRAM address" instruction.
  // The HD44780 is designed to control a 4 x 40 display, and the resulting
  // position depends on your LCD dimensions.
  // Best to find and define LINE1-LINE4, then navigate w.r.t those points.
  send_byte(0, 0x80 + pos, 1);
}

void lcd_clrscr()
{
  send_byte(0, 0x01, 1);
}

void lcd_home()
{
  send_byte(0, 0x02, 1);
  _delay_ms(2);
}

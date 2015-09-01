// I2C code for AVR master and mcp23008 port expander slave

// For use with the Adafruit I2C/SPI backpack for LCDs using the Hitachi
// HD44780 driver or equivalent. Uses 4 bit mode. Only I2C implemented here.

#include <avr/io.h>
#include <util/delay.h>
#include "MCP23008.h"

// Bits in byte written to HD44780 via MCP23008 GP1-GP7 pins (GP0 not connected)
#define LCD_RS     1 // GP1:   Register Select (0: cmd, 1: data)
#define LCD_E      2 // GP2:   Enable bit
#define LCD_DB4    3 // GP3-6: Highest 4 data bits
#define LCD_DB5    4 //        DB0-3 not used in 4-bit mode.
#define LCD_DB6    5 //        Instead, a byte is written to DB4-7 in 2 nibbles. 
#define LCD_DB7    6 //        
#define LCD_LITE   7 // GP7:   LCD backlight on/off (only on 16 pin LCDs)

// Line start positions for a 20x4 display
#define LINE1  0 // First char on line 1
#define LINE2 40 // First char on line 2
#define LINE3 20 // First char on line 3
#define LINE4 84 // First char on line 4

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

int main()
{
  init_lcd();

  uint8_t backlit = 1;

  while (1)
  {
    lcd_clrscr();

    lcd_goto(LINE1);
    lcd_puts("Line 1", backlit);

    lcd_goto(LINE2);
    lcd_puts("Line 2", backlit);

    lcd_goto(LINE3);
    lcd_puts("Line 3", backlit);

    lcd_goto(LINE4);
    lcd_puts("Line 4", backlit);

    _delay_ms(500);

    // Blink the backlight
    for (uint8_t i=0; i<6; i++)
    {
      send_byte(0, 0x80 + LINE4 + 6, i%2);
      _delay_ms(50);
    }

    _delay_ms(500);

  }

  return 0;
}

// I2C code for AVR master and mcp23008 port expander slave

// For use with the Adafruit I2C/SPI backpack for LCDs using the Hitachi
// HD44780 driver or equivalent. Uses 4 bit mode. Only I2C implemented here.

#ifndef __SLCD_H__
#define __SLCD_H__

#include "MCP23008.h"

// Bits in byte written to HD44780 via MCP23008 GP1-GP7 pins (GP0 not connected).
// In other words, these defs are w.r.t. MCP23008 (not HD44780) pins.
#define LCD_RS     1 // GP1:   Register Select (0: cmd, 1: data)
#define LCD_E      2 // GP2:   Enable bit
#define LCD_DB4    3 // GP3-6: Highest 4 data bits
#define LCD_DB5    4 //        DB0-3 not used in 4-bit mode.
#define LCD_DB6    5 //        Instead, a byte is written to DB4-7 in 2 nibbles. 
#define LCD_DB7    6 //        
#define LCD_LITE   7 // GP7:   LCD backlight on/off (only on 16 pin LCDs)

// LCD display size
// For sizes other than 4x20, LCD_LINE* below must be redefined.
#define LCD_LINES  4
#define LCD_WIDTH 20

// These are line start positions as DDRAM offsets from 0x80 (DB7). 
// The HD44780 is designed to control a 4 x 40 display.
#define LCD_LINE0 0x00 // First char on line 1
#define LCD_LINE1 0x40 // First char on line 2
#define LCD_LINE2 0x14 // First char on line 3
#define LCD_LINE3 0x54 // First char on line 4

void send_nibble(uint8_t rs, uint8_t nibble, uint8_t backlit);
void send_byte(uint8_t rs, uint8_t data, uint8_t backlit);
void lcd_write(uint8_t data, uint8_t backlit);
void lcd_command(uint8_t cmd);
void init_lcd();
void lcd_puts(const char *s, uint8_t backlit);
void lcd_goto(uint8_t line, uint8_t column); // Use e.g. LINE2 as 1st arg
void lcd_clrscr();
void lcd_home();

#endif /*__SLCD_H__*/

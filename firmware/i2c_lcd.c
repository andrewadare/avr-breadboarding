#include <avr/io.h>
#include <util/delay.h>
#include "slcd.h"

int main()
{
  init_lcd();

  while (1)
  {
    lcd_clrscr();

    lcd_goto(0, 0);
    lcd_puts("Line 1");

    lcd_goto(1, 0);
    lcd_puts("Line 2");

    lcd_goto(2, 0);
    lcd_puts("Line 3");

    lcd_goto(3, 0);
    lcd_puts("Line 4");

    _delay_ms(500);

    // Blink the backlight
    for (uint8_t i=0; i<6; i++)
    {
      send_byte(0, 0x80 + LCD_LINE3 + 6, i%2);
      _delay_ms(50);
    }

    _delay_ms(500);

    // Raster the cursor to check the goto function
    lcd_clrscr();
    for (uint8_t i=0; i<LCD_LINES; i++)
      for (uint8_t j=0; j<LCD_WIDTH; j++)
      {
        lcd_goto(i, j);
        _delay_ms(100);
      }

  }

  return 0;
}

#include <avr/io.h>
#include <util/delay.h>
#include "slcd.h"

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

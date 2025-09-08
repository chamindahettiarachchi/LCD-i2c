#ifndef F_CPU
#define F_CPU 3333333UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

int main(void) {
    lcd_init();
    lcd_clear();

    // demo variables
    int32_t   temp_c_x100 = 2537;    // 25.37°C as fixed-point (x100)
    uint32_t  counter     = 0;

    lcd_gotoxy(0,0);
    lcd_print("Temp:");
    lcd_print_fixed(temp_c_x100, 2);
    lcd_print_char((char)223);       // custom degree symbol? (depends on CGROM)
    lcd_print("C");

    while (1) {
        lcd_gotoxy(0,1);
        lcd_print("Cnt:");
        lcd_print_uint(counter++);
        lcd_print(" 0x");
        lcd_print_hex(counter, 4);   // show lower 4 hex digits

        _delay_ms(500);
    }
}

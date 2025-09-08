#ifndef LCD_H_
#define LCD_H_

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

/* ===== User Config ===== */
#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR   0x3F     // 0x27 or 0x3F are common
#endif

#ifndef LCD_COLS
#define LCD_COLS       16
#endif

#ifndef LCD_ROWS
#define LCD_ROWS       2
#endif

#ifndef I2C_FREQ
#define I2C_FREQ       100000UL  // you can set 10000UL if you like it slower
#endif

/* ===== Public API ===== */
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_gotoxy(uint8_t col, uint8_t row);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_print(const char *s);
void lcd_print_char(char c);
void lcd_backlight(bool on);

/* Value-print helpers (no stdio required) */
void lcd_print_int(int32_t v);
void lcd_print_uint(uint32_t v);
void lcd_print_hex(uint32_t v, uint8_t width);          // width=1..8, pads with zeros
void lcd_print_fixed(int32_t scaled, uint8_t decimals); // prints scaled/10^decimals, e.g. 2537,2 -> "25.37"

/* Optional: custom characters (index 0..7, pattern[8] 5-bit wide) */
void lcd_create_char(uint8_t index, const uint8_t pattern[8]);

#endif /* LCD_H_ */

#ifndef F_CPU
#define F_CPU 3333333UL
#endif

#include "lcd.h"
#include <util/delay.h>

/* ===== PCF8574 bit mapping (your working board) =====
   P0=RS, P1=RW, P2=EN, P3=BL, P4=D4, P5=D5, P6=D6, P7=D7
*/
#define LCD_RS   (1u<<0)
#define LCD_RW   (1u<<1)
#define LCD_EN   (1u<<2)
#define LCD_BL   (1u<<3)
#define D4       (1u<<4)
#define D5       (1u<<5)
#define D6       (1u<<6)
#define D7       (1u<<7)

/* local state */
static uint8_t g_lat = 0;             // last written expander state
static uint8_t g_bl_mask = LCD_BL;    // backlight bit mask (on/off)

/* ===== TWI0 (I2C) Master on ATtiny414: PA2=SDA, PA3=SCL ===== */
static inline void i2c_init(void) {
    uint8_t mbaud = (uint8_t)((F_CPU / (2*I2C_FREQ)) - 5);
    TWI0.MBAUD  = mbaud;
    TWI0.MCTRLA = TWI_ENABLE_bm;
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

static inline void i2c_write_byte(uint8_t addr, uint8_t data) {
    TWI0.MADDR = (addr << 1) | 0;                    // write
    while (!(TWI0.MSTATUS & TWI_WIF_bm)) {}
    if (TWI0.MSTATUS & TWI_RXACK_bm) {               // NACK on address
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return;
    }
    TWI0.MDATA = data;
    while (!(TWI0.MSTATUS & TWI_WIF_bm)) {}
    // ignore data NACK here, still stop
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

/* ===== PCF8574 out ===== */
static inline void pcf_out(uint8_t v) {
    g_lat = v;
    i2c_write_byte(LCD_I2C_ADDR, v);
}

/* ===== Low-level nibble write ===== */
static void lcd_write_nibble(uint8_t nibble, bool rs) {
    uint8_t out = 0;
    if (nibble & 0x01) out |= D4;
    if (nibble & 0x02) out |= D5;
    if (nibble & 0x04) out |= D6;
    if (nibble & 0x08) out |= D7;

    if (rs) out |= LCD_RS;       // RS=1 data, RS=0 cmd
    // RW always 0 (write). Keep BL as last state:
    out |= (g_lat & g_bl_mask);

    // EN pulse
    pcf_out(out | LCD_EN);
    _delay_us(1);
    pcf_out(out & ~LCD_EN);
    _delay_us(40);               // most ops ~37us
}

static void lcd_write_byte(uint8_t value, bool rs) {
    lcd_write_nibble((value >> 4) & 0x0F, rs);
    lcd_write_nibble(value & 0x0F, rs);
}

/* ===== Public core ===== */
void lcd_command(uint8_t cmd) { lcd_write_byte(cmd, false); }
void lcd_data(uint8_t data)    { lcd_write_byte(data, true); }

void lcd_backlight(bool on) {
    if (on) {
        g_bl_mask = LCD_BL;
        pcf_out((g_lat | LCD_BL) & 0xFF);
    } else {
        g_bl_mask = 0;
        pcf_out(g_lat & ~LCD_BL);
    }
}

void lcd_init(void) {
    i2c_init();
    _delay_ms(60);

    // make sure BL known
    pcf_out(0x00);
    lcd_backlight(true);

    // 4-bit init sequence (HD44780)
    lcd_write_nibble(0x03, false); _delay_ms(5);
    lcd_write_nibble(0x03, false); _delay_ms(5);
    lcd_write_nibble(0x03, false); _delay_ms(2);
    lcd_write_nibble(0x02, false); _delay_ms(2);

    lcd_command(0x28);  // 4-bit, 2-line, 5x8 dots
    lcd_command(0x08);  // display OFF
    lcd_clear();        // clear + delay
    lcd_command(0x06);  // entry mode: increment, no shift
    lcd_command(0x0C);  // display ON, cursor OFF, blink OFF
}

void lcd_clear(void) {
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_home(void) {
    lcd_command(0x02);
    _delay_ms(2);
}

void lcd_gotoxy(uint8_t col, uint8_t row) {
    static const uint8_t base[] = {0x00, 0x40, 0x14, 0x54}; // rows 0..3
    if (row >= LCD_ROWS) row = 0;
    lcd_command(0x80 | (base[row] + col));
}

void lcd_print(const char *s) {
    while (*s) lcd_data((uint8_t)*s++);
}

void lcd_print_char(char c) {
    lcd_data((uint8_t)c);
}

/* ===== Optional custom chars (index 0..7) ===== */
void lcd_create_char(uint8_t index, const uint8_t pattern[8]) {
    if (index > 7) return;
    lcd_command(0x40 | (index << 3));   // CGRAM addr
    for (uint8_t i = 0; i < 8; i++) {
        lcd_data(pattern[i] & 0x1F);    // only lower 5 bits used
    }
}

/* ===== Number printing helpers (no stdio) ===== */
static void _print_dec_unsigned(uint32_t v) {
    // buffer max 10 digits + null
    char buf[11];
    uint8_t i = 0;
    if (v == 0) {
        lcd_print_char('0');
        return;
    }
    while (v && i < sizeof(buf)) {
        buf[i++] = '0' + (v % 10u);
        v /= 10u;
    }
    // print reversed
    while (i--) lcd_print_char(buf[i]);
}

static void _print_dec_signed(int32_t v) {
    if (v < 0) {
        lcd_print_char('-');
        // careful with INT32_MIN
        uint32_t mag = (uint32_t)(-(v + 1)) + 1u;
        _print_dec_unsigned(mag);
    } else {
        _print_dec_unsigned((uint32_t)v);
    }
}

void lcd_print_uint(uint32_t v) { _print_dec_unsigned(v); }
void lcd_print_int(int32_t v)   { _print_dec_signed(v);   }

void lcd_print_hex(uint32_t v, uint8_t width) {
    if (width > 8) width = 8;
    for (int8_t i = (int8_t)width - 1; i >= 0; --i) {
        uint8_t nib = (v >> (i * 4)) & 0x0F;
        char c = (nib < 10) ? ('0' + nib) : ('A' + (nib - 10));
        lcd_print_char(c);
    }
}

/* Print fixed-point: scaled / 10^decimals, keeps sign and zero padding.
   Example: scaled= -2537, decimals=2  -> "-25.37" */
void lcd_print_fixed(int32_t scaled, uint8_t decimals) {
    bool neg = (scaled < 0);
    uint32_t mag = neg ? (uint32_t)(-(scaled + 1)) + 1u : (uint32_t)scaled;

    uint32_t pow10 = 1;
    for (uint8_t i = 0; i < decimals; ++i) pow10 *= 10u;

    uint32_t intpart = mag / pow10;
    uint32_t frac    = mag % pow10;

    if (neg) lcd_print_char('-');
    _print_dec_unsigned(intpart);

    if (decimals > 0) {
        lcd_print_char('.');
        // print frac with leading zeros to exactly 'decimals' digits
        char buf[10];
        uint8_t i = 0;
        for (uint8_t d = 0; d < decimals; ++d) {
            buf[decimals - 1 - d] = '0' + (frac % 10u);
            frac /= 10u;
        }
        for (uint8_t d = 0; d < decimals; ++d) lcd_print_char(buf[d]);
    }
}

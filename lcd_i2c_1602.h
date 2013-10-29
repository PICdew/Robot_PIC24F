

#ifndef _LCD_I2C_1602_
#define _LCD_I2C_1602_

#include "i2c.h"
#include "i2c_func.h"

//****************************************
// 定義1602內部命令
//****************************************
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// function on or off
#define LCD_FUNC_ON     0x01
#define LCD_FUNC_OFF    0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

void Lcd_1602_clear(void);
void Lcd_1602_home(void);
void Lcd_1602_set_cursor(int col, int row);
void Lcd_1602_display(int sw);
void Lcd_1602_cursor(int sw);
void Lcd_1602_blink(int sw);
void Lcd_1602_scroll_dir(int sw);
void Lcd_1602_display_dir(int sw);
void Lcd_1602_autoscroll(int sw);
void Lcd_1602_create_char(int location, int charmap[]);
void Lcd_1602_backlight(int sw);
void Lcd_1602_display_string(int x,int y, char *data, int n);
void Lcd_1602_display_dec(int x,int y, int value);
void Lcd_1602_display_hex(int x,int y, int value);
void Lcd_1602_init(int lcd_Addr,int lcd_cols,int lcd_rows, int dotsize);

#endif


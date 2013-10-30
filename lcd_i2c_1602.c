/*
 * Regis 10/29/2013
 *
 */
// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "led.h"

#include "i2c.h"
#include "i2c_func.h"
#include "lcd_i2c_1602.h"

int _Addr;
int _displayfunction;
int _displaycontrol;
int _displaymode;
int _numlines;
int _backlightval;

xQueueHandle xLCD_Queue;

void vTask_LCD(void *pvParameters)
{
    portTickType xLastWakeTime;
    portBASE_TYPE xStatus;

    (void)pvParameters;
    xLastWakeTime = xTaskGetTickCount();
    xLCD_Queue = xQueueCreate(20, sizeof(int));

    for (;;)
    {
	vTaskDelayUntil(&xLastWakeTime, 200);
	xStatus = uxQueueMessagesWaiting(xLCD_Queue);
	if (xStatus != pdPASS)
	{
            while(1){};
            /* The send operation could not complete because the queue was full -
            this must be an error as the queue should never contain more than
            one item! */
            //printf("xxxxxxxxxxxxx Could not send to the queue.\r\n");
	}
	//printf("T3 =>Q [%d]\n", value[a]);
    }
}


/*
LiquidCrystal_I2C lcd(0x4E,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
void setup()
{
  lcd.init();                      // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  lcd.print("Hello, world!");
}
*/

#if 0
//****************************************
//整數轉字符串
//****************************************
void Lcd_printf(int *s,int temp_data)
{
	if(temp_data<0)
	{
		temp_data=-temp_data;
		*s='-';
	}
	else *s=' ';
	*++s =temp_data/100+0x30;
	temp_data=temp_data%100;     //取余運算
	*++s =temp_data/10+0x30;
	temp_data=temp_data%10;      //取余運算
	*++s =temp_data+0x30;
}
#endif
/* atoi: convert s to an integer
 *
 * Here's the easy way:
 * int atoi(char *s){return (int)strtoul(s, NULL, 10);}
 * But I'll behave...
 */
static int atoi(char *s)
{
    int n, sign;

    while (isspace(*s))
        s++;
    sign = (*s == '+' || *s == '-') ? ((*s++ == '+') ? 1 : -1) : 1;
    for (n = 0; isdigit(*s); s++)
        n = (n * 10) + (*s - '0');  /* note to language lawyers --
                                     * the digits are in consecutive
                                     * order in the character set
                                     * C90 5.2.1
                                     */
    return sign * n;
}

static char *utoa(unsigned value, char *digits, int base)
{
    char *s, *p;

    s = "0123456789abcdefghijklmnopqrstuvwxyz"; /* don't care if s is in

                                                 * read-only memory
                                                 */
    if (base == 0)
        base = 10;
    if (digits == NULL || base < 2 || base > 36)
        return NULL;
    if (value < (unsigned) base) {
        digits[0] = s[value];
        digits[1] = '\0';
    } else {
        for (p = utoa(value / ((unsigned)base), digits, base);
             *p;
             p++);
        utoa( value % ((unsigned)base), p, base);
    }
    return digits;
}

static char *itoa(char *digits, int value, int base)
{
    char *d;
    unsigned u; /* assume unsigned is big enough to hold all the
                 * unsigned values -x could possibly be -- don't
                 * know how well this assumption holds on the
                 * DeathStation 9000, so beware of nasal demons
                 */

    d = digits;
    if (base == 0)
        base = 10;
    if (digits == NULL || base < 2 || base > 36)
        return NULL;
    if (value < 0) {
        *d++ = '-';
        u = -((unsigned)value);
    } else
        u = value;
    utoa(u, d, base);
    return digits;
}


/************ low level data pushing commands **********/
static unsigned char Lcd_i2c_write(unsigned char ControlByte, unsigned char data)
{
	unsigned char ErrorCode;

	IdleI2C();				//Ensure Module is Idle
	StartI2C();				//Generate Start COndition
	WriteI2C(ControlByte);			//Write Control byte
	IdleI2C();

	ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(data);				//Write Data
	IdleI2C();
	StopI2C();				//Initiate Stop Condition
	// Regis EEAckPolling(ControlByte);		//Perform ACK polling
	return(ErrorCode);
}

static void Lcd_1602_expanderWrite(int _data)
{
    Lcd_i2c_write(_Addr,  _data | _backlightval);
}

static void Lcd_1602_pulseEnable(int _data)
{
	Lcd_1602_expanderWrite(_data | En);	// En high
	delay_ms(1);		// enable pulse must be >450ns
	Lcd_1602_expanderWrite(_data & ~En);	// En low
	delay_ms(5);		// commands need > 37us to settle
}

static void Lcd_1602_write4bits(int value)
{
    Lcd_1602_expanderWrite(value);
    Lcd_1602_pulseEnable(value);
}

// write either command or data
static void Lcd_1602_send(int value, int mode)
{
    int highnib = value & 0xf0;
    int lownib = (value << 4) & 0xf0;
    Lcd_1602_write4bits((highnib)|mode);
    Lcd_1602_write4bits((lownib)|mode);
}

static void Lcd_1602_command(int value)
{
    Lcd_1602_send(value, 0);
}

static void Lcd_1602_write(int value)
{
    Lcd_1602_send(value, Rs);
}

/********** high level commands, for the user! */
void Lcd_1602_clear()
{
	Lcd_1602_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delay_ms(2);  // this command takes a long time!
}

void Lcd_1602_home(){
	Lcd_1602_command(LCD_RETURNHOME);  // set cursor position to zero
	delay_ms(2);  // this command takes a long time!
}

void Lcd_1602_set_cursor(int col, int row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > _numlines ) {
		row = _numlines-1;    // we count rows starting w/0
	}
	Lcd_1602_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void Lcd_1602_display(int sw)
{
    if (sw)
    {
        _displaycontrol |= LCD_DISPLAYON;
    }
    else
    {
        _displaycontrol &= ~LCD_DISPLAYON;
    }
    Lcd_1602_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void Lcd_1602_cursor(int sw)
{
    if (sw)
    {
        _displaycontrol |= LCD_CURSORON;
    }
    else
    {
        _displaycontrol &= ~LCD_CURSORON;
    }
    Lcd_1602_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void Lcd_1602_blink(int sw)
{
    if (sw)
    {
        _displaycontrol |= LCD_BLINKON;
    }
    else
    {
        _displaycontrol &= ~LCD_BLINKON;
    }
    Lcd_1602_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void Lcd_1602_scroll_dir(int sw)
{
    int func;
    if (sw)
    {
        // Left
        func = LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT;
    }
    else
    {
        // Right
        func = LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT;
    }
    Lcd_1602_command(func);
}

// This is for text that flows Left to Right or R2L
void Lcd_1602_display_dir(int sw)
{
    if (sw)
    {
        // Right to Left
        _displaymode &= ~LCD_ENTRYLEFT;
    }
    else
    {
        //Left to Right
        _displaymode |= LCD_ENTRYLEFT;
    }
    Lcd_1602_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void Lcd_1602_autoscroll(int sw)
{
    if (sw)
    {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
    }
    else
    {
       	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    }
    Lcd_1602_command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void Lcd_1602_create_char(int location, int charmap[]) {
    int i;
	location &= 0x7; // we only have 8 locations 0-7
	Lcd_1602_command(LCD_SETCGRAMADDR | (location << 3));
	for (i=0; i<8; i++) {
		Lcd_1602_write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void Lcd_1602_backlight(int sw)
{
    if (sw)
    {
        _backlightval=LCD_BACKLIGHT;
    }
    else
    {
        _backlightval=LCD_NOBACKLIGHT;
    }
    Lcd_1602_expanderWrite(0);
}

//****************************************
//LCD1602 show string
//****************************************
void Lcd_1602_display_string(int x,int y,char *data, int n)
{
	int a=0;

        y &= 0x1;
        while(n--)
	{
            x &= 0xF;
            if (y)
                x |= 0x40;
            x |= 0x80;
            Lcd_1602_command(x);
            Lcd_1602_write(data[a]);
            a++;
            x++;
	}
}

void Lcd_1602_display_dec(int x,int y, int value)
{
    char dis[8];

    //Lcd_printf(dis, value);         //轉換數據顯示
    itoa(dis, value, 10);
    Lcd_1602_display_string(x, y, dis, strlen(dis));	//啟始列，行，顯示數組，顯示長度
}

void Lcd_1602_display_hex(int x,int y, int value)
{
    char dis[8];

    //Lcd_printf(dis, value);         //轉換數據顯示
    itoa(dis, value, 16);
    Lcd_1602_display_string(x, y, dis, strlen(dis));	//啟始列，行，顯示數組，顯示長度
}

/// High level functions
 //Lcd_1602_Init(0x4E, 16, 2, LCD_5x8DOTS);  // set the LCD address to 0x27 for a 16 chars and 2 line display
void Lcd_1602_init(int lcd_Addr,int lcd_cols,int lcd_rows, int dotsize)
{
    char str[20] = "Regis is good!";

    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    _backlightval = LCD_NOBACKLIGHT;

    _Addr = lcd_Addr;
  
    if (lcd_rows > 1)
    {
        _displayfunction |= LCD_2LINE;
    }
    _numlines = lcd_rows;

    // for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lcd_rows == 1))
    {
	_displayfunction |= LCD_5x10DOTS;
    }

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 40ms
    delay_ms( 40 );

    //put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    Lcd_1602_write4bits(0x03 << 4);
    delay_ms(5); // wait min 4.1ms
    // second try
    Lcd_1602_write4bits(0x03 << 4);
    delay_ms(1); // wait min 100us
    // third go!
    Lcd_1602_write4bits(0x03 << 4);
    // finally, set to 4-bit interface
    Lcd_1602_write4bits(0x02 << 4);

    // set # lines, font size, etc.
    Lcd_1602_command(LCD_FUNCTIONSET | _displayfunction);

    // turn the display on with no cursor or blinking default
    //_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    Lcd_1602_display(LCD_FUNC_ON);

    // clear it off
    Lcd_1602_clear();

    // Initialize to default text direction (for roman languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

    // set the entry mode
    Lcd_1602_command(LCD_ENTRYMODESET | _displaymode);

    Lcd_1602_home();

    Lcd_1602_backlight(LCD_FUNC_ON);
    //Lcd_1602_autoscroll(LCD_FUNC_ON);
    Lcd_1602_display_string(0, 0, str, strlen(str));
    
    Lcd_1602_display_hex(0,1,255);
    Lcd_1602_display_dec(5,1,3000);

}

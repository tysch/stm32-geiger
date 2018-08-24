#ifndef __DRV_LCD5110__
#define __DRV_LCD5110__

#include <stdbool.h>

#define	LCD_COMMAND	0
#define LCD_DATA 1
#define LCD_RESET GPIO_Pin_8
#define LCD_DC GPIO_Pin_10
#define LCD_DATA_IN GPIO_Pin_11
#define LCD_CLK GPIO_Pin_12
#define LCD_CS GPIO_Pin_9
#define LCD_BACK_LIGHT GPIO_Pin_15
#define LCD_SETYADDR  0x40
#define LCD_SETXADDR  0x80
#define LEFT 0
#define RIGHT 9999
#define CENTER 9998
#define FONT_READ_BYTE(x) _font.font_ptr[x];


void LCD5110_init(void);
void LCD5110_write_char(unsigned char c);
void LCD5110_write_char_inv(unsigned char c);
void LCD5110_clear(void);
void LCD5110_set_XY(unsigned char X, unsigned char Y);
void LCD5110_write_string(char *s);
void LCD5110_write_Dec(unsigned int buffer);
void LCD5110_Led(unsigned char c);
void LCD5110_num_char(unsigned char c, int x, int row);
void printNumF(double num, char dec, int x, int y, char divider, int length, char filler);
void LCD5110_num_string(char *s, int x, int row);
void LCD5110_SetFont(bool isBig);
void LCD5110_drawBitmap(int x, int y, unsigned char* bitmap, int sx, int sy/*, bool flash*/);

struct lcd_font
{
    volatile unsigned char* font_ptr;
    char x_size;
    char y_size;
    char offset;
    char numchars;
    char inverted;
};

volatile static struct lcd_font _font;

#endif
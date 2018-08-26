#include "driver_5110_lcd.h"
#include "font.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

static void Delay_ms(unsigned long nCount)
{
    volatile unsigned long t;
	t = nCount * 72000;
	while(t--);
}

//Define the LCD Operation function
void LCD5110_LCD_write_byte(unsigned char dat, unsigned char LCD5110_MOde);

//Define the hardware operation function
static void LCD5110_GPIO_Config(void);
static void LCD5110_SCK(unsigned char temp);
static void LCD5110_DIN(unsigned char temp);
static void LCD5110_CS(unsigned char temp);
static void LCD5110_RST(unsigned char temp);
static void LCD5110_DC(unsigned char temp);


void LCD5110_SetFont(bool isBig) 
{
   if(isBig == true)
   {
      // You can obtain a pointer to the array block in a Byte array by taking 
      // the address of the first element and assigning it to a pointer.
      _font.font_ptr = &MediumNumbers[0];
   } 
   
   else if(isBig == false)
   {
      // You can obtain a pointer to the array block in a Byte array by taking 
      // the address of the first element and assigning it to a pointer.
      _font.font_ptr = &BigNumbers[0];
   }
   
    _font.x_size = FONT_READ_BYTE(0);     // 12;
    _font.y_size = FONT_READ_BYTE(1);     // 16;
    _font.offset = FONT_READ_BYTE(2);     // 45;
    _font.numchars = FONT_READ_BYTE(3);   // 13;
    _font.inverted = 0;
   
}

/**
 * Set pin configuration. Doesn't use SPI controller. Just regular pins.
 *
 *	PF11 : Reset
 *	PF7  : DC
 *	PF5  : MISO
 *	PF3  : CLK
 *	PF9  : CE
 *	PF1  : LED control
 *
 * @param None
 * @retval None
 */
void LCD5110_GPIO_Config(void) {
  
    GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_HSICmd(DISABLE);
//	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
//	RCC_PLLCmd(ENABLE);
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // Declare pins to configure
    GPIO_InitStructure.GPIO_Pin = LCD_RESET | LCD_DC |  LCD_DATA_IN | LCD_CLK | LCD_CS;// | LCD_BACK_LIGHT; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    
    // Init Port
    GPIO_Init(GPIOA, &GPIO_InitStructure);



	GPIO_InitStructure.GPIO_Pin = LCD_BACK_LIGHT; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Setup default font and size.
    LCD5110_SetFont(true);
    
}


/**
 * Initialize LCD module
 *
 * Input parameters : none
 * Return value		: none
 */
void LCD5110_init(void) {

	//Configure pins
	LCD5110_GPIO_Config();

	// Set pin initial state
	LCD5110_Led(0); //Turn back light off
	LCD5110_RST(0); //Set LCD reset = 0;
	LCD5110_DC(1); //Mode = command;
	LCD5110_DIN(1); //Set In at high level;
	LCD5110_SCK(1); //Set CLK high;
	LCD5110_CS(1); //Unselect chip;

	//Keep reset pin low for 10 ms
	Delay_ms(10);
	//Release Reset Pin
	LCD5110_RST(1); //LCD_RST = 1;

	//Configure LCD module
	LCD5110_LCD_write_byte(0x21, LCD_COMMAND); //Extended instruction set selected
	LCD5110_LCD_write_byte(0xBF, LCD_COMMAND); //Set LCD voltage (defined by experimentation...)
	LCD5110_LCD_write_byte(0x14, LCD_COMMAND); //Set Bias for 1/48
	LCD5110_LCD_write_byte(0x06, LCD_COMMAND); //Set temperature control (TC2)
	LCD5110_LCD_write_byte(0x20, LCD_COMMAND); //Revert to standard instruction set

	LCD5110_clear(); //Clear display (still off)
	LCD5110_LCD_write_byte(0x0c, LCD_COMMAND); //Set display on in "normal" mode (not inversed)
}


/**
 * Write byte to the module.
 *
 * @param dat  	data to write
 * @param mode  0 if command, 1 if data
 *
 * @retval		None
 */
void LCD5110_LCD_write_byte(unsigned char dat, unsigned char mode) {
	unsigned char i;
	LCD5110_CS(0); //SPI_CS = 0;

	if (0 == mode)
		LCD5110_DC(0); //LCD_DC = 0;
	else
		LCD5110_DC(1); //LCD_DC = 1;

	for (i = 0; i < 8; i++) {
		LCD5110_DIN(dat & 0x80); //SPI_MO = dat & 0x80;
		dat = dat << 1;
		LCD5110_SCK(0); //SPI_SCK = 0;
		LCD5110_SCK(1); //SPI_SCK = 1;
	}

	LCD5110_CS(1); //SPI_CS = 1;

}


/**
 * Write character to LCD at current position
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_write_char(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = font6_8[c][line];
		LCD5110_LCD_write_byte(ch, LCD_DATA);

	}
}

/**
 * Write character to LCD in inverse video at current location
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_write_char_inv(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = ~font6_8[c][line];
		LCD5110_LCD_write_byte(ch, LCD_DATA);

	}
}

/**
 * Write string to LCD at current position. String must be null terminated.
 *
 * @param s: string pointer
 * @retval None
 */
void LCD5110_write_string(char *s) {
	unsigned char ch;
	while (*s != '\0') {
		ch = *s;
		LCD5110_write_char(ch);
		s++;
	}
}



void LCD5110_num_string(char *s, int x, int row) 
{
  unsigned char ch;
  char m = 0;
  while(*s != '\0') {
    ch = *s;
    LCD5110_num_char(ch, x + (m * (_font.x_size)), row);
    s++;
    m++;
  } 
}



/**
 * Write character to LCD at current position
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_num_char(unsigned char c, int x, int row) 
{
   
//  printf("************* START *************** \n");
if(c == ' ') c = 10 + '0'; // Hack for padding whitespace
    // Caclulate the y-size pixel bank height.
  int max = _font.x_size == 12 ? 2 : 3;
    
    
    int font_idx  = 0;
    
     // look up
     uint16_t index = 0;
        
    for (int rowcnt = 0; rowcnt < max  ; rowcnt++) 
    {
        // Move the y-address pointer for every pointer.
      	LCD5110_LCD_write_byte(LCD_SETYADDR | (row + rowcnt), LCD_COMMAND); 
        // Move to the x-address.  
	LCD5110_LCD_write_byte(LCD_SETXADDR | x, LCD_COMMAND);
        
        // look up font index. 
        
        /*
        
            unsigned char = LOOKUP_TABLE[] = {}
            
            CONFIG = {0x0c, 0x10, 0x2d, 0x0d}
            offset = 45  ( ASCII code for char  "-" )
            x_size = 12
            y_size = 16
        
            x = (ASCII - offset )
            y = x_size *  ( y_size / 5)
            z = x + y + 4; 
        
            index = z;
            
            display char "1" dec "49"
            Example: 
        
            4 = (49 - offset) 
            8.8  =  (12 - (16 / 5))
            16.8 = (4 + 8.8 + 4) 
        
        */
        font_idx = (( c - _font.offset ) * ( _font.x_size * ( _font.y_size / 8 ))) + 4;
        
       // printf("y=> %d, x=> %d,  disp=> %c, font_idx=> %d, rowcnt=> %d \n", row, x, c, font_idx, rowcnt);
        

        for(uint16_t cnt = 0; cnt < _font.x_size ; cnt++)
        {
            index = font_idx + cnt + ( rowcnt * _font.x_size );
            char s = FONT_READ_BYTE( index );
            
         //   printf("cnt=> %d, index => %d, byte => %d \n", cnt, index, s);
            
            LCD5110_LCD_write_byte( s , LCD_DATA );
        }
    }
    
   // printf("************* END *************** \n");
  
   // reset 
   LCD5110_LCD_write_byte(LCD_SETYADDR, LCD_COMMAND);
   LCD5110_LCD_write_byte(LCD_SETXADDR, LCD_COMMAND);
}

void LCD5110_drawBitmap(int x, int y, unsigned char* bitmap, int sx, int sy/*, bool flash*/)
{
	int starty, rows;
    
	starty = y / 8;

	if (sy%8==0)
		rows=sy/8;  
	else
		rows=(sy/8)+1;

	for (int cy=0; cy<rows; cy++)
	{
		LCD5110_LCD_write_byte(0x40+(starty+cy), LCD_COMMAND);
		LCD5110_LCD_write_byte(0x80+x, LCD_COMMAND);
		for(int cx=0; cx<sx; cx++)
		{
			//if (flash)
			//LCD5110_LCD_write_byte( bitmap[cx+(cy*sx)], LCD_DATA);
				//else
			LCD5110_LCD_write_byte(bitmap[cx+(cy*sx)], LCD_DATA);
		}
	}      
	LCD5110_LCD_write_byte(0x40, LCD_COMMAND);
	LCD5110_LCD_write_byte(0x80, LCD_COMMAND);
}



/**
 * Clear display. Write 0 in all memory location.
 *
 * @param None
 * @retval None
 */
void LCD5110_clear(void) {
	unsigned char i, j;
	for (i = 0; i < 6; i++)
		for (j = 0; j < 84; j++)
			LCD5110_LCD_write_byte(0, LCD_DATA);
}

/**
 * Set memory current location for characters (set coordinates).
 * Applies only for Fonts with a 6 pixels width.
 *
 * @param X: Column (range from 0 to 13)
 * @param Y: Row (range from 0 to 5)
 * @retval None
 *
 */
void LCD5110_set_XY(unsigned char X, unsigned char Y) {
	unsigned char x;
	x = 6 * X;

	LCD5110_LCD_write_byte(0x40 | Y, LCD_COMMAND);
	LCD5110_LCD_write_byte(0x80 | x, LCD_COMMAND);
}

/**
 * Manage CS pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
static void LCD5110_CS(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIOA, LCD_CS);
	else
		GPIO_SetBits(GPIOA, LCD_CS);
}

/**
 * Manage Reset pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
static void LCD5110_RST(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIOA, LCD_RESET);
	else
		GPIO_SetBits(GPIOA, LCD_RESET);
}

/**
 * Manage DC pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
static void LCD5110_DC(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIOA, LCD_DC);
	else
		GPIO_SetBits(GPIOA, LCD_DC);
}

/**
 * Manage DIN pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
static void LCD5110_DIN(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIOA, LCD_DATA_IN);
	else
		GPIO_SetBits(GPIOA, LCD_DATA_IN);
}

/**
 * Manage CLK pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
static void LCD5110_SCK(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIOA, LCD_CLK);
	else
		GPIO_SetBits(GPIOA, LCD_CLK);
}

/**
 * Manage LED pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_Led(unsigned char state) {
	if (state)
		GPIO_SetBits(GPIOB, LCD_BACK_LIGHT);
	else
		GPIO_ResetBits(GPIOB, LCD_BACK_LIGHT);
}
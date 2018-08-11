#include "driver_5110_lcd.h"
#include "xprintf.h"
#include <stdint.h>

void strcat(char * dest, const char * src)
{
	while(*dest) dest++;
	while((*dest++ = *src++));
}

static void format_readings(char * prefix, char * valstr, uint32_t val)
{
	char mju = 'z' + 3;
    // Set range indication for main reading
	if(val >= 100000000)
	{
		int r = val/1000000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%4d", r);
		return;
	}
	if((val >= 10000000) && (val <= 99999999))
	{
		int r = val/1000000;
		int mr = (val % 1000000) / 100000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%2d.%01d", r, mr);
		return;
	}
	if((val >= 1000000) && (val <= 9999999))
	{
		int r = val/1000000;
		int mr = (val % 1000000) / 10000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%d.%02d", r, mr);
		return;
	}
	if((val >= 100000) && (val <= 999999))
	{
		int mr = val/1000;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%4d", mr);
		return;
	}
	if((val >= 10000) && (val <= 99999))
	{
		int mr = val/1000;
		int fr_mr = (val % 1000) / 100;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%2d.%01d", mr, fr_mr);
		return;
	}
	if((val >= 1000) && (val <= 9999))
	{
		int mr = val/1000;
		int fr_mr = (val % 1000) / 10;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%d.%02d", mr, fr_mr);
		return;
	}
	if(val < 1000) 
	{
		xsprintf(valstr, "%4d", val);
		xsprintf(prefix, "%c", mju); 
		return;
	}
}

static char str[128];
static char tmpstr[32];
static char main_val_str[16];
static char sec_val_str_prefix[2];

static char battery = 'z' + 4;

static char plus_minus = 'z' + 2 ;


void update_display(char sound, char battery_charge, uint32_t main_val, char main_err, uint32_t fast_val, char fast_err)
{
	LCD5110_set_XY(0,0);

	// Set ticks indication
	if(sound)
	{
	    str[0] = 'z' + 5; 
		str[1] = '\0';
	}
	else
	{
	    str[0] = ' '; 
		str[1] = '\0';
	}

	// Set battery indication
	xsprintf(tmpstr, "        %c%3d%%                        ", battery, battery_charge);
	strcat(str, tmpstr);

    // Set range indication for main reading
    format_readings(tmpstr, main_val_str, main_val);
	strcat(str, tmpstr);

	// Place units and error marking
	xsprintf(tmpstr, "R/h          %c", plus_minus); 
	strcat(str, tmpstr);

	// Place main error 
	xsprintf(tmpstr, "%2d%%              ", main_err);
	strcat(str, tmpstr);

	// Set range indication for secondary reading 
	format_readings(sec_val_str_prefix, tmpstr, fast_val);
	strcat(str, tmpstr);
	strcat(str, sec_val_str_prefix);
	xsprintf(tmpstr, "R/h %c%2d%%", plus_minus, fast_err);
	strcat(str, tmpstr);

	LCD5110_write_string(str);

	LCD5110_num_string(main_val_str,9,10);
}
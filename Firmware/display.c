#include "driver_5110_lcd.h"
#include "xprintf.h"
#include <stdint.h>

static void strcat(char * dest, const char * src)
{
    while(*dest) dest++;
    while((*dest++ = *src++));
}

static void format_readings(char * prefix, char * valstr, uint32_t val) // Value in nR/h
{
    const char mju = 'z' + 3;
    if((val >= 4000000000U))                        // ---- !R/h
    {
        xsprintf(prefix, "!"); 
        xsprintf(valstr, "----");
        return;
    }

    if((val >= 1000000000U) && (val < 4000000000U)) // 1.64 mR/h
    {
        uint32_t r = val/1000000000U;
        uint32_t mr = (val % 1000000000U) / 10000000U;
        xsprintf(prefix, " "); 
        xsprintf(valstr, "%d.%02d", r, mr);
        return;
    }

    if((val >= 100000000U) && (val < 1000000000U))  // 897 mR/h
    {
        uint32_t mr = val/1000000U;
        xsprintf(prefix, "m"); 
        xsprintf(valstr, "%4d", mr);
        return;
    }

    if((val >= 10000000U) && (val < 100000000U))    // 46.2 mR/h
    {
        uint32_t mr = val/1000000U;
        uint32_t ur = (val % 1000000U) / 100000U;
        xsprintf(prefix, "m"); 
        xsprintf(valstr, "%2d.%01d", mr, ur);
        return;
    }

    if((val >= 1000000U) && (val < 10000000U))      // 7.94 mR/h
    {
        int mr = val/1000000U;
        int ur = (val % 1000000U) / 10000U;
        xsprintf(prefix, "m"); 
        xsprintf(valstr, "%d.%02d", mr, ur);
        return;
    }

    if((val >= 100000U) && (val < 1000000U))        // 395 uR/h
    {
        int ur = val/1000U;
        xsprintf(prefix, "%c", mju); 
        xsprintf(valstr, "%4d", ur);
        return;
    }

    if((val >= 10000U) && (val < 100000U))          // 78.4 uR/h
    {
        int ur = val/1000U;
        int fr_ur = (val % 1000U) / 100U;
        xsprintf(prefix, "%c", mju);  
        xsprintf(valstr, "%2d.%01d", ur, fr_ur);
        return;
    }

    if((val >= 1000U) && (val < 10000U))            // 1.24 uR/h
    {
        int ur = val/1000U;
        int fr_ur = (val % 1000U) / 10U;
        xsprintf(prefix, "%c", mju); 
        xsprintf(valstr, "%d.%02d", ur, fr_ur);
        return;
    }

    if((val < 1000U))                               // 751 nR/h
    {
        xsprintf(valstr, "%4d", val);
        xsprintf(prefix, "n"); 
        return;
    }
}

static char str[128];
static char tmpstr[32];
static char main_val_str[16];
static char sec_val_str_prefix[2];


void update_display(char sound, char battery_charge, uint32_t main_val, char main_err, uint32_t fast_val, char fast_err)
{
    const static char tick = 'z' + 5;
    const static char battery = 'z' + 4;
    const static char plus_minus = 'z' + 2;

    LCD5110_set_XY(0,0);

    // Set ticks indication
    if(sound)
    {
        str[0] = tick; 
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
    xsprintf(tmpstr, " R/h %c%2d%%", plus_minus, fast_err);
    strcat(str, tmpstr);

    LCD5110_write_string(str);

    LCD5110_num_string(main_val_str,9,10);
}

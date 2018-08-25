#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>

void update_display(uint8_t sound, uint8_t battery_charge, uint32_t main_val, char main_err, uint32_t fast_val, char fast_err);

#endif

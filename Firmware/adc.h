#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

//Initialize internal ADC for ambient light measurements
void adc1_init(void);
uint16_t get_adc1_value(void);

void adc2_init(void);
uint16_t get_adc2_value(void);

void vcc_voltage_monitor_init(void);

uint16_t vcc_voltage(void); // in millivolts

#endif

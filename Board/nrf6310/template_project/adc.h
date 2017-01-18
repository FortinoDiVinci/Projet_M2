#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define PIN_ADC 26
#define PIN_ADC_ON 24

void init_adc();
uint8_t start_sampling();

#endif


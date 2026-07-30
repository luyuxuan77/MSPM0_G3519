#ifndef ADC0_H
#define ADC0_H
#include "bsp.h"

#define ANGLE_ADC_ZERO_RAW          2048
#define ANGLE_ADC_CDEG_PER_COUNT    1

void adc0_init(void);
void get_adc0_num_val(uint32_t *adc0);
uint16_t adc0_read_angle_raw(void);
int16_t angle_adc_to_cdeg(uint16_t raw);

#endif

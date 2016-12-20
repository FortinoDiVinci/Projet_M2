
#include "adc.h"
#include "nrf_gpio.h"
#include "nrf_gpio.h"

void init_adc()
{
  nrf_gpio_cfg_input(PIN_ADC,NRF_GPIO_PIN_NOPULL);
}

void start_sampling()
{
  
}
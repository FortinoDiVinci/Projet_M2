#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include <initialization.h>




 void gpiote_init(void)
{
//  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  nrf_gpio_cfg_input(BUTTON,NRF_GPIO_PIN_PULLUP);
  // Configure GPIOTE channel BUUTTON to generate event when MOTION_INTERRUPT_PIN_NUMBER goes from Low to High
  nrf_gpio_cfg_input(BUTTON,NRF_GPIO_PIN_PULLUP);
  nrf_gpiote_event_config(0, BUTTON, NRF_GPIOTE_POLARITY_LOTOHI);

  // Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
}
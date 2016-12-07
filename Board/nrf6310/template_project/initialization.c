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

  nrf_gpio_cfg_input(6,NRF_GPIO_PIN_NOPULL);
  // Configure GPIOTE channel BUUTTON to generate event when MOTION_INTERRUPT_PIN_NUMBER goes from Low to High
  
  nrf_gpiote_event_config(0, 6, NRF_GPIOTE_POLARITY_LOTOHI);

  // Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
}

void timerVib_init()
{
  NRF_TIMER0->TASKS_STOP=1;
  NRF_TIMER0->PRESCALER=0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER0->MODE=TIMER_MODE_MODE_Timer<< TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER0->BITMODE=TIMER_BITMODE_BITMODE_16Bit<< TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER0->CC[0]=0x1E85; //500 ms period
  NRF_TIMER0->INTENSET=TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER0->TASKS_CLEAR = 1;
  NRF_TIMER0->EVENTS_COMPARE[0]=0;
  NRF_TIMER0->SHORTS=TIMER_SHORTS_COMPARE0_CLEAR_Enabled<< TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
}

void timerSPI_init()
{
  NRF_TIMER1->TASKS_STOP=1;
  NRF_TIMER1->PRESCALER=0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER1->MODE=TIMER_MODE_MODE_Timer<< TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER1->BITMODE=TIMER_BITMODE_BITMODE_16Bit<< TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER1->CC[0]=0x139; // 20 ms period
  NRF_TIMER1->INTENSET=TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->EVENTS_COMPARE[0]=0;
  NRF_TIMER1->SHORTS=TIMER_SHORTS_COMPARE0_CLEAR_Enabled<< TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
}
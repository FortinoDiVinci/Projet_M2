/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example template
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example template.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "initialization.h"

#include "spi_master.h"
#include "common.h"
#include "spi_master_config.h"

#define MAX_LENGTH_SAMPLE 10
#define INC 4
#define DEC 1
#define THRESH 50
#define MAX 150
//static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
//static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */
uint8_t pulse_count = 0, sample_count = 1;
uint8_t start = 0;
int32_t x_acceleration=0,y_acceleration=0,z_acceleration=0;

static int16_t x_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration x samples*/
static int16_t y_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration y samples */
static int16_t z_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration z samples */

/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard. 
 */

void GPIOTE_IRQHandler(void)
{
  
  
  if (pulse_count == 0)
  {
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START=1;
    pulse_count += INC;
  }
  else if (pulse_count < MAX-INC)
  {
    pulse_count += INC;
  }
    
  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_IN[0] = 0;
  NVIC_DisableIRQ(GPIOTE_IRQn);
  
}

void TIMER0_IRQHandler(void)
{
  nrf_gpio_pin_toggle(LED);
  
  NVIC_EnableIRQ(GPIOTE_IRQn);
  if (pulse_count  <= DEC)
  {
      NVIC_DisableIRQ(TIMER0_IRQn);
//     NRF_TIMER0->TASKS_CLEAR = 1;
      pulse_count = 0;
  }
  else if (pulse_count > DEC)
  {
    pulse_count -= DEC;
  }
  if(pulse_count > THRESH)
  {
    nrf_gpio_pin_set(LED2);
    start =1;
  }
  else 
  {
    nrf_gpio_pin_clear(LED2);
    start = 0;
  }
  if((NRF_TIMER0->EVENTS_COMPARE[0]==1) && (NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
  {
    NRF_TIMER0->EVENTS_COMPARE[0]=0;
    //NRF_TIMER0->TASKS_START=1;
  }
}
void TIMER1_IRQHandler(void)
{
 
  read_ac_value(&x_acc_samples[sample_count],&y_acc_samples[sample_count],&z_acc_samples[sample_count]);
  sample_count += 1;
  
  if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
  {
    NRF_TIMER1->EVENTS_COMPARE[0]=0;
    //NRF_TIMER0->TASKS_START=1;
  }
}

int main(void)
{
/** GPIOTE interrupt handler.
* Triggered on motion interrupt pin input low-to-high transition.
*/
  
  
  gpiote_init();
  timerVib_init();
  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
  timerSPI_init();
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  nrf_gpio_cfg_output(LED2);
  nrf_gpio_cfg_output(LED);
  
  gpiote_init();
  NVIC_EnableIRQ(GPIOTE_IRQn);
//   // SPI0
 
  
    write_data(0x3F,0X10);  // set accelrometre (get mesure : 52 hz; scall:+-16g filter :50hz)
    //write_data(0x33,0x10);     // set accelerometre (get mesure: 52hz scall:+-2g filter :50hz)
    read_data(0x10);        // check value 
    write_data(0x10,0x15);  // disable high-performance mode for accelerometre 
    while (true)
    {
      
      if( start == 1)
      { 
        NRF_TIMER1->TASKS_START=1;
        NVIC_EnableIRQ(TIMER1_IRQn);
        while(start == 1)
        {
          __WFI();
          if(sample_count == MAX_LENGTH_SAMPLE)
          {
            x_acceleration=0,y_acceleration=0,z_acceleration=0;
            for (uint8_t i =0; i< MAX_LENGTH_SAMPLE; i++)
            {
               x_acceleration += x_acc_samples[i];
               y_acceleration += y_acc_samples[i];
               z_acceleration += z_acc_samples[i];
            }
            x_acceleration = x_acceleration/MAX_LENGTH_SAMPLE;
            y_acceleration = y_acceleration/MAX_LENGTH_SAMPLE;
            z_acceleration = z_acceleration/MAX_LENGTH_SAMPLE;
            sample_count = 1;
          }
        }
      }
      else 
      {
        NVIC_DisableIRQ(TIMER1_IRQn);
       __WFI();
      }
    }
}

/**
 *@}
 **/



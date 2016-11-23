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

static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */

static bool test_spi_tx_rx(SPIModuleNumber mod_num, uint8_t lsb_first)
{
  // Use SPI0, mode0 with lsb shifted as requested
  uint32_t *spi_base_address = spi_master_init(mod_num, SPI_MODE0, (bool)lsb_first);
  if (spi_base_address == 0)
  {
    return false;
  }

/** @def debug_init
 * If debug is enabled @ref DEBUG, then this function will configure @ref DEBUG_EVENT_READY_PIN to toggle (using GPIOTE) everytime
 * READY_EVENTS are generated in the SPI
 * @note This flag will configure GPIOTE CONFIG0 and PPI channel 0, do not enable DEBUG while using two spi modules in parallel
 */
#ifdef DEBUG
    *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals


    if ( NRF_SPI0_BASE == (uint32_t)spi_base_address )
    {
        nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN0);

        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                                (DEBUG_EVENT_READY_PIN0 << GPIOTE_CONFIG_PSEL_Pos) |
                                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

        NRF_PPI->CH[0].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
        NRF_PPI->CHEN |= (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
    }

    if ( NRF_SPI1_BASE == (uint32_t)spi_base_address )
    {
        nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN1);

        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                                (DEBUG_EVENT_READY_PIN1 << GPIOTE_CONFIG_PSEL_Pos) |
                                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

        NRF_PPI->CH[1].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
        NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
        NRF_PPI->CHEN |= (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
    }
#endif /* DEBUG */

  // Fill tx_data with some simple pattern, rx is filled with zero's so that after receiving from
  // slave we verify rx_Data is same as tx_data
    
   tx_data[0] = 0x8F;
   tx_data[1]=0;
   rx_data[0] = 0;
   rx_data[1]=0;

  // Transmit TX_RX_MSG_LENGTH bytes from tx_data and receive same number of bytes and data into rx_data
  if(!spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data) )
    return false;

  // Validate that we got all transmitted bytes back in the exact order
  for(uint8_t i = 0; i < TX_RX_MSG_LENGTH; i++)
  {
    if( tx_data[i] != rx_data[i] )
      return false;
  }
  return true;
}



/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard. 
 */

void timer_init();

void GPIOTE_IRQHandler(void)
{
  nrf_gpio_pin_toggle(LED);
  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_IN[0] = 0;
}

void TIMER0_IRQHandler(void)
{
  if((NRF_TIMER0->EVENTS_COMPARE[0]==1) && (NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
  {
    NRF_TIMER0->EVENTS_COMPARE[0]=0;
    //NRF_TIMER0->TASKS_START=1;
  }
}

int main(void)
{
/** GPIOTE interrupt handler.
* Triggered on motion interrupt pin input low-to-high transition.
*/

  gpiote_init();
  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
  NVIC_EnableIRQ(GPIOTE_IRQn);
  nrf_gpio_cfg_output(LED2);
  nrf_gpio_cfg_output(LED);
  
  timer_init();
  
  NVIC_EnableIRQ(TIMER0_IRQn);
  
   bool ret0;

  NRF_GPIO->DIRSET = (1UL << ERROR_PIN_SPI0);
  
   // SPI0
   ret0 = test_spi_tx_rx(SPI0, 1 );   /*!< test with shift Lsb first mode 0 */

   if (!ret0)
   {
     // Set gpio pin number ERROR_PIN to convey error, this pin can be connected to LED for visual debug
     NRF_GPIO->OUTSET = (1UL << ERROR_PIN_SPI0);
   }

    while (true)
    {
      //__WFI();
    }
}

/**
 *@}
 **/

void timer_init()
{
  NRF_TIMER0->TASKS_STOP=1;
  NRF_TIMER0->PRESCALER=0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER0->MODE=TIMER_MODE_MODE_Timer<< TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER0->BITMODE=TIMER_BITMODE_BITMODE_16Bit<< TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER0->CC[0]=0x2FFF;
  NRF_TIMER0->INTENSET=TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER0->TASKS_CLEAR = 1;
  NRF_TIMER0->EVENTS_COMPARE[0]=0;
  NRF_TIMER0->SHORTS=TIMER_SHORTS_COMPARE0_CLEAR_Enabled<< TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  NRF_TIMER0->TASKS_START=1; 
}


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
static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */
uint8_t pulse_count = 0, sample_count = 1;
uint8_t start = 0;
int32_t x_acceleration=0,y_acceleration=0,z_acceleration=0;

static int16_t x_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration x samples*/
static int16_t y_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration y samples */
static int16_t z_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration z samples */

/* The function write_data set the register */
bool write_data(uint8_t data, uint8_t adress )
{
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0,0);
  if (spi_base_address == 0)
  {
    return false;
  }
  tx_data[0]=adress&0x7F; // tranmit adress with MSB set to 0 must be changed for power consumption
  tx_data[1]=data;
  
  if(!spi_master_tx_rx(spi_base_address, 2, (const uint8_t *)tx_data, rx_data) )
    return false;
  return true;
}

//bool init_fifo()
//{
//   
//}

bool read_data(uint8_t adress)
{
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0,0);
  if (spi_base_address == 0)
  {
    return false;
  }
  tx_data[0]=adress|0x80; // tranmit adress with MSB set to 1 must be changed for power consumption
  tx_data[1]=0;
  
  if(!spi_master_tx_rx(spi_base_address, 2 , (const uint8_t *)tx_data, rx_data) )
    return false;
  return true;
}

bool read_ac_value(int16_t* x_acceleration,int16_t* y_acceleration,int16_t* z_acceleration)
{
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0, 0);
  if (spi_base_address == 0)
  {
    return false;
  }
  tx_data[0]=0xA8;
  tx_data[1]=0;
  if(!spi_master_tx_rx(spi_base_address, 7 , (const uint8_t *)tx_data, rx_data) )
    return false;
  
  *x_acceleration=(((int16_t)rx_data[2])<<8)+(int16_t)rx_data[1];
  *y_acceleration=(((int16_t)rx_data[4])<<8)+(int16_t)rx_data[3];
  *z_acceleration=(((int16_t)rx_data[6])<<8)+(int16_t)rx_data[5];
  
  return true;
}
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

void timerVib_init();
void timerSPI_init();
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
  bool ret0;
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
  
  ret0 = test_spi_tx_rx(SPI0, 1 );   /*!< test with shift Lsb first mode 0 */
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

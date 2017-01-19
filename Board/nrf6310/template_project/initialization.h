
#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#define DELAY_MS               100        /*!< Timer Delay in milli-seconds */
#define LED 18
#define LED2 19
#define BUTTON 17
#define PIN_BUCK 0
#define DEBEUG_PIN 2


void gpiote_init(void);
void timerVib_init();
void timerSPI_init();
void timerADC_init();

#endif


#include "stdio.h"
#include "stdlib.h"
#include "MK64F12.h"
#include "uart_drvr.h"
#include "gpio_drvr.h"
#include <string.h>
#include <math.h>

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#ifndef DEFAULT_SYSTEM_CLOCK
#define DEFAULT_SYSTEM_CLOCK 20485760u
#endif

void initialize(void);
void en_interrupts(void);
void delay(int del);

int main(void)
{  
	// Initialize UART
	initialize();
	Blue_LED(LED_ON);
  
  for(;;)
  {
    if(SW2_pressed())
    {
      Blue_LED(LED_TOGGLE);
      Red_LED(LED_ON);
      put("SW2 Pressed!\n\r");
      delay(50);
      Red_LED(LED_OFF);
      Blue_LED(LED_TOGGLE);
    }
    else if(SW3_pressed())
    {
      Blue_LED(LED_TOGGLE);
      Green_LED(LED_ON);
      put("SW3 Pressed!\n\r");
      delay(50);
      Green_LED(LED_OFF);
      Blue_LED(LED_TOGGLE);
    }
  }
  return 0;
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void initialize()
{
	// Initialize UART
	uart_init();
  LED_init();
  SW2_init();
  SW3_init();
}

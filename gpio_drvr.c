#include "MK64F12.h"
#include "gpio_drvr.h"

void LED_init(){
    // Enable clocks on Ports B and E for LED timing
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure the Signal Multiplexer for GPIO
    PORTB_PCR22 = PORT_PCR_MUX(1);
    PORTE_PCR26 = PORT_PCR_MUX(1);
    PORTB_PCR21 = PORT_PCR_MUX(1);	
	
    // Switch the GPIO pins to output mode
    GPIOB_PDDR = (1UL << 22) | (1UL << 21);
    GPIOE_PDDR = (1UL << 26);

    // Turn off the LEDs
    GPIOB_PSOR = (1UL << 22) | (1UL << 21);
    GPIOE_PSOR = (1UL << 26);
}

void SW2_init(){
	// Enable clock for Port C PTC6 button
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; 
	
	// Configure the Mux for the button
	PORTC_PCR6 = PORT_PCR_MUX(1);

	// Set the push button as an input
	GPIOC_PDDR &= ~(1UL << 6);
}

void SW3_init(){
  // Enable clock for Port A PTA4 button
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  
  // Configure the Mux for the button
  PORTA_PCR4 = PORT_PCR_MUX(1);
  
  // Set the button as an input
  GPIOA_PDDR &= ~(1UL << 4);
}

void FXOS8700CQ_INT_init()
{
  /*Initialize GPIO pin for receiving interrupts*/
  
  //Enable clock for Port C pin PTC13
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
  
  //Configure pin mux for PTC13
  PORTC_PCR13 = PORT_PCR_MUX(1); //INT2
  //PORTC_PCR6 = PORT_PCR_MUX(1); //INT1
  
  //Since the default INT2 operation is push-pull active low,
  //Configure PTC13 pin for falling edge interrupts
  PORTC_PCR13 |= PORT_PCR_IRQC(0xA) | PORT_PCR_ISF_MASK;
  //PORTC_PCR6 |= PORT_PCR_IRQC(0xA) | PORT_PCR_ISF_MASK;
  
  //Enable NVIC interrupt
  NVIC_EnableIRQ(PORTC_IRQn);
}

void Red_LED(int op)
{
  switch(op)
  {
    case LED_OFF:
      GPIOB_PSOR = (1UL << 22);	// Turn off red LED
      break;
    case LED_ON:
      GPIOB_PCOR = (1UL << 22); // Turn on red LED
      break;
    case LED_TOGGLE:
      GPIOB_PTOR = (1UL << 22); // Toggle red LED
      break;
  }
}

void Green_LED(int op)
{
  switch(op)
  {
    case LED_OFF:
      GPIOE_PSOR = (1UL << 26);	// Turn off green LED
      break;
    case LED_ON:
      GPIOE_PCOR = (1UL << 26); // Turn on green LED
      break;
    case LED_TOGGLE:
      GPIOE_PTOR = (1UL << 26); // Toggle green LED
      break;
  }
}

void Blue_LED(int op)
{
  switch(op)
  {
    case LED_OFF:
      GPIOB_PSOR = (1UL << 21);	// Turn off blue LED
      break;
    case LED_ON:
      GPIOB_PCOR = (1UL << 21); // Turn on blue LED
      break;
    case LED_TOGGLE:
      GPIOB_PTOR = (1UL << 21); // Toggle blue LED
      break;
  }
}

int SW2_pressed()
{
  return ((GPIOC_PDIR & (1UL << 6)) == 0);
}

int SW3_pressed()
{
  return ((GPIOA_PDIR & (1UL << 4)) == 0);
}

void PORTC_IRQHandler()
{
  PORTC_PCR13 |= PORT_PCR_ISF_MASK;
  DataReady = 1;
}

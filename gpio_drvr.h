#ifndef  GPIO_H
#define  GPIO_H

#include "MK64F12.h"

#define LED_OFF 0
#define LED_ON 1
#define LED_TOGGLE 2

void LED_init(void);
void SW2_init(void);
void SW3_init(void);
void FXOS8700CQ_INT_init(void);
void Red_LED(int op);
void Green_LED(int op);
void Blue_LED(int op);
int SW2_pressed(void);
int SW3_pressed(void);

//Global data valid flag
extern uint8_t DataReady;
#endif
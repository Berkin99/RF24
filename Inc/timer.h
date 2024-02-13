#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"

void timerInit(void);

uint32_t millis(void);
uint32_t micros(void);

void delay(uint32_t ms);
void delay_us(uint32_t us);

#endif /* TIMER_H_ */

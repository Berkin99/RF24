
#include <stdint.h>
#include "timer.h"
#include "debug.h"

TIM_HandleTypeDef *_htim;
volatile uint32_t time_ms = 0;
volatile uint32_t time_us = 0;

void timerInit(void)
{
#ifdef TIMER_H
	_htim = &TIMER_H;
	HAL_TIM_Base_Start_IT(_htim);
#endif
}

#ifdef TIMER_H

uint32_t millis(void){return time_ms;}

uint32_t micros(void){
	time_us = _htim->Instance->CNT;
	time_us += (time_ms*1000);
	return time_us;
}

void delay(uint32_t ms){
	uint32_t dt = millis() + ms;
	while (millis() < dt);
}

void delay_us(uint32_t us){
	uint32_t dt = micros() + us;
	while (micros() < dt);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == _htim->Instance) time_ms++;
}

#endif



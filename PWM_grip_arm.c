/*
 * PWM_grip_arm.c
 *
 * Created: 3/24/2017 11:18:58 AM
 *  Author: guswe541
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include "PWM_grip_arm.h"

void Timer2_init()
{
	TCCR2A |= (1 << WGM20) | (1 << COM2B1) | (1 << WGM21); // |(1<<COM2A1); fast PWM-mode, MAX = TOP
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // clock select 8 prescaler for 128000 kHz
}

void Open_grip_arm()
{
	OCR2B = 29; // 2 ms with 128000 kHz
}

void Center_grip_arm()
{
	OCR2B = 22; // 1.5 ms with 128000 kHz
}

void Close_grip_arm()
{
	OCR2B = 14; //15 1 ms with 128000 kHz
}

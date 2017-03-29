/*
 * TSEA56ROBOT.c
 *
 * Created: 3/24/2017 11:18:58 AM
 *  Author: guswe541
 */ 

#define F_CPU 128000UL

#include <avr/io.h>
#include <util/delay.h>
#include "PWM_grip_arm.h"

void Timer2_init()
{
	TCCR2A |= (1<<WGM20)|(1<<COM2B1)|(1<<WGM21); // |(1<<COM2A1); fast PWM-mode, MAX = TOP
	TCCR2B |= (1<<CS21); // clock select 8 prescaler for 128000 kHz
}

void Open_grip_arm()
{
	OCR2B = 30; // 2 ms with 128000 kHz
}

void Center_grip_arm()
{
	OCR2B = 22; // 1.5 ms with 128000 kHz
}

void Close_grip_arm()
{
	OCR2B = 15; //15 1 ms with 128000 kHz
}

int main()
{
	DDRD |= (1<<PORTD6); // OSC2A and OSC2B as outputs.
	CLKPR |= (1<<CLKPCE); // prescaler enabled
	Timer2_init(); // Timer initialization.
	
	Center_grip_arm();
	while(1)
	{
		// Infinite loop.		
		Open_grip_arm();
		_delay_ms(2000);
		Close_grip_arm();
		_delay_ms(2000);	
	}
}
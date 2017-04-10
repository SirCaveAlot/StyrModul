/*
 * LIDAR_PWM.c
 *
 * Created: 4/10/2017 9:00:56 AM
 *  Author: guswe541
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include "PWM_SirCave.h"

/*
void Timer2_init()
{
	TCCR2A |= (1 << WGM20) | (1 << COM2B1) | (1 << WGM21) | (1 << COM2A1); //fast PWM-mode, MAX = TOP
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // clock select 8 prescaler for 128000 kHz
}*/

void LIDAR_on()
{
	OCR2A = 128;
}

void LIDAR_off()
{
	OCR2A = 0;
}

int main(void)
{
	Timer2_init();
	DDRD |= (1<<PORTD6)|(1<<PORTD7); // OC2A  output.
	//OCR2A = 50;
	//OCR2B = 50;
    while(1)
    {
		LIDAR_on();
		_delay_ms(1000);
		LIDAR_off();
		_delay_ms(1000);
        //TODO:: Please write your application code 
    }
}
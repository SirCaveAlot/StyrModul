/*
 * TSEA56ROBOT.c
 *
 * Created: 3/24/2017 11:18:58 AM
 *  Author: guswe541
 */ 


#include <avr/io.h>
//#include <util/delay.h>

void Timer2_init()
{
	TCCR2A |= (1<<WGM20)|(1<<COM2B1)|(1<<WGM21); // |(1<<COM2A1); fast PWM-mode, MAX = TOP
	TCCR2B |= (1<<CS21) | (1 << CS20); //clock select 32 prescaler (Change later)
}

void Grip_power(float grip) // controls the grip power. 0 < grip < 1.
{
	OCR2B = grip*255;
}

int main()
{
	DDRD |= (1<<PORTD6); // |(1<<PORTD7); // OSC2A and OSC2B as outputs.

	Timer2_init(); // Timer initialization.
	
	Grip_power(0.02);
	//OCR2A = 127; PWM for LIDAR

	while(1)
	{
		// Infinite loop.
	}
}
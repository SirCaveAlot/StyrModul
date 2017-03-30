/*
 * PWM_SirCave.c
 *
 * Created: 3/27/2017 9:27:25 AM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>

void Timer1_init()
{
	CLKPCR |= (1 << CLKPCE); // Prescaler enabled.
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Intern clock, prescaler 8.
	TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // set fast PWM-mode and non-inverted mode
	
	ICR1 = 36863; // Top-value PWM. (2559 128kHz)
}

void Set_speed_right(float velocity_right)
{
	OCR1B = velocity_right*ICR1;
}

void Set_speed_left(float velocity_left)
{
	OCR1A = velocity_left*ICR1;
}

void Drive_forward(float velocity_left, float velocity_right)
{
	PORTA = (1 << PORTA0) | (1 << PORTA1); // Forward direction on both sides.
	Set_speed_left(velocity_left);
	Set_speed_right(velocity_right); // Same velocity on both sides.
}

void Drive_backwards(float velocity_left, float velocity_right)
{
	PORTA = (0 << PORTA0) | (0 << PORTA1); // Forward direction on both sides.
	Set_speed_left(velocity_left);
	Set_speed_right(velocity_right); // Same velocity on both sides.
}

void Rotate_clockwise(float velocity_left, float velocity_right)
{
	PORTA = (1 << PORTA0) | (0 << PORTA1); // Forward direction on the left side and backwards on the right side.
	Set_speed_left(velocity_left);
	Set_speed_right(velocity_right);	
}

void Rotate_counter_clockwise(float velocity_left, float velocity_right)
{
	PORTA = (0 << PORTA0) | (1 << PORTA1); // Forward direction on the right side and backwards on the left side.
	Set_speed_left(velocity_left);
	Set_speed_right(velocity_right);
}

int main(void)
{
	DDRD = 0b11000000;
	DDRA = 0xFF;	
	
	Timer1_init();
	
    while(1)
    {
        Drive_forward(0.7, 0.7);
		_delay_ms(1000);
		Drive_backwards(0.9, 0.9);
		_delay_ms(1000);
		Rotate_clockwise(0.5, 0.5);
		_delay_ms(1000);
		Rotate_counter_clockwise(0.8, 0.8);
		_delay_ms(1000);
		//TODO:: Please write your application code 
    }
}
/*
 * PWM_SirCave.c
 *
 * Created: 3/27/2017 9:27:25 AM
 *  Author: gusst967
 */ 


#include <avr/io.h>
//#include <avr/interrupt.h>

void timer1_init()
{
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Intern clock no prescaling (Change later)
	TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // set fast pwm-mode and non-inverted mode
	
	ICR1 = 2559; // Top-value PWM. 320000 with extern xtal
}

void set_speed_right(float velocity_right)
{
	OCR1B = velocity_right*ICR1;
}

void set_speed_left(float velocity_left)
{
	OCR1A = velocity_left*ICR1;
}

void drive_forward(float velocity)
{
	PORTA = (1 << PORTA0) | (1 << PORTA1); // Forward direction on both sides.
	set_speed_left(velocity);
	set_speed_right(velocity); // Same velocity on both sides.
}

void drive_backwards(float velocity)
{
	PORTA = (0 << PORTA0) | (0 << PORTA1); // Forward direction on both sides.
	set_speed_left(velocity);
	set_speed_right(velocity); // Same velocity on both sides.
}

void rotate_clockwise(float velocity)
{
	PORTA = (1 << PORTA0) | (0 << PORTA1); // Forward direction on the left side and backwards on the right side.
	set_speed_left(velocity);
	set_speed_right(velocity); // Same velocity on both sides.	
}

void rotate_counter_clockwise(float velocity)
{
	PORTA = (0 << PORTA0) | (1 << PORTA1); // Forward direction on the right side and backwards on the left side.
	set_speed_left(velocity);
	set_speed_right(velocity); // Same velocity on both sides.
}

int main(void)
{
	DDRD = 0xFF;
	DDRA = 0xFF;	
	
	timer1_init();
	
    while(1)
    {
        //TODO:: Please write your application code 
    }
}
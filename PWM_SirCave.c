/*
 * PWM_SirCave.c
 *
 * Created: 3/27/2017 
 * Author: Gustav Strandberg, gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include "PWM_SirCave.h"


//------------------------------Wheels-----------------------------------------

void Timer1_init()
{
	// Prescaler enabled.
	CLKPR |= (1 << CLKPCE); 
	// Fast PWM, ICR1 = TOP, prescaler 64.
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);  
	// Set fast PWM-mode, ICR1 = TOP, non-inverting mode.
	TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); 
	
	ICR1 = 4607; // Top-value PWM.
}

// Sets the speed on the left side to 0 < velocity < 1.
void Set_speed_left(float velocity)  
{
	OCR1B = velocity*ICR1;
}

// Sets the speed on the right side to 0 < velocity < 1.
void Set_speed_right(float velocity) 
{
	OCR1A = velocity*ICR1;
}

void Drive_forward(float velocity_left, float velocity_right)
{
	PORTA = (1 << PORTA0) | (1 << PORTA1); // Forward direction on both sides.
	Set_speed_left(velocity_left); // Velocity_left on left side.
	Set_speed_right(velocity_right); // Velocity_right on right side
}

void Drive_backwards(float velocity_left, float velocity_right)
{
	PORTA = (0 << PORTA0) | (0 << PORTA1); // Forward direction on both sides.
	Set_speed_left(velocity_left); // Velocity_left on left side
	Set_speed_right(velocity_right); // Velocity_right on right side
}

void Stop_motors()
{
	Set_speed_right(0);
	Set_speed_left(0);
}

void Rotate_clockwise(float velocity_left, float velocity_right)
{
	// Forward direction on the left side and backwards on the right side.
	PORTA = (0 << PORTA0) | (1 << PORTA1); 
	Set_speed_left(velocity_left); // Velocity_left on left side.
	Set_speed_right(velocity_right); // Velocity_right on right side.
}

void Rotate_counter_clockwise(float velocity_left, float velocity_right)
{
	// Forward direction on the right side and backwards on the left side.
	PORTA = (1 << PORTA0) | (0 << PORTA1); 
	Set_speed_left(velocity_left); // Velocity_left on left side.
	Set_speed_right(velocity_right); // Velocity_right on right side.
}

//-------------------------Grip arm and LIDAR tower----------------------------

void Timer2_init()
{
	// Fast PWM-mode, MAX = TOP
	TCCR2A |= (1 << WGM20) | (1 << COM2B1) | (1 << WGM21) | (1 << COM2A1); 
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024.
}

void Open_grip_arm()
{
	OCR2B = 26; // 2 ms 
}

void Center_grip_arm()
{
	OCR2B = 22; // 1.5 ms 
}

void Close_grip_arm()
{
	OCR2B = 17; // 1 ms 
}

void Rotate_LIDAR(float LIDAR_speed)
{
	OCR2A = LIDAR_speed * 255;
}

void Stop_LIDAR()
{
	Rotate_LIDAR(0);
}
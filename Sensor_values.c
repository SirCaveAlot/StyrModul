/*
 * Sensor_values.c
 *
 * Created: 4/20/2017 3:32:28 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>

uint16_t left_distance;
uint16_t right_distance;
uint8_t angle;
uint8_t gyro_rotation_speed;
uint8_t forward_distance;
uint8_t velocity;
uint8_t travelled_distance;
uint8_t LIDAR_angle;
uint8_t LIDAR_rotation_speed;
bool autonomous;

uint16_t left_IR_array[2] = {0, 0};
uint16_t right_IR_array[2] = {0, 0};
uint16_t LIDAR_array[2] = {0, 0};

//--------------------------Timer 0--------------------------------

// ISR(TIM0_OVF_vect)
// {
// 	
// }
// 
// void Timer0_init()
// {
// 	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); // fast PWM-mode, MAX = TOP
// 	TCCR0B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // clock select 8 prescaler for 128000 kHz
// }
  
//-------------------------IR sensors------------------------------

void IR_conversion(bool right, uint8_t IR_value)
{
	uint16_t distance = 10 * (55.25 * exp(-0.05762 * IR_value)) + (14.2 * exp(-0.009759 * IR_value));
	
	if(right)
	{
		right_distance = distance;
	}
	else
	{
		left_distance = distance;
	}	
}

bool Left_side_detectable()
{
	if(left_distance > 400)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Right_side_detectable()
{
	if(right_distance > 400)
	{
		return false;
	}
	else
	{
		return true;
	}
}

// void Store_in_IR_array(bool right, uint8_t IR_value)
// {
// 	if(right) // If true, use right IR array
// 	{
// 		right_IR_array[0] = right_IR_array[0] + IR_conversion(right, IR_value);
// 		right_IR_array[1] = right_IR_array[1] + 1;
// 	}
// 	else
// 	{
// 		left_IR_array[0] = left_IR_array[0] + IR_conversion(right, IR_value);
// 		left_IR_array[1] = left_IR_array[1] + 1;
// 	}
// }

//---------------------------Gyro----------------------------------



//--------------------------LIDAR----------------------------------


//--------------------------Wheels---------------------------------


//-------------------------IR sensors------------------------------


//-------------------------IR sensors------------------------------
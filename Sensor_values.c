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
#include <math.h>
#include <avr/interrupt.h>
#include "Sensor_values.h"

#define WHEEL_DIAMETER 63.34
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define WHEEL_SLICE (WHEEL_CIRCUMFERENCE/16)

int left_distance;
int right_distance;
bool left_side_detected;
bool right_side_detected;
uint8_t angle;
uint8_t gyro_rotation_speed;
float front_distance;
float travel_distance;
uint16_t wheel_sensor_counter;
uint8_t standing_still_counter;
uint8_t velocity;
uint8_t LIDAR_angle;
uint8_t LIDAR_rotation_speed;
uint8_t LIDAR_rotation_turns;
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
	int distance = floor(10 * ((81.42 * exp(-0.0435 * IR_value)) + (25.63 * exp(-0.007169 * IR_value))));
	if(right)
	{
		right_distance = distance;
	}
	else
	{
		left_distance = distance;
	}	
}

void Left_side_detectable()
{
	if(left_distance > 300)
	{
		left_side_detected = false;
	}
	else
	{
		left_side_detected = true;
	}
}

void Right_side_detectable()
{
	if(right_distance > 300)
	{
		right_side_detected = false;
	}
	else
	{
		right_side_detected = true;
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

void Set_front_distance(uint16_t distance)
{
	front_distance = distance;
}

void Distance_calculation(uint8_t data) //Calculates the distance
{
	Calculate_wheel_sensor_counter(data);
	Distance_travelled();
	front_distance -= travel_distance;
}

void Distance_travelled() //Calculates the travelled distance
{
	travel_distance = wheel_sensor_counter * WHEEL_SLICE; 
}

void Calculate_wheel_sensor_counter(uint8_t data) //Calculates the wheel sensor and standing still counter
{	
	if(data == 0x01)
	{
		standing_still_counter = 0;
		wheel_sensor_counter++; 
	}
	else if(data == 0x00)
	{
		standing_still_counter++;
	}
}

bool Standing_still() // Returns true if the robot is standing still
{
	if(standing_still_counter > 10 && front_distance < 250)
	{
		wheel_sensor_counter = 0;
		standing_still_counter = 0;
		return true;
	}
	return false;
}

//--------------------------Magnets--------------------------------

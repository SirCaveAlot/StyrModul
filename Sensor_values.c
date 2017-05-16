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
#include "UART.h"
#include "Modes.h"

#define WHEEL_DIAMETER 63.34
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define WHEEL_SLICE (WHEEL_CIRCUMFERENCE/16)
#define GYRO_OFFSET 1230
#define ROTATION_DISTANCE 2221.44 // Distance when rotating 90 degrees, 0.1 mm

int left_distance;
int right_distance;
int forward_IR_distance;
bool left_side_detected;
bool right_side_detected;
bool forward_IR_detected;

uint16_t angle;
uint16_t angle_to_rotate;
uint16_t gyro_rotation_speed;

int32_t distance_until_stop;
int32_t stop_distance;
int32_t travel_distance;
uint16_t wheel_sensor_counter;
uint8_t standing_still_counter;
uint8_t velocity;
uint8_t LIDAR_angle;
uint8_t LIDAR_rotation_speed;
uint8_t LIDAR_rotated_turns;
uint8_t LIDAR_turns;
bool line_detected;
float iteration_time = 0.02; // 20 ms

uint16_t left_IR_array[2] = {0, 0};
uint16_t right_IR_array[2] = {0, 0};
uint16_t LIDAR_array[2] = {0, 0};

//--------------------------Timer 0--------------------------------
  
//-------------------------IR sensors------------------------------

void IR_conversion(char direction, uint8_t IR_value)
{
	uint16_t distance = floor(10 * ((81.42 * exp(-0.0435 * IR_value)) + (25.63 * exp(-0.007169 * IR_value))));
	if(direction == 'r')
	{
		right_distance = distance;
	}
	else if(direction == 'l')
	{
		left_distance = distance;
	}
	else if(direction == 'f')
	{
		distance = floor(100 * ((81.42 * exp(-0.0435 * IR_value)) + (25.63 * exp(-0.007169 * IR_value))));
		forward_IR_distance = distance;
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

void Forward_IR_detectable()
{
	if(forward_IR_distance > 3000)
	{
		forward_IR_detected = false;
	}
	else
	{
		forward_IR_detected = true;
	}
}

//---------------------------Gyro----------------------------------

void Gyro_calculation(uint16_t gyro_data)
{
	if(mode == 'l')
	{
		gyro_rotation_speed = GYRO_OFFSET - gyro_data;
	}
	else if(mode == 'r')
	{
		gyro_rotation_speed = gyro_data - GYRO_OFFSET;
	}
}

void Angle_calculation()
{
	float delta_angle = gyro_rotation_speed * iteration_time * 100;
	angle += delta_angle;
}

void Set_angle_to_rotate(uint8_t data)
{
	angle_to_rotate = 100 * data;
}

void Set_rotation_distance(uint8_t data)
{
	distance_until_stop = round((data * ROTATION_DISTANCE) / 90);
	stop_distance = 150;
}


//--------------------------LIDAR----------------------------------

void Set_LIDAR_turns(uint8_t turns)
{
	LIDAR_turns = 360 * turns;
}

//--------------------------Wheels---------------------------------

void Set_distance_until_stop(uint8_t number_of_modules)
{
	distance_until_stop = 4000 * number_of_modules;
	stop_distance = 750 + (0.5 /*MAX_SPEED*/ * (number_of_modules - 1) * 25); 
}

void Distance_travelled() //Calculates the travelled distance
{
	travel_distance = round(wheel_sensor_counter * WHEEL_SLICE); 
}

void Calculate_wheel_sensor_counter(uint8_t data) //Calculates the wheel sensor and standing still counter
{	
	if(data == 0)
	{
		standing_still_counter++; 
	}
	else
	{
		wheel_sensor_counter += data;
		standing_still_counter = 0;
	}
}

bool Standing_still() // Returns true if the robot is standing still
{
	if(standing_still_counter > 30)
	{
		wheel_sensor_counter = 0;
		standing_still_counter = 0;
		distance_until_stop = 0;
		travel_distance = 0;
		angle = 0;
		angle_to_rotate = 0;
		return true;
	}
	return false;
}

//--------------------------Tape--------------------------------

void Line_detection(uint8_t tape_data)
{
	if(tape_data > 200)
	{
		line_detected = true;
	}
	else
	{
		line_detected = false;
	}
}
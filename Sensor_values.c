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
#include "Control.h"

#define WHEEL_DIAMETER 63.34
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define WHEEL_SLICE (WHEEL_CIRCUMFERENCE/16)
#define GYRO_OFFSET 1230
#define GYRO_CONSTANT_RIGHT 0.353 //0.366748166259168704156479
#define GYRO_CONSTANT_LEFT 0.375
#define ROTATION_DISTANCE 2221.44 // Distance when rotating 90 degrees, 0.1 mm

int left_distance;
int front_right_distance;
int back_right_distance;
int forward_IR_distance;
bool left_side_detected;
bool right_side_detected;
bool forward_IR_detected;

int16_t angle;
int16_t angle_to_rotate;
int16_t gyro_rotation_speed;

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

bool first_detection;

uint16_t left_IR_array[2] = {0, 0};
uint16_t right_IR_array[2] = {0, 0};
uint16_t LIDAR_array[2] = {0, 0};

//--------------------------Timer 0--------------------------------
  
//-------------------------IR sensors------------------------------

void IR_conversion(char direction, uint8_t IR_value)
{
	uint16_t distance = round(10 * ((81.42 * exp(-0.0435 * IR_value)) + (25.63 * exp(-0.007169 * IR_value))));
	if(direction == 'r')
	{
		front_right_distance = distance;
		Right_side_detectable();
	}
	else if(direction == 'b')
	{
		back_right_distance = distance;
	}
	else if(direction == 'l')
	{
		left_distance = distance;
		Left_side_detectable();
	}
	else if(direction == 'f')
	{
		distance = floor(100 * ((81.42 * exp(-0.0435 * IR_value)) + (25.63 * exp(-0.007169 * IR_value))));
		forward_IR_distance = distance;
		Forward_IR_detectable();
	}	
}

void Left_side_detectable()
{
	if(left_distance > 250)
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
	if(front_right_distance > 250)
	{
		right_side_detected = false;
	}
	else
	{
		right_side_detected = true;
		if(mode == 'f' && !first_detection)
		{
			first_detection = true;
		}
		if(after_right_turn)
		{
			after_right_turn = false;
		}
	}
}

void Forward_IR_detectable()
{
	if(forward_IR_distance > 2500)
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
	if(gyro_data > 1260 || gyro_data < 1200)
	{
		if(mode == 'r')
		{
			gyro_rotation_speed = (GYRO_OFFSET - gyro_data) * (GYRO_CONSTANT_RIGHT - (0.0005 * (180 - angle_to_rotate) / 90));
			return;
		}
		else if(mode == 'l')
		{
			gyro_rotation_speed = (gyro_data - GYRO_OFFSET) * (GYRO_CONSTANT_LEFT - (0.0005 * (180 - angle_to_rotate) / 90));
			return;
		}
	}
	gyro_rotation_speed = 0;
}

void Angle_calculation()
{
	uint16_t delta_angle = round(gyro_rotation_speed * iteration_time * 100);
	angle += delta_angle;
}

void Set_angle_to_rotate(uint8_t data)
{
	angle_to_rotate = 100 * data;
}

void Set_rotation_distance(uint8_t data)
{
	distance_until_stop = round((data * ROTATION_DISTANCE) / 90);
	stop_distance = 250 - data;
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
	stop_distance = 550 + (0.5 /*MAX_SPEED*/ * (number_of_modules - 1) * 25); 
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
	if(standing_still_counter > 20)
	{
		if(Correct_angle_to_wall())
		{
			last_mode = mode;
			wheel_sensor_counter = 0;
			standing_still_counter = 0;
			mode_complete = true;
			distance_until_stop = 0;
			travel_distance = 0;
			
			angle = 0;
			angle_to_rotate = 0;
			return true;
		}
		return false;
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
/*
 * Sensor_values.c
 *
 * Created: 4/20/2017
 * Author: Gustav Strandberg, gusst967
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

//----------------------------Global definitions-------------------------------
#define WHEEL_DIAMETER 63.34
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define WHEEL_SLICE (WHEEL_CIRCUMFERENCE/16)
#define GYRO_OFFSET 1230
#define GYRO_CONSTANT_RIGHT 0.353 
#define GYRO_CONSTANT_LEFT 0.33
// Distance when rotating 90 degrees using wheel sensors, 0.1 mm
#define ROTATION_DISTANCE 2221.44 

//--------------------------------Parameters-----------------------------------
int front_left_distance;
int back_left_distance;
int front_right_distance;
int back_right_distance;
int forward_IR_distance;
bool front_left_side_detected;
bool back_left_side_detected;
bool front_right_side_detected;
bool back_right_side_detected;
bool forward_IR_detected;

int16_t angle;
int16_t angle_to_rotate;
int16_t gyro_rotation_speed;

volatile int32_t distance_until_stop;
volatile int32_t stop_distance;
int32_t travel_distance;
volatile uint16_t wheel_sensor_counter;
uint8_t standing_still_counter;
uint8_t correct_angle_counter;
bool line_detected;
float iteration_time = 0.02; // 20 ms

bool first_detection;
  
//-------------------------IR sensors------------------------------

void IR_conversion(char direction, uint8_t IR_value)
{
	uint16_t distance = round(10 * ((81.42 * exp(-0.0435 * IR_value)) + 
				(25.63 * exp(-0.007169 * IR_value))));
	
	// The direction character tells what IR-sensor is the
	// one to be converted. 
	if(direction == 'r')
	{
		front_right_distance = distance;
		Front_right_side_detectable();
	}
	else if(direction == 'b')
	{
		back_right_distance = distance;
		Back_right_side_detectable();
	}
	else if(direction == 'l')
	{
		front_left_distance = distance;
		Front_left_side_detectable();
	}
	else if(direction == 'w')
	{
		back_left_distance = distance;
		Back_left_side_detectable();
	}
	else if(direction == 'f')
	{
		// The precision of forward_IR_distance is in 0.1 mm.
		distance = round(100 * ((81.42 * exp(-0.0435 * IR_value)) + 
			(25.63 * exp(-0.007169 * IR_value))));
		forward_IR_distance = distance;
		Forward_IR_detectable();
	}	
}

// Returns true if the front left IR has contact.
void Front_left_side_detectable()
{
	if(front_left_distance > 200)
	{
		front_left_side_detected = false;
	}
	else
	{
		front_left_side_detected = true;
	}
}

// Returns true if the back left IR has contact.
void Back_left_side_detectable()
{
	if(back_left_distance > 200)
	{
		back_left_side_detected = false;
	}
	else
	{
		back_left_side_detected = true;
	}
}

// Returns true if a wall is to the left of the robot.
bool Left_side_detectable()
{
	if(front_left_side_detected && back_left_side_detected)
	{
		return true;
	}
	return false;
}

// Returns true if the front right IR has contact.
void Front_right_side_detectable()
{
	if(front_right_distance > 200)
	{
		front_right_side_detected = false;
	}
	else
	{
		front_right_side_detected = true;
	}
}

// Returns true if the back right IR has contact.
void Back_right_side_detectable()
{
	if(back_right_distance > 200)
	{
		back_right_side_detected = false;
	}
	else
	{
		back_right_side_detected = true;
	}
}

// Returns true if a wall is to the right of the robot.
bool Right_side_detectable()
{
	if(front_right_side_detected && back_right_side_detected)
	{
		// Sets bools if true.
		if(mode == 'f' && !first_detection)
		{
			first_detection = true;
		}
		if(after_right_turn)
		{
			after_right_turn = false;
		}
		
		return true;
	}
	return false;
}

// Returns true if a wall is in front of the robot.
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

//-------------------------------Gyro------------------------------------------

// Calculation and conversion of the gyro data to rotation speed.
void Gyro_calculation(uint16_t gyro_data)
{
	// Compensation of hardware in gyro.
	if(gyro_data > 1310 || gyro_data < 1150)
	{
		if(mode == 'r')
		{
			gyro_rotation_speed = (GYRO_OFFSET - gyro_data) * 
				(GYRO_CONSTANT_RIGHT - (0.0005 * (180 - angle_to_rotate) / 90));
			return;
		}
		else if(mode == 'l')
		{
			gyro_rotation_speed = (gyro_data - GYRO_OFFSET) * 
				(GYRO_CONSTANT_LEFT - (0.0005 * (180 - angle_to_rotate) / 90));
			return;
		}
	}
	gyro_rotation_speed = 0;
}

void Angle_calculation()
{
	// Calculates angle out of the rotation speed
	int16_t delta_angle = round(gyro_rotation_speed * iteration_time * 100);
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

//--------------------------------Wheels---------------------------------------

void Set_distance_until_stop(uint8_t number_of_modules)
{
	distance_until_stop = 4000 * number_of_modules; // Precision of 0.1 mm.
	stop_distance = 550 + (0.5 * (number_of_modules - 1) * 25); 
}

void Distance_travelled()
{
	travel_distance = round(wheel_sensor_counter * WHEEL_SLICE); 
}

//Calculates the wheel sensor and standing still counter.
void Calculate_wheel_sensor_counter(uint8_t data) 
{	
	if(data == 0)
	{
		// If there hasn't been any wheel shifts, 
		// increment standing_still_counter.
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
		return true;
	}
	return false;
}

//-------------------------------Tape-----------------------------------------.

void Line_detection(uint8_t tape_data)
{
	if(tape_data == 1)
	{
		line_detected = true;
	}
	else
	{
		line_detected = false;
	}
}
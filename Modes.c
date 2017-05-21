/*
 * Modes.c
 *
 * Created: 2017-05-05 16:48:15
 *  Author: Deep
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "Modes.h"
#include "Sensor_values.h"
#include "UART.h"
#include "Control.h"
#include "PWM_SirCave.h"

uint8_t mode;
uint8_t last_mode;
uint8_t competition_mode;
bool change_competition_mode;
bool autonomous;
bool mode_complete;
bool turn_around;

void Mode_loop()
{
	if(autonomous) // Autonomous mode
	{
		Autonomous_mode();
	}
	else // Manual mode
	{
		Manual_mode();
	}
}

void Autonomous_mode()
{
// 	if(line_detected)
// 	{
// 		mode = 's';
// 		UART_transmission('t');
// 		line_detected = false;
// 		mode_complete = true;
// 		return;
// 	}

	switch(mode)
	{
		case 'f':
		Hallway_control(true);
		if(Standing_still())
		{
			mode = 's';
			last_mode = 'f';
			mode_complete = true;
			wheel_sensor_counter = 0;
			UART_transmission('d');
			
		}
		break;
		
		case 'b':
		Hallway_control(false);
		if(Standing_still())
		{
			mode = 's';
			last_mode = 'b';
			mode_complete = true;
			UART_transmission('d');
		}
		break;
		
		case 's':
		Stop_motors();
		mode_complete = true;
		standing_still_counter = 0;
		break;
		
		case 'r':
		Rotation_control(true);
		if(Standing_still())
		{
			mode = 's';
			last_mode = 'r';
			mode_complete = true;
			after_right_turn = true;
			angle = 0;
			angle_to_rotate = 0;
			wheel_sensor_counter = 0;
			UART_transmission('d');
		}
		break;
		
		case 'l':
		Rotation_control(false);
		if(Standing_still())
		{
			last_mode = 'l';
 			mode_complete = true;
			mode = 's';
			angle = 0;
			angle_to_rotate = 0;
			wheel_sensor_counter = 0;
			standing_still_counter = 0;
			UART_transmission('d');
		}
		break;
		
		case 'L':
		Rotate_LIDAR(0.2);
		break;
		
		case 'S':
		Stop_LIDAR();
		break;
		
		case 'o':
		Open_grip_arm();
		break;
		
		case 'c':
		Close_grip_arm();
		break;
		
		case 'm':
		Center_grip_arm();
		break;
	}		
}

void Manual_mode()
{
	switch(mode)
	{
		case 'f':
		Drive_forward(0.9, 0.9);
		break;
		
		case 'b':
		Drive_backwards(0.9, 0.9);
		break;
		
		case 'l':
		Rotate_counter_clockwise(0.5, 0.5);
		break;
		
		case 'r':
		Rotate_clockwise(0.5, 0.5);
		break;
		
		case 's':
		Stop_motors();
		break;
		
		case 'o':
		Open_grip_arm();
		break;
		
		case 'c':
		Close_grip_arm();
		break;
		
		case 'm':
		Center_grip_arm();
		break;
		
		case 'L':
		Rotate_LIDAR(0.2);
		break;
		
		case 'S':
		Stop_LIDAR();
		break;
	}	
}
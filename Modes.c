/*
 * Modes.c
 *
 * Created: 2017-05-05
 * Author: Gustav Strandberg, gusst967
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

//---------------------------Mode parameters-----------------------------------

volatile uint8_t mode;
uint8_t last_mode;
bool competition_mode;
bool change_competition_mode;
bool autonomous;
volatile bool mode_complete;
bool turn_around;
bool adjust_angle = false;

// This is the headloop for all the modes. It loops through the modes depending
// if the system is controlled autonomically or manually.  
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
	// If a line is detected.
	if(line_detected && mode != 'b' && mode != 's')
	{
		// If in mapping stage, the robot will stop as a line is detected.
		if(competition_mode)
		{
			mode = 's';
			line_detected = false;
			mode_complete = true;
		}
		
		UART_transmission('t'); // A 't' is sent to the communication module.
	}
	
	switch(mode)
	{
		case 'f':
		// If mode == 'f', the robot uses Hallway_control until it is parallel
		// to the walls to the sides and is standing still. A 'd' is then sent
		// to the communication module as a request of a new steering mode.
		 
		Hallway_control(true);
		
		if(Standing_still())
		{
			// If the robot has the correct angle to the walls and it is 
			// standing still, then parameters is reseted, the modes is set to 
			// 's' and a 'd' is sent to the communication module.
			 
			if(Correct_angle_to_wall())
			{
				mode = 's';
				last_mode = 'f';
				mode_complete = true;
				wheel_sensor_counter = 0;
				travel_distance = 0;
				distance_until_stop = 0;
				angle = 0;
				angle_to_rotate = 0;
				UART_transmission('d');
			}
			
			// If the angle to the sides isn't correct, the mode is set to 'C'.
			else
			{
				mode = 'C';
				last_mode = 'f';
				standing_still_counter = 0;
			}
		}
		break;
		
		case 'b':
		// Same as mode == 'f' but the direction is backwards.
		Hallway_control(false);
		if(Standing_still())
		{
			if(Correct_angle_to_wall())
			{
				mode = 's';
				last_mode = 'b';
				mode_complete = true;
				wheel_sensor_counter = 0;
				travel_distance = 0;
				distance_until_stop = 0;
				angle = 0;
				angle_to_rotate = 0;
				UART_transmission('d');
			}
			else
			{
				mode = 'C';
				last_mode = 'b';
				standing_still_counter = 0;
			}
		}
		break;
		
		case 's':
		// The case where the robot stands still. Parameters is set to 0
		// and Stop_motors() is made.
		Stop_motors();
		mode_complete = true;
		standing_still_counter = 0;
		wheel_sensor_counter = 0;
		travel_distance = 0;
		distance_until_stop = 0;
		angle = 0;
		angle_to_rotate = 0;
		break;
		
		case 'r':
		// Rotates clockwise using Rotation_control. 
		Rotation_control(true);
		if(Standing_still())
		{
			// If the robot has the correct angle to the walls and it is
			// standing still, then parameters is reseted, the modes is set to
			// 's' and a 'd' is sent to the communication module.
			
			if(Correct_angle_to_wall())
			{
				mode = 's';
				last_mode = 'r';
				mode_complete = true;
				after_right_turn = true;
				wheel_sensor_counter = 0;
				travel_distance = 0;
				distance_until_stop = 0;
				angle = 0;
				angle_to_rotate = 0;
				UART_transmission('d');
			}
			
			// If the angle to the sides isn't correct, the mode is set to 'C'.
			else
			{
				mode = 'C';
				last_mode = 'r';
				standing_still_counter = 0;
			}
		}
		break;
		
		case 'l':
		// Same as mode == 'r', but rotates counter clockwise.
		Rotation_control(false);
		if(UART_queue_peek(UART_queue_out + 1) == 'l')
		{
			UART_queue_remove();
			UART_queue_remove();
			UART_queue_remove();
		}
		if(Standing_still())
		{
			if(Correct_angle_to_wall())
			{
				if(last_mode == 'b')
				{
					first_detection = false;
					after_right_turn = true;
				}
				mode = 's';
				last_mode = 'l';
				mode_complete = true;
				travel_distance = 0;
				distance_until_stop = 0;
				angle = 0;
				angle_to_rotate = 0;
				wheel_sensor_counter = 0;
				standing_still_counter = 0;
				UART_transmission('d');
			}
			else
			{
				mode = 'C';
				last_mode = 'l';
				standing_still_counter = 0;
			}
		}
		break;
		
		case 'C':
		// The mode where the robot corrects its angle to the walls.
		// When the robot is standing still, parameters is reseted, mode is set 
		// to 's' and a 'd' is sent to the communication module.
		Straighten_up_robot();
		if(Standing_still())
		{
			if(last_mode == 'r')
			{
				after_right_turn = true;
			}
			mode_complete = true;
			mode = 's';
			travel_distance = 0;
			distance_until_stop = 0;
			angle = 0;
			angle_to_rotate = 0;
			wheel_sensor_counter = 0;
			standing_still_counter = 0;
			UART_transmission('d');
		}
		break;
		
		case 'L':
		// Rotates the LIDAR.
		Rotate_LIDAR(0.2);
		break;
		
		case 'S':
		// Stops the LIDAR.
		Stop_LIDAR();
		break;
		
		case 'o':
		// Opens the grip arm.
		Open_grip_arm();
		mode = 's';
		break;
		
		case 'c':
		// Close the grip arm.
		Close_grip_arm();
		mode = 's';
		break;
		
		case 'm':
		// Center the grip arm.
		Center_grip_arm();
		mode = 's';
		break;
	}		
}

// If the system is controlled manually, these are the cases used.
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
		
		case 'O':
		Drive_forward(0.8, 0.5);
		break;
		
		case 'P':
		Drive_forward(0.5, 0.8);
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
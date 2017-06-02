/*
 * Control.c
 *
 * Created: 4/19/2017
 * Author: Gustav Strandberg, gusst967
		   Gustaf Westerholm, guswe541
 */ 


#define F_CPU 14745600UL

#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <stdbool.h>
#include "Control.h"
#include "PWM_SirCave.h"
#include "Sensor_values.h"
#include "SPI.h"
#include "UART.h"
#include "Modes.h"

#define MAX_SPEED 0.5

//---------------------Control parameters---------------------

float error_prior_both;
float error_prior_right;
float error_prior_left;
float error_current_both;
float error_current_right;
float error_current_left;
uint8_t proportional_gain_both = 10;
uint8_t proportional_gain_right = 18;
uint8_t proportional_gain_left = 18;
float derivative_gain_both = 1.6;
float derivative_gain_right = 3.0;
float derivative_gain_left = 3.0;

bool update_control;
bool after_right_turn;

//----------------------------Hallway control------------------------------------

// Steersignal when both sides detectable.
float Steer_signal_both() 
{
	error_prior_both = error_current_both; // Set the last error
	error_current_both = front_right_distance - front_left_distance; // Set current error
	float steer_signal_both = proportional_gain_both * error_current_both + 
							  derivative_gain_both * (error_current_both - error_prior_both) * 
							  (1 / iteration_time); // Create the steersignal
	return steer_signal_both;
}

// Steersignal when it's only the right side detectable.
float Steer_signal_right() 
{
	error_prior_right = error_current_right; // Set the last error
	
	// If the distance from the right side is in the right interval.
	if(front_right_distance < 90 || front_right_distance > 130)
	{
		error_current_right = front_right_distance - 100; // Set current error 
	}
	else
	{
		error_current_right = 0; // Both current and last error = 0
		error_prior_right = 0;
	}
	// Create the steersignal
	float steer_signal_right = proportional_gain_right * error_current_right + 
			derivative_gain_right * (error_current_right - error_prior_right) * 
			(1 / iteration_time); 
	return steer_signal_right;
}

// Steersignal when it's only the left side is detectable. Works similar to the
// Steer_signal_right().
float Steer_signal_left() 
{
	error_prior_left = error_current_left;
	if(front_left_distance < 90 || front_left_distance > 130)
	{
		error_current_left = front_left_distance - 100;
	}
	else
	{
		error_current_left = 0;
		error_prior_right = 0;
	}
	float steer_signal_left = proportional_gain_left * error_current_left + 
			derivative_gain_left * (error_current_left - error_prior_left) * 
			(1 / iteration_time);
	
	return steer_signal_left;
}

//-----------------------------Control using walls-----------------------------

// Sets the direction of the wheels.
void Direction(bool forward) 
{
	if(forward)
	{
		PORTA = (1 << PORTA0) | (1 << PORTA1);
	}
	else
	{
		PORTA = (0 << PORTA0) | (0 << PORTA1);
	}
}

// Control when driving straight
void Hallway_control(bool forward)
{
	if(!update_control) // If update_control, then the system will control.
	{
		return;
	}
	
	update_control = false;
	Direction(forward);
	
	// Both sides detected
	if(Right_side_detectable() && Left_side_detectable()) 
	{
		Hallway_control_both();
	}
	// Only right side detected
	else if(Right_side_detectable() && !Left_side_detectable()) 
	{
		Hallway_control_right();
	}
	// After right turn during mapping, the robot should drive until a wall
	// to the right is found.
	else if(competition_mode && after_right_turn && !Right_side_detectable()) 
	{
		OCR1A = MAX_SPEED * ICR1;
		OCR1B = MAX_SPEED * ICR1;
	}
	// No side detected
	else if(!Right_side_detectable() && !Left_side_detectable()) 
	{
		float velocity = Set_speed();
		OCR1A = velocity * ICR1;
		OCR1B = velocity * ICR1;
	}
	// Only left side detected
	else if(!Right_side_detectable() && Left_side_detectable()) 
	{
		Hallway_control_left();
	}
}

// Control when both sides are detected.
void Hallway_control_both() 
{
	float steer_signal_both = Steer_signal_both();
	float set_speed = Set_speed(); 
	
	if (error_current_both >= 0) 
	{
		// The registers can't be set to negative values.
		if(set_speed * ICR1 - steer_signal_both < 0) 
		{
			OCR1A = 0; // Motors right side
			OCR1B = set_speed*ICR1; // Motors left side
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_both; // Right side
			OCR1B = set_speed*ICR1; // Left side
		}
	}
	else
	{
		// The registers can't be set to negative values.
		if(set_speed * ICR1 + steer_signal_both < 0) 
		{
			OCR1A = set_speed * ICR1; // Right side
			OCR1B = 0; // Left side
		}
		else
		{
			OCR1A = set_speed * ICR1; // Right side
			OCR1B = set_speed * ICR1 + steer_signal_both; // Left side
		}
	}
}

// Control when its only the right side detected.
void Hallway_control_right() 
{
	float steer_signal_right = Steer_signal_right();
	float set_speed = Set_speed();
	if(set_speed <= 0)
	{
		OCR1A = 0; // Motors right side
		OCR1B = 0; // Motors left side
		return;
	}
	if (error_current_right >= 0)
	{
		// The registers can't be set to negative values.
		if(set_speed * ICR1 - steer_signal_right < 0) 
		{
			OCR1A = 0; // Right side
			OCR1B = set_speed * ICR1; // Left side
		}
		else
		{
			OCR1A = set_speed * ICR1 - steer_signal_right; // Right side
			OCR1B = set_speed * ICR1; // Left side
		}
	}
	else
	{
		// The registers can't be set to negative values
		if(set_speed * ICR1 - steer_signal_right < 0) 
		{
			OCR1A = set_speed * ICR1; // Right side
			OCR1B = 0; // Left side
		}
		else
		{
			OCR1A = set_speed*ICR1; // Right side
			OCR1B = set_speed*ICR1 + steer_signal_right; // Left side
		}
	}
}

// Control when its only the left side detected.
void Hallway_control_left() 
{
	float steer_signal_left = Steer_signal_left();
	float set_speed = Set_speed();
	if(set_speed <= 0)
	{
		OCR1A = 0; // Motors right side
		OCR1B = 0; // Motors left side
		return;
	}
	
	if (error_current_left >= 0)
	{
		// The registers can't be set to negative values
		if(set_speed*ICR1 - steer_signal_left < 0) 
		{
			OCR1B = 0; // Left side
			OCR1A = set_speed*ICR1; // Right side
		}
		else
		{
			OCR1B = set_speed*ICR1 - steer_signal_left; // Left side
			OCR1A = set_speed*ICR1; // Right side
		}
	}
	else
	{
		// The registers can't be set to negative values
		if(set_speed*ICR1 - steer_signal_left < 0) 
		{
			OCR1B = set_speed*ICR1; // Left side
			OCR1A = 0; // Right side
		}
		else
		{
			OCR1B = set_speed*ICR1; // Left side
			OCR1A = set_speed*ICR1 + steer_signal_left; // Right side
		}
	}
}
//----------------------------Rotation control---------------------------------

// Rotates the robot until the correct angle is reached.
void Rotation_control(bool right) 
{
	if(!update_control)
	{
		return;
	}
	
	update_control = false;
	
	if(angle < angle_to_rotate) // Rotate if angle < angle_to_rotate
	{
		if(right)
		{
			PORTA = (0 << PORTA0) | (1 << PORTA1); // Rotate clockwise
			OCR1A = 0.5 * ICR1;
			OCR1B = 0.5 * ICR1;
		}
		else
		{
			PORTA = (1 << PORTA0) | (0 << PORTA1); // Rotate counter clockwise
			OCR1A = 0.5 * ICR1;
			OCR1B = 0.5 * ICR1;
		}
	}
	else
	{
		OCR1A = 0;
		OCR1B = 0;
	}
}

//----------------------------Speed control------------------------------------

// Sets speed to until the given distance is reached.
float Set_speed() 
{
	Distance_travelled(); // Calculates the distance travelled.
	int16_t delta_distance = 0;
	if(!competition_mode) // If it isn't in the mapping stage.
	{
		// If the robot drives forward and detects a wall in front of it.
		if(forward_IR_detected && mode == 'f') 
		{
			delta_distance = forward_IR_distance - 1000;
		}
		else
		{
			delta_distance = distance_until_stop - travel_distance;
		}
	}
	else if(competition_mode) // If it is in the mapping stage
	{
		if(forward_IR_detected && !turn_around)
		{
			delta_distance = forward_IR_distance - 1000;
		}
		
		// If the right side isn't detected, bool works the other way around
		// here for unknown reason.
		else if(Right_side_detectable()) 
		{
			if(first_detection) // If it's the first detection of no wall to
								// the right
			{
				distance_until_stop = travel_distance + 2100;
				first_detection = false;
			}
			
			if(turn_around) // If the robot has turned around
			{
				Set_distance_until_stop(15);
				turn_around = false;
			}
			
			delta_distance = distance_until_stop - travel_distance;
		}
		else
		{			
			delta_distance = distance_until_stop - travel_distance; 
		}
	}
	
	// If delta_distance < stop_distance the robot will stop.
	if(delta_distance < stop_distance) 
	{
		return Correct_to_center_of_tile();
	}
	else
	{
		return MAX_SPEED;
	}
}

//------------------------Position correction----------------------------------

// Corrects the position of the robot if there is a wall in front of it.
float Correct_to_center_of_tile() 
{
	if(forward_IR_detected && mode == 'f')
	{
		// Drive forward if it is more than 12 cm from the wall.
		if(forward_IR_distance > 1200) 
		{
			Direction(true);
			return 0.2;
		}	
		// Drive backward if it is less than 10 cm from the wall
		else if(forward_IR_distance < 1000) 
		{
			Direction(false);
			return 0.2;
		}
	}
	return 0;
}

// Returns true if the robot is parallel to any wall to the sides of the robot.
bool Correct_angle_to_wall() 
{	
	if(Right_side_detectable())
	{
		int16_t delta_right_distance = front_right_distance - back_right_distance;
		
		// Return false if the robot isn't parallel to the right wall.
		if(delta_right_distance > 5 || delta_right_distance < -5) 
		{
			return false;
		}
	}
	else if(Left_side_detectable())
	{
		int16_t delta_left_distance = front_left_distance - back_left_distance;
		
		// Return false if the robot isn't parallel to the left wall.
		if(delta_left_distance > 5 || delta_left_distance < -5)  
		{
			return false;
		}
	}
	
	return true;
}

// Straightens up the robot to any wall that is to the side of it.
void Straighten_up_robot()
{	
	if(Right_side_detectable())
	{
		int16_t delta_right_distance = front_right_distance - back_right_distance;
		
		// If front_right_distance > back_right_distance + 5 then rotate clockwise.
		if(delta_right_distance > 5) 
		{
			PORTA = (0 << PORTA0) | (1 << PORTA1);
			OCR1A = 0.3 * ICR1;
			OCR1B = 0.3 * ICR1;
		}
		
		// If front_right_distance > back_right_distance then rotate counter clockwise.
		else if(delta_right_distance < -5) 
		{
			PORTA = (1 << PORTA0) | (0 << PORTA1);
			OCR1A = 0.3 * ICR1;
			OCR1B = 0.3 * ICR1;
		}
		else
		{
			OCR1A = 0;
			OCR1B = 0;
		}
	}
	
	else if(Left_side_detectable())
	{
		int16_t delta_left_distance = front_left_distance - back_left_distance;
		
		// If front_left_distance > back_left_distance + 5 then rotate counter clockwise.
		if(delta_left_distance > 5) 
		{
			PORTA = (1<< PORTA0) | (0 << PORTA1);
			OCR1A = 0.3 * ICR1;
			OCR1B = 0.3 * ICR1;
		}
		// If front_left_distance + 5 < back_left_distance then rotate clockwise
		else if(delta_left_distance < -5) 
		{
			PORTA = (0 << PORTA0) | (1 << PORTA1);
			OCR1A = 0.3 * ICR1;
			OCR1B = 0.3 * ICR1;
		}
		else
		{
			OCR1A = 0;
			OCR1B = 0;
		}
	}
}


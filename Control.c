/*
 * Control.c
 *
 * Created: 4/19/2017 3:39:39 PM
 *  Author: gusst967
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

float error_prior_both;
float error_prior_right;
float error_prior_left;
float error_current_both;
float error_current_right;
float error_current_left;
uint8_t proportional_gain_both = 10;
uint8_t proportional_gain_right = 20;
uint8_t proportional_gain_left = 20;
float derivative_gain_both = 1.8;
float derivative_gain_right = 3.6;
float derivative_gain_left = 3.6;
//
float error_prior_speed;
float error_current_speed;
uint8_t proportional_gain_speed = 5;
float derivative_gain_speed = 0.5;

//
//
float error_prior_rotation;
float error_current_rotation;
uint8_t proportional_gain_rotation = 10;
float derivative_gain_rotation = 0.5;

bool update_control;
bool after_right_turn;
bool drive_anyways;
//



/*
>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM LIDAR WHEN POINTING FORWARD HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

/*
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM GYRO HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/







//----------------------------Hallway control------------------------------------


float Steer_signal_both() //steersignal when both sides detectable
{
	error_prior_both = error_current_both;
	error_current_both = right_distance - left_distance;
	float steer_signal_both = proportional_gain_both*error_current_both + derivative_gain_both*(error_current_both - error_prior_both)*(1/iteration_time);
	return steer_signal_both;
}

float Steer_signal_right() //steersignal when right side detectable
{
	error_prior_right = error_current_right;
	error_current_right = right_distance - 100; // Control using mm
	float steer_signal_right = proportional_gain_right*error_current_right + derivative_gain_right*(error_current_right - error_prior_right)*(1/iteration_time);
	return steer_signal_right;
}

float Steer_signal_left() //steersignal when left side detectable
{
	error_prior_left = error_current_left;
	error_current_left = left_distance - 100; // Control using mm
	float steer_signal_left = proportional_gain_left*error_current_left + derivative_gain_left*(error_current_left - error_prior_left)*(1/iteration_time);
	return steer_signal_left;
}

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

void Hallway_control(bool forward)
{
	if(!update_control)
	{
		return;
	}
	
	update_control = false;
	Direction(forward);
	
	if(right_side_detected && left_side_detected)
	{
		Hallway_control_both();
	}
	else if(right_side_detected && !left_side_detected)
	{
		Hallway_control_right();
	}
	else if(competition_mode == 1 && after_right_turn && !right_side_detected)
	{
		OCR1A = MAX_SPEED * ICR1;
		OCR1B = MAX_SPEED * ICR1;
	}
	else if(!right_side_detected && left_side_detected)
	{
		Hallway_control_left();
	}
	else
	{
		float velocity = Set_speed();
		OCR1A = velocity * ICR1;
		OCR1B = velocity * ICR1;
	}
	
}

void Hallway_control_both()
{
	float steer_signal_both = Steer_signal_both();
	float set_speed = Set_speed(); 
	
	if (error_current_both >= 0)
	{
		if(set_speed*ICR1 - steer_signal_both < 0)
		{
			OCR1A = 0; // Right
			OCR1B = set_speed*ICR1; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_both; // Right
			OCR1B = set_speed*ICR1; // Left
		}
	}
	else
	{
		if(set_speed * ICR1 + steer_signal_both < 0)
		{
			OCR1A = set_speed * ICR1; // Right
			OCR1B = 0; // Left
		}
		else
		{
			OCR1A = set_speed * ICR1; // Steersignal < 0 Right
			OCR1B = set_speed * ICR1 + steer_signal_both; //Left
		}
	}
}

void Hallway_control_right()
{
	float steer_signal_right = Steer_signal_right();
	float set_speed = Set_speed();
	if(set_speed <= 0)
	{
		OCR1A = 0; // Right
		OCR1B = 0; // Left
		return;
	}
	if (error_current_right >= 0)
	{
		if(set_speed*ICR1 - steer_signal_right < 0)
		{
			OCR1A = 0; // Right
			OCR1B = set_speed*ICR1; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_right; // Right
			OCR1B = set_speed*ICR1; // Left
		}
	}
	else
	{
		if(set_speed*ICR1 - steer_signal_right < 0)
		{
			OCR1A = set_speed*ICR1; // Right
			OCR1B = 0; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1; // Steersignal < 0 // Right
			OCR1B = set_speed*ICR1 + steer_signal_right; // Left
		}
	}
}

void Hallway_control_left()
{
	float steer_signal_left = Steer_signal_left();
	float set_speed = Set_speed();
	if(set_speed <= 0)
	{
		OCR1A = 0; // Right
		OCR1B = 0; // Left
		return;
	}
	
	if (error_current_left >= 0)
	{
		if(set_speed*ICR1 - steer_signal_left < 0)
		{
			OCR1B = 0; // Left
			OCR1A = set_speed*ICR1; // Right
		}
		else
		{
			OCR1B = set_speed*ICR1 - steer_signal_left; // Left
			OCR1A = set_speed*ICR1; // Right
		}
	}
	else
	{
		if(set_speed*ICR1 - steer_signal_left < 0)
		{
			OCR1B = set_speed*ICR1; // Left
			OCR1A = 0; // Right
		}
		else
		{
			OCR1B = set_speed*ICR1; // Steersignal < 0  // Left
			OCR1A = set_speed*ICR1 + steer_signal_left; // Right
		}
	}
}
//----------------------------Rotation control------------------------------------

float Steer_signal_rotation()
{
	Angle_calculation();
	error_prior_rotation = error_current_rotation;
	error_current_rotation = angle_to_rotate - angle; // Control using mm
	float steer_signal = proportional_gain_rotation*error_current_rotation + derivative_gain_rotation*(error_current_rotation - error_prior_rotation)*(1/iteration_time);
	return steer_signal;
}

void Rotation_control(bool right) //rotates robot
{
	if(!update_control)
	{
		return;
	}
	
	update_control = false;
	float set_speed = Set_speed();
	
	if(right)
	{
		PORTA = (0 << PORTA0) | (1 << PORTA1);
		OCR1A = set_speed * ICR1;
		OCR1B = set_speed * ICR1;
	}
	else
	{
		PORTA = (1 << PORTA0) | (0 << PORTA1);
		OCR1A = set_speed * ICR1;
		OCR1B = set_speed * ICR1;
	}
}

float Correct_angle_from_gyro()
{
	int16_t delta_angle = angle_to_rotate - angle;
	if(mode == 'r')
	{
		if(delta_angle < 300)
		{
			PORTA = (1 << PORTA0) | (0 << PORTA1); // If the robot has rotated to much, rotate back
			return 0.2;
		}
		else if(delta_angle > 600)
		{
			PORTA = (0 << PORTA0) | (1 << PORTA1); // If the robot hasn't rotated enough, rotate more
			return 0.2;
		}
		else
		{
			return 0;
		}
	}
	else if(mode == 'l')
	{
		if(delta_angle < 300)
		{
			PORTA = (0 << PORTA0) | (1 << PORTA1); // If the robot has rotated to much, rotate back
			return 0.2;
		}
		else if(delta_angle > 600)
		{
			PORTA = (1 << PORTA0) | (0 << PORTA1); // If the robot hasn't rotated enough, rotate more
			return 0.2;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

//----------------------------Speed control------------------------------------

float Set_speed() //sets speed given distance to obstacle ahead and then stops
{
	Distance_travelled();
	int16_t delta_distance = 0;
	if(competition_mode == 0)
	{
		if(forward_IR_detected && mode == 'f')
		{
			delta_distance = forward_IR_distance - 1000;
		}
		else
		{
			delta_distance = distance_until_stop - travel_distance;
		}
	}
	else if(competition_mode == 1)
	{
		if(forward_IR_detected && mode == 'f')
		{
			delta_distance = forward_IR_distance - 1000;
		}
		else if(mode == 'f' && right_side_detected)
		{
			if(first_detection)
			{
				distance_until_stop = travel_distance + 2000;
				first_detection = false;
			}
			
			delta_distance = distance_until_stop - travel_distance;
		}
		else
		{
			delta_distance = distance_until_stop - travel_distance;
		}
	}
	
//	if(mode == 'f' || mode == 'b')
//	{
		
		if(delta_distance <= stop_distance)
		{
			return Correct_to_center_of_tile();
		}
		else
		{
			return MAX_SPEED;
		}
//	}
// 	else if(mode == 'l' || mode == 'r')
// 	{
// 		if(delta_distance <= stop_distance)
// 		{
// 			return 0;//Correct_angle_from_gyro();
// 		}
// 		else
// 		{
// 			return MAX_SPEED;
// 		}
// 	}
// 	return 0;
}

float Correct_to_center_of_tile()
{
	if(forward_IR_detected && mode == 'f')
	{
		if(forward_IR_distance > 1150)
		{
			Direction(true);
			return 0.2;
		}	
		else if(forward_IR_distance < 850)
		{
			Direction(false);
			return 0.2;
		}
	}
	return 0;
}

//----------------------------LIDAR control------------------------------------


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

#define MAX_SPEED 1

float error_prior1;
float error_prior2;
float error_prior3;
float error_current1;
float error_current2;
float error_current3;
uint8_t proportional_gain1 = 9;
uint8_t proportional_gain2 = 6;
uint8_t proportional_gain3 = 6;
float derivative_gain1 = 0.7;
float derivative_gain2 = 1.0;
float derivative_gain3 = 1.0;
float iteration_time = 0.02; // 20 ms
//
float error_prior_speed;
float error_current_speed;
uint8_t proportional_gain_speed = 10;
float derivative_gain_speed = 0.5;
//
//
float rotation_speed;
float rotation_angle = 0;
float error_prior_rot;
float error_current_rot;
uint8_t proportional_gain_rot = 10;
float derivative_gain_rot = 0.5;
//



/*
>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM LIDAR WHEN POINTING FORWARD HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

/*
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM GYRO HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/







//----------------------------Hallway control------------------------------------


float Steer_signal1() //steersignal when both sides detectable
{
	error_prior1 = error_current1;
	error_current1 = right_distance - left_distance;
	float u1 = proportional_gain1*error_current1 + derivative_gain1*(error_current1 - error_prior1)*(1/iteration_time);
	return u1;
}

float Steer_signal2() //steersignal when right side detectable
{
	error_prior2 = error_current2;
	error_current2 = right_distance - 100; // Control using mm
	float u2 = proportional_gain2*error_current2 + derivative_gain2*(error_current2 - error_prior2)*(1/iteration_time);
	return u2;
}

float Steer_signal3() //steersignal when left side detectable
{
	error_prior3 = error_current3;
	error_current3 = 100 - left_distance; // Control using mm
	float u3 = proportional_gain3*error_current3 + derivative_gain3*(error_current3 - error_prior3)*(1/iteration_time);
	return u3;
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
	bool right_side_detected = Right_side_detectable();
	bool left_side_detected = Left_side_detectable();
	
	Direction(forward);
	
	if(right_side_detected)
	{
		PORTC |= (1 << 0);
	}
	else
	{
		PORTC &= ~(1 << 0);
	}
	
	if(left_side_detected)
	{
		PORTC |= (1 << 1);
	}
	else
	{
		PORTC &= ~(1 << 1);
	}
	
	if(right_side_detected && left_side_detected)
	{
		Hallway_control_both();
	}
	else if(right_side_detected && !left_side_detected)
	{
		Hallway_control_right();
	}
	else if(!right_side_detected && left_side_detected)
	{
		Hallway_control_left();
	}
	else
	{
		OCR1A = 0;
		OCR1B = 0;
		//Drive_forward(0, 0);
	}
}

void Hallway_control_both()
{
	float steer_signal_1 = Steer_signal1();
	float set_speed = 0.5; //Set_speed();, 
	
	if (error_current1 >= 0)
	{
		if(set_speed*ICR1 - steer_signal_1 < 0)
		{
			OCR1A = 0; // Right
			OCR1B = set_speed*ICR1; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_1; // Right
			OCR1B = set_speed*ICR1; // Left
		}
	}
	else
	{
		if(set_speed * ICR1 + steer_signal_1 < 0)
		{
			OCR1A = set_speed * ICR1; // Right
			OCR1B = 0; // Left
		}
		else
		{
			OCR1A = set_speed * ICR1; // Steersignal < 0 Right
			OCR1B = set_speed * ICR1 + steer_signal_1; //Left
		}
	}
}

void Hallway_control_left()
{
	float steer_signal_3 = Steer_signal3();
	float set_speed = 0.5; //Set_speed();
	if (error_current3 >= 0)
	{
		if(set_speed*ICR1 - steer_signal_3 < 0)
		{
			OCR1A = 0; // Right
			OCR1B = set_speed*ICR1; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_3; // Right
			OCR1B = set_speed*ICR1; // Left
		}
	}
	else
	{
		if(set_speed*ICR1 - steer_signal_3 < 0)
		{
			OCR1A = set_speed*ICR1; // Right
			OCR1B = 0; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1; // Steersignal < 0  // Right
			OCR1B = set_speed*ICR1 + steer_signal_3; // Left
		}
	}
}

void Hallway_control_right()
{
	float steer_signal_2 = Steer_signal2();
	float set_speed = 0.5; //Set_speed();
	if (error_current1 >= 0)
	{
		if(set_speed*ICR1 - steer_signal_2 < 0)
		{
			OCR1A = 0; // Right
			OCR1B = set_speed*ICR1; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1 - steer_signal_2; // Right
			OCR1B = set_speed*ICR1; // Left
		}
	}
	else
	{
		if(set_speed*ICR1 - steer_signal_2 < 0)
		{
			OCR1A = set_speed*ICR1; // Right
			OCR1B = 0; // Left
		}
		else
		{
			OCR1A = set_speed*ICR1; // Steersignal < 0 // Right
			OCR1B = set_speed*ICR1 + steer_signal_2; // Left
		}
	}
}
//----------------------------Rotation control------------------------------------

// void Rotate(uint16_t angle, bool right) //rotates robot
// {
// 	error_prior_rot = error_current_rot;
// 	rotation_angle = rotation_angle + (rotation_speed - 153) * iteration_time;
// 	error_prior_rot = angle - rotation_angle;
// 	float u_rot = proportional_gain_rot*error_current_rot + derivative_gain_rot*(error_current_rot - error_prior_rot)*(1/iteration_time);
// 	if (u_rot < MAX_SPEED)
// 	{
// 		if(right)
// 		{
// 			OCR1A = u_rot*ICR1;
// 			OCR1B = u_rot*ICR1;
// 			PORTA = (0 << PORTA0) | (1 << PORTA1);
// 		}
// 		else
// 		{
// 			OCR1A = u_rot*ICR1;
// 			OCR1B = u_rot*ICR1;
// 			PORTA = (1 << PORTA0) | (0 << PORTA1);
// 		}
// 	}
// /////////////////////////////////////////////////////////////////////
// // 	else if (u_rot < 0.01) //IF SMALL STEER SIGNAL GIVES PROBLEMS
// // 	{
// // 		if (direction == 'H')
// // 		 		{
// // 		 			OCR1A = 0;
// // 		 			OCR1B = 0;
// // 		 			PORTA = (1 << PORTA0) | (0 << PORTA1);
// // 		 		}
// // 		 		else
// // 		 		{
// // 		 			OCR1A = 0;
// // 		 			OCR1B = 0;
// // 		 			PORTA = (0 << PORTA0) | (1 << PORTA1);
// // 		 		}
// // 	}
// //////////////////////////////////////////////////////////////////////
// 	else
// 	{
// 		if (right)
// 		{
// 			OCR1A = MAX_SPEED * ICR1;
// 			OCR1B = MAX_SPEED * ICR1;
// 			PORTA = (0 << PORTA0) | (1 << PORTA1);
// 		}
// 		else
// 		{
// 			OCR1A = MAX_SPEED * ICR1;
// 			OCR1B = MAX_SPEED * ICR1;
// 			PORTA = (1 << PORTA0) | (0 << PORTA1);
// 		}
// 		
// 	}
// }
// 
// //----------------------------Speed control------------------------------------
// 
// float Set_speed() //sets speed given distance to obstacle ahead and then stops
// {
// 	error_prior_speed = error_current_speed;
// 	error_current_speed = front_distance - 25; //stops when LIDAR is 25 cm from obstacle ahead
// 	float velocity = proportional_gain_speed*error_current_speed + derivative_gain_speed*(error_current_speed - error_prior_speed)*(1/iteration_time);
// 	if (velocity < MAX_SPEED)
// 	{
// 		return velocity;
// 	}
// ///////////////////////////////////////////////////////////////////////
// // 	else if (velocity < 0.01) // IF SMALL STEER SIGNAL GIVES PROBLEMS
// // 	{
// // 		return 0;
// // 	}
// ///////////////////////////////////////////////////////////////////////
// 	else
// 	{
// 		return MAX_SPEED;
// 	}
// }


void Speed_test()
{
	
	
	SPI_queue_put(0xFF);
	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	
	SPI_queue_put(0xFF);
	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	
	SPI_queue_put(0xFF);
	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	
	SPI_queue_put(0xFF);
	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	
	SPI_queue_put(0xFF);
	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
}

//----------------------------LIDAR control------------------------------------










// int main(void)
// {
// 	Timer1_init();
// 	DDRD = 0xFF;
// 	TCNT1 = ICR1 - 2;
// 	
//     while(1)
//     {
// 		float steer_signal_1 = Steer_signal1();
// 		float steer_signal_2 = Steer_signal2();
// 		
// 		if (left_distance >= 0  && left_distance <= 40 && right_distance >= 0 && right_distance <= 40)
// 		{
// 			if (error_current1 >= 0)
// 			{
// 				OCR1A = 0.5*ICR1;
// 				OCR1B = 0.5*ICR1 - steer_signal_1;
// 			}
// 			else
// 			{
// 				OCR1A = 0.5*ICR1 + steer_signal_1;
// 				OCR1B = 0.5*ICR1;
// 			}
// 		}
// 		else if (left_distance > 40 && right_distance > 40)
// 		{
// 			//ROTERA 90 GRADER HÖGER
// 		}
// 		else if (right_distance > 0 && right_distance < 40 && left_distance > 40)
// 		{
// 			if (error_current1 >= 0)
// 			{
// 				OCR1A = 0.5*ICR1;
// 				OCR1B = 0.5*ICR1 - steer_signal_2;
// 			}
// 			else
// 			{
// 				OCR1A = 0.5*ICR1 + steer_signal_2;
// 				OCR1B = 0.5*ICR1;
// 			}
// 		}
// 		else 
// 		{
// 			//ROTERA 90 GRADER HÖGER
// 		}
// 		/* >>>>>>>>>>>>>>>>>>>>>>>THIS SECTION ALSO REGULATES THE set_speed<<<<<<<<<<<<<<<<<<<<<<<<<<
// 		float steer_signal_1 = Steer_signal1();
// 		float steer_signal_2 = Steer_signal2();
// 		float set_speed = Set_speed();
// 		
// 		if (left_distance >= 0  && left_distance <= 40 && right_distance >= 0 && right_distance <= 40)
// 		{
// 			if (error_current1 >= 0)
// 			{
// 				OCR1A = set_speed*ICR1;
// 				OCR1B = set_speed*ICR1 - steer_signal_1;
// 			}
// 			else
// 			{
// 				OCR1A = set_speed*ICR1 + steer_signal_1;
// 				OCR1B = set_speed*ICR1;
// 			}
// 		}
// 		else if (left_distance > 40 && right_distance > 40)
// 		{
// 			//ROTERA 90 GRADER HÖGER
// 		}
// 		else if (right_distance > 0 && right_distance < 40 && left_distance > 40)
// 		{
// 			if (error_current1 >= 0)
// 			{
// 				OCR1A = set_speed*ICR1;
// 				OCR1B = set_speed*ICR1 - steer_signal_2;
// 			}
// 			else
// 			{
// 				OCR1A = set_speed*ICR1 + steer_signal_2;
// 				OCR1B = set_speed*ICR1;
// 			}
// 		}
// 		else
// 		{
// 			//ROTERA 90 GRADER HÖGER
// 		}
// 		>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 		*/	
//     }
// }

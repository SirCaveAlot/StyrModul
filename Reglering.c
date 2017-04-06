/*
 * Reglering.c
 *
 * Created: 4/3/2017 1:40:07 PM
 *  Author: guswe541
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "PWM_SirCave.h"

unsigned short right_distance = 5;
unsigned short left_distance = 15;
short error_prior1 = 0;
short error_prior2 = 0;
short error_prior3 = 0;
short error_current1;
short error_current2;
short error_current3;
uint8_t proportional_gain1 = 10;
uint8_t proportional_gain2 = 10;
uint8_t proportional_gain3 = 10;
float derivative_gain1 = 0.5;
float derivative_gain2 = 0.5;
float derivative_gain3 = 0.5;
uint8_t iteration_time = 20;
//
unsigned short front_distance;
short error_prior_s;
short error_current_s;
uint8_t proportional_gain_s = 10;
float derivative_gain_s = 0.5;
//
//
unsigned short rotation_speed;
unsigned short rotation_angle = 0;
short error_prior_rot;
short error_current_rot;
uint8_t proportional_gain_rot = 10;
float derivative_gain_rot = 0.5;
//

void IR_conversion1(uint8_t val_left)
{
	left_distance = (55.25 * exp(-0.05762 * val_left)) + (14.2 * exp(-0.009759 * val_left));
}

void IR_conversion2(uint8_t val_right)
{
	right_distance = (55.25 * exp(-0.05762 * val_right)) + (14.2 * exp(-0.009759 * val_right));	
}

/*
>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM LIDAR WHEN POINTING FORWARD HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

/*
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PUT CONVERSION FROM GYRO HERE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

short Steer_signal1() //steersignal when both sides detectable
{
	error_prior1 = error_current1;
	error_current1 = right_distance - left_distance;
	short u1 = proportional_gain1*error_prior1 + derivative_gain1*(error_current1 - error_prior1)*(1/iteration_time);
	return u1;	
}

short Steer_signal2() //steersignal when right side detectable
{
	error_prior2 = error_current2;
	error_current2 = right_distance - 10;
	short u2 = proportional_gain2*error_prior2 + derivative_gain2*(error_current2 - error_prior2)*(1/iteration_time);	
	return u2;
}

short Steer_signal3() //steersignal when left side detectable
{
	error_prior3 = error_current3;
	error_current3 = 10 - left_distance;
	short u3 = proportional_gain3*error_prior3 + derivative_gain3*(error_current3 - error_prior3)*(1/iteration_time);
	return u3;
}

void Hallway_control1()
{
short steer_signal_1 = Steer_signal1();
short set_speed = Set_speed();
	if (error_current1 >= 0)
	{
		OCR1A = set_speed*ICR1;
		OCR1B = set_speed*ICR1 - steer_signal_1;
	}
	else
	{
		OCR1A = set_speed*ICR1 + steer_signal_1;
		OCR1B = set_speed*ICR1;
	}
}

void Hallway_control_right()
{
	short steer_signal_2 = Steer_signal2();
	short set_speed = Set_speed();
	if (error_current1 >= 0)
	{
		OCR1A = set_speed*ICR1;
		OCR1B = set_speed*ICR1 - steer_signal_2;
	}
	else
	{
		OCR1A = set_speed*ICR1 + steer_signal_2;
		OCR1B = set_speed*ICR1;
	}
}

short Set_speed() //sets speed given distance to obstacle ahead and then stops
{
	error_prior_s = error_current_s;
	error_current_s = front_distance - 25; //stops when LIDAR is 25 cm from obstacle ahead
	short u_s = proportional_gain_s*error_prior_s + derivative_gain_s*(error_current_s - error_prior_s)*(1/iteration_time);
	if (u_s <= 1)
	{
		return u_s;
	}
	else
	{
		return 1;
	}
}

void Hallway_control_left()
{
	short steer_signal_3 = Steer_signal3();
	short set_speed = Set_speed();
	if (error_current3 >= 0)
	{
		OCR1A = set_speed*ICR1;
		OCR1B = set_speed*ICR1 - steer_signal_3;
	}
	else
	{
		OCR1A = set_speed*ICR1 + steer_signal_3;
		OCR1B = set_speed*ICR1;
	}
}

void Rotate(uint16_t angle, char direction) //rotates robot 
{
	error_prior_rot = error_current_rot;
	rotation_angle = rotation_angle + rotation_speed*iteration_time;
	error_prior_rot = angle - rotation_angle;
	short u_rot = proportional_gain_rot*error_prior_rot + derivative_gain_rot*(error_current_rot - error_prior_rot)*(1/iteration_time);
	if (u_rot <= 1)
	{
		if (direction == 'H')
		{
			OCR1A = u_rot*ICR1;
			OCR1B = u_rot*ICR1;
			PORTA = (1 << PORTA0) | (0 << PORTA1);
		}
		else
		{
			OCR1A = u_rot*ICR1;
			OCR1B = u_rot*ICR1;
			PORTA = (0 << PORTA0) | (1 << PORTA1);
		}
	}
	else
	{
		if (direction == 'H')
		{
			OCR1A = ICR1;
			OCR1B = ICR1;
			PORTA = (1 << PORTA0) | (0 << PORTA1);
		}
		else
		{
			OCR1A = ICR1;
			OCR1B = ICR1;
			PORTA = (0 << PORTA0) | (1 << PORTA1);
		}
		
	}
}


int main(void)
{
	Timer1_init();
	DDRD = 0xFF;
	TCNT1 = ICR1 - 2;
	
    while(1)
    {
		short steer_signal_1 = Steer_signal1();
		short steer_signal_2 = Steer_signal2();
		
		if (left_distance >= 0  && left_distance <= 40 && right_distance >= 0 && right_distance <= 40)
		{
			if (error_current1 >= 0)
			{
				OCR1A = 0.5*ICR1;
				OCR1B = 0.5*ICR1 - steer_signal_1;
			}
			else
			{
				OCR1A = 0.5*ICR1 + steer_signal_1;
				OCR1B = 0.5*ICR1;
			}
		}
		else if (left_distance > 40 && right_distance > 40)
		{
			//ROTERA 90 GRADER HÖGER
		}
		else if (right_distance > 0 && right_distance < 40 && left_distance > 40)
		{
			if (error_current1 >= 0)
			{
				OCR1A = 0.5*ICR1;
				OCR1B = 0.5*ICR1 - steer_signal_2;
			}
			else
			{
				OCR1A = 0.5*ICR1 + steer_signal_2;
				OCR1B = 0.5*ICR1;
			}
		}
		else 
		{
			//ROTERA 90 GRADER HÖGER
		}
		/* >>>>>>>>>>>>>>>>>>>>>>>THIS SECTION ALSO REGULATES THE set_speed<<<<<<<<<<<<<<<<<<<<<<<<<<
		short steer_signal_1 = Steer_signal1();
		short steer_signal_2 = Steer_signal2();
		short set_speed = Set_speed();
		
		if (left_distance >= 0  && left_distance <= 40 && right_distance >= 0 && right_distance <= 40)
		{
			if (error_current1 >= 0)
			{
				OCR1A = set_speed*ICR1;
				OCR1B = set_speed*ICR1 - steer_signal_1;
			}
			else
			{
				OCR1A = set_speed*ICR1 + steer_signal_1;
				OCR1B = set_speed*ICR1;
			}
		}
		else if (left_distance > 40 && right_distance > 40)
		{
			//ROTERA 90 GRADER HÖGER
		}
		else if (right_distance > 0 && right_distance < 40 && left_distance > 40)
		{
			if (error_current1 >= 0)
			{
				OCR1A = set_speed*ICR1;
				OCR1B = set_speed*ICR1 - steer_signal_2;
			}
			else
			{
				OCR1A = set_speed*ICR1 + steer_signal_2;
				OCR1B = set_speed*ICR1;
			}
		}
		else
		{
			//ROTERA 90 GRADER HÖGER
		}
		>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		*/	
    }
}


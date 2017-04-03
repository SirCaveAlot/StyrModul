/*
 * Reglering.c
 *
 * Created: 4/3/2017 1:40:07 PM
 *  Author: guswe541
 */ 


#include <avr/io.h>
#include <math.h>

double right_distance;
double left_distance;
double error_prior1;
double error_current1;
double u1;
double proportional_gain1;
double derivative_gain1;
double iteration_time;
double forward_distance;


void IR_conversion1(uint8_t val_left)
{
	left_distance = (55.25 * exp(-0.05762 * val_left)) + (14.2 * exp(-0.009759 * val_left));
	
}

void IR_conversion2(uint8_t val_right)
{
	right_distance = (55.25 * exp(-0.05762 * val_right)) + (14.2 * exp(-0.009759 * val_right));
	
}

void steersignal1(double left_distance, double right_distance)
{
	error_prior1 = 0;
	while(1)
	{
		error_current1 = right_distance - left_distance;
		u1 = proportional_gain1*error_prior1 + derivative_gain1*(error_current1 - error_prior1)*(1/iteration_time);
		error_prior1 = error_current1;
	}
	
}

void steersignal2(double left_distance, double right_distance)
{
	error_prior1 = 0;
	while(1)
	{
		error_current1 = right_distance - 10;
		u1 = proportional_gain1*error_prior1 + derivative_gain1*(error_current1 - error_prior1)*(1/iteration_time);
		error_prior1 = error_current1;
	}
	
}

int main(void)
{
    while(1)
    {
		if (left_distance >= 3  && left_distance <= 40 && right_distance >= 3 && right_distance <= 40)
		{
			if (error_current1 >= 0)
			{
				Set_speed_left(velocity_left, velocity_right) = 0.5*ICR1;
				Set_speed_right(velocity_left, velocity_right) = 0.5*ICR1 - steersignal1(left_distance, right_distance);
			}
			else
			{
				Set_speed_left(velocity_left, velocity_right) = 0.5*ICR1 + steersignal1(left_distance, right_distance);
				Set_speed_right(velocity_left, velocity_right) = 0.5*ICR1;
			}
		}
		else if (left_distance > 40 && right_distance > 40)
		{
			//SVÄNG HÖGER
		}
		else if (right_distance > 0 && right_distance < 40 && left_distance > 40)
		{
			if (error_current1 >= 0)
			{
				Set_speed_left(velocity_left, velocity_right) = 0.5*ICR1;
				Set_speed_right(velocity_left, velocity_right) = 0.5*ICR1 - steersignal2(left_distance, right_distance);
			}
			else
			{
				Set_speed_left(velocity_left, velocity_right) = 0.5*ICR1 + steersignal2(left_distance, right_distance);
				Set_speed_right(velocity_left, velocity_right) = 0.5*ICR1;
			}
		}
		else 
		{
			//SVÄNG HÖGER
		}
    }
}


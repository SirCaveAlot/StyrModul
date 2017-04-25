/*
 * Sensor_values.c
 *
 * Created: 4/20/2017 3:32:28 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>

uint8_t left_distance;
uint8_t right_distance;
uint8_t angle;
uint8_t gyro_rotation_speed;
uint8_t forward_distance;
uint8_t velocity;
uint8_t travelled_distance;
uint8_t LIDAR_angle;
uint8_t LIDAR_rotation_speed;
 
//-------------------------IR sensors------------------------------

void IR_conversion_left(uint8_t value_left)
{
	left_distance = (55.25 * exp(-0.05762 * value_left)) + (14.2 * exp(-0.009759 * value_left));
}

void IR_conversion_right(uint8_t value_right)
{
	right_distance = (55.25 * exp(-0.05762 * value_right)) + (14.2 * exp(-0.009759 * value_right));
}


//---------------------------Gyro----------------------------------



//--------------------------LIDAR----------------------------------


//--------------------------Wheels---------------------------------


//-------------------------IR sensors------------------------------


//-------------------------IR sensors------------------------------
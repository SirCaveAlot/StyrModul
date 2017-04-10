/*
 * LIDAR_PWM.c
 *
 * Created: 4/10/2017 9:00:56 AM
 *  Author: guswe541
 */ 

#define F_CPU 14745600UL
#include <avr/io.h>
#include "PWM_SirCave.h"

void LIDAR_on()
{
	OCR2A = 0.95*ICR1;
}

void LIDAR_off()
{
	OCR2A = 0;
}

int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
    }
}
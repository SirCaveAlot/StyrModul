/*
 * PWM_SirCave.h
 *
 * Created: 3/31/2017 2:24:41 PM
 *  Author: gusst967
 */ 


#ifndef PWM_SIRCAVE_H_
#define PWM_SIRCAVE_H_

void Timer1_init();

void Set_speed_right(float);

void Set_speed_left(float);

void Drive_forward(float, float);

void Drive_backwards(float, float);

void Rotate_clockwise(float, float);

void Rotate_counter_clockwise(float, float);

#endif /* PWM_SIRCAVE_H_ */
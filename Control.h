/*
 * Control.h
 *
 * Created: 4/25/2017 2:13:30 PM
 *  Author: gusst967
 */ 


#ifndef CONTROL_H_
#define CONTROL_H_

float Steer_signal_both();

float Steer_signal_right();

float Steer_signal_left();

void Direction(bool);

void Hallway_control(bool);

void Hallway_control_both();

void Hallway_control_left();

void Hallway_control_right();

void Rotation_control(bool);

float Set_speed();

float Correct_to_center_of_tile();

void Speed_test();

extern bool update_control;

#endif /* CONTROL_H_ */
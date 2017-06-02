/*
 * Control.h
 *
 * Created: 4/19/2017 
 * Author: Gustav Strandberg, gusst967
		   Gustaf Westerholm, guswe541
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

bool Correct_angle_to_wall();

void Straighten_up_robot();

extern bool update_control;
extern bool after_right_turn;

#endif /* CONTROL_H_ */
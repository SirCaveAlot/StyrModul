/*
 * Control.h
 *
 * Created: 4/25/2017 2:13:30 PM
 *  Author: gusst967
 */ 


#ifndef CONTROL_H_
#define CONTROL_H_

float Steer_signal1();

float Steer_signal2();

float Steer_signal3();

void Direction(bool);

void Hallway_control(bool);

void Hallway_control_both();

void Hallway_control_left();

void Hallway_control_right();

// void Rotate(uint16_t, char);

float Set_speed();

#endif /* CONTROL_H_ */
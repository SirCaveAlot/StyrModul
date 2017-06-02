/*
 * Modes.h
 *
 * Created: 2017-05-05
 * Author: Gustav Strandberg, gusst967
 */ 


#ifndef MODES_H_
#define MODES_H_

void Mode_loop();

void Autonomous_mode();

void Manual_mode();

volatile extern uint8_t mode;
extern uint8_t last_mode;
extern bool competition_mode;
extern bool autonomous;
volatile extern bool mode_complete;
extern bool turn_around;

#endif /* MODES_H_ */
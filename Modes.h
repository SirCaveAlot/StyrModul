/*
 * Modes.h
 *
 * Created: 2017-05-05 16:51:46
 *  Author: Deep
 */ 


#ifndef MODES_H_
#define MODES_H_

void Mode_loop();

void Autonomous_mode();

void Manual_mode();

extern uint8_t mode;
extern uint8_t last_mode;
extern uint8_t competition_mode;
extern bool autonomous;
extern bool mode_complete;

#endif /* MODES_H_ */
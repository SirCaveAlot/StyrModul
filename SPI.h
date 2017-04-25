
/*
 * SPI.h
 *
 * Created: 4/3/2017 2:11:20 PM
 *  Author: gusst967
 */ 


#ifndef SPI_H_
#define SPI_H_

extern uint8_t right_distance;
extern uint8_t left_distance;
extern bool left_right;
extern uint8_t SPI_receiving_counter;

void Spi_init();

void QueueInit();

void QueuePut(uint8_t new);

void QueueGet(uint8_t *old);

#endif
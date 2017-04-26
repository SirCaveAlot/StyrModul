
/*
 * SPI.h
 *
 * Created: 4/3/2017 2:11:20 PM
 *  Author: gusst967
 */ 


#ifndef SPI_H_
#define SPI_H_

extern bool left_right;
extern volatile uint8_t SPI_receiving_counter;

void Spi_init();

void SPI_queue_init(void);

void SPI_queue_put(uint8_t);

void SPI_queue_get(uint8_t *old);

uint8_t SPI_queue_peek();

void SPI_queue_remove();

uint8_t SPI_queue_length();

void Dequeue_SPI_queue();

#endif
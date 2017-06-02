
/*
 * SPI.c
 *
 * Created: 4/3/2017
 * Author: Gustav Strandberg, gusst967
 */


#ifndef SPI_H_
#define SPI_H_

void Spi_init();

void SPI_queue_init(void);

void SPI_queue_put(uint8_t);

void SPI_queue_get(uint8_t *);

uint8_t SPI_queue_peek(uint8_t);

void SPI_queue_remove();

void Dequeue_SPI_queue();

void Start_dequeuing();

#endif
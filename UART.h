/*
 * UART.h
 *
 * Created: 3/31/2017
 * Author: Gustav Strandberg, gusst967
 */ 


#ifndef UART_H_
#define UART_H_

void USART_Init(unsigned int);

void UART_transmission(uint8_t);

void UART_queue_init(void);

void UART_queue_put(uint8_t);

void UART_queue_get(uint8_t *);

uint8_t UART_queue_peek(uint8_t);

void UART_queue_remove();

void Dequeue_UART_queue();

extern volatile uint8_t UART_queue_out;
extern volatile uint8_t UART_queue_in;
extern volatile uint8_t UART_queue_length;

#endif
/*
 * UART.h
 *
 * Created: 4/3/2017 1:54:29 PM
 *  Author: gusst967
 */


#ifndef UART_H_
#define UART_H_

void USART_Init(unsigned int);

void Data_transmission(char);

void UART_queue_init(void);

void UART_queue_put(uint8_t);

void UART_queue_get(uint8_t *);

uint8_t UART_queue_peek(uint8_t);

void UART_queue_remove();

void Dequeue_UART_queue();

extern uint8_t UART_queue_length;

extern uint8_t UART_queue_out;

extern uint8_t mode;

#endif

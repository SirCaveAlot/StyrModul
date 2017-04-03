/*
 * UART.h
 *
 * Created: 4/3/2017 1:54:29 PM
 *  Author: gusst967
 */ 


#ifndef UART_H_
#define UART_H_

void USART_Init(unsigned int);

void USART_Transmit(char);

char USART_Receive(void);

void Interrupt_Init();

#endif
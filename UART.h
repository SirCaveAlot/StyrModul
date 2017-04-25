/*
 * UART.h
 *
 * Created: 4/3/2017 1:54:29 PM
 *  Author: gusst967
 */ 


#ifndef UART_H_
#define UART_H_

void USART_Init(unsigned int);

void Interrupt_Init();

void Data_transmission(char);

extern char mode;

#endif
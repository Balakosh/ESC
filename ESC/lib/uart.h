/*
 * uart.h
 *
 * Created: 22.04.2018 11:54:00
 *  Author: Akeman
 */ 

#ifndef UART_H_
#define UART_H_

int8_t USART_0_init();
void uartWriteC(const uint8_t data);
void uart_puts(char *s);
uint8_t uartReadChar();

#endif /* UART_H_ */
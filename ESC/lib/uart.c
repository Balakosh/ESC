/*
 * uart.c
 *
 * Created: 22.04.2018 11:37:08
 *  Author: Akeman
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <stdio.h>
 #include <stdbool.h>
 #include "../main.h"

 void USART_0_write(const uint8_t data);

 int USART_0_printCHAR(char character, FILE *stream)
 {
	 USART_0_write(character);
	 return 0;
 }

 FILE USART_0_stream = FDEV_SETUP_STREAM(USART_0_printCHAR, NULL, _FDEV_SETUP_WRITE);

 int8_t USART_0_init()
 {
	 /* Enable USART0 */
	 PRR &= ~(1 << PRUSART0);

	 #define BAUD 9600

	 #include "setbaud.h"

	 UBRR0H = UBRRH_VALUE;
	 UBRR0L = UBRRL_VALUE;

	 UCSR0A = USE_2X << U2X0 | 0 << MPCM0;  /* Multi-processor Communication Mode: disabled */

	 UCSR0B = 1 << RXCIE0    /* RX Complete Interrupt Enable: enabled */
	 | 0 << UDRIE0  /* USART Data Register Empty Interupt Enable: disabled */
	 | 1 << RXEN0   /* Receiver Enable: enabled */
	 | 1 << TXEN0   /* Transmitter Enable: enabled */
	 | 0 << UCSZ02; /*  */

	 #if defined(__GNUC__)
	 stdout = &USART_0_stream;
	 #endif

	 return 0;
 }

 void USART_0_enable()
 {
	 UCSR0B |= ((1 << TXEN0) | (1 << RXEN0));
 }

 void USART_0_enable_rx()
 {
	 UCSR0B |= (1 << RXEN0);
 }

 void USART_0_enable_tx()
 {
	 UCSR0B |= (1 << TXEN0);
 }

 void USART_0_disable()
 {
	 UCSR0B &= ~((1 << TXEN0) | (1 << RXEN0));
 }

 uint8_t USART_0_get_data()
 {
	 return UDR0;
 }

 bool USART_0_is_tx_ready()
 {
	 return (UCSR0A & (1 << UDRE0));
 }

 bool USART_0_is_rx_ready()
 {
	 return (UCSR0A & (1 << RXC0));
 }

 bool USART_0_is_tx_busy()
 {
	 return (!(UCSR0A & (1 << TXC0)));
 }

 uint8_t USART_0_read()
 {
	 while (!(UCSR0A & (1 << RXC0)))
	 ;
	 return UDR0;
 }

 void USART_0_write(const uint8_t data)
 {
	 while (!(UCSR0A & (1 << UDRE0)))
	 ;
	 UDR0 = data;
	 ;
 }

ISR(USART_RX_vect)
{
	unsigned char nextChar;

	nextChar = UDR0;
	if( uart_str_complete == 0 )
	{
		if( nextChar != '\n' &&
		nextChar != '\r' &&
		uart_str_count < UART_MAXSTRLEN )
		{
			uart_string[uart_str_count] = nextChar;
			uart_str_count++;
		}
		else
		{
			uart_string[uart_str_count] = '\0';
			uart_str_count = 0;
			uart_str_complete = 1;
		}
	}
}
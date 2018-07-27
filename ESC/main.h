/*
 * main.h
 *
 * Created: 07.07.2018 11:48:05
 *  Author: Akeman
 */ 

#ifndef MAIN_H_
#define MAIN_H_

// ============ general definitions ==========================
#define FCPU    16000000               // Frequenz vom Quarz
#define PWM_F   FCPU/2000              // Für Berechnung der PWM-Frequenz

// ====================== parameters ==================

#define ANLAUF_DUTY_CYCLE   20         // 0-100% von PWM, während dem Anlauf
#define MINIMUM_DUTY_CYCLE  30         // 0-100% von PWM, direkt nach Anlauf
#define PWM_FREQUENZ        20        // in khz

// ==================== PIN-definitions =========================

#define LED1_OUTPUT     DDRD |= (1<<PD2);
#define LED1_ON         PORTD |= (1<<PD2);
#define LED1_OFF        PORTD &= ~(1<<PD2);
#define LED1_TOGGLE     PORTD ^= (1<<PD2);

#define PHASE_A_SD_DDR_OUTPUT  DDRD |= (1<<PD3);
#define PHASE_A_SD_HI    PORTD |= (1<<PD3);
#define PHASE_A_SD_LO    PORTD &= ~(1<<PD3);

#define PHASE_A_IN_DDR_OUTPUT    DDRD |= (1<<PD7);
#define PHASE_A_IN_DDR_PWM      DDRD &= ~(1<<PD7);
#define PHASE_A_IN_HI      PORTD |= (1<<PD7);
#define PHASE_A_IN_LO      PORTD &= ~(1<<PD7);

#define PHASE_B_SD_DDR_OUTPUT  DDRD |= (1<<PD4);
#define PHASE_B_SD_HI    PORTD |= (1<<PD4);
#define PHASE_B_SD_LO    PORTD &= ~(1<<PD4);

#define PHASE_B_IN_DDR_OUTPUT    DDRB |= (1<<PB0);
#define PHASE_B_IN_DDR_PWM      DDRB &= ~(1<<PB0);
#define PHASE_B_IN_HI      PORTB |= (1<<PB0);
#define PHASE_B_IN_LO      PORTB &= ~(1<<PB0);

#define PHASE_C_SD_DDR_OUTPUT  DDRD |= (1<<PD5);
#define PHASE_C_SD_HI    PORTD |= (1<<PD5);
#define PHASE_C_SD_LO    PORTD &= ~(1<<PD5);

#define PHASE_C_IN_DDR_OUTPUT    DDRB |= (1<<PB2);
#define PHASE_C_IN_DDR_PWM      DDRB &= ~(1<<PB2);
#define PHASE_C_IN_HI      PORTB |= (1<<PB2);
#define PHASE_C_IN_LO      PORTB &= ~(1<<PB2);

#define PWM_OUTPUT      DDRB |= (1<<PB1);
#define PWM_ON        PORTB |= (1<<PB1);
#define PWM_OFF        PORTB &= ~(1<<PB1);
#define PWM_STATUS      (PINB && (1<<PB1))

#define NULL_A_DDR      DDRC &= ~(1<<PC0); PORTC &= ~(1<<PC0);
#define NULL_B_DDR      DDRC &= ~(1<<PC1); PORTC &= ~(1<<PC1);
#define NULL_C_DDR      DDRC &= ~(1<<PC2); PORTC &= ~(1<<PC2);
#define MITTEL_DDR      DDRD &= ~(1<<PD6); PORTD &= ~(1<<PD6);

// ====================== UART ==================

#define UART_MAXSTRLEN 255

volatile uint8_t uart_str_complete;
volatile uint8_t uart_str_count;
char uart_string[UART_MAXSTRLEN + 1];

#endif /* MAIN_H_ */
